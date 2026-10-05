/*
  FrogCO2Alarm
  SparkFun Qwiic Pocket Dev Board - ESP32-C6
  + SparkFun CO2 Sensor - STCC4/SHT40 (Qwiic)
  + I2S class-D amp (e.g. MAX98357A) driving a small speaker

  - Reads CO2 once per second from the STCC4.
  - Prints "CO2_ppm" and "Threshold_ppm" in Arduino Serial Plotter format (two lines).
  - While CO2 is above the threshold, a calm frog chorus plays: random snippets of three
    clips, at random offsets, lengths, gaps and levels, up to two overlapping at once,
    so it never sounds like a straight loop. The whole chorus fades in smoothly when the
    alarm starts and fades out when CO2 drops below (threshold - hysteresis).
  - Type a number in the Serial Monitor/Plotter input (e.g. "1200") to change the threshold.

  Wiring (amp -> ESP32-C6 Pocket):
    DIN  -> IO2
    BCLK -> IO3
    LRC  -> IO4   (word select)
    VIN  -> 3.3V (or VUSB for more headroom), GND -> GND
  STCC4: Qwiic cable (SDA IO6, SCL IO7, I2C address 0x64)

  Requires:
    - Arduino-ESP32 core 3.x (for ESP_I2S.h)
    - "SparkFun STCC4 Arduino Library" (Library Manager)
    - frog_sounds.h in this sketch folder (~587 KB of audio; fits the default 1.2 MB app
      partition)

  Notes: the ESP32-C6 has no FPU, so the audio mixer uses integer (fixed-point) math only.
*/

#include <Wire.h>
#include <ESP_I2S.h>
#include <SparkFun_STCC4.h>
#include "frog_sounds.h"

// =================== User settings ===================
static int16_t co2ThresholdPpm = 1000;            // alarm level (ppm), changeable over Serial
static const int16_t CO2_HYSTERESIS_PPM = 50;     // must fall this far below threshold to stop

static const uint32_t MAX_VOLUME_PCT  = 60;       // overall volume at full fade-in (0-100)
static const uint32_t FADE_IN_MS      = 4000;     // chorus fade-in when the alarm starts
static const uint32_t FADE_OUT_MS     = 2000;     // chorus fade-out when the alarm clears

// Chorus "randomizer"
static const uint8_t  MAX_VOICES      = 2;        // snippets allowed to overlap
static const uint32_t SNIPPET_MIN_MS  = 2500;     // length of each random snippet
static const uint32_t SNIPPET_MAX_MS  = 6000;
static const uint32_t SPAWN_MIN_MS    = 1500;     // time between snippet starts
static const uint32_t SPAWN_MAX_MS    = 4500;
static const uint32_t SNIPPET_FADE_MS = 300;      // soft edges on each snippet
static const uint32_t SNIPPET_GAIN_MIN_PCT = 50;  // per-snippet level, randomized so the
static const uint32_t SNIPPET_GAIN_MAX_PCT = 80;  //   "frogs" sound near and far

// =================== Pins ===================
static const int PIN_I2S_DOUT = 2;   // -> amp DIN
static const int PIN_I2S_BCLK = 3;   // -> amp BCLK
static const int PIN_I2S_WS   = 4;   // -> amp LRC
static const int PIN_SDA      = 6;   // Qwiic
static const int PIN_SCL      = 7;   // Qwiic

// =================== Globals ===================
I2SClass       i2s;
SfeSTCC4ArdI2C co2Sensor;

static volatile bool alarmActive = false;   // written by loop(), read by audio task
static int16_t lastCO2 = 0;

// =================== Audio mixer ===================
static const uint32_t SR = FROG_SAMPLE_RATE;
static inline uint32_t msToSamples(uint32_t ms) { return (uint32_t)((uint64_t)ms * SR / 1000); }

static inline uint32_t randRange(uint32_t lo, uint32_t hi)   // inclusive
{
    return lo + (esp_random() % (hi - lo + 1));
}

struct Voice
{
    bool     active;
    uint32_t base;     // index into frogPCM where this snippet starts
    uint32_t n;        // samples played so far
    uint32_t total;    // snippet length in samples
    uint32_t fade;     // edge fade length in samples
    int32_t  gainQ15;  // per-snippet level
};

static Voice   voices[MAX_VOICES];
static uint8_t lastClip = 0xFF;

static void spawnVoice()
{
    Voice *v = nullptr;
    for (auto &cand : voices)
        if (!cand.active) { v = &cand; break; }
    if (!v) return;

    // Pick a clip, avoiding the same clip twice in a row
    uint8_t c = esp_random() % FROG_NUM_CLIPS;
    if (FROG_NUM_CLIPS > 1 && c == lastClip) c = (c + 1 + esp_random() % (FROG_NUM_CLIPS - 1)) % FROG_NUM_CLIPS;
    lastClip = c;

    const FrogClip &clip = frogClips[c];
    uint32_t len = msToSamples(randRange(SNIPPET_MIN_MS, SNIPPET_MAX_MS));
    if (len > clip.length) len = clip.length;
    uint32_t offset = randRange(0, clip.length - len);   // random start point in the clip

    v->base    = clip.start + offset;
    v->n       = 0;
    v->total   = len;
    v->fade    = min(msToSamples(SNIPPET_FADE_MS), len / 2);
    v->gainQ15 = (int32_t)(randRange(SNIPPET_GAIN_MIN_PCT, SNIPPET_GAIN_MAX_PCT) * 32767 / 100);
    v->active  = true;
}

// Streams continuously (silence when idle) so the amp never sees a stalled clock / pop.
static void audioTask(void *)
{
    const uint32_t ENV_ONE  = 1UL << 24;                 // master envelope full scale
    const uint32_t stepIn   = ENV_ONE / msToSamples(FADE_IN_MS);
    const uint32_t stepOut  = ENV_ONE / msToSamples(FADE_OUT_MS);

    const size_t FRAMES = 256;
    static int16_t buf[FRAMES * 2];                      // interleaved L/R

    uint32_t env = 0;                                    // master envelope 0..ENV_ONE
    uint32_t spawnCountdown = 0;                         // samples until next snippet

    for (;;)
    {
        for (size_t i = 0; i < FRAMES; i++)
        {
            bool on = alarmActive;

            // ---- Master envelope ----
            if (on)  env = (env + stepIn  > ENV_ONE) ? ENV_ONE : env + stepIn;
            else     env = (env < stepOut) ? 0 : env - stepOut;

            if (!on && env == 0)
            {
                // Fully faded out: reset so the next alarm starts fresh
                for (auto &v : voices) v.active = false;
                spawnCountdown = 0;
                buf[2 * i] = buf[2 * i + 1] = 0;
                continue;
            }

            // ---- Schedule new snippets (only while the alarm is active) ----
            if (on)
            {
                if (spawnCountdown == 0)
                {
                    spawnVoice();
                    spawnCountdown = msToSamples(randRange(SPAWN_MIN_MS, SPAWN_MAX_MS));
                }
                else
                {
                    spawnCountdown--;
                }
            }

            // ---- Mix active snippets ----
            int32_t acc = 0;
            for (auto &v : voices)
            {
                if (!v.active) continue;

                int32_t e = 32767;                       // snippet edge envelope, Q15
                if (v.n < v.fade)                    e = (int32_t)(v.n * 32767 / v.fade);
                else if (v.total - v.n < v.fade)     e = (int32_t)((v.total - v.n) * 32767 / v.fade);

                int32_t s = frogPCM[v.base + v.n];
                s = (s * v.gainQ15) >> 15;
                acc += (s * e) >> 15;

                if (++v.n >= v.total) v.active = false;
            }

            // ---- Master gain: smoothstep curve for a soft start and soft landing ----
            int64_t e16 = env >> 8;                                          // 0..65536
            int64_t ss  = (((e16 * e16) >> 16) * (3 * 65536 - 2 * e16)) >> 16;  // Q16
            int64_t g   = ss * MAX_VOLUME_PCT / 100;

            int32_t out = (int32_t)(((int64_t)acc * g) >> 16);
            if (out > 32767)  out = 32767;
            if (out < -32768) out = -32768;

            buf[2 * i]     = (int16_t)out;   // left
            buf[2 * i + 1] = (int16_t)out;   // right (MAX98357A default is (L+R)/2)
        }
        i2s.write((uint8_t *)buf, sizeof(buf));   // blocks; paces this task at the sample rate
    }
}

// =================== Helpers ===================
static void checkSerialForThreshold()
{
    if (!Serial.available()) return;
    long v = Serial.parseInt();
    while (Serial.available()) Serial.read();      // flush rest of line
    if (v >= 400 && v <= 32000) co2ThresholdPpm = (int16_t)v;
}

// =================== Setup / loop ===================
void setup()
{
    Serial.begin(115200);
    delay(500);

    Wire.begin(PIN_SDA, PIN_SCL);
    while (!co2Sensor.begin())
    {
        Serial.println("STCC4 not detected on Qwiic - retrying...");
        delay(1000);
    }
    co2Sensor.startContinuousMeasurement();

    i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT);
    if (!i2s.begin(I2S_MODE_STD, SR, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO))
    {
        Serial.println("I2S init failed!");
        while (1) delay(1000);
    }

    xTaskCreate(audioTask, "audio", 4096, nullptr, 3, nullptr);

    delay(1000);   // STCC4 needs ~1 s after start before the first read
}

void loop()
{
    static uint32_t lastRead = 0;

    checkSerialForThreshold();

    if (millis() - lastRead >= 1000)
    {
        lastRead = millis();

        if (co2Sensor.readMeasurement() == ksfTkErrOk)
        {
            lastCO2 = co2Sensor.getCO2();

            // Threshold with hysteresis
            if (!alarmActive && lastCO2 > co2ThresholdPpm)
                alarmActive = true;
            else if (alarmActive && lastCO2 < co2ThresholdPpm - CO2_HYSTERESIS_PPM)
                alarmActive = false;
        }

        // Serial Plotter: label:value pairs -> two named lines
        Serial.print("CO2_ppm:");
        Serial.print(lastCO2);
        Serial.print(",Threshold_ppm:");
        Serial.println(co2ThresholdPpm);
    }

    delay(10);
}