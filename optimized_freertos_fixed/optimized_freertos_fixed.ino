#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX30105 particleSensor;

// ─────────────────────────────────────────────
//  HRV / Stress Classification Thresholds (ms)
// ─────────────────────────────────────────────
#define T1_RMSSD  40.0f   // Relaxed vs Moderate boundary
#define T2_RMSSD  25.0f   // Moderate vs High Stress boundary

// ─────────────────────────────────────────────
//  Signal Processing Parameters
// ─────────────────────────────────────────────
#define SAMPLE_RATE_HZ     100           // PPG sampling rate
#define RR_BUFFER_SIZE     30            // # of RR intervals for RMSSD window
#define MIN_RR_MS          300           // ~200 BPM max (physiological limit)
#define MAX_RR_MS          2000          // ~30 BPM min  (physiological limit)
#define PEAK_DETECTION_AVG 4             // Beats averaged for BPM display

// Low-pass filter coefficient (simple 1st order IIR)
// Cutoff ~5 Hz at 100 Hz sample rate: alpha = 0.24
#define LPF_ALPHA 0.24f

// RR interval circular buffer (ms)
float   rrBuffer[RR_BUFFER_SIZE];
uint8_t rrHead      = 0;
uint8_t rrCount     = 0;

// Peak detection state
long    lastPeakTime = 0;          // millis() of last detected peak
long    lastIR       = 0;          // previous filtered IR sample

// BPM rolling average (SparkFun helper)
byte    rateBuffer[PEAK_DETECTION_AVG];
byte    rateHead    = 0;
long    beatAvg     = 0;

// Low-pass filter state
float   filteredIR  = 0.0f;
bool    filterInit  = false;

// Display refresh throttle
unsigned long lastDisplayUpdate = 0;
#define DISPLAY_INTERVAL_MS 500

enum StressLevel { RELAXED, MODERATE, HIGH_STRESS, UNKNOWN };

const char* stressLabel[] = {
  "Relaxed",
  "Moderate",
  "High Stress",
  "Measuring..."
};

float     computeRMSSD();
StressLevel classifyStress(float rmssd);
void      updateDisplay(long bpm, float rmssd, StressLevel stress);
bool      initSensor();
void      addRRInterval(float rrMs);

void setup() {
  Serial.begin(115200);
  Serial.println(F("HRV Stress Monitor — Starting..."));

  // --- OLED init ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("OLED init failed — check wiring!"));
    while (true) delay(100);
  }
  // --- MAX30102 init ---
  if (!initSensor()) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println(F("MAX30102 not found!"));
    display.println(F("Check wiring."));
    display.display();
    while (true) delay(100);
  }

  Serial.println(F("System ready. Place finger on sensor."));
}

// ─────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────
void loop() {
  // Read raw IR value (100 Hz)
  long rawIR = particleSensor.getIR();

  // ── 1. Low-pass filter (IIR, ~5 Hz cutoff) ──
  if (!filterInit) {
    filteredIR = (float)rawIR;
    filterInit = true;
  } else {
    filteredIR = LPF_ALPHA * (float)rawIR + (1.0f - LPF_ALPHA) * filteredIR;
  }

  // ── 2. Finger presence check ──
  // IR < 50000 typically means no finger
  if (rawIR < 50000) {
    // Reset state when finger removed
    rrCount    = 0;
    rrHead     = 0;
    filterInit = false;
    beatAvg    = 0;

    if (millis() - lastDisplayUpdate > DISPLAY_INTERVAL_MS) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(8, 20);
      display.println(F("Place finger on"));
      display.setCursor(22, 32);
      display.println(F("sensor..."));
      display.display();
      lastDisplayUpdate = millis();
    }
    delay(10);   // maintain ~100 Hz
    return;
  }

  // ── 3. Peak detection (adaptive threshold) ──
  bool peakDetected = checkForBeat((long)filteredIR);

  if (peakDetected) {
    long now      = millis();
    long rrMs     = now - lastPeakTime;
    lastPeakTime  = now;

    // Physiological range gate
    if (rrMs >= MIN_RR_MS && rrMs <= MAX_RR_MS) {

      // ── 4. BPM rolling average ──
      int bpmInstant = 60000 / rrMs;
      rateBuffer[rateHead++] = (byte)constrain(bpmInstant, 20, 200);
      rateHead %= PEAK_DETECTION_AVG;

      beatAvg = 0;
      for (byte i = 0; i < PEAK_DETECTION_AVG; i++) beatAvg += rateBuffer[i];
      beatAvg /= PEAK_DETECTION_AVG;

      // ── 5. Store RR interval ──
      addRRInterval((float)rrMs);

      // Debug output (matches paper's CSV-style logging)
      Serial.print(F("RR_ms="));
      Serial.print(rrMs);
      Serial.print(F("  BPM="));
      Serial.print(beatAvg);
      if (rrCount >= 2) {
        float rmssd = computeRMSSD();
        StressLevel s = classifyStress(rmssd);
        Serial.print(F("  RMSSD="));
        Serial.print(rmssd, 1);
        Serial.print(F("ms  State="));
        Serial.println(stressLabel[s]);
      } else {
        Serial.println();
      }
    }
  }

  // ── 6. Refresh display at 2 Hz ──
  if (millis() - lastDisplayUpdate > DISPLAY_INTERVAL_MS) {
    float rmssd  = (rrCount >= 2) ? computeRMSSD() : -1.0f;
    StressLevel s = (rrCount >= 2) ? classifyStress(rmssd) : UNKNOWN;
    updateDisplay(beatAvg, rmssd, s);
    lastDisplayUpdate = millis();
  }

  // ── 7. Maintain ~100 Hz sampling ──
  delay(10);
}

// ─────────────────────────────────────────────
//  Add an RR interval to the circular buffer
// ─────────────────────────────────────────────
void addRRInterval(float rrMs) {
  rrBuffer[rrHead] = rrMs;
  rrHead = (rrHead + 1) % RR_BUFFER_SIZE;
  if (rrCount < RR_BUFFER_SIZE) rrCount++;
}

// ─────────────────────────────────────────────
//  Compute RMSSD over the current RR window
//
//  RMSSD = sqrt( (1/(N-1)) * sum( (RR[i+1]-RR[i])^2 ) )
//  Window = min(rrCount, RR_BUFFER_SIZE) samples (~30 s at normal HR)
// ─────────────────────────────────────────────
float computeRMSSD() {
  if (rrCount < 2) return 0.0f;

  uint8_t n = min((int)rrCount, (int)RR_BUFFER_SIZE);

  // Rebuild ordered sequence from circular buffer
  float ordered[RR_BUFFER_SIZE];
  for (uint8_t i = 0; i < n; i++) {
    // Most recent n entries ending at (rrHead - 1)
    int8_t idx = ((int)rrHead - (int)n + (int)i + RR_BUFFER_SIZE) % RR_BUFFER_SIZE;
    ordered[i] = rrBuffer[idx];
  }

  double sumSqDiff = 0.0;
  for (uint8_t i = 1; i < n; i++) {
    double diff = ordered[i] - ordered[i - 1];
    sumSqDiff += diff * diff;
  }

  return (float)sqrt(sumSqDiff / (double)(n - 1));
}

// ─────────────────────────────────────────────
//  Threshold-based stress classifier
//    Relaxed    : RMSSD >  T1 (40 ms)
//    Moderate   : T2 < RMSSD <= T1
//    High Stress: RMSSD <= T2 (25 ms)
// ─────────────────────────────────────────────
StressLevel classifyStress(float rmssd) {
  if (rmssd > T1_RMSSD)  return RELAXED;
  if (rmssd > T2_RMSSD)  return MODERATE;
  return HIGH_STRESS;
}

void updateDisplay(long bpm, float rmssd, StressLevel stress) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // ── Header ──
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("HRV Stress Monitor"));
  display.drawLine(0, 9, SCREEN_WIDTH - 1, 9, SSD1306_WHITE);

  // ── BPM ──
  display.setCursor(0, 13);
  display.print(F("HR: "));
  if (bpm > 0) {
    display.setTextSize(2);
    display.print(bpm);
    display.setTextSize(1);
    display.print(F(" BPM"));
  } else {
    display.print(F("---"));
  }

  // ── RMSSD ──
  display.setCursor(0, 33);
  display.print(F("RMSSD: "));
  if (rmssd >= 0) {
    display.print(rmssd, 1);
    display.print(F(" ms"));
  } else {
    display.print(F("---"));
  }

  // ── Stress Level ──
  display.setCursor(0, 45);
  display.print(F("Stress: "));

  // Invert background for stress label
  int labelX = 48;
  int labelW = strlen(stressLabel[stress]) * 6 + 2;
  display.fillRect(labelX - 1, 44, labelW, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(labelX, 45);
  display.print(stressLabel[stress]);
  display.setTextColor(SSD1306_WHITE);

  // ── Sample count indicator ──
  display.setCursor(0, 57);
  display.print(F("Samples: "));
  display.print(min((int)rrCount, (int)RR_BUFFER_SIZE));
  display.print(F("/"));
  display.print(RR_BUFFER_SIZE);

  display.display();
}
bool initSensor() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30102 not found. Check wiring / 3.3V supply."));
    return false;
  }

  byte ledBrightness = 60;    // 0=off, 255=50mA
  byte sampleAverage = 4;     // 1, 2, 4, 8, 16, 32
  byte ledMode       = 2;     // 1=Red only, 2=Red+IR, 3=Red+IR+Green
  int  sampleRate    = 100;   // 25, 50, 100, 200, 400, 800, 1000
  int  pulseWidth    = 411;   // 69, 118, 215, 411 (µs)
  int  adcRange      = 4096;  // 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode,
                       sampleRate, pulseWidth, adcRange);

  particleSensor.setPulseAmplitudeRed(0x0A);   // low red for proximity
  particleSensor.setPulseAmplitudeGreen(0);     // green off

  Serial.println(F("MAX30102 initialized at 100 Hz."));
  return true;
}