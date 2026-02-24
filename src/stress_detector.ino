/*
* Multi-sensor Stress Detection System (Polygraph-inspired)
* Measures Heart Rate (MAX30102), Galvanic Skin Response (GSR), 
* Temperature (DS18B20), and Micro-movements (Piezo).
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 16, 2);

// MAX30102 
MAX30105 particleSensor;

//  DS18B20 
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//  PIEZO 
// Piezo module: +5V, GND, S -> A0
#define PIEZO_PIN A0

//  GSR 
// GSR module: VCC->5V, GND->GND, SIG->A1
#define GSR_PIN A1

// GSR sampling
const int GSR_SAMPLE_MS = 120;
unsigned long lastGsrSample = 0;

// GSR calibration (after baselineDone)
bool gsrCalibrating = false;
bool gsrCalDone = false;
unsigned long gsrCalStart = 0;
long gsrCalAcc = 0;
int gsrCalCnt = 0;

// GSR filters
float gsrBase = 0;   // very slow baseline
float gsrFast = 0;   // faster tracking
float gsrScoreSmooth = 0;
int gsrLastRaw = 0;
int gsrLastPhasic = 0;
int gsrLastScore = 0;

// GSR parameters (tunable)
const unsigned long GSR_CAL_MS = 60000; // 60 sec
const int GSR_PHASIC_CLAMP = 40;        // map 0..40 -> 0..100

// PIEZO sampling
const int PIEZO_SAMPLE_MS = 10;    // 100 Hz
unsigned long lastPiezoSample = 0;

// piezo processing
int piezoBaselineRaw = 0;
int piezoEnvelope = 0;

// piezo baseline calibration
long piezoBaseSum = 0;
int  piezoBaseN = 0;
int  piezoBaseEnv = 1;     // baseline envelope (can be 1-3 for calm breathers)
int  piezoSpikeTH = 80;

// piezo window stats (5 sec)
const int PIEZO_WIN_MS = 5000;
unsigned long piezoWinStart = 0;
int piezoCurMax = 0;
int piezoActiveSamples = 0;
int piezoSpikeCount = 0;

// latest piezo classification
enum PiezoState { PZ_NORMAL = 0, PZ_MED = 1, PZ_HIGH = 2, PZ_MOVE = 3 };
PiezoState piezoState = PZ_NORMAL;

// debug values (for adding score / printing)
int piezoLastRatioMax = 0;
int piezoLastActiveMs = 0;
int piezoLastCurMax = 0;

//  -BPM-
long lastBeat = 0;
float BPM = 0;
float bpmBuffer[5] = {0};
int bpmIndex = 0;
bool bpmFilled = false;

//  -TEMP-
unsigned long lastTempMillis = 0;
float tempC = 0;

//  -BASELINE-
float bpmBaseline = 0;
float tempBaseline = 0;
int baselineCount = 0;
const int baselineSamples = 30;
bool baselineDone = false;

//  -LCD timing & Scoring-
unsigned long lastLCD = 0;
const unsigned long lcdInterval = 400;
int stressScore = 0;

// -Helper Functions-	
float getBpmAverage() {
  float sum = 0;
  int count = bpmFilled ? 5 : bpmIndex;
  if (count == 0) return BPM;
  for (int i = 0; i < count; i++) sum += bpmBuffer[i];
  return sum / count;
}

// -PIEZO HELPERS-
void piezoInitBaselineRaw() {
  long s = 0;
  for (int i = 0; i < 300; i++) {
    s += analogRead(PIEZO_PIN);
    delay(5);
  }
  piezoBaselineRaw = (int)(s / 300);
  piezoWinStart = millis();
}

void piezoUpdateOneSample() {
  int v = analogRead(PIEZO_PIN);

  // diff from slow baseline
  int diff = v - piezoBaselineRaw;
  if (diff < 0) diff = -diff;

  // very slow baseline tracking (so it doesn't "eat" breathing)
  piezoBaselineRaw = (piezoBaselineRaw * 9995 + v * 5) / 10000;

  // envelope: rise faster, decay slower
  if (diff > piezoEnvelope) piezoEnvelope = (piezoEnvelope * 7 + diff * 3) / 10;
  else                     piezoEnvelope = (piezoEnvelope * 97 + diff * 3) / 100;

  // spike detection (movement/noise)
  static int prevEnv = 0;
  if (piezoEnvelope - prevEnv > piezoSpikeTH) piezoSpikeCount++;
  prevEnv = piezoEnvelope;

  // window stats
  if (piezoEnvelope > piezoCurMax) piezoCurMax = piezoEnvelope;

  // "active" if above 2x baseline (works even when baseline is small)
  if (piezoEnvelope > piezoBaseEnv * 2) piezoActiveSamples++;
}

void piezoFinalizeWindowIfNeeded() {
  unsigned long now = millis();
  if (now - piezoWinStart < PIEZO_WIN_MS) return;

  // compute metrics
  int ratioMax = (int)((long)piezoCurMax * 100 / piezoBaseEnv);
  int activeMs = piezoActiveSamples * PIEZO_SAMPLE_MS;

  piezoLastRatioMax = ratioMax;
  piezoLastActiveMs = activeMs;
  piezoLastCurMax = piezoCurMax;

  // thresholds (tune these if needed)
  const int ABS_MED  = 15;
  const int ABS_HIGH = 25;

  const int R_MED  = 200;   // 2.0x
  const int R_HIGH = 300;   // 3.0x

  const int ACTIVE_MED_MS  = 800;
  const int ACTIVE_HIGH_MS = 1500;

  const int MOVE_SPIKES = 8;

  if (piezoSpikeCount >= MOVE_SPIKES) {
    piezoState = PZ_MOVE;
  } else if ((piezoCurMax >= ABS_HIGH && ratioMax >= R_HIGH) ||
             (activeMs >= ACTIVE_HIGH_MS && ratioMax >= R_MED)) {
    piezoState = PZ_HIGH;
  } else if ((piezoCurMax >= ABS_MED && ratioMax >= R_MED) ||
             (activeMs >= ACTIVE_MED_MS)) {
    piezoState = PZ_MED;
  } else {
    piezoState = PZ_NORMAL;
  }

  // reset window
  piezoCurMax = 0;
  piezoActiveSamples = 0;
  piezoSpikeCount = 0;
  piezoWinStart = now;
}

int piezoScoreContribution() {
  // If movement/noise, ignore (score 0)
  if (piezoState == PZ_MOVE) return 0;

  int score = 0;

  // base contribution by state
  if (piezoState == PZ_MED) score += 15;
  else if (piezoState == PZ_HIGH) score += 30;

  // extra small bonus by intensity (kept safe)
  int extra = piezoLastCurMax;          // typical: 0..80+ in your setup
  if (extra > 60) extra = 60;
  extra = extra / 3;                    // 0..20
  score += extra;

  if (score > 40) score = 40;           // piezo max contribution
  return score;
}

const char* piezoStateText() {
  switch (piezoState) {
    case PZ_NORMAL: return "N";
    case PZ_MED:    return "M";
    case PZ_HIGH:   return "H";
    case PZ_MOVE:   return "X"; // movement/noise
  }
  return "?";
}

//  -GSR HELPERS-
int gsrReadAvg() {
  long sum = 0;
  for (int i = 0; i < 15; i++) {
    sum += analogRead(GSR_PIN);
    delay(3);
  }
  return (int)(sum / 15);
}

// returns 0..30
int gsrScoreContribution() {
  if (!gsrCalDone) return 0;

  // map phasic 0..GSR_PHASIC_CLAMP -> 0..100
  int p = gsrLastPhasic;
  if (p < 0) p = 0;
  if (p > GSR_PHASIC_CLAMP) p = GSR_PHASIC_CLAMP;
  int pct = (p * 100) / GSR_PHASIC_CLAMP;

  // smooth score already 0..100
  int s = gsrLastScore;

  // convert to 0..30 contribution
  int contrib = (s * 30) / 100;
  if (contrib < 0) contrib = 0;
  if (contrib > 30) contrib = 30;
  return contrib;
}

void gsrStartCalibration() {
  gsrCalibrating = true;
  gsrCalDone = false;

  gsrCalStart = millis();
  gsrCalAcc = 0;
  gsrCalCnt = 0;

  // init filters with current reading
  int r = gsrReadAvg();
  gsrBase = r;
  gsrFast = r;
  gsrScoreSmooth = 0;
  gsrLastRaw = r;
  gsrLastPhasic = 0;
  gsrLastScore = 0;
}

void gsrUpdate() {
  unsigned long now = millis();
  if (now - lastGsrSample < GSR_SAMPLE_MS) return;
  lastGsrSample = now;

  int raw = gsrReadAvg();
  gsrLastRaw = raw;

  // If calibration hasn't started, do nothing
  if (!gsrCalibrating && !gsrCalDone) return;

  // During calibration: accumulate raw mean for 60 sec
  if (gsrCalibrating) {
    gsrCalAcc += raw;
    gsrCalCnt++;

    if (now - gsrCalStart >= GSR_CAL_MS) {
      float b = (gsrCalCnt > 0) ? ((float)gsrCalAcc / gsrCalCnt) : raw;
      gsrBase = b;
      gsrFast = b;          // IMPORTANT: align fast/base after calibration
      gsrCalibrating = false;
      gsrCalDone = true;
      gsrScoreSmooth = 0;
    }
    return;
  }

  // After calibration:
  // slow baseline follows drift very slowly
  gsrBase = 0.998f * gsrBase + 0.002f * raw;
  // fast follows quicker
  gsrFast = 0.85f * gsrFast + 0.15f * raw;

  int phasic = (int)gsrFast - (int)gsrBase;
  gsrLastPhasic = phasic;

  int p = phasic;
  if (p < 0) p = 0;
  if (p > GSR_PHASIC_CLAMP) p = GSR_PHASIC_CLAMP;

  float target = (p * 100.0f) / (float)GSR_PHASIC_CLAMP;

  // smooth score (realistic)
  gsrScoreSmooth = 0.95f * gsrScoreSmooth + 0.05f * target;
  gsrLastScore = (int)gsrScoreSmooth;
  if (gsrLastScore < 0) gsrLastScore = 0;
  if (gsrLastScore > 100) gsrLastScore = 100;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  lcd.init();
  lcd.backlight();
  lcd.print("Stress Detector");
  delay(1500);
  lcd.clear();

  if (!particleSensor.begin(Wire)) {
    lcd.print("MAX30102 ERR");
    while (1);
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeGreen(0);

  sensors.begin();
  sensors.setResolution(9);
  sensors.setWaitForConversion(false);

  // piezo init
  piezoInitBaselineRaw();

  lcd.print("PLACE FINGER");
}

void loop() {
  // -BPM-
  long irValue = particleSensor.getIR();
  bool fingerDetected = irValue > 20000;

  if (fingerDetected && checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    if (delta > 0) {
      float newBPM = 60000.0 / delta;
      if (newBPM >= 45 && newBPM <= 120) {
        BPM = newBPM;
        bpmBuffer[bpmIndex++] = BPM;
        if (bpmIndex >= 5) {
          bpmIndex = 0;
          bpmFilled = true;
        }
      }
    }
  }

  float bpmAvg = getBpmAverage();

  //  -TEMP-
  if (millis() - lastTempMillis >= 1000) {
    sensors.requestTemperatures();
    lastTempMillis = millis();
  }
  tempC = sensors.getTempCByIndex(0);

  //  -PIEZO sampling (non-blocking)-
  if (millis() - lastPiezoSample >= PIEZO_SAMPLE_MS) {
    lastPiezoSample = millis();

    piezoUpdateOneSample();
    piezoFinalizeWindowIfNeeded();
  }

  //  -BASELINE (BPM/TEMP + PIEZO baseline)-
  // We only build baseline when finger is detected and bpmAvg valid (so user is "ready")
  if (!baselineDone && fingerDetected && bpmAvg > 0) {
    bpmBaseline += bpmAvg;
    tempBaseline += tempC;
    baselineCount++;

    // piezo baseline during the same period
    piezoBaseSum += piezoEnvelope;
    piezoBaseN++;

    if (baselineCount >= baselineSamples) {
      bpmBaseline /= baselineSamples;
      tempBaseline /= baselineSamples;
      baselineDone = true;

      // finalize piezo baseline
      piezoBaseEnv = (piezoBaseN > 0) ? (int)(piezoBaseSum / piezoBaseN) : 1;
      if (piezoBaseEnv < 1) piezoBaseEnv = 1;

      // set adaptive spike threshold
      piezoSpikeTH = piezoBaseEnv * 10;
      if (piezoSpikeTH < 20) piezoSpikeTH = 20;

      // reset window after baseline
      piezoCurMax = 0;
      piezoActiveSamples = 0;
      piezoSpikeCount = 0;
      piezoWinStart = millis();

      // start GSR calibration NOW (user already has finger on MAX + electrodes on)
      gsrStartCalibration();

      // debug
      Serial.print("BASELINES DONE. bpmBase=");
      Serial.print(bpmBaseline, 1);
      Serial.print(" tempBase=");
      Serial.print(tempBaseline, 2);
      Serial.print(" piezoBaseEnv=");
      Serial.print(piezoBaseEnv);
      Serial.print(" piezoSpikeTH=");
      Serial.print(piezoSpikeTH);
      Serial.println();
      Serial.println("GSR calibration started (60 sec)...");
    }
  }

  gsrUpdate();

  //  -Stress Score-
  if (baselineDone) {
    stressScore = 0;

    // BPM contribution
    if (bpmAvg > bpmBaseline)
      stressScore += (int)((bpmAvg - bpmBaseline) * 2);

    // TEMP contribution (temp drop -> stress)
    if (tempC < tempBaseline)
      stressScore += (int)((tempBaseline - tempC) * 20);

    // PIEZO contribution
    stressScore += piezoScoreContribution();

    // GSR contribution (max 30)
    stressScore += gsrScoreContribution();

    stressScore = constrain(stressScore, 0, 100);

    // Optional debug to Serial
    static unsigned long lastDbg = 0;
    if (millis() - lastDbg > 1000) {
      lastDbg = millis();
      Serial.print("bpm=");
      Serial.print((int)bpmAvg);
      Serial.print(" base=");
      Serial.print((int)bpmBaseline);

      Serial.print(" temp=");
      Serial.print(tempC, 2);
      Serial.print(" base=");
      Serial.print(tempBaseline, 2);

      Serial.print(" piezoEnv=");
      Serial.print(piezoEnvelope);
      Serial.print(" piezoBase=");
      Serial.print(piezoBaseEnv);
      Serial.print(" pzState=");
      Serial.print(piezoStateText());

      Serial.print(" GSRraw=");
      Serial.print(gsrLastRaw);
      Serial.print(" GSRbase=");
      Serial.print((int)gsrBase);
      Serial.print(" GSRfast=");
      Serial.print((int)gsrFast);
      Serial.print(" GSRph=");
      Serial.print(gsrLastPhasic);
      Serial.print(" GSRs=");
      Serial.print(gsrLastScore);
      Serial.print(" gsrContrib=");
      Serial.print(gsrScoreContribution());

      Serial.print(" score=");
      Serial.println(stressScore);
    }
  }

  //  -LCD-
  if (millis() - lastLCD >= lcdInterval) {
    lastLCD = millis();

    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");

    if (!fingerDetected) {
      lcd.setCursor(0, 0);
      lcd.print("PLACE FINGER");
      lcd.setCursor(0, 1);
      lcd.print("ON SENSOR");
    }
    else if (!baselineDone) {
      lcd.setCursor(0, 0);
      lcd.print("CALIBRATING...");
      lcd.setCursor(0, 1);
      lcd.print(baselineCount);
      lcd.print("/");
      lcd.print(baselineSamples);
    }
    else if (!gsrCalDone) {
      // show GSR calibration progress
      lcd.setCursor(0, 0);
      lcd.print("GSR CALIBRATION");
      lcd.setCursor(0, 1);
      unsigned long passed = millis() - gsrCalStart;
      int pct = (passed >= GSR_CAL_MS) ? 100 : (int)(passed * 100 / GSR_CAL_MS);
      lcd.print(pct);
      lcd.print("%");
    }
    else {
      lcd.setCursor(0, 0);
      lcd.print("BPM:");
      lcd.print((int)bpmAvg);
      lcd.print(" PZ:");
      lcd.print(piezoStateText()); // N/M/H/X

      lcd.setCursor(0, 1);
      lcd.print("STRESS SCORE:");
      lcd.print(stressScore);
      lcd.print("%");
    }
  }
}
