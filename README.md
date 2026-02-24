# 🩺 Multi-Sensor Stress Detection System (Polygraph-inspired)

An advanced embedded system developed for monitoring physiological reactions associated with stress. The project uses real-time data fusion from multiple sensors to establish a baseline and detect deviations caused by emotional or physical stress.

## 🚀 Overview
This system collects and processes biometric data to calculate a dynamic **Stress Score (0-100%)**. It features a structured operation flow: initialization, adaptive baseline calibration, and real-time monitoring with signal filtering.

## 🛠️ Hardware Components
- **Microcontroller**: Arduino Uno/Nano
- **Biometric Sensors**: 
  - **MAX30102**: Heart Rate (BPM) & SpO2 monitoring.
  - **GSR Sensor**: Electrodermal activity (sweat gland activation).
  - **DS18B20**: High-precision skin temperature monitoring.
- **Piezoelectric Sensor**: 
  - **Dual-Role Logic**: Contributes to the overall **Stress Score** by detecting psychomotor agitation (micro-tremors) and serves as a **Data Validation** tool by identifying movement noise.

## 💻 Key Software Features
- **Signal Processing**: Implementation of digital filters for GSR (phasic conductance) and Piezo signals (envelope detection).
- **Adaptive Calibration**: 60-second baseline establishment to tailor the scoring algorithm to each individual user.
- **Non-blocking Architecture**: Optimized code using `millis()` for efficient multitasking across high-frequency sensors.
- **State Machine Logic**: `Initialization` -> `Baseline Setup` -> `GSR Calibration` -> `Active Monitoring`.

## 📊 Stress Scoring Logic
The final score is a weighted sum derived from:
1. **BPM Rise**: Heart rate increase relative to the baseline.
2. **Temperature Drop**: Physiological "cold sweat" response (vasoconstriction).
3. **GSR Spikes**: Rapid changes in skin conductance (Phasic levels).
4. **Piezo Intensity**: Physical tension and micro-movements detected by the vibration sensor.

---

## 📄 Documentation & Research
The project is backed by technical research on biometric data fusion. You can find the complete project report, including the mathematical model for stress calculation and the electronic schematics, here:
- 📑 **[Multi-Sensor Stress Detection System Report (PDF)](./docs/Stress_detection_system_documentation.pdf)**

---
## 📂 Repository Structure
- `/src`: Arduino source code (`.ino`).
- `/docs`: Technical documentation (UTCN), schematics, and experimental results.
