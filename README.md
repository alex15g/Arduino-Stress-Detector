# 🩺 Stress Detector System (Polygraph-inspired)

An advanced embedded system designed to monitor and analyze physiological responses associated with stress in real-time.

## 🚀 Overview
This project uses an **Arduino** microcontroller to collect data from multiple sensors, processing signals to calculate a "Stress Score" (0-100%). It features an automated calibration phase for baseline establishment.

## 🛠️ Hardware Components
- **Microcontroller**: Arduino Uno/Nano
- **Biometric Sensors**: 
  - MAX30102 (Heart Rate & SpO2)
  - GSR Sensor (Galvanic Skin Response)
  - DS18B20 (Skin Temperature)
- **Activity Monitoring**: Piezo Vibration Sensor (to detect movement noise)
- **Display**: LCD 16x2 I2C

## 💻 Key Software Features
- **Layered Logic**: Distinct modules for sensor sampling, signal filtering, and scoring.
- **Adaptive Calibration**: 60-second baseline period to adjust thresholds to each user.
- **Non-blocking Architecture**: Implemented using `millis()` for real-time responsiveness.
- **Signal Filtering**: Pulse-width filtering and GSR phasic conductance calculation.

## 📊 How it Works
1. **Baseline Setup**: Measures normal BPM and Temperature.
2. **GSR Calibration**: Calibrates skin conductance for 60 seconds.
3. **Real-time Scoring**: 
   - 📈 Heart rate increase adds to score.
   - 📉 Temperature drops (common in stress) increase score.
   - ⚡ GSR spikes indicate emotional response.
   - ⚠️ Piezo detects if the user moves too much (invalidating the test).

## 📂 Repository Structure
- `/src`: Contains the `.ino` source code.
- `/docs`: Schematics, assembly photos, and technical documentation.
