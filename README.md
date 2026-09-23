# 🩺 Multi-Sensor Biometric Data Fusion & HiL Validation System

An advanced embedded system engineered for the real-time monitoring and processing of physiological data. The project utilizes sensor fusion to establish an adaptive biometric baseline and detect deviations caused by physiological stress, validated through an automated Python-based testing architecture.

## 🚀 Overview
This system collects and processes high-frequency biometric data to compute a dynamic **Physiological Stress Score (0-100%)**. It features a strict Finite State Machine (FSM) operation flow: initialization, adaptive baseline calibration, real-time digital filtering, and continuous monitoring. To ensure data integrity and system reliability, the architecture integrates a custom **Hardware-in-the-Loop (HiL) automated testing bench**.

## 🛠️ Hardware Architecture
- **Microcontroller**: Arduino Uno/Nano (C++ Firmware)
- **Biometric Sensors**: 
  - **MAX30102 (I2C)**: Optical pulse oximetry for continuous Heart Rate (BPM) extraction.
  - **GSR Sensor (Analog)**: Electrodermal activity measurement to track phasic skin conductance (sweat gland activation).
  - **DS18B20 (OneWire)**: High-precision digital skin temperature monitoring.
- **Piezoelectric Sensor (Analog)**: 
  - **Dual-Role Logic**: Monitors psychomotor agitation (micro-tremors) and serves as an active **Data Validation** tool by filtering out mechanical noise and movement artifacts.

## 💻 Key Software & Testing Features
- **Hardware-in-the-Loop (HiL) Automation**: A custom Python/pySerial script (`hil_test_runner.py`) acts as an automated test bench, parsing CSV telemetry over UART and running live assertions on physiological bounds and state machine integrity.
- **Industrial Telemetry Payload**: Replaced standard debug prints with a strictly formatted CSV data stream optimized for machine reading and automated validation.
- **Digital Signal Processing (DSP)**: Custom implementation of digital filters for GSR (slow/fast baseline tracking) and Piezo signals (envelope detection).
- **Non-blocking Architecture**: Engineered using `millis()` for concurrent task scheduling and efficient polling across multiple sensor buses without blocking delays.
- **Adaptive Calibration Engine**: 60-second baseline establishment that dynamically tailors the algorithmic thresholds to individual user physiology.

## 📊 Algorithmic Scoring Logic
The final telemetry output includes a weighted physiological score derived from:
1. **BPM Rise**: Heart rate elevation relative to the adaptive baseline.
2. **Peripheral Temperature Drop**: Physiological vasoconstriction response.
3. **GSR Phasic Spikes**: Rapid transient changes in skin conductance.
4. **Piezo Intensity**: Physical tension and micro-movements detected via envelope tracking.

---

## 📄 Documentation & Research
The project is backed by technical research on biometric data fusion. You can find the complete project report, including the mathematical model for stress calculation and the electronic schematics, here:
- 📑 **[Multi-Sensor Stress Detection System Report (PDF)](./docs/Stress_detection_system_documentation.pdf)**

---
## 📂 Repository Structure
- `/src`: Contains the non-blocking C++ firmware (`stress_detector.ino`) and the Python automated testing script (`hil_test_runner.py`).
- `/docs`: Technical documentation (UTCN), schematics, and experimental results.
