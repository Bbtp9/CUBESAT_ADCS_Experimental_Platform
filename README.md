# CubeSat ADCS Experimental Platform

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b+-007672?style=flat&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-v10.8+-ef5350?style=flat&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/simulink.html)
[![Arduino](https://img.shields.io/badge/Arduino-ESP32--C6-00979D?style=flat&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**Bachelor Thesis**  
**Author:** Bianca-Andreea Topliceanu  
**Faculty:** Aerospace Engineering, National University of Science and Technology POLITEHNICA Bucharest  
**Academic Year:** 2025 - 2026  

---

## 🛰️ Project Abstract

This project presents the design, modeling, simulation, and experimental validation of a **CubeSat Attitude Determination and Control System (ADCS)** based on a single reaction wheel actuator. 

Modern CubeSats demand high-precision pointing and detumbling capabilities under strict size, weight, and power (SWaP) constraints. This platform implements a low-cost, high-performance experimental testbed using an **ESP32-C6 microcontroller** that communicates wirelessly over **Bluetooth Low Energy (BLE)** with a **MATLAB/Simulink** workstation. It supports both high-fidelity numerical simulation and real-time **Hardware-in-the-Loop (HIL)** testing.

---

## ✨ Key Features

- **Reaction Wheel Attitude Control:** Bidirectional motor control using a dedicated H-bridge driver.
- **PID Pointing Controller:** Closed-loop pointing stabilization algorithm.
- **Detumbling Algorithm:** B-dot style detumbling control for initial stabilization.
- **Wireless BLE Communication:** Native 10 Hz telemetry pipeline between the physical CubeSat and MATLAB.
- **Dual Verification Modes:** 
  1. *Ideal Simulation:* Pure numerical models of satellite and actuator dynamics.
  2. *HIL Simulation:* Real-time sensor input (IMU & magnetometer) combined with hardware actuator feedback.
- **State Estimation:** Complementary attitude estimation combining gyroscope, accelerometer, and calibrated magnetometer data.

---

## 📂 Repository Contents

The repository is organized into a clean, minimalist structure to facilitate navigation:

```
CUBESAT_ADCS_Experimental_Platform/
├── README.md                           # This documentation guide
├── LICENSE                             # MIT License
├── BachelorThesis_Final_Bianca.pdf     # The complete Bachelor Thesis PDF manuscript
│
├── ESP32/                              # ESP32-C6 Arduino Firmware
│   ├── CubeSat_HIL_BLE/                # Production firmware for wireless HIL telemetry
│   ├── Check_Sensors/                  # I2C scanner & sensor diagnostic tool
│   ├── Motor_Driver_Test/              # Actuator PWM & direction tester
│   ├── BLE_Test/                       # Simple BLE connection test
│   └── BT_Classic_Test/                # Bluetooth Classic connection test (fallback)
│
├── MATLAB/                             # MATLAB Scripts & Data
│   ├── init_simulation_lqr.m           # Pre-load LQR simulation parameters
│   ├── init_simulation_pd.m            # Pre-load PD/PID simulation parameters
│   ├── plot_cubesat_results.m          # Ideal simulation plotting script
│   ├── magnetometer_calibration.m      # Hard-iron & soft-iron calibration script
│   ├── mag1.txt                        # Raw magnetometer calibration data
│   ├── Test_CubeSat_BLE_HIL.m          # Standalone MATLAB BLE telemetry scanner
│   ├── MATLAB_to_BLE_POINTING.m        # Real-time BLE pointing controller coordinator
│   ├── MATLAB_to_BLE_DETUMBL.m         # Real-time BLE detumbling coordinator
│   ├── SerialHILSystemObject.m         # MATLAB System Object for USB Serial HIL
│   ├── BLEHILSystemObject.m            # MATLAB System Object for BLE HIL
│   ├── compare_sim_real.m              # Simulator vs. Real experimental comparison script
│   ├── compare_pointing.m              # Comparative analysis of pointing controllers
│   └── compare_detumble.m              # Comparative analysis of detumbling trials
│
└── Simulink/                           # Simulink Control Models
    ├── Cubesat_Control_LQR.slx         # LQR Attitude Control Loop Simulation
    ├── Cubesat_Control_PD.slx          # PD Attitude Control Loop Simulation
    ├── Motor_Satellite_Dynamics.slx    # Unified physical dynamics simulator
    ├── PID1.slx                        # Hardware-in-the-Loop model (Detumbling / Pointing)
    └── PID2.slx                        # Alternative HIL model configuration
```

---

## 🔧 Hardware & Software Setup

### Hardware Requirements
- **Microcontroller:** ESP32-C6 Super Mini (or compatible ESP32 board).
- **IMU Sensor:** MPU6050 (connected via I2C: SDA=GPIO 1, SCL=GPIO 0).
- **Magnetometer:** QMC5883P / GY-273 (connected via I2C).
- **Motor Driver:** DRV8833 Dual H-Bridge (IN1=GPIO 2, IN2=GPIO 3, EEP=GPIO 14).
- **Actuator:** DC Motor with inertial reaction wheel.

### Required Software & Libraries
1. **MATLAB & Simulink** (R2023b or newer recommended) with the following toolboxes:
   - Aerospace Blockset / System Identification Toolbox (optional but recommended).
2. **Arduino IDE** (v2.x) with the following libraries:
   - `Adafruit_MPU6050` & `Adafruit_Sensor`
   - `QMC5883LCompass` (or raw I2C readings)
   - ESP32 BLE Stack (included in the ESP32 Arduino Core v3.0+)

---

## 🚀 Setup & Execution Guide

### 1. Calibration (Magnetometer)
Before running experiments, the magnetometer must be calibrated to eliminate hard/soft-iron distortions:
1. Run `ESP32/Check_Sensors` to ensure both I2C sensors are detected.
2. Spin the CubeSat along its axes while reading raw values, or collect them to `mag1.txt`.
3. Open MATLAB and run `MATLAB/magnetometer_calibration.m` to calculate the correction offset matrix.

### 2. Running Simulations (Ideal)
To test the controller design theoretically:
1. Open MATLAB and navigate to the `MATLAB/` folder.
2. Run `init_simulation_pd.m` or `init_simulation_lqr.m` to load parameters into the workspace.
3. Open `Simulink/Cubesat_Control_PD.slx` or `Cubesat_Control_LQR.slx` and run the simulation.
4. Use `plot_cubesat_results.m` to plot the response.

### 3. Running Hardware-in-the-Loop (HIL) Tests
To run live wireless hardware experiments:
1. Flash the ESP32 firmware located in `ESP32/CubeSat_HIL_BLE/` to the ESP32-C6 board.
2. Ensure your computer's Bluetooth is turned on.
3. Open and run `MATLAB/MATLAB_to_BLE_POINTING.m` (for active attitude control) or `MATLAB/MATLAB_to_BLE_DETUMBL.m` (for active detumbling).
4. The script will automatically scan, connect, stream telemetry at 10 Hz, and command the motor driver in real time.

---

## 📜 Thesis Manuscript

The complete Bachelor Thesis document detailing all mathematical equations (Euler equations, LQR state-space formulations, complementary filtering, and HIL architectures) is available in the root folder:  
📄 **[BachelorThesis_Final_Bianca.pdf](BachelorThesis_Final_Bianca.pdf)**

---

## ⚖️ License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
