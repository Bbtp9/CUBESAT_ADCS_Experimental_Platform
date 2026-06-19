# CubeSat Thesis - Arduino Code Reference

This directory contains organized, clean, and well-documented Arduino reference codes for your CubeSat Attitude Control (Detumbling and Pointing) project.

## Directory Structure

```
Thesi/
├── 1_Sensors_I2C/
│   └── Check_Sensors/
│       └── Check_Sensors.ino        # Scans the I2C bus and verifies MPU6050 and Magnetometer addresses.
├── 2_Motor_Driver/
│   └── Motor_Driver_Test/
│       └── Motor_Driver_Test.ino    # Tests motor driver IN1/IN2/EEP pins with 10-bit PWM.
├── 3_Bluetooth_BLE_Tests/
│   ├── ArduinoBLE_Test/
│   │   └── ArduinoBLE_Test.ino      # Tests simple BLE connection using standard ArduinoBLE.h library.
│   └── BT_Classic_Test/
│       └── BT_Classic_Test.ino      # Test sketch for Bluetooth Classic (only for boards supporting it).
└── 4_Full_HIL_System/
    └── CubeSat_HIL_BLE/
        └── CubeSat_HIL_BLE.ino      # The fully integrated wireless Hardware-in-the-Loop (HIL) system.
```

---

## Folder Descriptions

### 1. `1_Sensors_I2C`
Use this folder to verify that your sensors are wired correctly:
* **MPU6050 Address:** `0x69` (standard is `0x68`, but your board has AD0 high or a custom chip at `0x69`).
* **Magnetometer Address:** `0x2C` (QMC5883P / GY-273).
* If a sensor is not detected, check your SDA (GPIO 1) and SCL (GPIO 0) physical connections.

### 2. `2_Motor_Driver`
Use this folder to test that the motor driver is wired correctly:
* **IN1 (GPIO 2)** and **IN2 (GPIO 3)** are configured with **10-bit resolution** (values `0 - 1023`) and a **500 Hz** PWM frequency.
* **EEP (GPIO 14)** is driven `HIGH` to enable the driver board.
* The test script runs the motor in one direction, stops, runs in reverse, and stops.

### 3. `3_Bluetooth_BLE_Tests`
* **ArduinoBLE_Test:** Demonstrates how to use the modern, lightweight `ArduinoBLE.h` library on the ESP32-C6. It publishes mock telemetry data and intercepts writes.
* **BT_Classic_Test:** A serial pass-through test. *Note: ESP32-C6 does not support Bluetooth Classic, so this code will only compile on the original ESP32 (WROOM-32).*

### 4. `4_Full_HIL_System`
This is the **production-ready code** for your thesis:
1. It reads real sensor data from MPU6050 and GY-273.
2. It formats and publishes **11 telemetry values** (`Mx My Mz Gx Gy Gz Ax Ay Az Temp Hdg`) at **10 Hz** via the BLE notify characteristic (`TELEMETRY_UUID`).
3. It listens for motor commands (signed integers like `-500` to `500` or comma-separated pairs like `500,0`) written to the BLE characteristic (`MOTOR_UUID`).
4. It parses these commands and controls the physical motor driver in real time.
5. It includes a fallback mechanism: you can also send commands and read telemetry over standard USB Serial at `115200 baud`.
