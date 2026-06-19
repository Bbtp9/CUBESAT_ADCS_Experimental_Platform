# Magnetometer Calibration

This directory contains the MATLAB calibration script and raw dataset used to calibrate the QMC5883P magnetometer on the CubeSat.

## Files
- [magnetometer_calibration.m](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/2_Magnetometer_Calibration/magnetometer_calibration.m): Calibration script that reads raw measurements, computes offsets, and plots raw vs. calibrated data.
- [mag1.txt](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/2_Magnetometer_Calibration/mag1.txt): Raw magnetometer measurements gathered during a full 360-degree rotation of the CubeSat in the horizontal plane.

---

## Calibration Methodology (Hard-Iron Calibration)

Hard-iron distortion is caused by permanent magnets or magnetized metal on the CubeSat chassis. It acts as a constant bias (offset) shifting the center of the magnetic measurement circle in the 2D plane (X-Y) away from the origin $(0,0)$.

### 1. Offset Calculation in MATLAB
The script reads the raw data columns $M_x$ and $M_y$ from `mag1.txt` and computes their arithmetic mean:
- $X_{\text{offset}} = \text{mean}(M_x) \approx 163$
- $Y_{\text{offset}} = \text{mean}(M_y) \approx -57$

To center the measurement circle back to the $(0,0)$ origin, these offsets are subtracted from raw readings:
$$M_{x0} = M_x - X_{\text{offset}}$$
$$M_{y0} = M_y - Y_{\text{offset}}$$

### 2. Arduino C++ Integration
The calculated calibration parameters are integrated directly into the ESP32 code inside `CubeSat_HIL_BLE.ino` to correct the readings in real-time before sending telemetry over BLE:

```cpp
// Hard-iron correction based on calibration offsets:
// mx_calibrated = mx - 163
// my_calibrated = my - (-57) = my + 57
heading = atan2((float)my + 57, (float)mx - 163) * 180.0 / PI;

if (heading < 0) {
  heading += 360.0;
}
```

This correction ensures that the calculated heading (yaw angle) is accurate and zero-centered.
