# MATLAB & Simulink Workspace Reference

This directory contains the cleaned-up, main active scripts and Simulink models for your CubeSat Attitude Control (PD & LQR) thesis.

## Active Directory Files

### 📈 Simulink Models
* **[Cubesat_Control_PD.slx](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/Cubesat_Control_PD.slx)**: The Simulink model configured with the PD controller architecture.
* **[Cubesat_Control_LQR.slx](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/Cubesat_Control_LQR.slx)**: The Simulink model configured with the Optimal LQR controller architecture.

### ⚙️ Initialization Scripts (Run these first in MATLAB)
* **[init_simulation_pd.m](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/init_simulation_pd.m)**: Sets up workspace parameters, dynamics matrices, and gains for the PD simulation.
* **[init_simulation_lqr.m](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/init_simulation_lqr.m)**: Sets up optimal state-feedback gains (solving Riccati equations) and parameters for the LQR simulation.

### 📶 Bluetooth (BLE) Scripts
* **[Test_CubeSat_BLE_HIL.m](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/Test_CubeSat_BLE_HIL.m)**: A simple wireless test script to establish a BLE connection with `ESP32_IMU`, send a short motor test command, and print 11 real sensor values in a 10 Hz loop.
* **[Matlab_to_BLE2.m](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/Matlab_to_BLE2.m)**: The advanced real-time Hardware-in-the-Loop script which connects to the BLE satellite, executes a live control loop, integrates angular rates, plots attitude vs. reference, and exports logs to the workspace.

### 🛡️ Core HIL System Blocks (Do not delete or move)
* **[BLEHILSystemObject.m](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/BLEHILSystemObject.m)**: Custom MATLAB System block used inside the Simulink files to communicate wirelessly via BLE.
* **[SerialHILSystemObject.m](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/SerialHILSystemObject.m)**: Custom MATLAB System block used if you choose to communicate via wired USB Serial.

---

## 🗄️ Archived Items
All older trial scripts, results plotters, model-builders, and temporary test files have been safely archived to:
👉 **[Archive_Old_Scripts/](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab/Archive_Old_Scripts/)**
*(Keep these for future reference, but they are out of the way to avoid clutter).*
