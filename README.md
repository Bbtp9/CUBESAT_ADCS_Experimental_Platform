# CUBESAT_THESI
Bachelors Thesis
# ADCS CubeSat: Full Simulation + Real OBC + Real Orbital Data

## Project Overview
This project focuses on the **Attitude Determination and Control System (ADCS)** for a CubeSat, utilizing a Hardware-in-the-Loop (HIL) approach.

### System Components:
* **On-Board Computer (OBC):** Raspberry Pi Pico/Zero.
* **Simulation Environment:** Simulink (Dynamics, Sensors, and Actuators).
* **Orbit Propagation:** STK (Satellite Tool Kit) for real orbital data (LEO/GEO).
* **Hardware Demonstrator:** A custom 1U LEGO CubeSat frame.

## Technical Implementation
### A) Sensor Acquisition & Estimation
* **Sensors:** Gyroscope (ω), Accelerometer, and Pi Camera for Earth Horizon sensing.
* **Estimation:** Implementing q-method for attitude determination.

### B) Control System
* **Algorithm:** ADCS control loops running on real hardware (Raspberry Pi).
* **Communication:** Data exchange between Simulink and Raspberry Pi via USB/Ethernet/UDP.

### C) Hardware (LEGO Cubesat)
* **Structure:** 10x10x10 cm (1U) rigid double-layered LEGO walls.
* **Features:** Integrated Pi Camera mount and rotating platform for Earth horizon detection.