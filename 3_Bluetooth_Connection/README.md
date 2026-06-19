# Bluetooth BLE Connection & Interface

This directory details the wireless Bluetooth Low Energy (BLE) interface established between the MATLAB control scripts and the ESP32 microcontroller onboard the CubeSat.

## Included Files
- [Test_CubeSat_BLE_HIL.m](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/3_Bluetooth_Connection/Test_CubeSat_BLE_HIL.m): MATLAB test script used to list BLE advertising devices, connect to the ESP32 server, read sensor telemetry in a 10 Hz loop, and test basic motor torque sending.
- [BLE_Test/](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/3_Bluetooth_Connection/BLE_Test/): Core ESP32 BLE server testing sketch.
- [BT_Classic_Test/](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/3_Bluetooth_Connection/BT_Classic_Test/): Bluetooth Classic server alternative testing sketch.

---

## BLE Communication Architecture

The system operates as a **Client-Server** model where:
1. **ESP32 (Server)**: Advertises as `ESP32_IMU` and hosts the GATT service and characteristics.
2. **MATLAB (Client)**: Scans for the device, establishes the connection, subscribes to the sensor telemetry, and writes PWM motor commands.

### BLE GATT Profile configuration
The communication uses custom UUIDs:
- **Service UUID**: `12345678-1234-1234-1234-123456789abc`
- **Telemetry Characteristic UUID (Read/Notify)**: `87654321-4321-4321-4321-cba987654321`
- **Motor Control Characteristic UUID (Write Without Response)**: `87654321-4321-4321-4321-cba987654326`

---

## Data Packet Structure

To minimize BLE transmission overhead, all data is streamed in a single ASCII string format.

### 1. Telemetry Packet (ESP32 -> MATLAB)
The ESP32 reads the sensors and formats a string containing 11 space-separated numbers:
`"Ax Ay Az Gx Gy Gz Mx My Mz Temp Heading"`

- **Ax, Ay, Az**: Accelerometer readings [$g$] (3 values)
- **Gx, Gy, Gz**: Gyroscope angular velocities [$\text{rad/s}$] (3 values)
- **Mx, My, Mz**: Magnetometer readings (3 values)
- **Temp**: Sensor temperature [$\text{C}^\circ$] (1 value)
- **Heading**: Magnetometer calibrated heading angle [$^\circ$] (1 value)

#### MATLAB Telemetry Parser
```matlab
raw = read(c); % read telemetry characteristic
data = sscanf(char(raw), '%f'); % parse values into an array
gz = data(6) - gz_offset; % extract gyro z
theta_sensor = data(11); % extract absolute heading angle
```

### 2. Control Command (MATLAB -> ESP32)
The control effort calculated in MATLAB is mapped to an integer command string representing a PWM value:
- **Command range**: `[-1023, 1023]` (with a deadband addition of $\pm 150$ applied on the ESP32)

#### MATLAB Command Writer
```matlab
cmd_str = sprintf("%i", floor(pwm_cmd));
write(m, uint8(char(cmd_str))); % write string as byte array to motor characteristic
```
