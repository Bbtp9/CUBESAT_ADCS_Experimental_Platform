%% TEST_CUBESAT_BLE_HIL.m
% =========================================================================
%   CUBESAT WIRELESS BLE HIL TEST SCRIPT
% =========================================================================
%   - Connects to the ESP32-C6 (named "ESP32_IMU") over BLE.
%   - Obtains telemetry and motor control characteristics.
%   - Reads sensor data (Magnetometer, Gyroscope, Accelerometer, Temp, Heading).
%   - Writes a test command to the motor driver characteristic.
% =========================================================================

clear; clc; close all;

disp('==================================================');
disp('   CUBESAT BLE WIRELESS TELEMETRY & CONTROL TEST  ');
disp('==================================================');

%% 1. Establish BLE Connection
disp('[*] Scanning and connecting to "ESP32_IMU" BLE device...');
try
    d = ble("ESP32_IMU");
    disp('[+] Connected to ESP32_IMU successfully!');
catch ME
    error('[-] Connection failed! Make sure the ESP32 is powered on and advertising. Error: %s', ME.message);
end

%% 2. Set Up Characteristics
serviceUUID = "12345678-1234-1234-1234-123456789ABC";
telemetryUUID = "87654321-4321-4321-4321-CBA987654321";
motorUUID = "87654321-4321-4321-4321-CBA987654326";

try
    % Characteristic for reading telemetry data
    c = characteristic(d, serviceUUID, telemetryUUID);
    % Characteristic for writing motor driver commands
    m = characteristic(d, serviceUUID, motorUUID);
    disp('[+] Characteristics mapped successfully!');
catch ME
    error('[-] Mapping characteristics failed: %s', ME.message);
end

%% 3. Perform a Test Motor Write
disp(' ');
disp('[*] Sending test motor command...');
% Command: Drive motor forward at 500 PWM (approx 50% power)
test_cmd = "500"; 
disp(['    -> Writing speed/torque: ', test_cmd]);
write(m, uint8(char(test_cmd)));

% Let it run for 3 seconds
pause(3.0);

% Command: Stop the motor
disp('    -> Stopping motor (Writing: 0)');
write(m, uint8(char("0")));

%% 4. Read Telemetry Data in a Loop
disp(' ');
disp('[*] Starting wireless sensor telemetry read (Press Ctrl+C to stop)...');
disp('--------------------------------------------------');

try
    while true
        raw = read(c);
        txt = char(raw);
        
        % Parse the 11 telemetry values:
        % Mx My Mz Gx Gy Gz Ax Ay Az Temp Hdg
        values = sscanf(txt, "%f");

        if numel(values) == 11
            fprintf("Mx=%4d My=%4d Mz=%4d | Gx=%7.4f Gy=%7.4f Gz=%7.4f | Ax=%7.4f Ay=%7.4f Az=%7.4f | Temp=%5.2f°C Hdg=%6.2f°\n", ...
                values(1), values(2), values(3), ...
                values(4), values(5), values(6), ...
                values(7), values(8), values(9), ...
                values(10), values(11));
        else
            fprintf("[-] Raw package: %s\n", txt);
        end

        pause(0.1); % Read at approx 10 Hz
    end
catch ME
    disp(' ');
    disp('[-] Telemetry loop stopped or connection lost.');
end

disp('==================================================');
disp('                 TEST SEQUENCE END                ');
disp('==================================================');
