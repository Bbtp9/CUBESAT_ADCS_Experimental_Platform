% ==========================================
%    CUBESAT BLE CONNECTION CODE
% ==========================================

clear;
clc;

% 1. Connect to the CubeSat via Bluetooth BLE
disp('Connecting to CubeSat...');
d = ble("ESP32_IMU");

% 2. Map characteristics (Telemetry and Motor)
c = characteristic(d, "12345678-1234-1234-1234-123456789abc", "87654321-4321-4321-4321-cba987654321");
m = characteristic(d, "12345678-1234-1234-1234-123456789abc", "87654321-4321-4321-4321-cba987654326");

% 3. Spin the motor LEFT (Counter-Clockwise using a negative value)
disp('Spinning motor LEFT (writing: -500)...');
write(m, uint8(char("-500")));
pause(3.0); % Spin left for 3 seconds

% 4. Stop the motor 
disp('Stopping motor briefly (writing: 0)...');
write(m, uint8(char("0")));
pause(1.0); % Stop for 1 second

% 5. Spin the motor RIGHT (Clockwise using a positive value)
disp('Spinning motor RIGHT (writing: 500)...');
write(m, uint8(char("500")));
pause(3.0); % Spin right for 3 seconds

% 6. Stop the motor again
disp('Stopping motor (writing: 0)...');
write(m, uint8(char("0")));
pause(1.0);

% 7. Read raw sensor telemetry from the CubeSat for 5 seconds
disp('Reading sensor telemetry:');
for i = 1:50
    raw = read(c);
    txt = char(raw);
    disp(txt); % Display the raw string of 11 numbers received from ESP32
    pause(0.1);
end

% 8. Final safety stop command
disp('Ensuring motor is stopped (writing: 0)...');
write(m, uint8(char("0")));
disp('Sequence complete.');
