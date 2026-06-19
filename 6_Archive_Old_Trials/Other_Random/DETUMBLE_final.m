%% DETUMBLE_Final.m
% =========================================================================
%   CUBESAT REAL-TIME DETUMBLING CONTROL over BLE (HIL - Hardware in the Loop)
% =========================================================================
%   - Streamlined single-write BLE protocol (reduces BLE overhead by 50%)
%   - Dynamic BLE connection to "CubeSat_ESP32" (resolves macOS UUID issues)
%   - Live numerical integration of Gz to calculate Theta (Attitude Angle)
%   - Real-time PID controller architecture
%   - High-fidelity control torque calculation (saturating at +/- 0.002 Nm)
%   - dark-theme double real-time plotter (Attitude & Control effort)
%   - Structured workspace export (timeseries) for Simulink validation
% =========================================================================

clear; clc; close all;

disp('==================================================');
disp('   CUBESAT REAL-TIME CONTROL & TELEMETRY VIEWER   ');
disp('==================================================');

%% 1. Dynamic BLE Connection Establishment
disp('[*] Scanning for advertising BLE devices...');
try
    devices = blelist;
catch ME
    error('BLE is not supported or Bluetooth is turned off on this machine. Error: %s', ME.message);
end

% Search for the ESP32_IMU device name
idx = find(strcmp(devices.Name, "ESP32_IMU"), 1);
if isempty(idx)
    error('[-] ESP32_IMU device not found! Make sure the ESP32 is powered on and advertising.');
end

deviceAddress = devices.Address(idx);
fprintf('[+] Found ESP32_IMU! Address/UUID: %s\n', deviceAddress);
disp('[*] Connecting to ESP32_IMU BLE Service...');

serviceUUID   = "12345678-1234-1234-1234-123456789abc";
telemetryUUID = "87654321-4321-4321-4321-cba987654321";
motorUUID     = "87654321-4321-4321-4321-cba987654326";

try
    b = ble(deviceAddress);
    c = characteristic(b, serviceUUID, telemetryUUID);
    m = characteristic(b, serviceUUID, motorUUID);
    disp('[+] Connection established successfully!');
catch ME
    error('[-] Failed to connect to BLE device: %s', ME.message);
end

%% 2. Controller Parameters
ctrl_name = 'Analytical PID';

% Satellite Physical Constants
J  = 0.000634;     % Spacecraft body inertia [kg*m^2]
Jw = 4.607e-5;     % Reaction wheel inertia [kg*m^2]
tau_max = 1024;    % Maximum control torque [Nm]
gz_offset = 0.012;
tau_delay = 2.3;   % measured time delay [s]

% PParameters from PID1
Kp = -142;
Ki = -117;
Kd = 47;
N  = 0.73;

fprintf('Analytical PID Gains (for delay tau = %.2f s):\n', tau_delay);
fprintf('  Kp = %.6f, Ki = %.6f, Kd = %.6f\n', Kp, Ki, Kd);
fprintf('\n[+] Selected Pointing Mode: %s\n', ctrl_name);
fprintf('    PID Gains: Kp = %.6f, Ki = %.6f, Kd = %.6f\n', Kp, Ki, Kd);
disp('Press ENTER to start Phase 1 (Detumbling)...');
input('');

%% 3. Setup Dark-Theme Live Visualizer
fig = figure('Color', 'k', 'Name', ['Real-Time CubeSat Mission - ' ctrl_name], 'Position', [100, 100, 1000, 700]);

% Subplot 1: Spacecraft State (Velocity in Phase 1, Angle in Phase 2)
ax1 = subplot(2, 1, 1);
h_theta = animatedline('Color', [0.00 0.80 0.80], 'LineWidth', 2.5); % Turquoise
h_ref   = yline(0, 'w:', 'LineWidth', 1.5);                         % Reference line
set(h_ref, 'Visible', 'off');
grid on;
ylabel('Angular Velocity [deg/s]', 'Color', 'w');
title('Real-Time Spacecraft Angular Velocity (Phase 1: Detumbling)', 'Color', 'w', 'FontSize', 12);
legend('Body Rate (\omega_z)', 'TextColor', 'w', 'Location', 'southeast', 'Color', 'none', 'EdgeColor', 'none');

% Subplot 2: Estimated Reaction Wheel Velocity
ax2 = subplot(2, 1, 2);
h_omega_w = animatedline('Color', [1.00 0.55 0.15], 'LineWidth', 2.0);  % Orange
grid on;
ylabel('Wheel Speed \omega_w [rad/s]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Real-Time Estimated Reaction Wheel Velocity (\omega_w)', 'Color', 'w', 'FontSize', 12);
legend('Reaction Wheel Speed', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

% Apply dark styling to axes
for ax = [ax1, ax2]
    set(ax, 'Color', 'k', ...
            'XColor', 'w', ...
            'YColor', 'w', ...
            'GridColor', [0.5 0.5 0.5], ...
            'GridAlpha', 0.3, ...
            'LineWidth', 1.2, ...
            'FontSize', 11);
end

%% 4. Real-Time Acquisition & Mission Control Loop
thetaZ = 0;             % Integrated attitude angle [rad]
omega_w_est = 0;        % Estimated reaction wheel speed [rad/s]
lastT = tic;            % Timer for dt calculation
t0 = tic;               % Global timer
sampling_rate = 10;     % Loop frequency (approx 10 Hz)
pause_time = 1 / sampling_rate;

% History arrays for Workspace Export
history_time  = [];
history_theta = [];
history_omega = [];
history_tau   = [];

% ==================================================
%   PHASE 1: DETUMBLING (RATE DAMPING)
% ==================================================
disp('[*] Starting Phase 1: Detumbling (Damping rates)...');
disp('--------------------------------------------------');

stable_count = 0;
stable_threshold = deg2rad(0.5); % Stable if below 0.5 deg/s
stable_required = 10;            % 10 samples (1 second at 10 Hz)
max_detumble_iterations = 150;    % Max 15 seconds

tau_sat = 0;
gz = 0;
tau_cmd = 0;

% Filter initialization
e = 0;
xF = 0;
xI = 0;


for k = 1:max_detumble_iterations
    if ~ishandle(fig)
        disp('[-] Figure closed by user. Terminating loop.');
        break;
    end
    
    % Send torque command and read telemetry
    pwm_cmd = tau_sat;
    if pwm_cmd > 0
        pwm_cmd = pwm_cmd + 150;
    else
        pwm_cmd = pwm_cmd - 150;
    end
    pwm_cmd = max(min(pwm_cmd, 1023), -1023);

    cmd_str = sprintf("%i", floor(pwm_cmd));
    disp([num2str([gz tau_cmd tau_sat]) ' ' cmd_str])
    try
        write(m, char(cmd_str));
        pause(0.1);
        raw = read(c);
        data = sscanf(char(raw), '%f');
        pause(0.05);
    catch ME
        warning('BLE communication dropped a package. Retrying... Error: %s', ME.message);
        continue;
    end
    
    if numel(data) == 11 && all(~isnan(data))
        gz = data(6) - gz_offset;  % Gyroscope Z-axis [rad/s]
        dt = toc(lastT);
        lastT = tic;
        
        % Integrate angle and estimate reaction wheel speed
        thetaZ = thetaZ + gz * dt;
        thetaDeg = rad2deg(thetaZ);
        omega_w_est = omega_w_est + (tau_sat / Jw - tau_delay * omega_w_est) * dt;
        t = toc(t0);
        
        % Damping controller: u = -Kd * omega
        e = gz - 0; % error
        xF = xF + N * (e - xF);
        xI = xI + dt * e;
        D = Kd * N * (e - xF);

        tau_cmd = Kp * e + Ki * xI + D; %+kdd*dgz;

       
        % Motorul este invers
        tau_sat = -tau_cmd;  
        
        % Log history
        history_time  = [history_time; t];
        history_theta = [history_theta; thetaDeg];
        history_omega = [history_omega; rad2deg(gz)];
        history_tau   = [history_tau; tau_sat];
        
        % Update Plots
        addpoints(h_theta, t, rad2deg(gz));
        addpoints(h_omega_w, t, omega_w_est);
        drawnow limitrate;
        
        fprintf("t=%5.2fs | DETUMBLE | Gz=%7.4f rad/s (%5.2f deg/s) | Theta=%6.2f deg\n", ...
                t, gz, rad2deg(gz), thetaDeg);
        
        % Check stability
        if abs(gz) < stable_threshold
            stable_count = stable_count + 1;
        else
            stable_count = 0;
        end
        
        if stable_count >= stable_required
            disp('[+] Stabilization detected.');
            break;
        end
    end
end

return