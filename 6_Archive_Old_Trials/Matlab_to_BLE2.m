%% Matlab_to_BLE2.m
% =========================================================================
%   CUBESAT REAL-TIME ATTITUDE CONTROL over BLE (HIL - Hardware in the Loop)
% =========================================================================
%   - Streamlined single-write BLE protocol (reduces BLE overhead by 50%)
%   - Dynamic BLE connection to "CubeSat_ESP32" (resolves macOS UUID issues)
%   - Live numerical integration of Gz to calculate Theta (Attitude Angle)
%   - Sequential control: Phase 1 (Detumbling using PD/rate damping) ->
%                         Phase 2 (Pointing using tuned PID with derivative filter)
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

%% 2. Controller Parameters & Gains
% Satellite Physical Constants
J  = 0.000634;     % Spacecraft body inertia [kg*m^2]
Jw = 4.607e-5;     % Reaction wheel inertia [kg*m^2]
tau_delay = 2.3;   % Command transmission delay [s]
tau_max = 1024;    % Maximum control torque / PWM limit

% Pointing PID Controller Gains (Tuned values)
Kp = -142.325;
Ki = -117.452;
Kd = 47.716;
N = 0.739;

disp(' ');
disp('--------------------------------------------------');
disp('   LOADED CONTROLLER CONFIGURATIONS               ');
disp('--------------------------------------------------');
fprintf('Detumbling Mode:\n');
fprintf('  Damping Gain (Kd) = %.6f\n', Kd);
fprintf('Pointing Mode (PID with Filtered Derivative):\n');
fprintf('  Kp = %.6f, Ki = %.6f, Kd = %.6f, N = %.6f\n', Kp, Ki, Kd, N);
disp('--------------------------------------------------');
disp('Press ENTER to start Phase 1 (Detumbling)...');
input('');

%% 3. Setup Dark-Theme Live Visualizer
fig = figure('Color', 'k', 'Name', 'Real-Time CubeSat Mission - PD & PID Control', 'Position', [100, 100, 1000, 700]);

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

tau_sat = 0; gz=0; tau_cmd=0;

for k = 1:max_detumble_iterations
    if ~ishandle(fig)
        disp('[-] Figure closed by user. Terminating loop.');
        break;
    end
    
    % Send torque command & read telemetry
    pwm_cmd = round((tau_sat / tau_max) * 1023);
    pwm_cmd = max(min(pwm_cmd, 1023), -1023);

    cmd_str = sprintf("%d", pwm_cmd);
    disp([num2str([gz tau_cmd tau_sat]) ' ' cmd_str])
    try
        write(m, uint8(char(cmd_str)));
        pause(0.1);
        raw = read(c);
        data = sscanf(char(raw), '%f');
    catch ME
        warning('BLE communication dropped a package. Retrying... Error: %s', ME.message);
        continue;
    end
    
    if numel(data) == 11 && all(~isnan(data))
        gz = data(6);  % Gyroscope Z-axis [rad/s]
        dt = toc(lastT);
        lastT = tic;
        
        % Safeguard dt anomalies
        if dt <= 0 || dt > 0.5
            dt = 0.1;
        end
        
        % Integrate angle and estimate reaction wheel speed
        thetaZ = thetaZ + gz * dt;
        thetaDeg = rad2deg(thetaZ);
        omega_w_est = omega_w_est + (tau_sat / Jw - tau_delay * omega_w_est) * dt;
        t = toc(t0);
        
        % Detumbling control law (Damping): u = Kd * gz
        tau_cmd = Kd * gz;
        tau_sat = max(min(tau_cmd, tau_max), -tau_max);
        
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
    pause(pause_time - 0.08);
end

% Ensure motor is stopped immediately
try
    write(m, uint8(char("0")));
catch
end
disp('[*] Motor stopped. Waiting for CubeSat to settle...');
pause(2.0);

% Read settled heading from sensor
try
    raw = read(c);
    data = sscanf(char(raw), '%f');
    if numel(data) == 11 && all(~isnan(data))
        settled_heading = data(11);
    else
        settled_heading = rad2deg(thetaZ);
    end
catch
    settled_heading = rad2deg(thetaZ);
end

fprintf('\n==================================================\n');
fprintf('   [+] DETUMBLING COMPLETE & SETTLED!\n');
fprintf('   CubeSat settled at:\n');
fprintf('   -> Current Sensor Heading: %.2f deg\n', settled_heading);
fprintf('==================================================\n\n');

% Initialize attitude angle for pointing
thetaZ = deg2rad(settled_heading);

% ==================================================
%   INTER-PHASE USER INPUT
% ==================================================
theta_ref_deg = input('>> Enter Target Pointing Heading/Angle [deg] (0 to 360): ');
if isempty(theta_ref_deg) || ~isnumeric(theta_ref_deg)
    theta_ref_deg = 45;
end
theta_ref = deg2rad(theta_ref_deg);

% Set Subplot 1 titles and labels for Pointing Phase
title(ax1, 'Real-Time Spacecraft Attitude Angle (Phase 2: Pointing)', 'Color', 'w', 'FontSize', 12);
ylabel(ax1, 'Attitude Angle \theta_z [deg]', 'Color', 'w');

% Set reference line visible
set(h_ref, 'Value', theta_ref_deg);
set(h_ref, 'Visible', 'on');
legend(ax1, 'Live Angle (\theta_z)', 'Target Reference', 'TextColor', 'w', 'Location', 'southeast', 'Color', 'none', 'EdgeColor', 'none');

disp(' ');
disp('[*] Starting Phase 2: Attitude Pointing...');
disp('--------------------------------------------------');

% Start with an initial torque command of 0
tau_sat = 0;
lastT = tic; 
int_err = 0; 
xF_pointing = 0; 

pointing_duration = 30; 
num_pointing_iterations = pointing_duration * sampling_rate;

for k = 1:num_pointing_iterations
    if ~ishandle(fig)
        disp('[-] Figure closed by user. Terminating loop.');
        break;
    end
    
    pwm_cmd = round((tau_sat / tau_max) * 1023);
    pwm_cmd = max(min(pwm_cmd, 1023), -1023);
    cmd_str = sprintf("%d", pwm_cmd);
    
    try
        write(m, uint8(char(cmd_str)));
        pause(0.04);
        raw = read(c);
        data = sscanf(char(raw), '%f');
    catch ME
        warning('BLE communication dropped a package. Retrying... Error: %s', ME.message);
        continue;
    end
    
    if numel(data) == 11 && all(~isnan(data))
        gz = data(6);  
        dt = toc(lastT);
        lastT = tic;
        
        if dt <= 0 || dt > 0.5
            dt = 0.1;
        end
        
        thetaZ = thetaZ + gz * dt;
        thetaDeg = rad2deg(thetaZ);
        omega_w_est = omega_w_est + (tau_sat / Jw - tau_delay * omega_w_est) * dt;
        t = toc(t0);
        
        % Calculate wrapped error (shortest path)
        theta_err = wrapToPi(thetaZ - theta_ref);
        
        % PID Controller with Filtered Derivative
        int_err = int_err + theta_err * dt;
        xF_pointing = xF_pointing + N * (theta_err - xF_pointing) * dt;
        D_term = Kd * N * (theta_err - xF_pointing);
        tau_cmd = Kp * theta_err + Ki * int_err + D_term;
        tau_sat = max(min(tau_cmd, tau_max), -tau_max);
        
        % Log history
        history_time  = [history_time; t];
        history_theta = [history_theta; thetaDeg];
        history_omega = [history_omega; rad2deg(gz)];
        history_tau   = [history_tau; tau_sat];
        
        % Update Plots
        addpoints(h_theta, t, thetaDeg);
        addpoints(h_omega_w, t, omega_w_est);
        drawnow limitrate;
        
        fprintf("t=%5.2fs | POINTING | Gz=%7.4f rad/s | Theta=%6.2f deg | Error=%6.2f deg | Torque=%8.5f mNm\n", ...
                t, gz, thetaDeg, rad2deg(theta_err), tau_sat * 1000);
    end
    pause(pause_time - 0.05);
end

disp('--------------------------------------------------');
disp('[+] Real-Time Loop Finished. Disconnecting BLE...');
clear b;

%% 5. Export Data to MATLAB Workspace for Simulink
if ~isempty(history_time)
    disp('[*] Constructing timeseries objects for Simulink...');
    sensor_theta = timeseries(deg2rad(history_theta), history_time, 'Name', 'sensor_theta');
    sensor_omega = timeseries(deg2rad(history_omega), history_time, 'Name', 'sensor_omega');
    sensor_tau   = timeseries(history_tau, history_time, 'Name', 'sensor_tau');
    assignin('base', 'sensor_theta', sensor_theta);
    assignin('base', 'sensor_omega', sensor_omega);
    assignin('base', 'sensor_tau', sensor_tau);
    
    disp('[+] TELEMETRY EXPORTED TO WORKSPACE SUCCESSFUL:');
    disp('    -> "sensor_theta" (timeseries of live attitude angle [rad])');
    disp('    -> "sensor_omega" (timeseries of live angular velocity [rad/s])');
    disp('    -> "sensor_tau"   (timeseries of live applied control torque [Nm])');
    disp(' ');
    disp('>>> You can now open your Simulink models ("Cubesat_Control_PD")');
    disp('    and use these timeseries as inputs to evaluate how your simulated models perform');
    disp('    using real hardware-in-the-loop sensor telemetry!');
else
    disp('[-] No valid data recorded. Workspace export skipped.');
end

disp('==================================================');
disp('                SEQUENCE COMPLETE                 ');
disp('==================================================');
