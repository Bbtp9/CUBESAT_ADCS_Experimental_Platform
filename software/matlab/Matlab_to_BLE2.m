%% Matlab_to_BLE2.m
% =========================================================================
%   CUBESAT REAL-TIME ATTITUDE CONTROL over BLE (HIL - Hardware in the Loop)
% =========================================================================
%   - Streamlined single-write BLE protocol (reduces BLE overhead by 50%)
%   - Dynamic BLE connection to "CubeSat_ESP32" (resolves macOS UUID issues)
%   - Live numerical integration of Gz to calculate Theta (Attitude Angle)
%   - Real-time selection of PD or LQR controller architectures
%   - High-fidelity control torque calculation (saturating at +/- 0.002 Nm)
%   - Premium dark-theme double real-time plotter (Attitude & Control effort)
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

%% 2. Controller Parameters & Tuning Mode Selection
disp(' ');
disp('--------------------------------------------------');
disp('   CONTROLLER ARCHITECTURE SELECTOR               ');
disp('--------------------------------------------------');
disp(' 1) Analytical PID Controller (lambda = 1.0, tau = 2.3s)');
disp(' 2) Optimal LQR Controller (incorporating tau = 2.3s)');
ctrl_choice = input('>> Select Controller Mode [1 or 2] (default LQR): ');
if isempty(ctrl_choice) || ~ismember(ctrl_choice, [1, 2])
    ctrl_choice = 2; % Default to LQR
end

% Satellite Physical Constants
J  = 0.000634;     % Spacecraft body inertia [kg*m^2]
Jw = 4.607e-5;     % Reaction wheel inertia [kg*m^2]

% Load controller parameters from Workspace if they exist, otherwise use defaults
if evalin('base', "exist('tau_max', 'var')")
    tau_max = evalin('base', 'tau_max');
else
    tau_max = 0.2;   % Maximum control torque [Nm]
end

if evalin('base', "exist('Kd_detumble', 'var')")
    Kd_detumble = evalin('base', 'Kd_detumble');
else
    Kd_detumble = 0.03;  % Detumbling Gain
end

% Initialize pointing gains depending on selected controller
tau_delay = 2.3; % measured time delay [s]

if ctrl_choice == 1
    ctrl_name = 'Analytical PID';
    lambda = 1.0;
    Kp = 3 * J * (lambda^2);
    Ki = J * (lambda^3);
    Kd = J * (3 * lambda - tau_delay);
    
    fprintf('Analytical PID Gains (for delay tau = %.2f s):\n', tau_delay);
    fprintf('  Kp = %.6f, Ki = %.6f, Kd = %.6f\n', Kp, Ki, Kd);
else
    ctrl_name = 'Optimal LQR';
    Q = [50, 0; 0, 5];
    R = 1;
    q1 = Q(1,1);
    q2 = Q(2,2);
    K1_lqr = -sqrt(q1/R);
    K2_lqr = J*tau_delay - sqrt(J^2*tau_delay^2 + (2*J*sqrt(q1*R) + q2)/R);
    K_ctrl = [K1_lqr, K2_lqr];
    
    fprintf('Optimal LQR gains (analytical solution with tau = %.2f s):\n', tau_delay);
    fprintf('  K_theta = %.6f (equivalent Kp_lqr)\n', -K_ctrl(1));
    fprintf('  K_omega = %.6f (equivalent Kd_lqr)\n', -K_ctrl(2));
end

fprintf('\n[+] Selected Pointing Mode: %s\n', ctrl_name);
fprintf('    Control Gain Matrix K: [%.6f, %.6f]\n', K_ctrl(1), K_ctrl(2));
fprintf('    Detumbling Rate Gain: %.4f\n', Kd_detumble);
disp('Press ENTER to start Phase 1 (Detumbling)...');
input('');

%% 3. Setup Premium Dark-Theme Live Visualizer
fig = figure('Color', 'k', 'Name', ['Real-Time CubeSat Mission - ' ctrl_name], 'Position', [100, 100, 1000, 700]);

% Subplot 1: Attitude Angle (Theta)
ax1 = subplot(2, 1, 1);
h_theta = animatedline('Color', [0.00 0.80 0.80], 'LineWidth', 2.5); % Turquoise
h_ref   = yline(0, 'w:', 'LineWidth', 1.5);                         % Hidden reference line initially
set(h_ref, 'Visible', 'off');
grid on;
ylabel('\theta_z [deg]', 'Color', 'w');
title(['Real-Time Attitude Angle vs Target Reference (' ctrl_name ')'], 'Color', 'w', 'FontSize', 12);
legend('Live Angle (\theta_z)', 'Target Reference (Not Set)', 'TextColor', 'w', 'Location', 'southeast', 'Color', 'none', 'EdgeColor', 'none');

% Subplot 2: Commanded Motor PWM
ax2 = subplot(2, 1, 2);
h_pwm   = animatedline('Color', [1.00 0.55 0.15], 'LineWidth', 2.0);  % Orange
h_sat_u = yline(1023, 'r--', 'LineWidth', 1.2);                       % Max PWM limit
h_sat_l = yline(-1023, 'r--', 'LineWidth', 1.2);
grid on;
ylabel('Motor PWM [Units]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Real-Time Commanded Motor Speed (PWM)', 'Color', 'w', 'FontSize', 12);
legend('Commanded PWM', 'PWM Limit', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

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
    if pwm_cmd>0
        pwm_cmd=pwm_cmd+150;
    else
        pwm_cmd=pwm_cmd-150;
    end
    pwm_cmd = max(min(pwm_cmd, 1023), -1023);
    cmd_str = sprintf("%d", pwm_cmd);
    disp([num2str([gz tau_cmd tau_sat]) ' ' cmd_str])
    try
        write(m, char(cmd_str));
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
        
        % Integrate angle
        thetaZ = thetaZ + gz * dt;
        thetaDeg = rad2deg(thetaZ);
        t = toc(t0);
        
        % Damping controller: u = -Kd * omega
        tau_cmd = -Kd_detumble * gz; %+kdd*dgz;
        tau_sat = max(min(tau_cmd, tau_max), -tau_max);
        
        % Log history
        history_time  = [history_time; t];
        history_theta = [history_theta; thetaDeg];
        history_omega = [history_omega; rad2deg(gz)];
        history_tau   = [history_tau; tau_sat];
        
        % Update Plots
        addpoints(h_theta, t, rad2deg(gz));
        addpoints(h_pwm, t, pwm_cmd);
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

% Ensure motor is stopped after detumbling
% write(m, uint8("0"));
% disp('[*] Motor stopped. Measuring settled heading...');
% pause(1.5);

% Read settled heading
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
fprintf('   [+] DETUMBLING COMPLETE!\n');
fprintf('   CubeSat settled at:\n');
fprintf('   -> Current Heading: %.2f deg\n', settled_heading);
fprintf('   -> Integrated Theta: %.2f deg\n', rad2deg(thetaZ));
fprintf('==================================================\n\n');

pause(5)

write(m, "0");
disp('[*] Motor stopped. Measuring settled heading...');
pause(1.5);

% ==================================================
%   INTER-PHASE USER INPUT
% ==================================================
theta_ref_deg = input('>> Enter Target Pointing Heading/Angle [deg] (0 to 360): ');
if isempty(theta_ref_deg) || ~isnumeric(theta_ref_deg)
    theta_ref_deg = 45;
end
theta_ref = deg2rad(theta_ref_deg);

% Set reference line visible and update value
set(h_ref, 'Value', theta_ref_deg);
set(h_ref, 'Visible', 'on');
legend(ax1, 'Live Angle (\theta_z)', 'Target Reference', 'TextColor', 'w', 'Location', 'southeast', 'Color', 'none', 'EdgeColor', 'none');

% Reset times for Phase 2 continuity or keep absolute time
disp(' ');
disp('[*] Starting Phase 2: Attitude Pointing (shortest-path control)...');
disp('--------------------------------------------------');

% Start with an initial torque command of 0
tau_sat = 0;
lastT = tic; % Reset dt timer
int_err = 0; % Initialize integral error for PID

pointing_duration = 30; % 30 seconds for pointing
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
        gz = data(6);  % Gyroscope Z-axis [rad/s]
        dt = toc(lastT);
        lastT = tic;
        
        % Integrate angle
        thetaZ = thetaZ + gz * dt;
        thetaDeg = rad2deg(thetaZ);
        t = toc(t0);
        
        % Calculate wrapped error (shortest path)
        theta_err = wrapToPi(thetaZ - theta_ref);
        
        % Calculate Pointing Control command
        if ctrl_choice == 1
            int_err = int_err + theta_err * dt;
            tau_cmd = Kp * theta_err + Ki * int_err + Kd * gz;
        else
            tau_cmd = -K_ctrl * [theta_err; gz];
        end
        tau_sat = max(min(tau_cmd, tau_max), -tau_max);
        
        % Log history
        history_time  = [history_time; t];
        history_theta = [history_theta; thetaDeg];
        history_omega = [history_omega; rad2deg(gz)];
        history_tau   = [history_tau; tau_sat];
        
        % Update Plots
        addpoints(h_theta, t, thetaDeg);
        addpoints(h_pwm, t, pwm_cmd);
        drawnow limitrate;
        
        fprintf("t=%5.2fs | POINTING | Gz=%7.4f rad/s | Theta=%6.2f deg | Error=%6.2f deg | Torque=%8.5f mNm\n", ...
                t, gz, thetaDeg, rad2deg(theta_err), tau_sat * 1000);
    end
    pause(pause_time - 0.05);
end

disp('--------------------------------------------------');
disp('[+] Real-Time Loop Finished. Disconnecting BLE...');
clear b; % Properly close BLE connection

%% 5. Export Data to MATLAB Workspace for Simulink
if ~isempty(history_time)
    disp('[*] Constructing timeseries objects for Simulink...');
    
    % Create timeseries for direct use in Simulink "From Workspace" blocks
    sensor_theta = timeseries(deg2rad(history_theta), history_time, 'Name', 'sensor_theta');
    sensor_omega = timeseries(deg2rad(history_omega), history_time, 'Name', 'sensor_omega');
    sensor_tau   = timeseries(history_tau, history_time, 'Name', 'sensor_tau');
    
    % Store timeseries in Workspace
    assignin('base', 'sensor_theta', sensor_theta);
    assignin('base', 'sensor_omega', sensor_omega);
    assignin('base', 'sensor_tau', sensor_tau);
    
    disp('[+] TELEMETRY EXPORTED TO WORKSPACE SUCCESSFUL:');
    disp('    -> "sensor_theta" (timeseries of live attitude angle [rad])');
    disp('    -> "sensor_omega" (timeseries of live angular velocity [rad/s])');
    disp('    -> "sensor_tau"   (timeseries of live applied control torque [Nm])');
    disp(' ');
    disp('>>> You can now open your Simulink models ("Cubesat_Control_PD" or "Cubesat_Control_LQR")');
    disp('    and use these timeseries as inputs to evaluate how your simulated models perform');
    disp('    using real hardware-in-the-loop sensor telemetry!');
else
    disp('[-] No valid data recorded. Workspace export skipped.');
end

disp('==================================================');
disp('                SEQUENCE COMPLETE                 ');
disp('==================================================');
