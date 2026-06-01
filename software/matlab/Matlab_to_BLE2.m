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

% Search for the CubeSat_ESP32 device name
idx = find(strcmp(devices.Name, "CubeSat_ESP32"), 1);
if isempty(idx)
    error('[-] CubeSat_ESP32 device not found! Make sure the ESP32 is powered on and advertising.');
end

deviceAddress = devices.Address(idx);
fprintf('[+] Found CubeSat_ESP32! Address/UUID: %s\n', deviceAddress);
disp('[*] Connecting to CubeSat_ESP32 BLE Service...');

serviceUUID = "12345678-1234-1234-1234-1234567890AB";
charUUID    = "87654321-4321-4321-4321-BA0987654321";

try
    b = ble(deviceAddress);
    c = characteristic(b, serviceUUID, charUUID);
    disp('[+] Connection established successfully!');
catch ME
    error('[-] Failed to connect to BLE device: %s', ME.message);
end

%% 2. Controller Parameters & Tuning Mode Selection
disp(' ');
disp('--------------------------------------------------');
disp('   CONTROLLER ARCHITECTURE SELECTOR               ');
disp('--------------------------------------------------');
disp(' 1) Manual PD Controller (Kp = 0.02, Kd = 0.01)');
disp(' 2) Optimal LQR Controller (Kp_lqr analytical)');
ctrl_choice = input('>> Select Controller Mode [1 or 2] (default LQR): ');
if isempty(ctrl_choice) || ~ismember(ctrl_choice, [1, 2])
    ctrl_choice = 2; % Default to LQR
end

theta_ref_deg = input('>> Enter Target Reference Angle [deg] (default 45): ');
if isempty(theta_ref_deg) || ~isnumeric(theta_ref_deg)
    theta_ref_deg = 45;
end
theta_ref = deg2rad(theta_ref_deg);

% Satellite Physical Constants
J  = 0.000634;     % Spacecraft body inertia [kg*m^2]
Jw = 4.607e-5;     % Reaction wheel inertia [kg*m^2]
tau_max = 0.002;   % Maximum control torque [Nm]

% Initialize gains depending on selected controller
if ctrl_choice == 1
    ctrl_name = 'Manual PD';
    % Gains in terms of K = [-Kp, -Kd] as defined in compare_PD_LQR
    Kp = 0.02;
    Kd = 0.01;
    K_ctrl = [-Kp, -Kd];
else
    ctrl_name = 'Optimal LQR';
    % Analytical Riccati solver matching compare_PD_LQR
    Q = [50, 0; 0, 5];
    R = 1;
    q1 = Q(1,1);
    q2 = Q(2,2);
    K1_lqr = -sqrt(q1/R);
    K2_lqr = -sqrt((2*J*sqrt(q1*R) + q2)/R);
    K_ctrl = [K1_lqr, K2_lqr];
end

fprintf('\n[+] Selected: %s\n', ctrl_name);
fprintf('    Control Gain Matrix K: [%.6f, %.6f]\n', K_ctrl(1), K_ctrl(2));
fprintf('    Target Reference Angle: %.2f deg (%.4f rad)\n', theta_ref_deg, theta_ref);
disp('Press ENTER to start the real-time control loop...');
input('');

%% 3. Setup Premium Dark-Theme Live Visualizer
fig = figure('Color', 'k', 'Name', ['Real-Time CubeSat Control - ' ctrl_name], 'Position', [100, 100, 1000, 700]);

% Subplot 1: Attitude Angle (Theta)
ax1 = subplot(2, 1, 1);
h_theta = animatedline('Color', [0.00 0.80 0.80], 'LineWidth', 2.5); % Turquoise
h_ref   = yline(theta_ref_deg, 'w:', 'LineWidth', 1.5);              % White dotted reference line
grid on;
ylabel('\theta_z [deg]', 'Color', 'w');
title(['Real-Time Attitude Angle vs Target Reference (' ctrl_name ')'], 'Color', 'w', 'FontSize', 12);
legend('Live Angle (\theta_z)', 'Target Reference', 'TextColor', 'w', 'Location', 'southeast', 'Color', 'none', 'EdgeColor', 'none');

% Subplot 2: Control Torque (Tau)
ax2 = subplot(2, 1, 2);
h_tau   = animatedline('Color', [1.00 0.55 0.15], 'LineWidth', 2.0);  % Orange
h_sat_u = yline(tau_max * 1000, 'r--', 'LineWidth', 1.2);             % Saturation limits in mNm
h_sat_l = yline(-tau_max * 1000, 'r--', 'LineWidth', 1.2);
grid on;
ylabel('\tau [mN m]', 'Color', 'w');                                  % Plotted in mNm for clear visual resolution!
xlabel('Time [s]', 'Color', 'w');
title('Real-Time Control Torque Command (\tau)', 'Color', 'w', 'FontSize', 12);
legend('Control Torque (\tau)', 'Saturation Limit', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

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

%% 4. Real-Time Acquisition & Control Loop
thetaZ = 0;             % Integrated attitude angle [rad]
lastT = tic;            % Timer for dt calculation
t0 = tic;               % Global timer
total_duration = 30;    % Run loop for 30 seconds
sampling_rate = 10;     % Loop frequency (approx 10 Hz)
pause_time = 1 / sampling_rate;
num_iterations = total_duration * sampling_rate;

% History arrays for Workspace Export
history_time  = [];
history_theta = [];
history_omega = [];
history_tau   = [];

% Start with an initial torque command of 0
tau_sat = 0;

disp('[*] Real-Time Control Loop Engaged. Telemetry streaming starting...');
disp('--------------------------------------------------');

for k = 1:num_iterations
    if ~ishandle(fig)
        disp('[-] Figure closed by user. Terminating loop.');
        break;
    end
    
    % Streamlined Single-Write BLE Architecture:
    % MATLAB writes the current saturated torque command back to the ESP32.
    % The ESP32 immediately applies it, reads the MPU6050, and updates its
    % characteristic value with the fresh IMU data in one step!
    cmd_str = sprintf("CMD_TAU:%.6f", tau_sat);
    try
        write(c, uint8(char(cmd_str)), "WithResponse");
        pause(0.06); % Allow ESP32 a brief window to process write and update MPU6050
        raw = read(c);
        data = str2double(split(string(char(raw)), ","));
    catch ME
        warning('BLE communication dropped a package. Retrying... Error: %s', ME.message);
        continue;
    end
    
    % Verify valid package formatting (expecting 7 numbers: Ax, Ay, Az, Gx, Gy, Gz, Temp)
    if numel(data) == 7 && all(~isnan(data))
        % Extract values
        gz = data(6);  % Gyroscope Z-axis [rad/s]
        
        % Calculate actual dt (elapsed time)
        dt = toc(lastT);
        lastT = tic;
        
        % Integrate Gyro Z rate to calculate current Attitude Angle
        thetaZ = thetaZ + gz * dt;
        thetaDeg = rad2deg(thetaZ);
        t = toc(t0);
        
        % Calculate State Error (wrap angle to standard range [-pi, pi])
        theta_err = wrapToPi(thetaZ - theta_ref);
        
        % Calculate required Control Torque command (u = -K * x)
        tau_cmd = -K_ctrl * [theta_err; gz];
        
        % Apply Actuator Saturation limits
        tau_sat = max(min(tau_cmd, tau_max), -tau_max);
        
        % Log history for export
        history_time  = [history_time; t];
        history_theta = [history_theta; thetaDeg];
        history_omega = [history_omega; rad2deg(gz)];
        history_tau   = [history_tau; tau_sat];
        
        % Update Real-Time Plots
        addpoints(h_theta, t, thetaDeg);
        addpoints(h_tau, t, tau_sat * 1000); % Plotted in mNm for visual clarity!
        drawnow limitrate;
        
        % Print Real-Time Telemetry to Command Window
        fprintf("t=%5.2fs | Gz=%7.4f rad/s | Theta=%6.2f° | Error=%6.2f° | Torque=%8.5f mNm\n", ...
                t, gz, thetaDeg, rad2deg(theta_err), tau_sat * 1000);
    else
        fprintf("[-] Invalid BLE packet received: [%s]\n", char(raw));
    end
    
    pause(pause_time - 0.06); % Adjust for BLE delay to maintain ~10Hz
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
