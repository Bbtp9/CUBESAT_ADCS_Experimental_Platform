%% run_simulink_hil.m
% =========================================================================
%   CUBESAT SIMULINK REAL-TIME HIL CONTROLLER LAUNCHER
% =========================================================================
%   - Sets up workspace gains and inertia parameters
%   - Prompts you to select: 1) USB Serial or 2) Wireless BLE
%   - Runs the update script to integrate the HIL block and switches
%   - Programmatically swaps the driver block class depending on choice
%   - Configures serial port name (if USB Serial mode is active)
%   - Automatically enables Simulation Pacing and opens the model
% =========================================================================

clearvars; clc; close all;

disp('==================================================');
disp('   CUBESAT SIMULINK REAL-TIME HIL CONTROLLER      ');
disp('==================================================');

%% 1. Set Workspace Parameters (Shared with Simulink Model)
J  = 0.000634;        % Spacecraft body inertia [kg*m^2]
Jw = 4.607e-5;        % Reaction wheel inertia [kg*m^2]
tau_max = 0.002;      % Maximum control torque [Nm]

% PD Controller Gains
Kp = 0.02;            % Proportional pointing gain
Kd = 0.01;            % Derivative pointing gain
Kd_detumble = 0.03;   % Detumbling gain (positive for motor command damping)

% Thresholds
omega_th_low = deg2rad(2.0);  % Detumbling completion threshold (2.0 deg/s)
omega_th_high = deg2rad(10);  % Re-engage detumbling
t_stop = Inf;                 % Simulation stop time (infinite for real-time HIL run)

% Export to base workspace
assignin('base', 'J', J);
assignin('base', 'Jw', Jw);
assignin('base', 'Kp', Kp);
assignin('base', 'Kd', Kd);
assignin('base', 'Kd_detumble', Kd_detumble);
assignin('base', 'tau_max', tau_max);
assignin('base', 'omega_th_low', omega_th_low);
assignin('base', 'omega_th_high', omega_th_high);
assignin('base', 't_stop', t_stop);

%% 2. Choose Connection Mode (USB Serial vs Wireless BLE)
disp('Connection Mode Selection:');
disp('  1) USB Serial (Wired Cable Connection)');
disp('  2) Wireless BLE (Bluetooth Low Energy for Battery Power)');
mode_choice = input('>> Select connection mode [1 or 2] (default 1): ');
if isempty(mode_choice) || ~ismember(mode_choice, [1, 2])
    mode_choice = 1;
end

portName = '';
if mode_choice == 1
    systemClass = 'SerialHILSystemObject';
    disp(' ');
    disp('[*] Scanning for available Serial/USB ports...');
    ports = serialportlist("all");

    if isempty(ports)
        warning('No serial ports found! Make sure your ESP32 is connected.');
        portName = input('>> Enter port name manually (e.g. "/dev/tty.usbmodem-10" or "COM3"): ', 's');
    else
        disp('Available ports:');
        for i = 1:length(ports)
            fprintf('  %d) %s\n', i, ports(i));
        end
        choice = input('>> Select port number (or press ENTER to type manually): ');
        if isempty(choice) || choice < 1 || choice > length(ports)
            portName = input('>> Enter port name manually: ', 's');
        else
            portName = ports(choice);
        end
    end
    if isempty(portName)
        error('[-] Invalid port name. Aborting.');
    end
    fprintf('[+] Target Serial Port: %s\n\n', portName);
else
    systemClass = 'BLEHILSystemObject';
    fprintf('[+] Wireless BLE Mode Selected. Connecting to "ESP32_IMU_BLE" dynamically...\n\n');
end

%% 3. Add Paths and Update Simulink Model
script_dir = fileparts(mfilename('fullpath'));
simulink_dir = fullfile(script_dir, '..', 'simulink');
addpath(script_dir);
addpath(simulink_dir);

% Programmatically configure HIL path and switches
integrate_hil_into_pd_model;

% 4. Load the model and configure block parameters
modelName = 'Cubesat_Control_PD';
load_system(modelName);

% Set selected SystemClass (Serial vs BLE)
try
    set_param([modelName '/HIL_Interface'], 'System', systemClass);
    fprintf('[+] Configured block "%s/HIL_Interface" to use "%s"\n', modelName, systemClass);
catch ME
    warning('Failed to configure block SystemClass: %s', ME.message);
end

% Set selected PortName if in USB mode
if mode_choice == 1
    try
        set_param([modelName '/HIL_Interface'], 'PortName', portName);
        fprintf('[+] Configured block "%s/HIL_Interface" with port "%s"\n', modelName, portName);
    catch ME
        warning('Failed to configure block portName: %s', ME.message);
    end
end

% Enable Pacing (Real-Time Mode)
set_param(modelName, 'EnablePacing', 'on');
set_param(modelName, 'PacingRate', '1');
save_system(modelName);

% Open Model
open_system(modelName);

disp(' ');
disp('==================================================');
disp('   SIMULINK HIL MODEL IS READY!');
disp('   How to run:');
disp('   1. Double-click the manual switches inside the model');
disp('      to toggle them from "Sim" to "HIL" mode.');
disp('   2. Double-click "Live_Scope" to open the real-time graph.');
disp('   3. Click the RUN button in Simulink.');
disp('==================================================');
