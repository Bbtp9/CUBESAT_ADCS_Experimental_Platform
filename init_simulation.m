% init_simulation.m
% Initialization script for complete CubeSat Attitude Control Run

clear; close all; clc;

disp('==================================================');
disp('   CUBESAT DETUMBLING AND POINTING CONTROLLER     ');
disp('==================================================');

%% 1. Prompt User for Inputs
disp(' ');
omega0_deg = input('>> Enter INITIAL angular velocity of the CubeSat [deg/s]: ');
while isempty(omega0_deg) || ~isnumeric(omega0_deg)
    omega0_deg = input('Invalid input. Enter a number [deg/s]: ');
end

theta_ref_deg = input('>> Enter TARGET Pointing Angle [deg]: ');
while isempty(theta_ref_deg) || ~isnumeric(theta_ref_deg)
    theta_ref_deg = input('Invalid input. Enter a number [deg]: ');
end

%% 2. Set Physical and Controller Parameters
% Cubesat and Wheel Inertias
J  = 0.002;      % spacecraft inertia [kg*m^2]
Jw = 1e-5;       % reaction wheel inertia [kg*m^2]

% Controller Gains
Kp = 0.02;             % proportional pointing gain
Kd = 0.01;             % derivative pointing gain
Kd_detumble = 0.03;    % derivative detumbling gain

% Constraints & Thresholds
tau_max  = 0.002;                % max torque [Nm]
omega_th_high = deg2rad(10000);  % artificially huge so it never switches BACK to detumbling
omega_th_low  = deg2rad(0.3);    % switch to pointing threshold

% Initial Conditions (Convert user inputs to radians)
theta0   = deg2rad(0);             % start from 0 deg for pure attitude tracking 
                                   % (or we could use previous theta0)
theta_ref= deg2rad(theta_ref_deg); % commanded angle
omega0   = deg2rad(omega0_deg);
omega_w0 = 0;                      % wheel starts at 0

t_stop = 300;                      % Simulate for 300 seconds

disp(' ');
disp('[*] Parameters Loaded. Generating the full Simulink Model...');

%% 3. Build Simulink Model
% build_simulink_model; % Model is already built, no need to recreate

%% 4. Run the Simulation
disp('[*] Simulating Cubesat_Control_System.slx ...');
simOut = sim('Cubesat_Control_System.slx');

disp('[+] Simulation complete! Extracting data for plotting...');

%% 5. Plot Results
plot_cubesat_results;

disp('==================================================');
disp('                SEQUENCE COMPLETE                 ');
disp('==================================================');
