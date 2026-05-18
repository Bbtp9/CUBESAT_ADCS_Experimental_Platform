% init_simulation_lqr.m
% Initialization script for Complete CubeSat Control using LQR

clear; close all; clc;

disp('==================================================');
disp('   CUBESAT LQR CONTROL & REAL-TIME PLOTTING       ');
disp('==================================================');

%% 1. Prompt User for Inputs
disp(' ');
omega0_deg = input('>> Enter INITIAL angular velocity of the CubeSat [deg/s] (e.g. 50): ');
while isempty(omega0_deg) || ~isnumeric(omega0_deg)
    omega0_deg = input('Invalid input. Enter a number [deg/s]: ');
end

theta_ref_deg = input('>> Enter TARGET Pointing Angle [deg] (e.g. 180): ');
while isempty(theta_ref_deg) || ~isnumeric(theta_ref_deg)
    theta_ref_deg = input('Invalid input. Enter a number [deg]: ');
end

%% 2. Set Physical Parameters
J  = 0.002;      % spacecraft inertia [kg*m^2]
Jw = 1e-5;       % reaction wheel inertia [kg*m^2]

tau_max  = 0.002;                % max torque [Nm]
omega_th_high = deg2rad(10000);  % artificially huge so it never switches BACK to detumbling
omega_th_low  = deg2rad(0.3);    % switch to pointing threshold

theta0   = deg2rad(0);             
theta_ref= deg2rad(theta_ref_deg); 
omega0   = deg2rad(omega0_deg);
omega_w0 = 0;                      

t_stop = 30;                      

disp(' ');
disp('[*] Physical Parameters Loaded.');
disp('[*] Computing Optimal LQR Gains...');

%% 3. LQR Control Synthesis

% --- POINTING LQR ---
% State: x = [theta, omega]'
% Dynamics: d(omega)/dt = (1/J) * tau_sat
A = [0 1; 0 0];
B = [0; 1/J];

% Q and R weightings (Tuning matrices)
% We penalize theta error heavily, and control effort moderately to respect tau_max
Q = diag([10, 15]); 
R = 5000;

Kp_lqr = lqr(A, B, Q, R); % Returns row vector [k1, k2]
fprintf('    -> Pointing LQR Matrix Gain Kp: [%.4f, %.4f]\n', Kp_lqr(1), Kp_lqr(2));

% --- DETUMBLING LQR ---
% State: x = omega
% Dynamics: d(omega)/dt = (1/J) * tau_sat
A_d = 0;
B_d = 1/J;
Q_d = 100;
R_d = 5000;

Kd_detumble = lqr(A_d, B_d, Q_d, R_d);
fprintf('    -> Detumbling LQR Scalar Gain Kd: %.4f\n', Kd_detumble);

%% 4. Build Simulink Model
disp(' ');
disp('[*] Constructing LQR Simulink Architecture...');
build_lqr_model; 

%% 5. Run the Simulation
disp('[*] Simulating Cubesat_Control_LQR.slx ...');
simOut = sim('Cubesat_Control_LQR.slx');
disp('[+] Simulation complete! Extracting data for real-time visualization...');

%% 6. Real-Time Telemetry Plot
disp('[*] Launching Real-Time Animated Graph Viewer...');
plot_cubesat_results_rt;

disp('==================================================');
disp('                SEQUENCE COMPLETE                 ');
disp('==================================================');
