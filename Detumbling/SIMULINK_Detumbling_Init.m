%% SIMULINK_Detumbling_Init.m
% Initial parameters for CubeSat single-axis detumbling + pointing model

close all
clc
clear

%-------------------- MODEL NAME --------------------
model_name = 'DETUMBLE_POINTING';   % name of your .slx model

% -------------------- PHYSICAL PARAMETERS --------------------
J  = 0.002;      % spacecraft inertia [kg*m^2]
Jw = 1e-4;       % reaction wheel inertia [kg*m^2]

% -------------------- CONTROL GAINS --------------------
Kp          = 0.02;   % proportional gain for pointing
Kd          = 0.01;   % derivative gain for pointing
Kd_detumble = 0.03;   % detumbling gain

% -------------------- ACTUATOR LIMIT --------------------
tau_max = 5e-4;      % maximum control torque [N*m]

% -------------------- SWITCHING THRESHOLD --------------------
omega_high = deg2rad(0.7);
omega_low  = deg2rad(0.3);

% -------------------- INITIAL CONDITIONS --------------------
theta0   = deg2rad(15);    % initial attitude angle [rad]
omega0   = deg2rad(8);     % initial body angular rate [rad/s]
omega_w0 = 0;              % initial reaction wheel speed [rad/s]

% -------------------- SIMULATION TIME --------------------
t_stop = 5;               % simulation stop time [s]

% -------------------- REFERENCE INPUT --------------------
theta_ref_deg = input('Enter desired reference angle theta_ref [deg]: ');

while isempty(theta_ref_deg) || ~isnumeric(theta_ref_deg) || ~isscalar(theta_ref_deg)
    theta_ref_deg = input('Invalid input. Enter theta_ref as a scalar in degrees: ');
end

theta_ref = deg2rad(theta_ref_deg);

fprintf('\n');
fprintf('Reference angle:\n');
fprintf('theta_ref = %.2f deg = %.4f rad\n', theta_ref_deg, theta_ref);
fprintf('\n');

% -------------------- OPEN MODEL --------------------
open_system(model_name);

% -------------------- SET MODEL STOP TIME --------------------
set_param(model_name, 'StopTime', num2str(t_stop));

% -------------------- RUN SIMULATION --------------------
simOut = sim(model_name);

% -------------------- CHECK OUTPUTS --------------------
if isempty(simOut)
    error('Simulation failed: simOut is empty.');
end

fprintf('Simulation completed successfully.\n');
fprintf('Model: %s\n', model_name);
fprintf('Stop time: %.2f s\n', t_stop);

% -------------------- OPTIONAL: SAVE ALL PARAMETERS --------------------
save('detumbling_pointing_params.mat', ...
    'J', 'Jw', ...
    'Kp', 'Kd', 'Kd_detumble', ...
    'tau_max', 'omega_th', ...
    'theta0', 'omega0', 'omega_w0', ...
    'theta_ref', 'theta_ref_deg', ...
    't_stop');