% init_simulation_pd.m
% Initialization script for complete CubeSat Attitude Control Run

clear; close all; clc;

% Add the simulink folder to the MATLAB search path
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'simulink'));

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
% Physical parameters & Measured Time Delay
J   = 0.000634;     % spacecraft inertia [kg*m^2]
Jw  = 4.607e-5;     % reaction wheel inertia [kg*m^2]
tau = 2.3;          % measured command transmission delay [s]

% Analytical PID Controller Design (Pole Placement)
% Target closed-loop bandwidth lambda (must be > tau/3 to keep Kd > 0)
lambda = 1.0; 
Kp = 3 * J * (lambda^2);
Kd = J * (3 * lambda - tau);
Ki = J * (lambda^3);

Kd_detumble = 0.003;    % derivative detumbling gain

fprintf('\n[*] Calculated Analytical PID Gains (for delay tau = %.2f s, lambda = %.2f rad/s):\n', tau, lambda);
fprintf('    -> Proportional Gain Kp = %.6f\n', Kp);
fprintf('    -> Integral Gain Ki = %.6f\n', Ki);
fprintf('    -> Derivative Gain Kd = %.6f\n\n', Kd);

% Constraints & Thresholds
tau_max  = 0.002;                % max torque [Nm]
omega_th_high = deg2rad(10000);  % artificially huge so it never switches BACK to detumbling once pointing is active
omega_th_low  = deg2rad(0.3);    % switch to pointing threshold

% Initial Conditions (Convert user inputs to radians)
theta0   = deg2rad(0);             % start from 0 deg for pure attitude tracking 
                                   % (or we could use previous theta0)
theta_ref= deg2rad(theta_ref_deg); % commanded angle
omega0   = deg2rad(omega0_deg);
omega_w0 = 0;                      % wheel starts at 0

t_stop = 20;                       % Simulate for 30 seconds

disp(' ');
disp('[*] Parameters Loaded. Using the existing Cubesat_Control_PD.slx model...');

%% 3. Apply shortest path wrapping to Cubesat_Control_PD.slx if not already done
try
    disp('[*] Checking/Applying shortest path logic to Cubesat_Control_PD.slx...');
    modelName = 'Cubesat_Control_PD';
    load_system(modelName);
    ctrlPath = [modelName '/Attitude_Controller'];
    if isempty(find_system(ctrlPath, 'Name', 'Wrap_To_Pi'))
        % Add MATLAB Fcn block to wrap angle error
        add_block('simulink/User-Defined Functions/MATLAB Fcn', [ctrlPath '/Wrap_To_Pi']);
        set_param([ctrlPath '/Wrap_To_Pi'], 'MATLABFcn', 'wrapToPi', 'Position', [170, 60, 210, 90]);
        
        % Delete direct connection between Sum_Error and Kp_Gain
        delete_line(ctrlPath, 'Sum_Error/1', 'Kp_Gain/1');
        
        % Re-route: Sum_Error -> Wrap_To_Pi -> Kp_Gain
        add_line(ctrlPath, 'Sum_Error/1', 'Wrap_To_Pi/1', 'autorouting', 'on');
        add_line(ctrlPath, 'Wrap_To_Pi/1', 'Kp_Gain/1', 'autorouting', 'on');
        
        save_system(modelName);
        disp('[+] Shortest path wrapping (wrapToPi) applied to Cubesat_Control_PD.slx!');
    else
        disp('[*] Shortest path wrapping (wrapToPi) is already configured in the model.');
    end
    close_system(modelName);
catch ME
    disp(['[!] Could not modify Simulink model programmatically: ' ME.message]);
    disp('[!] Please ensure Cubesat_Control_PD.slx is closed and in the MATLAB path.');
end

%% 4. Run the Simulation
disp('[*] Simulating Cubesat_Control_PD.slx ...');
simOut = sim('Cubesat_Control_PD.slx');

disp('[+] Simulation complete! Extracting data for plotting...');

%% 5. Plot Results
plot_cubesat_results_rt;

disp('==================================================');
disp('                SEQUENCE COMPLETE                 ');
disp('==================================================');
