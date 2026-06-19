%% compare_sim_real.m
% =========================================================================
%   VALIDATION TOOL: SIMULINK SIMULATION VS. PHYSICAL HIL EXPERIMENT
% =========================================================================
%   - Loads a chosen experimental run (.mat file).
%   - Automatically configures initial states and gains in the workspace.
%   - Runs the Cubesat_Control_PD or Cubesat_Control_LQR Simulink model.
%   - Compares the simulation trajectories with the real BLE data.
%   - Plots comparison curves for Attitude, Body Rate, and Control Torque.
%   - Calculates RMSE (Root Mean Square Error) for model validation.
% =========================================================================

clear; clc; close all;

disp('==================================================');
disp('   SIMULINK SIMULATION VS. REALITY COMPARISON     ');
disp('==================================================');

% 1. Prompt user for experiment file
file_name = input('>> Enter the experiment data file name to load (e.g. exp1.mat): ', 's');
if isempty(file_name)
    file_name = 'exp1.mat';
end

if ~exist(file_name, 'file')
    error('[-] File %s not found! Make sure you run Matlab_to_BLE2 and save the workspace.', file_name);
end

% Load experiment data
load(file_name);
fprintf('[+] Loaded experiment data from: %s\n', file_name);

% 2. Prompt user for controller mode used in this experiment
disp(' ');
disp('Select the controller mode that was used in this experiment:');
disp(' 1) PID Controller (Cubesat_Control_PD.slx)');
disp(' 2) LQR Controller (Cubesat_Control_LQR.slx)');
ctrl_choice = input('>> Select Controller Mode [1 or 2] (default LQR): ');
if isempty(ctrl_choice) || ~ismember(ctrl_choice, [1, 2])
    ctrl_choice = 2; % Default LQR
end

% 3. Set Physical Parameters
J   = 0.000634;     % spacecraft inertia [kg*m^2]
Jw  = 4.607e-5;     % reaction wheel inertia [kg*m^2]
tau = 2.3;          % command transmission delay [s]
tau_max = 0.002;    % max control torque [Nm]

% Initialize gains depending on mode
if ctrl_choice == 1
    modelName = 'Cubesat_Control_PD';
    lambda = 1.0;
    Kp = 3 * J * (lambda^2);
    Kd = J * (3 * lambda - tau);
    Ki = J * (lambda^3);
    Kd_detumble = 0.03;
    disp('[*] Configured PID Gains for simulation run.');
else
    modelName = 'Cubesat_Control_LQR';
    Q = [50, 0; 0, 5];
    R = 1;
    q1 = Q(1,1);
    q2 = Q(2,2);
    K1_lqr = -sqrt(q1/R);
    K2_lqr = J*tau - sqrt(J^2*tau^2 + (2*J*sqrt(q1*R) + q2)/R);
    Kp_lqr = [K1_lqr, K2_lqr];
    Kd_detumble = 0.03;
    disp('[*] Configured LQR Gains for simulation run.');
end

% Set Hysteresis thresholds matching Matlab_to_BLE2.m
omega_th_high = deg2rad(10000);  % High threshold to prevent switching back to detumble
omega_th_low  = deg2rad(0.3);    % Low threshold to switch to pointing

% Extract initial conditions from the experimental run
% We map the starting heading and rates of Phase 2 (or Phase 1 depending on where data starts)
t_real = history_time;
theta_real = history_theta;      % in degrees
omega_real = history_omega;      % in deg/s
tau_real = history_tau;          % in Nm

% Identify initial states
theta0 = deg2rad(theta_real(1));
omega0 = deg2rad(omega_real(1));
omega_w0 = 0;                    % Reaction wheel starts from 0 speed
theta_ref = deg2rad(theta_ref_deg);
t_stop = t_real(end);            % Stop simulation at the same final time

% Push initial states to Base Workspace (so Simulink can access them)
assignin('base', 'J', J);
assignin('base', 'Jw', Jw);
assignin('base', 'tau', tau);
assignin('base', 'tau_max', tau_max);
assignin('base', 'omega0', omega0);
assignin('base', 'omega_w0', omega_w0);
assignin('base', 'theta0', theta0);
assignin('base', 'theta_ref', theta_ref);
assignin('base', 'omega_th_high', omega_th_high);
assignin('base', 'omega_th_low', omega_th_low);
assignin('base', 'Kd_detumble', Kd_detumble);
assignin('base', 't_stop', t_stop);

if ctrl_choice == 1
    assignin('base', 'Kp', Kp);
    assignin('base', 'Ki', Ki);
    assignin('base', 'Kd', Kd);
else
    assignin('base', 'Kp_lqr', Kp_lqr);
end

% 4. Run Simulink Simulation
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'simulink'));
fprintf('[*] Loading and simulating model "%s.slx" for %.2f seconds...\n', modelName, t_stop);

try
    load_system(modelName);
    simOut = sim(modelName);
    
    % Extract Sim Results (handle both direct workspace variables or simOut fields)
    if isprop(simOut, 'tout')
        t_sim = simOut.tout;
    else
        t_sim = tout;
    end
    
    if isprop(simOut, 'theta_out')
        theta_sim = rad2deg(simOut.theta_out);
        omega_sim = rad2deg(simOut.omega_out);
        tau_sim = simOut.tau_out;
    else
        theta_sim = rad2deg(theta_out);
        omega_sim = rad2deg(omega_out);
        tau_sim = tau_out;
    end
    
    close_system(modelName, 0);
    disp('[+] Simulation completed successfully!');
catch ME
    error('[-] Simulink execution failed: %s', ME.message);
end

% Scale torque outputs to PWM (for ease of comparison)
pwm_real = (tau_real / tau_max) * 1023;
pwm_sim = (tau_sim / tau_max) * 1023;

% 5. Interpolate simulation data to real time indices for error calculation
theta_sim_interp = interp1(t_sim, theta_sim, t_real, 'linear', 'extrap');
omega_sim_interp = interp1(t_sim, omega_sim, t_real, 'linear', 'extrap');

% Calculate Root Mean Square Error (RMSE)
rmse_theta = sqrt(mean((theta_real - theta_sim_interp).^2));
rmse_omega = sqrt(mean((omega_real - omega_sim_interp).^2));

fprintf('\n==================================================\n');
fprintf('           MODEL VALIDATION METRICS (RMSE)        \n');
fprintf('==================================================\n');
fprintf('  Attitude Angle RMSE:   %.4f degrees\n', rmse_theta);
fprintf('  Body Angular Rate RMSE: %.4f deg/s\n', rmse_omega);
fprintf('==================================================\n\n');

% 6. Plot Comparison Figures (Dark Theme)
fig = figure('Color', 'k', 'Name', 'Simulink Simulation vs. Physical HIL Run', 'Position', [100, 100, 1100, 800]);

% Custom color scheme
color_real = [0.00 0.80 0.80];    % Turquoise (Reality)
color_sim = [1.00 0.55 0.15];     % Orange (Simulation)

% Subplot 1: Attitude Angle (Theta)
ax1 = subplot(3, 1, 1);
plot(t_real, theta_real, 'Color', color_real, 'LineWidth', 2.2); hold on;
plot(t_sim, theta_sim, 'Color', color_sim, 'LineStyle', '--', 'LineWidth', 2.2);
yline(theta_ref_deg, 'w:', 'LineWidth', 1.2);
grid on;
ylabel('\theta_z [deg]', 'Color', 'w');
title(sprintf('Attitude Angle Comparison (RMSE = %.3f°)', rmse_theta), 'Color', 'w', 'FontSize', 12);
legend('Physical Telemetry (BLE)', 'Simulink Simulation', 'Target Reference', 'TextColor', 'w', 'Color', 'none', 'EdgeColor', 'none');

% Subplot 2: Spacecraft Body Rate (Omega)
ax2 = subplot(3, 1, 2);
plot(t_real, omega_real, 'Color', color_real, 'LineWidth', 2.0); hold on;
plot(t_sim, omega_sim, 'Color', color_sim, 'LineStyle', '--', 'LineWidth', 2.0);
grid on;
ylabel('\omega [deg/s]', 'Color', 'w');
title(sprintf('Spacecraft Angular Velocity (RMSE = %.3f deg/s)', rmse_omega), 'Color', 'w', 'FontSize', 12);
legend('Physical Telemetry (BLE)', 'Simulink Simulation', 'TextColor', 'w', 'Color', 'none', 'EdgeColor', 'none');

% Subplot 3: Commanded Motor PWM
ax3 = subplot(3, 1, 3);
plot(t_real, pwm_real, 'Color', color_real, 'LineWidth', 2.0); hold on;
plot(t_sim, pwm_sim, 'Color', color_sim, 'LineStyle', '--', 'LineWidth', 2.0);
yline(1023, 'r--', 'LineWidth', 1.2);
yline(-1023, 'r--', 'LineWidth', 1.2);
grid on;
ylabel('Motor PWM [Units]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Motor Command Input (PWM Limit = \pm 1023)', 'Color', 'w', 'FontSize', 12);
legend('Physical Telemetry (BLE)', 'Simulink Simulation', 'PWM Limit', 'TextColor', 'w', 'Color', 'none', 'EdgeColor', 'none');

% Format subplots axes
for ax = [ax1, ax2, ax3]
    set(ax, 'Color', 'k', ...
            'XColor', 'w', ...
            'YColor', 'w', ...
            'GridColor', [0.5 0.5 0.5], ...
            'GridAlpha', 0.25, ...
            'FontSize', 11, ...
            'LineWidth', 1.2);
end

% Save high-res plot
saveas(fig, 'Simulation_vs_Reality_Comparison.png');
fprintf('[+] Comparison plot successfully saved as "Simulation_vs_Reality_Comparison.png".\n');
