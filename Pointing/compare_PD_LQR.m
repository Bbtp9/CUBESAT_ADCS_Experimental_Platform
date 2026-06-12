%% compare_PD_LQR.m
% Simultaneous comparison between Manual PD controller and LQR controller
% for CubeSat attitude control (single-axis, reaction wheel actuated).
%
% Simulation time: 10 seconds (as requested).
% The script runs both simulations in parallel and plots them in two formats:
% 1) A 4x1 vertical layout (aligned in time)
% 2) A 2x2 grid layout (side-by-side comparison)
% Both are exported as premium dark-theme PNG images.

clear; clc; close all;

%% 1. PHYSICAL PARAMETERS & DELAY (CubeSat + Reaction Wheel)
J  = 0.000634;     % Spacecraft moment of inertia [kg*m^2]
Jw = 4.607e-5;     % Reaction wheel moment of inertia [kg*m^2]
tau_delay = 2.3;   % Measured command transmission delay [s]

% Actuator torque limits
tau_max = 0.002;   % Maximum control torque [N*m]

%% 2. INITIAL AND REFERENCE CONDITIONS
theta0   = deg2rad(15);    % Initial attitude angle [rad] (15 degrees)
omega0   = deg2rad(8);     % Initial body angular velocity [rad/s] (8 deg/s)
omega_w0 = 0;              % Initial reaction wheel speed [rad/s]

x0 = [theta0; omega0; omega_w0]; % Initial state vector [theta; omega; omega_w]

% Get reference angle from the user (defaults to 45 degrees)
fprintf('========================================================\n');
fprintf('  COMPARATIVE SIMULATION: PD vs LQR CONTROLLER (10 SEC) \n');
fprintf('========================================================\n');
theta_ref_deg = input('Enter desired reference angle [deg] (default 45): ');
if isempty(theta_ref_deg) || ~isnumeric(theta_ref_deg) || ~isscalar(theta_ref_deg)
    theta_ref_deg = 45;
end
theta_ref = deg2rad(theta_ref_deg);
fprintf('Reference angle set to: %.2f deg (%.4f rad)\n\n', theta_ref_deg, theta_ref);

%% 3. CONTROLLER DESIGN

% --- 3.1. Analytical PID Controller ---
% Target closed-loop bandwidth lambda (must be > tau/3 to keep Kd > 0)
lambda = 1.0; 
Kp = 3 * J * (lambda^2);
Kd = J * (3 * lambda - tau_delay);
Ki = J * (lambda^3);

% --- 3.2. LQR (Linear Quadratic Regulator) Controller ---
% State-space model updated with delay:
% e_theta = theta - theta_ref
% dx_sub/dt = A*x_sub + B*u
A = [0, 1;
     0, -tau_delay];
B = [0;
     -1/J];

% Weighting matrices Q and R
Q = [50, 0;    % Penalty for attitude error (e_theta)
     0,  5];   % Penalty for body angular rate (omega)
R = 1;         % Penalty for control effort (u = tau)

% Analytical LQR gains calculation for the system with time delay
q1 = Q(1,1);
q2 = Q(2,2);
K1_lqr = -sqrt(q1/R);
K2_lqr = J*tau_delay - sqrt(J^2*tau_delay^2 + (2*J*sqrt(q1*R) + q2)/R);
K_lqr = [K1_lqr, K2_lqr];

fprintf('Optimal LQR gains (analytical solution with tau = %.2f s):\n', tau_delay);
fprintf('  K_theta = %.6f (equivalent Kp_lqr)\n', -K_lqr(1));
fprintf('  K_omega = %.6f (equivalent Kd_lqr)\n\n', -K_lqr(2));

%% 4. RUN SIMULATIONS (Duration: 10 seconds)
tspan = [0 10]; % 10-second simulation span

% --- Simulation 1: PID ---
fprintf('Running PID simulation...\n');
x0_pid = [theta0; omega0; omega_w0; 0]; % theta, omega, omega_w, int_error
[t_pd, x_pd] = ode45(@(t, x) pid_dynamics(t, x, J, Jw, Kp, Ki, Kd, theta_ref, tau_max, tau_delay), tspan, x0_pid);

% --- Simulation 2: LQR ---
fprintf('Running Optimal LQR simulation...\n');
[t_lqr, x_lqr] = ode45(@(t, x) spacecraft_dynamics_lqr(t, x, J, Jw, K_lqr, theta_ref, tau_max, tau_delay), tspan, x0);

%% 5. DATA EXTRACTION AND UNIT CONVERSION
% Extract PID states
theta_pd   = rad2deg(x_pd(:,1));
omega_pd   = rad2deg(x_pd(:,2));
omega_w_pd = x_pd(:,3);

pwm_pd = zeros(length(t_pd), 1);
for i = 1:length(t_pd)
    tau_val = control_law_pid(x_pd(i, :)', Kp, Ki, Kd, theta_ref, tau_max);
    pwm_pd(i) = (tau_val / tau_max) * 1023;
end

% Extract Optimal LQR states
theta_lqr   = rad2deg(x_lqr(:,1));
omega_lqr   = rad2deg(x_lqr(:,2));
omega_w_lqr = x_lqr(:,3);

pwm_lqr = zeros(length(t_lqr), 1);
for i = 1:length(t_lqr)
    tau_val = control_law(x_lqr(i, 1:2)', K_lqr, theta_ref, tau_max);
    pwm_lqr(i) = (tau_val / tau_max) * 1023;
end

%% 6. SAVE SIMULATION DATA FOR EXPORT
save('comparison_data.mat', 't_pd', 'theta_pd', 'omega_pd', 'omega_w_pd', 'pwm_pd', ...
                            't_lqr', 'theta_lqr', 'omega_lqr', 'omega_w_lqr', 'pwm_lqr', ...
                            'J', 'Jw', 'Kp', 'Ki', 'Kd', 'K_lqr', 'theta_ref_deg');
fprintf('Simulation data successfully exported to "comparison_data.mat".\n\n');

%% 7. FORMAT 1: VERTICAL TIME-ALIGNED VISUALIZATION (4x1)
fig1 = figure('Color', 'k', 'Name', 'PD vs LQR Vertical Comparison (10s)', 'Position', [50, 50, 1200, 850]);

% Custom color palette for premium design
turquoise      = [0.00 0.80 0.80];
orange         = [1.00 0.55 0.15];
midnightpurple = [0.50 0.40 0.85];
magenta        = [0.95 0.35 0.65];
darkyellow     = [0.85 0.70 0.15];
brightyellow   = [1.00 0.90 0.10];
pastelgreen    = [0.55 0.85 0.60];
cyan           = [0.20 0.70 1.00];

% --- Subplot 1: Attitude Angle (theta) ---
ax1 = subplot(4, 1, 1);
plot(t_pd, theta_pd, 'LineWidth', 2.5, 'Color', turquoise); hold on;
plot(t_lqr, theta_lqr, 'LineWidth', 2.5, 'Color', orange, 'LineStyle', '--');
yline(theta_ref_deg, 'w:', 'LineWidth', 1.5); % Reference line
grid on;
ylabel('\theta [deg]', 'Color', 'w');
title('Attitude Angle Comparison (\theta)', 'Color', 'w', 'FontSize', 12);
legend('PID Controller', 'Optimal LQR', 'Reference', 'TextColor', 'w', 'Location', 'southeast', 'Color', 'none', 'EdgeColor', 'none');

% --- Subplot 2: Spacecraft Body Rate (omega) ---
ax2 = subplot(4, 1, 2);
plot(t_pd, omega_pd, 'LineWidth', 2, 'Color', midnightpurple); hold on;
plot(t_lqr, omega_lqr, 'LineWidth', 2, 'Color', magenta, 'LineStyle', '--');
grid on;
ylabel('\omega [deg/s]', 'Color', 'w');
title('Spacecraft Body Angular Rate (\omega)', 'Color', 'w', 'FontSize', 12);
legend('PID Controller', 'Optimal LQR', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

% --- Subplot 3: Motor PWM Command ---
ax3 = subplot(4, 1, 3);
plot(t_pd, pwm_pd, 'LineWidth', 2, 'Color', darkyellow); hold on;
plot(t_lqr, pwm_lqr, 'LineWidth', 2, 'Color', brightyellow, 'LineStyle', '--');
yline(1023, 'r--', 'LineWidth', 1.2);  % Upper limit
yline(-1023, 'r--', 'LineWidth', 1.2); % Lower limit
grid on;
ylabel('Motor PWM [Units]', 'Color', 'w');
title('Motor Speed Command (PWM)', 'Color', 'w', 'FontSize', 12);
legend('PID Controller', 'Optimal LQR', 'PWM Limit', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

% --- Subplot 4: Reaction Wheel Speed (omega_w) ---
ax4 = subplot(4, 1, 4);
plot(t_pd, omega_w_pd, 'LineWidth', 2, 'Color', pastelgreen); hold on;
plot(t_lqr, omega_w_lqr, 'LineWidth', 2, 'Color', cyan, 'LineStyle', '--');
grid on;
ylabel('\omega_w [rad/s]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Reaction Wheel Angular Velocity (\omega_w)', 'Color', 'w', 'FontSize', 12);
legend('PID Controller', 'Optimal LQR', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

% Style configuration for modern axes look (Dark theme)
axs1 = [ax1, ax2, ax3, ax4];
for ax = axs1
    set(ax, 'Color', 'k', ...
            'XColor', 'w', ...
            'YColor', 'w', ...
            'GridColor', [0.5 0.5 0.5], ...
            'GridAlpha', 0.3, ...
            'MinorGridColor', [0.3 0.3 0.3], ...
            'MinorGridAlpha', 0.15, ...
            'FontSize', 11, ...
            'LineWidth', 1.2);
end

% Save high-resolution vertical comparison plot to PNG
saveas(fig1, 'PD_vs_LQR_10s.png');
fprintf('Vertical comparative plot successfully saved to "PD_vs_LQR_10s.png".\n');

%% 8. FORMAT 2: SIDE-BY-SIDE 2x2 GRID VISUALIZATION
fig2 = figure('Color', 'k', 'Name', 'PD vs LQR Grid Comparison (10s)', 'Position', [150, 80, 1200, 750]);

% --- Subplot 1: Attitude Angle (Top Left) ---
ax_g1 = subplot(2, 2, 1);
plot(t_pd, theta_pd, 'LineWidth', 2.5, 'Color', turquoise); hold on;
plot(t_lqr, theta_lqr, 'LineWidth', 2.5, 'Color', orange, 'LineStyle', '--');
yline(theta_ref_deg, 'w:', 'LineWidth', 1.5);
grid on;
ylabel('\theta [deg]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Attitude Angle (\theta)', 'Color', 'w', 'FontSize', 12);
legend('PID Controller', 'Optimal LQR', 'Reference', 'TextColor', 'w', 'Location', 'southeast', 'Color', 'none', 'EdgeColor', 'none');

% --- Subplot 2: Spacecraft Body Rate (Top Right) ---
ax_g2 = subplot(2, 2, 2);
plot(t_pd, omega_pd, 'LineWidth', 2, 'Color', midnightpurple); hold on;
plot(t_lqr, omega_lqr, 'LineWidth', 2, 'Color', magenta, 'LineStyle', '--');
grid on;
ylabel('\omega [deg/s]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Spacecraft Body Angular Rate (\omega)', 'Color', 'w', 'FontSize', 12);
legend('PID Controller', 'Optimal LQR', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

% --- Subplot 3: Commanded Motor PWM (Bottom Left) ---
ax_g3 = subplot(2, 2, 3);
plot(t_pd, pwm_pd, 'LineWidth', 2, 'Color', darkyellow); hold on;
plot(t_lqr, pwm_lqr, 'LineWidth', 2, 'Color', brightyellow, 'LineStyle', '--');
yline(1023, 'r--', 'LineWidth', 1.2);
yline(-1023, 'r--', 'LineWidth', 1.2);
grid on;
ylabel('Motor PWM [Units]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Motor Speed Command (PWM)', 'Color', 'w', 'FontSize', 12);
legend('PID Controller', 'Optimal LQR', 'PWM Limit', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

% --- Subplot 4: Reaction Wheel Speed (Bottom Right) ---
ax_g4 = subplot(2, 2, 4);
plot(t_pd, omega_w_pd, 'LineWidth', 2, 'Color', pastelgreen); hold on;
plot(t_lqr, omega_w_lqr, 'LineWidth', 2, 'Color', cyan, 'LineStyle', '--');
grid on;
ylabel('\omega_w [rad/s]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Reaction Wheel Angular Velocity (\omega_w)', 'Color', 'w', 'FontSize', 12);
legend('PID Controller', 'Optimal LQR', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

% Style configuration for the 2x2 grid
axs_grid = [ax_g1, ax_g2, ax_g3, ax_g4];
for ax = axs_grid
    set(ax, 'Color', 'k', ...
            'XColor', 'w', ...
            'YColor', 'w', ...
            'GridColor', [0.5 0.5 0.5], ...
            'GridAlpha', 0.3, ...
            'MinorGridColor', [0.3 0.3 0.3], ...
            'MinorGridAlpha', 0.15, ...
            'FontSize', 11, ...
            'LineWidth', 1.2);
end

% Save high-resolution grid plot to PNG
saveas(fig2, 'PD_vs_LQR_10s_grid.png');
fprintf('Comparative grid plot successfully saved to "PD_vs_LQR_10s_grid.png".\n');

fprintf('Simulation completed successfully!\n');

%% ==================== AUXILIARY FUNCTIONS ====================

% Control law formulation with saturation limits for LQR
function tau = control_law(x_sub, K, theta_ref, tau_max)
    % e_theta = theta - theta_ref
    err = wrapToPi(x_sub(1) - theta_ref);
    omega = x_sub(2);
    
    % Control command formulation: u = -K * x_sub
    tau_cmd = -K * [err; omega];
    
    % Apply actuator torque saturation
    tau = max(min(tau_cmd, tau_max), -tau_max);
end

% Control law formulation with saturation limits for PID
function tau = control_law_pid(x_pid, Kp, Ki, Kd, theta_ref, tau_max)
    theta = x_pid(1);
    omega = x_pid(2);
    int_err = x_pid(4);
    
    err = wrapToPi(theta - theta_ref);
    tau_cmd = Kp * err + Ki * int_err + Kd * omega;
    
    tau = max(min(tau_cmd, tau_max), -tau_max);
end

% Non-linear differential equations representing satellite + reaction wheel dynamics for LQR
function dx = spacecraft_dynamics_lqr(~, x, J, Jw, K, theta_ref, tau_max, tau_delay)
    theta   = x(1);
    omega   = x(2);
    omega_w = x(3);
    
    tau = control_law([theta; omega], K, theta_ref, tau_max);
    
    dtheta   = omega;
    domega   = -tau / J - tau_delay * omega;
    domega_w =  tau / Jw - tau_delay * omega_w;
    
    dx = [dtheta; domega; domega_w];
end

% Non-linear differential equations representing satellite + reaction wheel dynamics for PID
function dx = pid_dynamics(~, x, J, Jw, Kp, Ki, Kd, theta_ref, tau_max, tau_delay)
    theta   = x(1);
    omega   = x(2);
    omega_w = x(3);
    int_err = x(4);
    
    tau = control_law_pid(x, Kp, Ki, Kd, theta_ref, tau_max);
    
    dtheta   = omega;
    domega   = -tau / J - tau_delay * omega;
    domega_w =  tau / Jw - tau_delay * omega_w;
    dint_err = wrapToPi(theta - theta_ref);
    
    dx = [dtheta; domega; domega_w; dint_err];
end
