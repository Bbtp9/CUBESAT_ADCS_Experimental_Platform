% reaction_wheel_sim.m
% Single-axis CubeSat with one reaction wheel: detumble + pointing
clear; clc; close all;

% --- Parameters ---
J   = 0.002;        % spacecraft inertia [kg*m^2]
Jw  = 1e-5;         % reaction wheel inertia [kg*m^2]

Kp  = 0.05;         % pointing proportional gain
Kd  = 0.02;         % derivative gain (pointing)
Kd_detumble = 0.03; % derivative-only detumble gain

tau_max = 0.002;    % max wheel torque [N*m]
omega_th = deg2rad(0.5);  % detumble threshold [rad/s]

% --- Mission / simulation settings ---
theta_ref_deg = 90;            % desired pointing angle [deg]
theta_ref = deg2rad(theta_ref_deg);

% Initial conditions - simulate a random tumble
theta0   = deg2rad( rand*360 - 180 );    % random initial attitude [rad]
omega0   = deg2rad( 20 * (2*rand-1) );   % random initial body rate up to +/-20 deg/s
omega_w0 = 0;                            % initial wheel speed [rad/s]

x0 = [theta0; omega0; omega_w0];

tspan = [0 200]; % seconds

% --- Run ODE ---
[t,x] = ode45(@(t,x) dynamics_rw(t,x,J,Jw,Kp,Kd,Kd_detumble,tau_max,theta_ref,omega_th), tspan, x0);

theta   = wrapToPi(x(:,1));
omega   = x(:,2);
omega_w = x(:,3);

% Compute control history
tau = zeros(size(t));
for i = 1:length(t)
    tau(i) = control_rw([theta(i); omega(i)], Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th);
end

color = [0 0.70 0.70];

figure('Name','Attitude (deg)');
plot(t, rad2deg(theta), 'Color', color, 'LineWidth', 1.6); grid on;
xlabel('Time [s]'); ylabel('\theta [deg]'); title(['Attitude — target ' num2str(theta_ref_deg) ' deg']);

figure('Name','Body Rate (deg/s)');
plot(t, rad2deg(omega), 'Color', color, 'LineWidth', 1.6); grid on;
xlabel('Time [s]'); ylabel('\omega [deg/s]'); title('Body angular rate');

figure('Name','Reaction Wheel Speed (rad/s)');
plot(t, omega_w, 'Color', color, 'LineWidth', 1.6); grid on;
xlabel('Time [s]'); ylabel('\omega_w [rad/s]'); title('Reaction wheel speed');

figure('Name','Control Torque (N*m)');
plot(t, tau, 'Color', color, 'LineWidth', 1.6); grid on;
xlabel('Time [s]'); ylabel('\tau [N m]'); title('Control torque');

% --- Functions ---
function tau = control_rw(x, Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th)
    theta = x(1);
    omega = x(2);

    % Mode switching: detumble when body rate large, otherwise pointing
    if abs(omega) > omega_th
        % Detumble: derivative-only damping
        tau_cmd = -Kd_detumble * omega;
    else
        % Pointing: PD on angle error (wrap error to [-pi,pi])
        err = wrapToPi(theta - theta_ref);
        tau_cmd = -Kp * err - Kd * omega;
    end

    % Torque saturation
    tau = max(min(tau_cmd, tau_max), -tau_max);
end

function dx = dynamics_rw(~, x, J, Jw, Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th)
    theta   = x(1);
    omega   = x(2);
    omega_w = x(3);

    tau = control_rw([theta; omega], Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th);

    dtheta   = omega;
    domega   = -tau / J;    % torque on spacecraft
    domega_w =  tau / Jw;   % equal and opposite on wheel

    dx = [dtheta; domega; domega_w];
end
