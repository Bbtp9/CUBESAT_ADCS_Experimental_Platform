% filepath: /Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/Pointing/Reaction_Wheel_sim.m
% reaction_wheel_sim.m
% Single-axis CubeSat with one reaction wheel: detumble first, then pointing

clear; clc; close all;

% --- Spacecraft parameters ---
J   = 0.000634;     % spacecraft inertia [kg*m^2]
Jw  = 4.607e-5;     % reaction wheel inertia [kg*m^2]

% --- Control gains ---
Kp = 0.06;          % pointing proportional term
Kd = 0.03;          % pointing derivative term
Kd_detumble = 0.04; % detumble derivative-only term

% --- Actuator limits ---
tau_max = 0.002;    % max wheel torque [N*m]
omega_th_high = deg2rad(1.0); % detumble active above this
omega_th_low  = deg2rad(0.3); % pointing active below this

% --- Mission settings ---
theta_ref_deg = 90;             % desired pointing angle [deg]
theta_ref = deg2rad(theta_ref_deg);

% --- Initial tumble conditions ---
theta0_deg = input('Enter initial attitude angle in degrees: ');
theta0 = deg2rad(theta0_deg);
omega0_deg = input('Enter initial angular velocity in degrees per second: ');
omega0 = deg2rad(omega0_deg);
omega_w0 = 0;

x0 = [theta0; omega0; omega_w0];
tspan = [0 200];

% --- Run ODE ---
[t,x] = ode45(@(t,x) dynamics_rw(t,x,J,Jw,Kp,Kd,Kd_detumble,tau_max,theta_ref,omega_th_high,omega_th_low), tspan, x0);

theta   = wrapToPi(x(:,1));
omega   = x(:,2);
omega_w = x(:,3);

% --- Compute history (including mode) ---
tau = zeros(size(t));
mode = zeros(size(t));
for i = 1:length(t)
    [tau(i), mode(i)] = control_rw([theta(i); omega(i)], Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th_high, omega_th_low);
end

% --- Plot results ---
figure('Name','Attitude and Mode');
subplot(3,1,1);
plot(t, -rad2deg(theta), 'Color', [0 0.8 0.8], 'LineWidth', 1.8); grid on;
ylabel('-\theta [deg]');
title(['Attitude (target ' num2str(theta_ref_deg) '°)']);

subplot(3,1,2);
plot(t, rad2deg(omega), 'Color', [0.3 0.9 0.5], 'LineWidth', 1.6); grid on;
ylabel('\omega [deg/s]');
title('Body rate');

subplot(3,1,3);
plot(t, mode, 'Color', [0.2 0.0 0.3], 'LineWidth', 1.6); grid on;
ylabel('Mode'); ylim([0.5 2.5]);
yticks([1 2]);
yticklabels({'Detumble', 'Pointing'});
xlabel('Time [s]');

figure('Name','Reaction Wheel & Torque');
subplot(2,1,1);
plot(t, omega_w, 'Color', [0 0.8 0.8], 'LineWidth', 1.6); grid on;
ylabel('\omega_w [rad/s]');
title('Reaction wheel speed');

subplot(2,1,2);
plot(t, tau, 'Color', [0.3 0.9 0.5], 'LineWidth', 1.6); grid on;
ylabel('\tau [N m]');
xlabel('Time [s]');
title('Applied control torque');

disp('Simulation ended. Close figures manually or press Enter in Command Window.');
fig = findall(0,'Type','figure');
if isempty(fig)
    warning('No figure windows found.');
else
    waitfor(fig);  % mentine ferestrele pana la inchidere manuala
end

% --- Functions ---
function [tau, mode] = control_rw(x, Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th_high, omega_th_low)
    theta = x(1);
    omega = x(2);

    persistent mode_active;
    if isempty(mode_active)
        mode_active = 1; % detumble by default
    end

    if mode_active == 1
        if abs(omega) < omega_th_low
            mode_active = 2; % switch to pointing permanently
        end
    end
    mode = mode_active;

    if mode == 1
        tau_cmd = Kd_detumble * omega;
    else
        err = wrapToPi(theta_ref - theta);
        tau_cmd = Kp * err + Kd * omega;
    end

    tau = max(min(tau_cmd, tau_max), -tau_max);
end

function dx = dynamics_rw(~, x, J, Jw, Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th_high, omega_th_low)
    [tau, ~] = control_rw(x, Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th_high, omega_th_low);

    dtheta   = x(2);
    domega   = -tau / J;
    domega_w =  tau / Jw;
    dx = [dtheta; domega; domega_w];
end