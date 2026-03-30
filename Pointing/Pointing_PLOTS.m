close all
clc

if ~exist('simOut','var')
    error('simOut nu exista in workspace. Ruleaza mai intai simularea.');
end

% Extract data
t       = simOut.tout;
theta   = simOut.theta.Data;
omega   = simOut.omega.Data;
tau     = simOut.tau.Data;
omega_w = simOut.omega_w.Data;

% Unit conversions
theta_deg = theta * 180/pi;
omega_deg = omega * 180/pi;

% Colors
turquoise      = [0.00 0.80 0.80];
midnightpurple = [0.35 0.25 0.55];
darkyellow     = [0.85 0.65 0.13];
pastelgreen    = [0.55 0.80 0.60];

% Figure + dark style
fig = figure('Color','k','Name','Pointing Results');

ax1 = subplot(4,1,1);
plot(t, theta_deg, 'LineWidth', 2, 'Color', turquoise)
grid on
ylabel('\theta [deg]', 'Color','w')
title('Attitude angle', 'Color','w')

ax2 = subplot(4,1,2);
plot(t, omega_deg, 'LineWidth', 2, 'Color', midnightpurple)
grid on
ylabel('\omega [deg/s]', 'Color','w')
title('Body angular rate', 'Color','w')

ax3 = subplot(4,1,3);
plot(t, tau, 'LineWidth', 2, 'Color', darkyellow)
grid on
ylabel('\tau [N m]', 'Color','w')
title('Control torque', 'Color','w')

ax4 = subplot(4,1,4);
plot(t, omega_w, 'LineWidth', 2, 'Color', pastelgreen)
grid on
ylabel('\omega_w [rad/s]', 'Color','w')
xlabel('Time [s]', 'Color','w')
title('Reaction wheel speed', 'Color','w')

% Apply dark axes formatting
axs = [ax1 ax2 ax3 ax4];
for ax = axs
    set(ax, 'Color', 'k', ...
            'XColor', 'w', ...
            'YColor', 'w', ...
            'GridColor', [0.6 0.6 0.6], ...
            'GridAlpha', 0.25, ...
            'MinorGridColor', [0.4 0.4 0.4], ...
            'MinorGridAlpha', 0.15, ...
            'FontSize', 11)
end