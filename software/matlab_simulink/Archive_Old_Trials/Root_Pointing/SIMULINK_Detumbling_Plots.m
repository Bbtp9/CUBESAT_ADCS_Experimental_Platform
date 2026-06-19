%% SIMULINK_Detumbling_Plots.m
% Plot results for CubeSat detumbling + pointing simulation

close all
clc

% -------------------- CHECK SIMULATION OUTPUT --------------------
if ~exist('simOut','var')
    error('simOut does not exist in workspace. Run the initialization/simulation script first.');
end

if ~exist('omega_th','var')
    error('omega_th does not exist in workspace. Run the initialization script first.');
end

% -------------------- EXTRACT DATA --------------------
t = simOut.tout;

theta   = simOut.theta.Data;
omega   = simOut.omega.Data;
tau     = simOut.tau.Data;
omega_w = simOut.omega_w.Data;

% -------------------- UNIT CONVERSION --------------------
theta_deg    = rad2deg(theta);
omega_deg    = rad2deg(omega);
omega_th_deg = rad2deg(omega_th);

% -------------------- FIND SWITCHING TIME --------------------
idx_switch = find(abs(omega) <= omega_th, 1, 'first');

if isempty(idx_switch)
    t_switch = NaN;
    fprintf('Switching threshold not reached during simulation.\n');
else
    t_switch = t(idx_switch);
    fprintf('Detumbling -> Pointing switch at t = %.3f s\n', t_switch);
end

% -------------------- COLORS --------------------
turquoise      = [0.00 0.80 0.80];
midnightpurple = [0.50 0.40 0.85];
darkyellow     = [0.90 0.75 0.20];
pastelgreen    = [0.55 0.85 0.60];
lightred       = [0.95 0.35 0.35];
lightgray      = [0.70 0.70 0.70];

% -------------------- FIGURE --------------------
figure('Color','k', ...
       'Name','Detumbling + Pointing Results', ...
       'Position',[100 50 1100 900]);

% -------------------- PLOT 1: ATTITUDE ANGLE --------------------
ax1 = subplot(4,1,1);
plot(t, theta_deg, 'LineWidth', 2, 'Color', turquoise)
hold on
grid on
ylabel('\theta [deg]', 'Color','w')
title('Attitude Angle', 'Color','w')

if ~isnan(t_switch)
    xline(t_switch, '--', 'Color', lightred, 'LineWidth', 1.5);
end

% -------------------- PLOT 2: BODY ANGULAR RATE --------------------
ax2 = subplot(4,1,2);
plot(t, omega_deg, 'LineWidth', 2, 'Color', midnightpurple)
hold on
yline( omega_th_deg, '--', 'Color', lightgray, 'LineWidth', 1.2);
yline(-omega_th_deg, '--', 'Color', lightgray, 'LineWidth', 1.2);
grid on
ylabel('\omega [deg/s]', 'Color','w')
title('Body Angular Rate', 'Color','w')

if ~isnan(t_switch)
    xline(t_switch, '--', 'Color', lightred, 'LineWidth', 1.5);
end

% -------------------- PLOT 3: CONTROL TORQUE --------------------
ax3 = subplot(4,1,3);
plot(t, tau, 'LineWidth', 2, 'Color', darkyellow)
hold on
grid on
ylabel('\tau [N m]', 'Color','w')
title('Control Torque', 'Color','w')

if ~isnan(t_switch)
    xline(t_switch, '--', 'Color', lightred, 'LineWidth', 1.5);
end

% -------------------- PLOT 4: REACTION WHEEL SPEED --------------------
ax4 = subplot(4,1,4);
plot(t, omega_w, 'LineWidth', 2, 'Color', pastelgreen)
hold on
grid on
ylabel('\omega_w [rad/s]', 'Color','w')
xlabel('Time [s]', 'Color','w')
title('Reaction Wheel Speed', 'Color','w')

if ~isnan(t_switch)
    xline(t_switch, '--', 'Color', lightred, 'LineWidth', 1.5);
end

% -------------------- AXES FORMATTING --------------------
axs = [ax1 ax2 ax3 ax4];

for ax = axs
    set(ax, 'Color', 'k', ...
            'XColor', 'w', ...
            'YColor', 'w', ...
            'GridColor', [0.7 0.7 0.7], ...
            'GridAlpha', 0.25, ...
            'FontSize', 11, ...
            'LineWidth', 1);
end

% -------------------- LABEL DETUMBLING / POINTING --------------------
if ~isnan(t_switch)
    yl = ylim(ax1);

    text(ax1, t_switch/2, yl(2)-0.12*(yl(2)-yl(1)), ...
        'DETUMBLING', ...
        'Color','w', ...
        'FontWeight','bold', ...
        'HorizontalAlignment','center', ...
        'BackgroundColor','k');

    text(ax1, (t_switch + t(end))/2, yl(2)-0.12*(yl(2)-yl(1)), ...
        'POINTING', ...
        'Color','w', ...
        'FontWeight','bold', ...
        'HorizontalAlignment','center', ...
        'BackgroundColor','k');
end