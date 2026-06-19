% plot_cubesat_results_rt.m
% Animates the plotting to simulate real-time telemetry

% Ensure data exists
if ~exist('simOut','var')
    error('No simulation data found (simOut variable missing).');
end

% Extract Data Arrays
t       = simOut.tout;
theta   = simOut.theta_out(1:length(t), 1);
omega   = simOut.omega_out(1:length(t), 1);
omega_w = simOut.omega_w_out(1:length(t), 1);
tau     = simOut.tau_out(1:length(t), 1);
pwm     = (tau / tau_max) * 1023;

% Constants
theta_deg = rad2deg(theta);
omega_deg = rad2deg(omega);
theta_ref_deg = rad2deg(theta_ref);
omega_th_low_deg = rad2deg(omega_th_low);

%% Setup Real-Time Plot Figure
turquoise      = [0.00 0.80 0.80];
midnightpurple = [0.50 0.40 0.85];
darkyellow     = [0.90 0.75 0.20];
pastelgreen    = [0.55 0.85 0.60];
lightred       = [0.95 0.35 0.35];

hFig = figure('Color','k', ...
       'Name','Real-Time Control Sequence (Telemetry Simulation)', ...
       'Position',[100 50 1100 900]);

% 1. Attitude Angle subplot
ax1 = subplot(4,1,1);
set(ax1, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
ylabel('\theta [deg]', 'Color', 'w');
title('Attitude Angle (Pointing)', 'Color', 'w');
grid on; hold on;
xlim([0 t_stop]);
ylim([min(min(theta_deg), 0)-10, max(max(theta_deg), theta_ref_deg)+10]);
yline(theta_ref_deg, '--w', 'LineWidth', 1.5, 'Label', 'Target Angle');
line_theta = animatedline('Color', turquoise, 'LineWidth', 2);

% 2. Body Angular Rate subplot
ax2 = subplot(4,1,2);
set(ax2, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
ylabel('\omega [deg/s]', 'Color', 'w');
title('Satellite Angular Velocity (Detumbling)', 'Color', 'w');
grid on; hold on;
xlim([0 t_stop]);
ylim([min(omega_deg)-5, max(omega_deg)+5]);
yline(omega_th_low_deg, '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.2);
yline(-omega_th_low_deg, '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.2);
line_omega = animatedline('Color', midnightpurple, 'LineWidth', 2);

% 3. Commanded Motor PWM subplot
ax3 = subplot(4,1,3);
set(ax3, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
ylabel('Motor PWM [Units]', 'Color', 'w');
title('Reaction Wheel Motor Command (PWM)', 'Color', 'w');
grid on; hold on;
xlim([0 t_stop]);
ylim([-1200, 1200]);
yline(1023, 'r--', 'LineWidth', 1.2);
yline(-1023, 'r--', 'LineWidth', 1.2);
line_pwm = animatedline('Color', darkyellow, 'LineWidth', 2);

% 4. Wheel Speed subplot
ax4 = subplot(4,1,4);
set(ax4, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.5 0.5 0.5]);
ylabel('\omega_w [rad/s]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Reaction Wheel Speed', 'Color', 'w');
grid on; hold on;
xlim([0 t_stop]);
ylim([min(omega_w)-5, max(omega_w)+5]);
line_omega_w = animatedline('Color', pastelgreen, 'LineWidth', 2);

%% Animation Loop
disp('Starting Real-Time Telemetry Animation...');
tic;
% Run 10x faster than real-life so user doesn't wait 300s
% while maintaining the "live data stream" feel
speedup_factor = 10; 

max_t = t(end);
current_t = 0;

% Setup a status text
annotation_str = annotation('textbox', [0.75 0.92 0.2 0.05], 'String', 'MODE: STARTING', ...
             'EdgeColor', 'none', 'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');

last_idx = 1;
while current_t <= max_t
    % elapsed time scaled
    current_t = toc * speedup_factor;
    
    % Find logical index
    idx = find(t <= current_t, 1, 'last');
    
    if ~isempty(idx) && idx > last_idx
        % Add points
        addpoints(line_theta, t(last_idx:idx), theta_deg(last_idx:idx));
        addpoints(line_omega, t(last_idx:idx), omega_deg(last_idx:idx));
        addpoints(line_pwm, t(last_idx:idx), pwm(last_idx:idx));
        addpoints(line_omega_w, t(last_idx:idx), omega_w(last_idx:idx));
        
        % Update Status Label matching the threshold logic
        if abs(omega(idx)) > omega_th_low
            set(annotation_str, 'String', 'MODE: DETUMBLING', 'Color', lightred);
        else
            set(annotation_str, 'String', 'MODE: POINTING', 'Color', turquoise);
        end
        
        drawnow limitrate;
        last_idx = idx;
    end
    
    if ~ishghandle(hFig)
        % User closed figure manually
        disp('Animation stopped manually.');
        break;
    end
    
    pause(0.02);
end

% Make sure end points are plotted
if isvalid(line_theta)
    addpoints(line_theta, t(last_idx:end), theta_deg(last_idx:end));
    addpoints(line_omega, t(last_idx:end), omega_deg(last_idx:end));
    addpoints(line_pwm, t(last_idx:end), pwm(last_idx:end));
    addpoints(line_omega_w, t(last_idx:end), omega_w(last_idx:end));
    
    % Final state check
    idx = length(t);
    if abs(omega(idx)) > omega_th_low
        set(annotation_str, 'String', 'MODE: DETUMBLING', 'Color', lightred);
    else
        set(annotation_str, 'String', 'MODE: POINTING', 'Color', turquoise);
    end
    
    drawnow;
end

disp('Animation Completed!');
