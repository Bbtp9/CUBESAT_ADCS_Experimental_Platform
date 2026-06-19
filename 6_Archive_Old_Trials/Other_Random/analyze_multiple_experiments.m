%% analyze_multiple_experiments.m
% =========================================================================
%   POST-PROCESSING TOOL: MULTI-EXPERIMENT ANALYSIS & PERFORMANCE METRICS
% =========================================================================
%   - Loads up to 5 experimental data files (exp1.mat, exp2.mat, etc.).
%   - Calculates control metrics: Overshoot, Settling Time, Steady-State Error.
%   - Generates a 5x5 grid comparison (5 rows for experiments, 5 columns for metrics).
%   - Uses a uniform, premium color palette for each variable.
%   - Prints a structured comparison table in the MATLAB Command Window.
% =========================================================================

clear; clc; close all;

% List of experiment files
exp_files = {'exp1.mat', 'exp2.mat', 'exp3.mat', 'exp4.mat', 'exp5.mat'};
num_exp = length(exp_files);

% Preallocate arrays for metrics table
metrics_overshoot_deg = zeros(num_exp, 1);
metrics_overshoot_pct = zeros(num_exp, 1);
metrics_settling_time = zeros(num_exp, 1);
metrics_steady_state_error = zeros(num_exp, 1);
metrics_active = false(num_exp, 1);

% Colors for consistent plotting (Premium Dark Theme)
color_omega = [0.00 0.80 0.80];       % Turquoise (Angular velocity)
color_pwm   = [1.00 0.55 0.15];       % Orange (Commanded Motor PWM)
color_theta = [0.50 0.40 0.85];       % Midnight Purple (Absolute Heading)
color_error = [0.95 0.35 0.65];       % Magenta (Pointing Error)
color_bar   = [0.20 0.70 1.00];       % Cyan (Performance Metrics)

% Create Figure for 5x5 grid
fig = figure('Color', 'k', 'Name', 'CubeSat Multi-Experiment Analysis Grid', 'Position', [50, 50, 1500, 950]);

for i = 1:num_exp
    file_name = exp_files{i};
    
    % Check if file exists
    if ~exist(file_name, 'file')
        % Plot placeholder message if experiment file is missing
        for col = 1:5
            subplot(5, 5, (i-1)*5 + col);
            text(0.5, 0.5, ['Missing ' file_name], 'Color', [0.5 0.5 0.5], 'HorizontalAlignment', 'center', 'FontSize', 12);
            axis off;
            set(gca, 'Color', 'k');
        end
        continue;
    end
    
    metrics_active(i) = true;
    
    % Load variables: history_time, history_theta, history_omega, history_tau, theta_ref_deg
    load(file_name);
    
    % Ensure data arrays are present
    if ~exist('history_time', 'var') || ~exist('history_theta', 'var') || ...
       ~exist('history_omega', 'var') || ~exist('history_tau', 'var') || ~exist('theta_ref_deg', 'var')
        warning('File %s is missing required variables.', file_name);
        continue;
    end
    
    % Extract arrays
    t = history_time;
    theta = history_theta;      % in degrees
    omega = history_omega;      % in deg/s
    tau = history_tau;          % in Nm
    theta_ref = theta_ref_deg;  % in degrees
    
    % Scale control torque to Motor PWM (from -1023 to 1023)
    tau_max = 0.002;
    pwm = (tau / tau_max) * 1023;
    pwm = max(min(pwm, 1023), -1023);
    
    % Calculate pointing error in degrees (shortest path)
    error_deg = zeros(length(theta), 1);
    for k = 1:length(theta)
        error_deg(k) = rad2deg(wrapToPi(deg2rad(theta(k) - theta_ref)));
    end
    
    % --- PERFORMANCE METRICS CALCULATIONS ---
    theta0 = theta(1);
    step_size = abs(theta_ref - theta0);
    
    % 1) Steady-State Error: average error over final 10% of time
    num_samples = length(t);
    last_samples = round(0.1 * num_samples);
    ss_error = mean(abs(error_deg(end-last_samples:end)));
    metrics_steady_state_error(i) = ss_error;
    
    % 2) Overshoot
    step_dir = sign(theta_ref - theta0);
    peak_val = max(step_dir * theta);
    overshoot_deg = max(0, peak_val - step_dir * theta_ref);
    metrics_overshoot_deg(i) = overshoot_deg;
    if step_size > 0
        metrics_overshoot_pct(i) = (overshoot_deg / step_size) * 100;
    else
        metrics_overshoot_pct(i) = 0;
    end
    
    % 3) Settling Time: time to enter and stay within 2% band of step size (min 0.5 degrees)
    settling_band = max(0.02 * step_size, 0.5);
    idx_outside = find(abs(error_deg) > settling_band, 1, 'last');
    if isempty(idx_outside)
        s_time = 0;
    else
        s_time = t(idx_outside);
    end
    metrics_settling_time(i) = s_time;
    
    % --- PLOTTING 5 SUBPLOTS FOR EXPERIMENT i (ROW i) ---
    
    % 1. Spacecraft Angular Velocity (Col 1)
    ax = subplot(5, 5, (i-1)*5 + 1);
    plot(t, omega, 'Color', color_omega, 'LineWidth', 1.8);
    grid on;
    ylabel(['Exp ' num2str(i) ' [\omega_z]'], 'Color', 'w', 'FontWeight', 'bold');
    if i == 1, title('Body Rate \omega_z [deg/s]', 'Color', 'w', 'FontSize', 10); end
    if i == 5, xlabel('Time [s]', 'Color', 'w'); end
    
    % 2. Commanded Motor PWM (Col 2)
    subplot(5, 5, (i-1)*5 + 2);
    plot(t, pwm, 'Color', color_pwm, 'LineWidth', 1.8);
    grid on;
    ylim([-1100 1100]);
    if i == 1, title('Commanded PWM [Units]', 'Color', 'w', 'FontSize', 10); end
    if i == 5, xlabel('Time [s]', 'Color', 'w'); end
    
    % 3. Absolute Heading (Col 3)
    subplot(5, 5, (i-1)*5 + 3);
    plot(t, theta, 'Color', color_theta, 'LineWidth', 1.8); hold on;
    yline(theta_ref, 'w:', 'LineWidth', 1.2);
    grid on;
    if i == 1, title('Absolute Heading \theta_z [deg]', 'Color', 'w', 'FontSize', 10); end
    if i == 5, xlabel('Time [s]', 'Color', 'w'); end
    
    % 4. Pointing Error (Col 4)
    subplot(5, 5, (i-1)*5 + 4);
    plot(t, error_deg, 'Color', color_error, 'LineWidth', 1.8);
    grid on;
    if i == 1, title('Pointing Error [deg]', 'Color', 'w', 'FontSize', 10); end
    if i == 5, xlabel('Time [s]', 'Color', 'w'); end
    
    % 5. Performance Metrics Bar Chart (Col 5)
    subplot(5, 5, (i-1)*5 + 5);
    y_values = [s_time, overshoot_deg, ss_error];
    b = bar(y_values, 'FaceColor', color_bar, 'EdgeColor', 'none', 'BarWidth', 0.5);
    set(gca, 'XTickLabel', {'T_s [s]', 'Overshoot [°]', 'SS Err [°]'});
    grid on;
    if i == 1, title('Metrics Summary', 'Color', 'w', 'FontSize', 10); end
    
    % Set text values above bars for clarity
    for col_bar_idx = 1:length(y_values)
        text(col_bar_idx, y_values(col_bar_idx) + (max(y_values)*0.05), ...
             sprintf('%.2f', y_values(col_bar_idx)), ...
             'Color', 'w', 'HorizontalAlignment', 'center', 'FontSize', 8);
    end
    
    % Style configuration for modern axes look (Dark theme)
    for col = 1:5
        ax_curr = subplot(5, 5, (i-1)*5 + col);
        set(ax_curr, 'Color', 'k', ...
                     'XColor', 'w', ...
                     'YColor', 'w', ...
                     'GridColor', [0.5 0.5 0.5], ...
                     'GridAlpha', 0.25, ...
                     'FontSize', 9);
    end
end

% Save high-resolution comparative grid plot
saveas(fig, 'Multi_Experiment_Comparison.png');
fprintf('[+] Comparative grid plot successfully saved to "Multi_Experiment_Comparison.png".\n\n');

% --- DISPLAY STRUCTURED PERFORMANCE TABLE ---
fprintf('=========================================================================================\n');
fprintf('                             MULTI-EXPERIMENT PERFORMANCE ANALYSIS TABLE                 \n');
fprintf('=========================================================================================\n');
fprintf('  Experiment    |  Settling Time (Ts) |  Overshoot (deg)  |  Overshoot (%%)  |  SS Error (deg)\n');
fprintf('-----------------------------------------------------------------------------------------\n');
for i = 1:num_exp
    if metrics_active(i)
        fprintf('  Experiment %d  |      %6.2f s        |      %6.2f°       |     %6.1f%%      |     %6.4f°\n', ...
                i, metrics_settling_time(i), metrics_overshoot_deg(i), metrics_overshoot_pct(i), metrics_steady_state_error(i));
    else
        fprintf('  Experiment %d  |      [NO DATA]      |      [NO DATA]    |     [NO DATA]    |     [NO DATA]\n', i);
    end
end
fprintf('=========================================================================================\n\n');

% Save metrics to a file for table generation in Word/Excel
save('experiments_performance_summary.mat', 'metrics_settling_time', 'metrics_overshoot_deg', 'metrics_overshoot_pct', 'metrics_steady_state_error');
fprintf('[+] Metrics summary exported to "experiments_performance_summary.mat".\n');
