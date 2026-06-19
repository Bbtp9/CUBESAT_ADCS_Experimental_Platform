%% plot_compare_pointing_runs.m

clear; clc; close all;

archive_folder = "Pointing_Archive";
files = dir(fullfile(archive_folder, "pointing_run_*.mat"));

if isempty(files)
    error('No pointing experiment files found.');
end

figure('Color', 'w', 'Name', 'Pointing Comparison');

for i = 1:length(files)
    load(fullfile(files(i).folder, files(i).name), 'experiment_data');

    subplot(length(files), 3, 3*i-2)
    plot(experiment_data.Time_s, experiment_data.Theta_deg, 'LineWidth', 1.3)
    grid on
    ylabel(['Run ' num2str(i)])
    title('\theta_z [deg]')

    subplot(length(files), 3, 3*i-1)
    plot(experiment_data.Time_s, experiment_data.Omega_deg_s, 'LineWidth', 1.3)
    grid on
    title('\omega_z [deg/s]')

    subplot(length(files), 3, 3*i)
    plot(experiment_data.Time_s, experiment_data.Command, 'LineWidth', 1.3)
    grid on
    title('Command')
end

sgtitle('Pointing Experimental Runs Comparison')