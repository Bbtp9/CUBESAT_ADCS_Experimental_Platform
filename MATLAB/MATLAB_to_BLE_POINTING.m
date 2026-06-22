%% Matlab_to_BLE_POINTING.m
% =========================================================================
%   CUBESAT REAL-TIME POINTING CONTROL over BLE
% =========================================================================

clear; clc; close all;

disp('==================================================');
disp('        CUBESAT REAL-TIME POINTING CONTROL        ');
disp('==================================================');

%% 1. BLE CONNECTION
disp('[*] Scanning for advertising BLE devices...');

try
    devices = blelist;
catch ME
    error('BLE is not supported or Bluetooth is turned off. Error: %s', ME.message);
end

idx = find(strcmp(devices.Name, "ESP32_IMU"), 1);

if isempty(idx)
    error('[-] ESP32_IMU device not found! Make sure ESP32 is powered on and advertising.');
end

deviceAddress = devices.Address(idx);
fprintf('[+] Found ESP32_IMU! Address/UUID: %s\n', deviceAddress);

serviceUUID   = "12345678-1234-1234-1234-123456789abc";
telemetryUUID = "87654321-4321-4321-4321-cba987654321";
motorUUID     = "87654321-4321-4321-4321-cba987654326";

try
    b = ble(deviceAddress);
    c = characteristic(b, serviceUUID, telemetryUUID);
    m = characteristic(b, serviceUUID, motorUUID);
    disp('[+] BLE connection established successfully!');
catch ME
    error('[-] Failed to connect to BLE device: %s', ME.message);
end

%% 2. CONTROLLER PARAMETERS
disp(' ');
disp('--------------------------------------------------');
disp('   CONTROLLER ARCHITECTURE SELECTOR');
disp('--------------------------------------------------');
disp(' 1) Analytical PID Controller');
disp(' 2) Optimal LQR Controller');

ctrl_choice = input('>> Select Controller Mode [1 or 2] default LQR: ');

if isempty(ctrl_choice) || ~ismember(ctrl_choice, [1, 2])
    ctrl_choice = 2;
end

J  = 0.000634;       % CubeSat body inertia [kg*m^2]
Jw = 4.607e-5;       % Reaction wheel inertia [kg*m^2]

sampling_rate = 10;

tau_max = 300;      % Max command saturation
tau_delay = 2.3;     % measured delay [s]
gz_offset = 0.012;   % gyro offset [rad/s]

if ctrl_choice == 1
    ctrl_name = 'Analytical PID';

    %Valori luate din PID2
    Kp = -8;
    Ki = -2;
    Kd = -16;
    N  = 17.65;

    fprintf('PID Gains:\n');
    fprintf('Kp = %.6f, Ki = %.6f, Kd = %.6f\n', Kp, Ki, Kd);

else
    ctrl_name = 'Optimal LQR';

    Q = [50, 0; 0, 5];
    R = 1;

    q1 = Q(1,1);
    q2 = Q(2,2);

    K1_lqr = -sqrt(q1/R);
    K2_lqr = J*tau_delay - sqrt(J^2*tau_delay^2 + (2*J*sqrt(q1*R) + q2)/R);

    K_ctrl = [K1_lqr, K2_lqr];

    fprintf('LQR Gains:\n');
    fprintf('K_theta = %.6f\n', -K_ctrl(1));
    fprintf('K_omega = %.6f\n', -K_ctrl(2));
end

fprintf('\n[+] Selected Controller: %s\n', ctrl_name);

%% 3. READ INITIAL HEADING
write(m, "0");
disp('[*] Motor stopped. Waiting for CubeSat to settle...');
pause(2.0);

thetaZ = 0;

try
    raw = read(c);
    data = sscanf(char(raw), '%f');

    if numel(data) == 11 && all(~isnan(data))
        settled_heading = data(11);
    else
        settled_heading = 0;
    end
catch
    settled_heading = 0;
end

fprintf('\n==================================================\n');
fprintf('   Initial CubeSat heading:\n');
fprintf('   -> Current Sensor Heading: %.2f deg\n', settled_heading);
fprintf('==================================================\n\n');

thetaZ = deg2rad(settled_heading);

%% 4. TARGET INPUT
theta_ref_deg = input('>> Enter Target Pointing Heading/Angle [deg] 0 to 360: ');

if isempty(theta_ref_deg) || ~isnumeric(theta_ref_deg)
    theta_ref_deg = 45;
end

theta_ref = deg2rad(theta_ref_deg);

%% 5. DARK-THEME LIVE PLOTS
fig = figure('Color', 'k', ...
             'Name', ['Real-Time CubeSat Pointing - ' ctrl_name], ...
             'Position', [100, 100, 1000, 700]);

ax1 = subplot(2, 1, 1);
h_theta = animatedline('Color', [0.00 0.80 0.80], 'LineWidth', 2.5);
h_ref = yline(theta_ref_deg, 'w:', 'LineWidth', 1.5);

grid on;
ylabel('Attitude Angle \theta_z [deg]', 'Color', 'w');
title('Real-Time Spacecraft Attitude Angle - Pointing', 'Color', 'w', 'FontSize', 12);
legend('Live Angle (\theta_z)', 'Target Reference', ...
       'TextColor', 'w', ...
       'Location', 'southeast', ...
       'Color', 'none', ...
       'EdgeColor', 'none');

ax2 = subplot(2, 1, 2);
h_omega_w = animatedline('Color', [1.00 0.55 0.15], 'LineWidth', 2.0);

grid on;
ylabel('Wheel Speed \omega_w [rad/s]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Real-Time Estimated Reaction Wheel Velocity', 'Color', 'w', 'FontSize', 12);
legend('Reaction Wheel Speed', ...
       'TextColor', 'w', ...
       'Location', 'northeast', ...
       'Color', 'none', ...
       'EdgeColor', 'none');

for ax = [ax1, ax2]
    set(ax, 'Color', 'k', ...
            'XColor', 'w', ...
            'YColor', 'w', ...
            'GridColor', [0.5 0.5 0.5], ...
            'GridAlpha', 0.3, ...
            'LineWidth', 1.2, ...
            'FontSize', 11);
end

%% 6. INITIALIZE REAL-TIME LOOP VARIABLES
tau_sat = 0;

lastT = tic;
t0 = tic;


history_time  = [];
history_theta = [];
history_omega = [];
history_tau   = [];

int_err = 0;

pointing_duration = 30;
num_pointing_iterations = pointing_duration * sampling_rate;

disp(' ');
disp('[*] Starting Attitude Pointing Control...');
disp('--------------------------------------------------');

e = 0;
xF = 0;
xI = 0;
tau_cmd = 0;
csi = 0.9;

%% 7. POINTING LOOP
for k = 1:num_pointing_iterations

    if ~ishandle(fig)
        disp('[-] Figure closed by user. Terminating loop.');
        break;
    end

   % Send torque command & read telemetry
    pwm_cmd = tau_sat;
    if pwm_cmd>0
        pwm_cmd=pwm_cmd+150;
    else
        pwm_cmd=pwm_cmd-150;
    end
    pwm_cmd = max(min(pwm_cmd, 1023), -1023);

    cmd_str = sprintf("%i", floor(pwm_cmd));
    disp([num2str([thetaZ tau_cmd tau_sat]) ' ' cmd_str])


    try
        write(m, uint8(char(cmd_str)));
        pause(0.1);

        raw = read(c);
        data = sscanf(char(raw), '%f');
        pause(0.05);

    catch ME
        warning('BLE communication dropped a package. Retrying... Error: %s', ME.message);
        continue;
    end

    if numel(data) == 11 && all(~isnan(data))

        gz = data(6) - gz_offset;
        dt = toc(lastT);
        lastT = tic;
        thetaM = data(11); % grade
        
        thetaZ = csi * (thetaZ + dt * gz * 180 / pi) + (1 - csi) * thetaM;

        e=wrapToPi((thetaZ-theta_ref_deg)/180 * pi) * 180 / pi; % error
     
        xF=xF+N*(e-xF);
        xI=xI+dt*e;
        %  D=Kd*N*(e-xF);
        D = Kd * rad2deg(gz);
       

        tau_cmd = Kp*e+Ki*xI+D;
        disp([xF, xI, e])
        disp([Kp * e, Ki * xI, D])

        t = toc(t0);

  

        % Motorul e invers
        tau_sat = -max(min(tau_cmd, tau_max), -tau_max);

      

        history_time  = [history_time; t];
        history_theta = [history_theta; thetaZ];
        history_omega = [history_omega; rad2deg(gz)];
        history_tau   = [history_tau; tau_sat];

        addpoints(h_theta, t, thetaZ);
        addpoints(h_omega_w, t, rad2deg(gz));
        drawnow limitrate;

        fprintf("t=%5.2fs | POINTING | Gz=%7.4f rad/s | Theta=%6.2f deg | Error=%6.2f deg | Command=%8.2f\n", ...
                t, gz, thetaZ, e, tau_sat);
    end

end

%% 8. STOP MOTOR AND DISCONNECT
disp('--------------------------------------------------');
disp('[*] Stopping motor...');

try
    write(m, "0");
catch
    warning('Could not send final motor stop command.');
end

disp('[+] Real-Time Loop Finished. Disconnecting BLE...');
clear b;

return

%% 9. EXPORT TO WORKSPACE
if ~isempty(history_time)

    disp('[*] Constructing timeseries objects for Simulink...');

    sensor_theta = timeseries(deg2rad(history_theta), history_time, 'Name', 'sensor_theta');
    sensor_omega = timeseries(deg2rad(history_omega), history_time, 'Name', 'sensor_omega');
    sensor_tau   = timeseries(history_tau, history_time, 'Name', 'sensor_tau');

    assignin('base', 'sensor_theta', sensor_theta);
    assignin('base', 'sensor_omega', sensor_omega);
    assignin('base', 'sensor_tau', sensor_tau);

    disp('[+] TELEMETRY EXPORTED TO WORKSPACE:');
    disp('    -> sensor_theta');
    disp('    -> sensor_omega');
    disp('    -> sensor_tau');

else
    disp('[-] No valid data recorded. Workspace export skipped.');
end

disp('==================================================');
disp('                POINTING COMPLETE                 ');
disp('==================================================');


%% 10. SAVE POINTING DATA TO ARCHIVE

disp('[*] Saving pointing experiment data...');

archive_folder = "Pointing_Archive";

if ~exist(archive_folder, 'dir')
    mkdir(archive_folder);
end

run_id = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
save_name = archive_folder + "/pointing_run_" + run_id;

experiment_data = table( ...
    history_time, ...
    history_theta, ...
    history_omega, ...
    history_tau, ...
    'VariableNames', {'Time_s', 'Theta_deg', 'Omega_deg_s', 'Command'} ...
);

save(save_name + ".mat", ...
     'experiment_data', ...
     'history_time', ...
     'history_theta', ...
     'history_omega', ...
     'history_tau', ...
     'ctrl_name', ...
     'ctrl_choice', ...
     'theta_ref_deg', ...
     'settled_heading', ...
     'tau_delay', ...
     'gz_offset');

writetable(experiment_data, save_name + ".csv");

fig_save = figure('Color', 'w', 'Name', 'Saved Pointing Result');

subplot(3,1,1)
plot(history_time, history_theta, 'LineWidth', 1.5)
hold on
yline(theta_ref_deg, '--', 'Target')
grid on
ylabel('\theta_z [deg]')
title('Pointing - Attitude Angle')

subplot(3,1,2)
plot(history_time, history_omega, 'LineWidth', 1.5)
grid on
ylabel('\omega_z [deg/s]')

subplot(3,1,3)
plot(history_time, history_tau, 'LineWidth', 1.5)
grid on
ylabel('Command')
xlabel('Time [s]')

saveas(fig_save, save_name + ".png");

disp('[+] Pointing data saved successfully.');
disp("    MAT file: " + save_name + ".mat");
disp("    CSV file: " + save_name + ".csv");
disp("    PNG file: " + save_name + ".png");