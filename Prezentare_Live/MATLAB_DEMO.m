% DETUMBLE & POINTING DEMO



clear; clc; close all;

disp('==================================================');
disp('   CUBESAT REAL-TIME CONTROL & TELEMETRY VIEWER   ');
disp('==================================================');

%% 1. Dynamic BLE Connection Establishment
disp('[*] Scanning for advertising BLE devices...');
try
    devices = blelist;
catch ME
    error('BLE is not supported or Bluetooth is turned off on this machine. Error: %s', ME.message);
end

% Search for the ESP32_IMU device name
idx = find(strcmp(devices.Name, "ESP32_IMU"), 1);
if isempty(idx)
    error('[-] ESP32_IMU device not found! Make sure the ESP32 is powered on and advertising.');
end

deviceAddress = devices.Address(idx);
fprintf('[+] Found ESP32_IMU! Address/UUID: %s\n', deviceAddress);
disp('[*] Connecting to ESP32_IMU BLE Service...');

serviceUUID   = "12345678-1234-1234-1234-123456789abc";
telemetryUUID = "87654321-4321-4321-4321-cba987654321";
motorUUID     = "87654321-4321-4321-4321-cba987654326";

try
    b = ble(deviceAddress);
    c = characteristic(b, serviceUUID, telemetryUUID);
    m = characteristic(b, serviceUUID, motorUUID);
    disp('[+] Connection established successfully!');
catch ME
    error('[-] Failed to connect to BLE device: %s', ME.message);
end

%% 2. Controller Parameters
ctrl_name = 'Analytical PID';

% Satellite Physical Constants
J  = 0.000634;     % Spacecraft body inertia [kg*m^2]
Jw = 4.607e-5;     % Reaction wheel inertia [kg*m^2]
tau_max = 1024;    % Maximum control torque [Nm]
gz_offset = 0.012;
tau_delay = 2.3;   % measured time delay [s]
csi = 0.25;

Kp = -142;
Ki = -117;
Kd = 47;
N  = 0.73;

AKp = 0.12;
AKi = 0.03;
AKd = 5;
AN  = 100;

% seq=[0 20 50  80  110 300;
%      0 1 90 250 180 180];

seq=[0 30 70  110  150 300;
     0 45 90  250  180 180];


fprintf('Analytical PID Gains (for delay tau = %.2f s):\n', tau_delay);
fprintf('  Kp = %.6f, Ki = %.6f, Kd = %.6f\n', Kp, Ki, Kd);
fprintf('\n[+] Selected Pointing Mode: %s\n', ctrl_name);
fprintf('    PID Gains: Kp = %.6f, Ki = %.6f, Kd = %.6f\n', Kp, Ki, Kd);
disp('Press ENTER to start Phase 1 (Detumbling)...');
input('');

%% 3. Setup Dark-Theme Live Visualizer
fig = figure('Color', 'k', 'Name', ['Real-Time CubeSat Mission - ' ctrl_name], 'Position', [100, 100, 1000, 700]);

% Subplot 1: Spacecraft State (Velocity in Phase 1, Angle in Phase 2)
ax1 = subplot(2, 2, 1);
h_theta = animatedline('Color', [0.00 0.80 0.80], 'LineWidth', 2.5); % Turquoise
h_ref   = yline(0, 'w:', 'LineWidth', 1.5);                          % Reference line
set(h_ref, 'Visible', 'off');
grid on;
ylabel('Angular Velocity [deg/s]', 'Color', 'w');
title('Real-Time Spacecraft Angular Velocity', 'Color', 'w', 'FontSize', 12);
legend('Body Rate (\omega_z)', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

% Subplot 2: Estimated Reaction Wheel Velocity
ax2 = subplot(2, 2, 2);
h_omega_w = animatedline('Color', [1.00 0.55 0.15], 'LineWidth', 2.0);  % Orange
grid on;
ylabel('Wheel Speed \omega_w [rad/s]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Real-Time Estimated Reaction Wheel Velocity (\omega_w)', 'Color', 'w', 'FontSize', 12);
legend('Reaction Wheel Speed', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

ax3 = subplot(2, 2, 3);
h_cmd = animatedline('Color', [0 1 0], 'LineWidth', 2.0);  % Orange
grid on;
ylabel('Command [PWM]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Real-Time Command ', 'Color', 'w', 'FontSize', 12);
legend('Command', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');

% Unghi
ax4 = subplot(2, 2, 4);
h_unghi = animatedline('Color', [1 1 1], 'LineWidth', 2.0);  % Orange
h_ref = animatedline('Color', [1 1 0.5], 'LineWidth', 2.0);
% h_phi_cmd = animatedline('Color', [0 1 0], ...
                         % 'LineStyle', '--', ...
                         % 'LineWidth', 2.0);
grid on;
ylabel('Angle [Deg]', 'Color', 'w');
xlabel('Time [s]', 'Color', 'w');
title('Pointing Angle ', 'Color', 'w', 'FontSize', 12);
% legend('Pointing Angle', 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'none');
legend('Pointing Angle', ...
       'Commanded Angle', ...
       'TextColor', 'w', ...
       'FontSize', 12, ...
       'Location', 'northeast', ...
       'Color', 'none', ...
       'EdgeColor', 'none');
% legend('Measured Pointing Angle', 'Commanded Pointing Angle', ...
%         'TextColor', 'w', 'Location', 'northeast', ...
%         'Color', [1 1 0.5], 'EdgeColor', 'none');

% schimb legenda

% Apply dark styling to axes
for ax = [ax1, ax2, ax3, ax4]
    set(ax, 'Color', 'k', ...
            'XColor', 'w', ...
            'YColor', 'w', ...
            'GridColor', [0.7 0.7 0.7], ...
            'GridAlpha', 1, ...
            'LineWidth', 1, ...
            'FontSize', 11);
end

%% 4. Real-Time Acquisition & Mission Control Loop
% thetaZ = 0;             % Integrated attitude angle [rad]
phiZ=0;
memphiZ = 0;
omega_w_est = 0;        % Estimated reaction wheel speed [rad/s]
lastT = tic;            % Timer for dt calculation
t0 = tic;               % Global timer
sampling_rate = 10;     % Loop frequency (approx 10 Hz)
pause_time = 1 / sampling_rate;

% History arrays for Workspace Export
history_time  = [];
% history_theta = [];
history_omega = [];
history_tau   = [];
history_phi   = [];

% ==================================================
%   PHASE 1: DETUMBLING (RATE DAMPING)
% ==================================================
disp('[*] Starting Phase 1: Detumbling (Damping rates)...');
disp('--------------------------------------------------');

stable_count = 0;
stable_threshold = deg2rad(0.5); % Stable if below 0.5 deg/s
stable_required = 10;            % 10 samples (1 second at 10 Hz)

max_detumble_iterations = 800;    % Max 160 seconds

tau_sat = 0;
gz = 0;
tau_cmd = 0;

% Filter initialization
e = 0;
xF = 0;
xI = 0;

Ae = 0;
AxI =  0;

cmd_speed = 0;

t=0; 
for k = 1:max_detumble_iterations

    ii=find(seq(1,:)<=t);
    ii=ii(end);
    if isempty(ii)
        ii=1;
    end
    phi_cmd=seq(2,ii);

    if ~ishandle(fig)
        disp('[-] Figure closed by user. Terminating loop.');
        break;
    end
    
    % Send torque command and read telemetry
    pwm_cmd = tau_sat;
    if pwm_cmd > 0
        pwm_cmd = pwm_cmd + 210;
    else
        pwm_cmd = pwm_cmd - 210;
    end
    pwm_cmd = max(min(pwm_cmd, 1023), -1023);

    cmd_str = sprintf("%i", floor(pwm_cmd));
    disp([num2str([gz tau_cmd tau_sat]) ' ' cmd_str])
    try
        if k > 20
            write(m, char(cmd_str)); 
        else
            xF = 0;
            xI = 0;
            pwm_cmd = 0;
        end 
        pause(0.1);
        raw = read(c);
        data = sscanf(char(raw), '%f');
        pause(0.05);
    catch ME
        warning('BLE communication dropped a package. Retrying... Error: %s', ME.message);
        continue;
    end
    
    dt = toc(lastT);
    t=t+dt;
    lastT = tic;

    if numel(data) == 11 && all(~isnan(data))
        gz = data(6) - gz_offset;  % Gyroscope Z-axis [rad/s]
        phiM=data(11);             % grade
       
        phiZ = phiZ + csi * 180 / pi * wrapToPi((phiM - phiZ)/180 * pi);
        if phiZ < 0
            phiZ = phiZ + 360;
        elseif phiZ > 360
            phiZ = phiZ - 360;
        end 
       %  phiZ = phiM;
        % Integrate angle and estimate reaction wheel speed
        % thetaZ = thetaZ + gz * dt;
        % thetaDeg = rad2deg(thetaZ);
        omega_w_est = omega_w_est + (tau_sat / Jw - tau_delay * omega_w_est) * dt;
        t = toc(t0);
    
        % angle controller 
        if phi_cmd~=0
            % Ae=phi_cmd-phiZ;
            Ae = wrapToPi((phi_cmd - phiZ) / 180 * pi) * 180 / pi; % error
            AxI = AxI + dt * Ae;
            AxI = min(max(AxI, -90),90);
            disp(AxI)
            cmd_speed = AKp * Ae + AKi * AxI + AKd * gz;
        end

        % Damping controller: u = -Kd * omega
        e = gz - cmd_speed *pi / 180;   % error
        xF = xF + N * (e - xF);
        xI = xI + dt * e;
        D = Kd * N * (e - xF);

        tau_cmd = Kp * e + Ki * xI + D; %+kdd*dgz;

 

     
        % Motorul este invers
        tau_sat = -tau_cmd;  
        
        % Log history
        history_time  = [history_time; t];
        % history_theta = [history_theta; thetaDeg];
        history_omega = [history_omega; rad2deg(gz)];
        history_tau   = [history_tau; tau_sat];
        history_phi   = [history_phi; phiZ];
        
        % Update Plots
        addpoints(h_theta, t, rad2deg(gz));
        addpoints(h_omega_w, t, omega_w_est);
        addpoints(h_cmd, t, pwm_cmd);

        if abs(phiZ - memphiZ) < 180
           addpoints(h_unghi, t, phiZ);
           addpoints(h_ref, t, phi_cmd);
        else
            addpoints(h_unghi, t, NaN)
            addpoints(h_ref, t, phi_cmd);
        end
       memphiZ = phiZ;

      

        drawnow limitrate;
        
        fprintf("t=%5.2fs | DETUMBLE | Gz=%7.4f rad/s (%5.2f deg/s) | Theta=%6.2f deg\n", ...
                t, gz, rad2deg(gz), phiZ);
        
       
    end
end

return

