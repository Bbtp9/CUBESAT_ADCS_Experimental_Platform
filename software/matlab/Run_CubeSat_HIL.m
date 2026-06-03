%% Run_CubeSat_HIL.m
% =========================================================================
%   CUBESAT REAL-TIME ATTITUDE CONTROL over Bluetooth/Serial (HIL - Hardware in the Loop)
% =========================================================================
%   - Automatically loads and opens your Simulink model: Cubesat_Control_PD.slx
%   - Prompts for serial port connection (supports USB and Bluetooth Classic)
%   - Fuses Gyro Z and Magnetometer using a complementary filter for drift-free heading
%   - Runs a 10 Hz State Machine:
%       1. DETUMBLING: Damps rotation until angular speed < omega_th_low
%       2. WAITING: Holds motor and displays the stabilized heading, waiting for pointing angle
%       3. POINTING: Drives the satellite to the target heading using PD control laws
%   - Premium dark-theme 2x2 dashboard displaying all telemetry in real-time
%   - Interactive non-blocking GUI controls to change angles and stop safely
% =========================================================================

function Run_CubeSat_HIL()
    clearvars -except fig; % Clear variables, keep figure reference if re-running
    clc;
    close all;

    disp('==================================================');
    disp('   CUBESAT REAL-TIME HIL CONTROL & DASHBOARD     ');
    disp('==================================================');

    %% 1. Set Workspace Parameters & Auto-Open Simulink
    % These parameters are shared with the Cubesat_Control_PD.slx model
    J  = 0.000634;        % Spacecraft body inertia [kg*m^2]
    Jw = 4.607e-5;        % Reaction wheel inertia [kg*m^2]
    tau_max = 0.002;      % Maximum control torque [Nm]
    
    % PD Controller Gains
    Kp = 0.02;            % Proportional pointing gain
    Kd = 0.01;            % Derivative pointing gain
    Kd_detumble = 0.03;   % Detumbling gain (positive for motor command damping)
    
    % Thresholds
    omega_th_low = deg2rad(2.0);  % Detumbling completion threshold (2.0 deg/s)
    omega_th_high = deg2rad(10);  % Re-engage detumbling (huge to avoid switching back)
    
    % Export parameters to Base Workspace so Simulink can use them
    assignin('base', 'J', J);
    assignin('base', 'Jw', Jw);
    assignin('base', 'Kp', Kp);
    assignin('base', 'Kd', Kd);
    assignin('base', 'Kd_detumble', Kd_detumble);
    assignin('base', 'tau_max', tau_max);
    assignin('base', 'omega_th_low', omega_th_low);
    assignin('base', 'omega_th_high', omega_th_high);
    
    % Add directories to path and open the Simulink model
    script_dir = fileparts(mfilename('fullpath'));
    simulink_dir = fullfile(script_dir, '..', 'simulink');
    addpath(script_dir);
    addpath(simulink_dir);
    
    disp('[*] Loading and opening Simulink model (Cubesat_Control_PD.slx)...');
    try
        load_system('Cubesat_Control_PD.slx');
        open_system('Cubesat_Control_PD.slx');
        disp('[+] Simulink model successfully loaded and opened!');
    catch ME
        warning('Could not open Cubesat_Control_PD.slx automatically. Please make sure it is in the path. Error: %s', ME.message);
    end

    %% 2. Establish Bluetooth / Serial Connection
    disp(' ');
    disp('[*] Scanning for available Serial/Bluetooth ports...');
    ports = serialportlist("all");
    
    if isempty(ports)
        warning('No serial ports found! Make sure your Bluetooth SPP (HC-05) or USB is connected.');
        portName = input('>> Enter port name manually (e.g. "/dev/tty.HC-05-Port" or "COM3"): ', 's');
    else
        disp('Available ports:');
        for i = 1:length(ports)
            fprintf('  %d) %s\n', i, ports(i));
        end
        choice = input('>> Select port number (or press ENTER to type manually): ');
        if isempty(choice) || choice < 1 || choice > length(ports)
            portName = input('>> Enter port name manually: ', 's');
        else
            portName = ports(choice);
        end
    end
    
    if isempty(portName)
        error('[-] Invalid port name. Aborting.');
    end
    
    fprintf('[*] Connecting to %s at 115200 baud...\n', portName);
    try
        device = serialport(portName, 115200);
        configureTerminator(device, "LF");
        flush(device);
        disp('[+] Bluetooth/Serial connection established successfully!');
    catch ME
        error('[-] Failed to connect to port %s: %s', portName, ME.message);
    end

    %% 3. HIL State Variables (Shared with UI)
    run_loop = true;
    state = "DETUMBLING"; % Starting state
    target_heading_deg = 0.0;
    target_heading_rad = 0.0;
    
    % Fused state estimator variables
    theta_fused = []; % Initialized on first valid sample or state transition
    detumble_stable_count = 0; % Counter for stable samples to trigger completion
    
    % History vectors for live plotting
    t_hist = [];
    omega_hist = [];
    heading_hist = [];
    fused_hist = [];
    tau_hist = [];
    mx_hist = [];
    my_hist = [];
    mz_hist = [];

    %% 4. Create Premium Dark-Theme Dashboard GUI
    fig = figure('Color', 'k', ...
                 'Name', 'CubeSat HIL Attitude Control & Telemetry', ...
                 'Position', [100, 100, 1200, 800]);

    % Define subplots in 2x2 grid
    % Leave Y=[0, 100] for bottom control panel
    
    % Subplot 1: Angular Velocity (Gyro Z)
    ax1 = subplot(2, 2, 1);
    h_omega = animatedline('Parent', ax1, 'Color', [0.50 0.40 0.85], 'LineWidth', 2); % Midnight Purple
    h_omega_lim1 = yline(ax1, rad2deg(omega_th_low), '--r', 'LineWidth', 1.2);
    h_omega_lim2 = yline(ax1, -rad2deg(omega_th_low), '--r', 'LineWidth', 1.2);
    title('Angular Velocity (\omega_z)', 'Color', 'w', 'FontSize', 12);
    ylabel('\omega_z [deg/s]', 'Color', 'w');
    grid on;
    
    % Subplot 2: Heading (Magnetometer Hdg vs Fused Heading vs Target Reference)
    ax2 = subplot(2, 2, 2);
    h_hdg_raw = animatedline('Parent', ax2, 'Color', [0.7 0.7 0.7], 'LineStyle', ':', 'LineWidth', 1); % Grey dotted raw heading
    h_hdg_fused = animatedline('Parent', ax2, 'Color', [0.00 0.80 0.80], 'LineWidth', 2); % Turquoise Fused
    h_hdg_ref = yline(ax2, target_heading_deg, '--w', 'LineWidth', 1.5, 'Label', 'Target Heading');
    title('Heading Angle (\theta_z)', 'Color', 'w', 'FontSize', 12);
    ylabel('Heading [deg]', 'Color', 'w');
    legend('Raw Mag Hdg', 'Fused Heading', 'Target Heading', 'TextColor', 'w', 'Location', 'best', 'Color', 'none', 'EdgeColor', 'none');
    grid on;

    % Subplot 3: Motor Torque Command
    ax3 = subplot(2, 2, 3);
    h_tau = animatedline('Parent', ax3, 'Color', [1.00 0.55 0.15], 'LineWidth', 2); % Orange
    h_tau_sat_p = yline(ax3, tau_max * 1000, '--r', 'LineWidth', 1.2);
    h_tau_sat_n = yline(ax3, -tau_max * 1000, '--r', 'LineWidth', 1.2);
    title('Applied Reaction Wheel Torque (\tau)', 'Color', 'w', 'FontSize', 12);
    ylabel('\tau [mN m]', 'Color', 'w');
    xlabel('Time [s]', 'Color', 'w');
    grid on;

    % Subplot 4: Raw Magnetometer Components (Mx, My, Mz)
    ax4 = subplot(2, 2, 4);
    h_mx = animatedline('Parent', ax4, 'Color', [0.95 0.35 0.35], 'LineWidth', 1.5); % Light Red
    h_my = animatedline('Parent', ax4, 'Color', [0.55 0.85 0.60], 'LineWidth', 1.5); % Pastel Green
    h_mz = animatedline('Parent', ax4, 'Color', [0.20 0.60 0.90], 'LineWidth', 1.5); % Sky Blue
    title('Raw Magnetometer Readings (QMC5883P)', 'Color', 'w', 'FontSize', 12);
    ylabel('Field Strength [LSB]', 'Color', 'w');
    xlabel('Time [s]', 'Color', 'w');
    legend('Mx', 'My', 'Mz', 'TextColor', 'w', 'Location', 'best', 'Color', 'none', 'EdgeColor', 'none');
    grid on;

    % Apply premium styling to all axes
    all_axes = [ax1, ax2, ax3, ax4];
    for ax = all_axes
        set(ax, 'Color', 'k', ...
                'XColor', 'w', ...
                'YColor', 'w', ...
                'GridColor', [0.5 0.5 0.5], ...
                'GridAlpha', 0.25, ...
                'LineWidth', 1.2, ...
                'FontSize', 10);
    end

    % --- Interactive GUI controls at the bottom ---
    % Main Panel Background
    uicontrol('Style', 'text', 'Position', [0, 0, 1200, 70], 'BackgroundColor', [0.1, 0.1, 0.1], 'String', '');
    
    % Status Label
    hStateLabel = uicontrol('Style', 'text', ...
                            'String', 'STATE: DETUMBLING', ...
                            'ForegroundColor', [0.95 0.35 0.35], ...
                            'BackgroundColor', [0.1 0.1 0.1], ...
                            'FontSize', 12, ...
                            'FontWeight', 'bold', ...
                            'HorizontalAlignment', 'left', ...
                            'Position', [50, 20, 400, 30]);
                        
    % Input for Target Angle
    uicontrol('Style', 'text', ...
              'String', 'Target Pointing Angle [deg]:', ...
              'ForegroundColor', 'w', ...
              'BackgroundColor', [0.1 0.1 0.1], ...
              'FontSize', 11, ...
              'HorizontalAlignment', 'right', ...
              'Position', [500, 22, 220, 25]);
          
    hEditAngle = uicontrol('Style', 'edit', ...
                           'String', '0', ...
                           'BackgroundColor', 'w', ...
                           'FontSize', 11, ...
                           'Position', [730, 22, 60, 25]);
                       
    hBtnAngle = uicontrol('Style', 'pushbutton', ...
                          'String', 'Set Heading', ...
                          'FontSize', 10, ...
                          'FontWeight', 'bold', ...
                          'Position', [800, 21, 110, 27], ...
                          'Callback', @updateAngleCallback);
                      
    % Stop Button
    uicontrol('Style', 'pushbutton', ...
              'String', 'STOP & DISCONNECT', ...
              'FontSize', 10, ...
              'FontWeight', 'bold', ...
              'ForegroundColor', 'w', ...
              'BackgroundColor', [0.8 0.1 0.1], ...
              'Position', [980, 21, 170, 27], ...
              'Callback', @stopCallback);

    %% 5. Nested Callbacks
    function updateAngleCallback(~, ~)
        val = str2double(hEditAngle.String);
        if ~isnan(val)
            target_heading_deg = val;
            target_heading_rad = deg2rad(val);
            % Update the target indicator in the plot
            h_hdg_ref.Value = target_heading_deg;
            
            if state == "WAITING"
                state = "POINTING";
                hStateLabel.String = sprintf('STATE: POINTING (Target: %.1f°)', target_heading_deg);
                hStateLabel.ForegroundColor = [0.00 0.80 0.80]; % Turquoise
            end
            disp(['[+] Updated pointing reference to: ', num2str(val), ' deg']);
        else
            errordlg('Please enter a valid scalar angle (in degrees).', 'Invalid Reference');
        end
    end

    function stopCallback(~, ~)
        run_loop = false;
    end

    %% 5.5 Gyroscope Calibration Phase
    disp(' ');
    disp('[*] Calibrating Gyroscope bias... PLEASE KEEP THE CUBESAT STILL!');
    hStateLabel.String = 'STATE: CALIBRATING GYRO... DO NOT MOVE';
    hStateLabel.ForegroundColor = [1.00 0.55 0.15]; % Orange
    drawnow;
    
    gyro_samples = [];
    calibration_start = tic;
    % Read for 2 seconds (around 20 samples at 10 Hz)
    while length(gyro_samples) < 20
        if toc(calibration_start) > 5.0 % 5-second timeout safeguard
            break;
        end
        if device.NumBytesAvailable > 0
            try
                line = readline(device);
                line = strtrim(line);
                if startsWith(line, "Mx")
                    continue;
                end
                data = str2double(split(line, " "));
                if numel(data) == 11 && all(~isnan(data))
                    gyro_samples = [gyro_samples, data(6)]; % data(6) is raw gz
                end
            catch
            end
        end
        pause(0.01);
    end
    
    if isempty(gyro_samples)
        gz_bias = 0.0;
        warning('[!] Gyro calibration timed out. Using 0.0 bias offset.');
    else
        gz_bias = mean(gyro_samples);
        fprintf('[+] Gyro Calibration Complete! Gz Bias Offset: %.5f rad/s (%.3f deg/s)\n', gz_bias, rad2deg(gz_bias));
    end
    
    % Update label to start detumbling
    hStateLabel.String = 'STATE: DETUMBLING';
    hStateLabel.ForegroundColor = [0.95 0.35 0.35]; % Red
    drawnow;

    %% 6. Real-Time HIL Control Loop
    t0 = tic;
    last_loop_time = toc(t0);
    tau_sat = 0; % Initial torque sent to motor
    
    disp(' ');
    disp('[*] Starting HIL Control Loop. Listening to telemetry...');
    disp('--------------------------------------------------');
    
    while run_loop
        % Stop loop if user closed the figure window
        if ~ishandle(fig)
            disp('[-] Figure closed by user. Exiting loop.');
            break;
        end
        
        % Check if serial data is available
        if device.NumBytesAvailable > 0
            try
                line = readline(device);
                line = strtrim(line);
                
                % Skip header line if present
                if startsWith(line, "Mx")
                    continue;
                end
                
                % Parse space-separated data
                data = str2double(split(line, " "));
            catch ME
                % Catch serial read glitch
                continue;
            end
            
            % Confirm packet integrity (11 values)
            if numel(data) == 11 && all(~isnan(data))
                % 6.1 Parse Telemetry variables
                mx = data(1);
                my = data(2);
                mz = data(3);
                
                gx = data(4);
                gy = data(5);
                gz = data(6) - gz_bias;  % Gyroscope Z rate [rad/s] (corrected for bias offset)
                
                ax = data(7);
                ay = data(8);
                az = data(9);
                
                temp = data(10);
                hdg_raw = data(11); % Raw magnetometer heading [deg] (0 to 360)
                
                % 6.2 Timing calculation
                curr_t = toc(t0);
                dt = curr_t - last_loop_time;
                last_loop_time = curr_t;
                
                % Safeguard dt anomalies (first loop or delay spikes)
                if dt <= 0 || dt > 0.5
                    dt = 0.1;
                end
                
                % Map Raw Heading to [-pi, pi]
                mag_theta_rad = wrapToPi(deg2rad(hdg_raw));
                
                % Initialize fused heading on first valid data point
                if isempty(theta_fused)
                    theta_fused = mag_theta_rad;
                end
                
                % Complementary Filter: Fuse high-frequency gyro integration with low-frequency absolute magnetometer heading
                theta_fused = 0.98 * (theta_fused + gz * dt) + 0.02 * mag_theta_rad;
                theta_fused = wrapToPi(theta_fused);
                
                fused_heading_deg = rad2deg(theta_fused);
                if fused_heading_deg < 0
                    fused_heading_deg = fused_heading_deg + 360.0; % Remap back to 0-360 for UI/Plotting consistency
                end

                % 6.3 Controller State Machine
                switch state
                    case "DETUMBLING"
                        % Pure damping control law: tau_motor = Kd_detumble * omega
                        % Note: Motor torque matches the signs of the Cubesat_Control_PD controller
                        tau_cmd = Kd_detumble * gz;
                        tau_sat = max(min(tau_cmd, tau_max), -tau_max);
                        
                        % Check if detumbled: abs(omega) must stay below threshold
                        if abs(gz) < omega_th_low
                            detumble_stable_count = detumble_stable_count + 1;
                        else
                            detumble_stable_count = 0; % reset if it spikes
                        end
                        
                        % Stabilized for 10 consecutive loops (~1 second at 10Hz)
                        if detumble_stable_count >= 10
                            tau_sat = 0; % disable motor torque
                            % Send zero torque to the board immediately
                            try
                                write(device, uint8(char("CMD_TAU:0.0\n")), "uint8");
                            catch
                            end
                            
                            stopped_heading = hdg_raw;
                            fprintf('\n[+] DETUMBLING COMPLETE! CubeSat stabilized.\n');
                            fprintf('    Stopped Magnetic Heading: %.2f°\n', stopped_heading);
                            
                            % Update UI status to alert user of popup
                            hStateLabel.String = sprintf('STATE: ENTER POINTING ANGLE (Stopped at %.1f°)', stopped_heading);
                            hStateLabel.ForegroundColor = [1.00 0.70 0.00]; % Orange
                            drawnow;
                            
                            % Prompt the user with a dialog box (blocks loop briefly)
                            prompt = {sprintf('Detumbling complete!\nStopped at Heading: %.1f°\n\nEnter target pointing angle [deg]:', stopped_heading)};
                            dlgtitle = 'Target Pointing Angle';
                            dims = [1 50];
                            definput = {num2str(round(stopped_heading))};
                            answer = inputdlg(prompt, dlgtitle, dims, definput);
                            
                            if isempty(answer) || isnan(str2double(answer{1}))
                                disp('[!] No valid input received. Defaulting to 0 deg.');
                                target_heading_deg = 0.0;
                            else
                                target_heading_deg = str2double(answer{1});
                            end
                            
                            target_heading_rad = deg2rad(target_heading_deg);
                            h_hdg_ref.Value = target_heading_deg;
                            
                            % Initialize fused heading to the stopped heading to prevent jumps
                            theta_fused = mag_theta_rad;
                            
                            % Transition directly to POINTING state (no way to go back to detumble)
                            state = "POINTING";
                            hStateLabel.String = sprintf('STATE: POINTING (Target: %.1f°)', target_heading_deg);
                            hStateLabel.ForegroundColor = [0.00 0.80 0.80]; % Turquoise
                            
                            % Flush accumulated serial data during dialog wait
                            flush(device);
                            last_loop_time = toc(t0);
                        end
                        
                    case "WAITING"
                        % Hold state, no active torque command
                        tau_sat = 0;
                        
                    case "POINTING"
                        % Pointing PD Control: tau_motor = -Kp * error + Kd * omega
                        % error = current_angle - target_angle
                        theta_err = wrapToPi(theta_fused - target_heading_rad);
                        
                        tau_cmd = -Kp * theta_err + Kd * gz;
                        tau_sat = max(min(tau_cmd, tau_max), -tau_max);
                        
                        % Print pointing telemetry to console
                        fprintf("t=%6.2fs | Fused Hdg=%6.2f° | Error=%6.2f° | Torque=%7.4f mNm\n", ...
                                curr_t, fused_heading_deg, rad2deg(theta_err), tau_sat * 1000);
                end
                
                % 6.4 Write control command back to Arduino
                cmd_str = sprintf("CMD_TAU:%.6f\n", tau_sat);
                write(device, uint8(char(cmd_str)), "uint8");
                
                % 6.5 Update GUI real-time plots
                addpoints(h_omega, curr_t, rad2deg(gz));
                addpoints(h_hdg_raw, curr_t, hdg_raw);
                addpoints(h_hdg_fused, curr_t, fused_heading_deg);
                addpoints(h_tau, curr_t, tau_sat * 1000); % Plotted in mNm
                addpoints(h_mx, curr_t, mx);
                addpoints(h_my, curr_t, my);
                addpoints(h_mz, curr_t, mz);
                
                % Dynamic X-axis sliding window (show last 30 seconds of data)
                xlim(ax1, [max(0, curr_t - 30), max(30, curr_t)]);
                xlim(ax2, [max(0, curr_t - 30), max(30, curr_t)]);
                xlim(ax3, [max(0, curr_t - 30), max(30, curr_t)]);
                xlim(ax4, [max(0, curr_t - 30), max(30, curr_t)]);
                
                drawnow limitrate;
            end
        end
        
        pause(0.01); % Allow MATLAB context switching
    end

    %% 7. Post-HIL Cleanup
    disp('--------------------------------------------------');
    disp('[*] Disengaging HIL control loop...');
    
    % Send a final command to stop the motor
    try
        write(device, uint8(char("CMD_TAU:0.0\n")), "uint8");
        disp('[+] Sent stop command (0.0 torque) to CubeSat.');
    catch
        warning('Could not send final zero torque command (serial port disconnected).');
    end
    
    % Close serial connection
    disp('[*] Closing serial connection...');
    clear device;
    disp('[+] Disconnected successfully.');
    disp('==================================================');
end
