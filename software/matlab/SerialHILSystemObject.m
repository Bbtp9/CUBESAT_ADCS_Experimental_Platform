classdef SerialHILSystemObject < matlab.System & matlab.system.mixin.Propagates
    % SerialHILSystemObject custom block for real-time ESP32 serial communication.
    % Handles port initialization, gyro calibration, complementary filtering, and zero-order hold state buffering.
    
    %#codegen
    properties (Nontunable)
        PortName = 'COM3'
        BaudRate = 115200
    end
    
    properties (Access = private)
        SerialDevice = []
        GzBias = 0
        LastTime = []
        ThetaFused = []
        LastFusedHdg = 0
        LastRawHdg = 0
        LastGz = 0
        LastMx = 0
        LastMy = 0
        LastMz = 0
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Initialize serial port connection
            try
                obj.SerialDevice = serialport(obj.PortName, obj.BaudRate);
                configureTerminator(obj.SerialDevice, 'LF');
                flush(obj.SerialDevice);
                
                disp(' ');
                disp('==================================================');
                disp('   STARTING SIMULINK REAL-TIME GYRO CALIBRATION ');
                disp('   KEEP CUBESAT COMPLETELY STILL!                ');
                disp('==================================================');
                
                gyro_samples = [];
                calib_start = tic;
                
                % Read 20 samples to estimate bias (approx 2 seconds at 10 Hz)
                while length(gyro_samples) < 20
                    if toc(calib_start) > 5.0 % 5 second safeguard timeout
                        break;
                    end
                    if obj.SerialDevice.NumBytesAvailable > 0
                        try
                            line = readline(obj.SerialDevice);
                            line = strtrim(line);
                            if ~startsWith(line, "Mx") && ~isempty(line)
                                data = str2double(split(line, " "));
                                if numel(data) == 11 && all(~isnan(data))
                                    gyro_samples = [gyro_samples, data(6)]; % data(6) is Gz
                                end
                            end
                        catch
                        end
                    end
                    pause(0.01);
                end
                
                if isempty(gyro_samples)
                    obj.GzBias = 0;
                    warning('[!] Gyro calibration timed out. Using 0.0 bias offset.');
                else
                    obj.GzBias = mean(gyro_samples);
                    fprintf('[+] Gyro Calibration Complete! Gz Bias Offset: %.5f rad/s (%.3f deg/s)\n', ...
                        obj.GzBias, rad2deg(obj.GzBias));
                end
                
                % Initialize variables
                obj.LastTime = tic;
                obj.ThetaFused = [];
                obj.LastFusedHdg = 0;
                obj.LastRawHdg = 0;
                obj.LastGz = 0;
                obj.LastMx = 0;
                obj.LastMy = 0;
                obj.LastMz = 0;
                
                disp('[+] HIL Simulation Interface Ready!');
                disp('--------------------------------------------------');
            catch ME
                error('[-] Failed to open serial port %s: %s', obj.PortName, ME.message);
            end
        end
        
        function [fused_hdg_deg, raw_hdg_deg, gz_deg_s, mx, my, mz, tau_mNm] = stepImpl(obj, tau_cmd)
            % 1. Send the control torque command back to the ESP32
            if ~isempty(obj.SerialDevice) && obj.SerialDevice.Writable
                cmd_str = sprintf("CMD_TAU:%.6f\n", tau_cmd);
                write(obj.SerialDevice, uint8(cmd_str), "uint8");
            end
            
            % 2. Read sensor data from the ESP32
            if ~isempty(obj.SerialDevice) && obj.SerialDevice.NumBytesAvailable > 0
                try
                    line = readline(obj.SerialDevice);
                    line = strtrim(line);
                    
                    if ~startsWith(line, "Mx") && ~isempty(line)
                        data = str2double(split(line, " "));
                        
                        if numel(data) == 11 && all(~isnan(data))
                            % Parse values
                            obj.LastMx = data(1);
                            obj.LastMy = data(2);
                            obj.LastMz = data(3);
                            
                            gz = data(6) - obj.GzBias; % Corrected gyro rate [rad/s]
                            obj.LastGz = rad2deg(gz);  % Gyro rate [deg/s]
                            
                            obj.LastRawHdg = data(11); % Raw magnetometer heading [deg]
                            
                            % Compute dt (elapsed time)
                            dt = toc(obj.LastTime);
                            obj.LastTime = tic;
                            if dt <= 0 || dt > 0.5
                                dt = 0.1;
                            end
                            
                            % Fused Complementary Filter logic
                            mag_theta_rad = wrapToPi(deg2rad(obj.LastRawHdg));
                            if isempty(obj.ThetaFused)
                                obj.ThetaFused = mag_theta_rad;
                            end
                            
                            obj.ThetaFused = 0.98 * (obj.ThetaFused + gz * dt) + 0.02 * mag_theta_rad;
                            obj.ThetaFused = wrapToPi(obj.ThetaFused);
                            
                            fused_hdg = rad2deg(obj.ThetaFused);
                            if fused_hdg < 0
                                fused_hdg = fused_hdg + 360;
                            end
                            obj.LastFusedHdg = fused_hdg;
                        end
                    end
                catch
                    % Maintain previous state on parse failure
                end
            end
            
            % 3. Propagate outputs using Zero-Order Hold state buffers
            fused_hdg_deg = obj.LastFusedHdg;
            raw_hdg_deg = obj.LastRawHdg;
            gz_deg_s = obj.LastGz;
            mx = obj.LastMx;
            my = obj.LastMy;
            mz = obj.LastMz;
            tau_mNm = tau_cmd * 1000; % Convert to mNm for scaling consistency
        end
        
        function releaseImpl(obj)
            % Safely turn off reaction wheel motor and release the serial port
            if ~isempty(obj.SerialDevice)
                try
                    write(obj.SerialDevice, uint8(sprintf("CMD_TAU:0.0\n")), "uint8");
                    disp('[+] Sent stop command (0.0 torque) to CubeSat.');
                catch
                end
                clear obj.SerialDevice;
                disp('[+] Serial connection successfully closed.');
            end
        end
        
        function num = getNumInputsImpl(~)
            num = 1; % input: tau_cmd
        end
        
        function num = getNumOutputsImpl(~)
            num = 7; % outputs: fused_hdg_deg, raw_hdg_deg, gz_deg_s, mx, my, mz, tau_mNm
        end
        
        function varargout = getOutputDataTypeImpl(obj)
            varargout = cell(1, obj.getNumOutputsImpl());
            for i = 1:obj.getNumOutputsImpl()
                varargout{i} = 'double';
            end
        end
        
        function varargout = getOutputSizeImpl(obj)
            varargout = cell(1, obj.getNumOutputsImpl());
            for i = 1:obj.getNumOutputsImpl()
                varargout{i} = [1 1];
            end
        end
        
        function varargout = isOutputComplexImpl(obj)
            varargout = cell(1, obj.getNumOutputsImpl());
            for i = 1:obj.getNumOutputsImpl()
                varargout{i} = false;
            end
        end
        
        function varargout = isOutputFixedSizeImpl(obj)
            varargout = cell(1, obj.getNumOutputsImpl());
            for i = 1:obj.getNumOutputsImpl()
                varargout{i} = true;
            end
        end
        
        function sts = getSampleTimeImpl(obj)
            % Discrete 10 Hz loop execution
            sts = obj.createSampleTime('Type', 'Discrete', 'SampleTime', 0.1);
        end
    end
end
