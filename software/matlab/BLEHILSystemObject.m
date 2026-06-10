classdef BLEHILSystemObject < matlab.System & matlab.system.mixin.Propagates
    % BLEHILSystemObject custom block for wireless ESP32 HIL communication over BLE.
    % Connects to 'CubeSat_ESP32', writes torque commands, reads MPU6050 data,
    % integrates Gyro Z, and feeds attitude states to Simulink. Exposes 7 outputs
    % to remain fully plug-and-play compatible with SerialHILSystemObject.
    
    %#codegen
    properties (Nontunable)
        DeviceName = 'CubeSat_ESP32'
        ServiceUUID = '12345678-1234-1234-1234-1234567890AB'
        CharacteristicUUID = '87654321-4321-4321-4321-BA0987654321'
    end
    
    properties (Access = private)
        BleDevice = []
        BleChar = []
        LastTime = []
        ThetaFused = 0
        LastFusedHdg = 0
        LastGz = 0
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            % Initialize BLE communication
            disp(' ');
            disp('==================================================');
            disp('   CONNECTING TO CUBESAT ESP32 OVER BLE...        ');
            disp('   KEEP CUBESAT COMPLETELY STILL!                ');
            disp('==================================================');
            try
                % Scan for BLE devices
                devices = blelist;
                idx = find(strcmp(devices.Name, obj.DeviceName), 1);
                if isempty(idx)
                    error('CubeSat_ESP32 BLE device not found! Make sure the ESP32 is powered on and advertising.');
                end
                
                deviceAddress = devices.Address(idx);
                fprintf('[+] Found %s at Address: %s\n', obj.DeviceName, deviceAddress);
                
                % Connect to device and characteristic
                obj.BleDevice = ble(deviceAddress);
                obj.BleChar = characteristic(obj.BleDevice, obj.ServiceUUID, obj.CharacteristicUUID);
                disp('[+] BLE Connection established successfully!');
                
                % Initialize states
                obj.LastTime = tic;
                obj.ThetaFused = 0; % Start attitude angle at 0 degrees
                obj.LastFusedHdg = 0;
                obj.LastGz = 0;
                
                disp('[+] Wireless BLE HIL Interface Ready!');
                disp('--------------------------------------------------');
            catch ME
                error('[-] BLE Connection failed: %s', ME.message);
            end
        end
        
        function [fused_hdg_deg, raw_hdg_deg, gz_deg_s, mx, my, mz, tau_mNm] = stepImpl(obj, tau_cmd)
            % 1. Send the current torque command to the ESP32 via BLE
            if ~isempty(obj.BleChar)
                cmd_str = sprintf("CMD_TAU:%.6f", tau_cmd);
                try
                    write(obj.BleChar, uint8(char(cmd_str)), "WithResponse");
                    
                    % 2. Read back the updated IMU sensor data
                    raw = read(obj.BleChar);
                    data = str2double(split(string(char(raw)), ","));
                    
                    if numel(data) == 7 && all(~isnan(data))
                        gz = data(6); % Gyro Z rate in rad/s
                        obj.LastGz = rad2deg(gz);
                        
                        % Compute dt
                        dt = toc(obj.LastTime);
                        obj.LastTime = tic;
                        if dt <= 0 || dt > 0.5
                            dt = 0.1;
                        end
                        
                        % Integrate Gyro Z to compute attitude angle
                        obj.ThetaFused = obj.ThetaFused + gz * dt;
                        obj.ThetaFused = wrapToPi(obj.ThetaFused);
                        
                        fused_hdg = rad2deg(obj.ThetaFused);
                        if fused_hdg < 0
                            fused_hdg = fused_hdg + 360;
                        end
                        obj.LastFusedHdg = fused_hdg;
                    end
                catch
                    % Maintain last states on communication glitch
                end
            end
            
            % 3. Output values (mx, my, mz, raw_hdg_deg are set to dummy 0s for compatibility)
            fused_hdg_deg = obj.LastFusedHdg;
            raw_hdg_deg = obj.LastFusedHdg; % Copy fused heading to raw for plotting compatibility
            gz_deg_s = obj.LastGz;
            mx = 0;
            my = 0;
            mz = 0;
            tau_mNm = tau_cmd * 1000;
        end
        
        function releaseImpl(obj)
            % Send stop command and disconnect BLE
            if ~isempty(obj.BleChar)
                try
                    write(obj.BleChar, uint8("CMD_TAU:0.0"), "WithResponse");
                    disp('[+] Sent stop command (0.0 torque) to CubeSat.');
                catch
                end
            end
            if ~isempty(obj.BleDevice)
                clear obj.BleDevice;
                disp('[+] BLE wireless connection successfully closed.');
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
            sts = obj.createSampleTime('Type', 'Discrete', 'SampleTime', 0.1);
        end
    end
end
