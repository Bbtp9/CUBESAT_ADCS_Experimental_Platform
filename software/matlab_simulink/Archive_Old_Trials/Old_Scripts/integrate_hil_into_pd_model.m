% integrate_hil_into_pd_model.m
% Programmatically modifies the existing Cubesat_Control_PD.slx model to add
% real-time HIL capability with Manual Switches.

modelName = 'Cubesat_Control_PD';

% 1. Load the model
disp('[*] Loading Cubesat_Control_PD.slx...');
load_system(modelName);

% 2. Clean up any existing HIL block additions and lines to avoid duplicate block/connection errors
disp('[*] Cleaning up existing HIL blocks and lines...');

hil_blocks = {'HIL_Interface', 'HIL_Theta_Gain', 'HIL_Omega_Gain', ...
              'Switch_Theta', 'Switch_Omega', 'Switch_Tau', 'Const_Zero'};

ports_to_clear = {
    'Attitude_Controller', 2;
    'Attitude_Controller', 3;
    'Cubesat_Dynamics', 1;
    'Out_Theta', 1;
    'Out_Omega', 1;
    'Out_Tau', 1;
    'Live_Scope', 1;
    'Live_Scope', 2;
    'Live_Scope', 3
};

% Try to find and delete all lines connected to HIL blocks or to-be-cleared ports
try
    lines = find_system(modelName, 'FindAll', 'on', 'Type', 'line');
    for i = 1:length(lines)
        try
            srcBlock = get_param(lines(i), 'SrcBlockHandle');
            dstBlock = get_param(lines(i), 'DstBlockHandle');
            
            srcName = '';
            if srcBlock ~= -1
                srcName = get_param(srcBlock, 'Name');
            end
            
            dstName = '';
            if dstBlock ~= -1
                dstName = get_param(dstBlock, 'Name');
            end
            
            is_hil_line = ismember(srcName, hil_blocks) || ismember(dstName, hil_blocks);
            
            is_cleared_port = false;
            if dstBlock ~= -1
                dstPort = get_param(lines(i), 'DstPortHandle');
                for dp = dstPort(:)'
                    portNumber = get_param(dp, 'PortNumber');
                    for k = 1:size(ports_to_clear, 1)
                        if strcmp(dstName, ports_to_clear{k, 1}) && portNumber == ports_to_clear{k, 2}
                            is_cleared_port = true;
                            break;
                        end
                    end
                    if is_cleared_port, break; end
                end
            end
            
            if is_hil_line || is_cleared_port
                delete_line(lines(i));
            end
        catch
        end
    end
catch ME
    warning('Error during initial line cleanup: %s', ME.message);
end

% Also try standard delete_line as a backup for non-HIL connections
try, delete_line(modelName, 'Cubesat_Dynamics/1', 'Out_Theta/1'); catch, end
try, delete_line(modelName, 'Cubesat_Dynamics/2', 'Out_Omega/1'); catch, end
try, delete_line(modelName, 'Cubesat_Dynamics/1', 'Attitude_Controller/2'); catch, end
try, delete_line(modelName, 'Cubesat_Dynamics/2', 'Attitude_Controller/3'); catch, end
try, delete_line(modelName, 'Cubesat_Dynamics/1', 'Live_Scope/1'); catch, end
try, delete_line(modelName, 'Cubesat_Dynamics/2', 'Live_Scope/2'); catch, end
try, delete_line(modelName, 'Attitude_Controller/1', 'Cubesat_Dynamics/1'); catch, end
try, delete_line(modelName, 'Attitude_Controller/1', 'Live_Scope/3'); catch, end
try, delete_line(modelName, 'Attitude_Controller/1', 'Out_Tau/1'); catch, end

% Now delete the HIL blocks themselves
for i = 1:length(hil_blocks)
    blockPath = [modelName '/' hil_blocks{i}];
    try
        delete_block(blockPath);
    catch
    end
end

% 4. Add the Custom HIL Block (MATLAB System referencing SerialHILSystemObject)
disp('[*] Adding custom MATLAB System Block (HIL_Interface)...');
add_block('simulink/User-Defined Functions/MATLAB System', [modelName '/HIL_Interface']);
set_param([modelName '/HIL_Interface'], ...
          'System', 'SerialHILSystemObject', ...
          'Position', [450, 270, 600, 390]);

% 5. Add conversion gains (sensors output degrees/s and deg, we need radians)
add_block('simulink/Math Operations/Gain', [modelName '/HIL_Theta_Gain']);
set_param([modelName '/HIL_Theta_Gain'], 'Gain', 'pi/180', 'Position', [650, 270, 690, 300]);

add_block('simulink/Math Operations/Gain', [modelName '/HIL_Omega_Gain']);
set_param([modelName '/HIL_Omega_Gain'], 'Gain', 'pi/180', 'Position', [650, 310, 690, 340]);

% 6. Add Manual Switches for Mode Toggling
disp('[*] Adding Manual Switches...');
add_block('simulink/Signal Routing/Manual Switch', [modelName '/Switch_Theta']);
set_param([modelName '/Switch_Theta'], 'Position', [750, 80, 780, 140]);

add_block('simulink/Signal Routing/Manual Switch', [modelName '/Switch_Omega']);
set_param([modelName '/Switch_Omega'], 'Position', [750, 160, 780, 220]);

add_block('simulink/Signal Routing/Manual Switch', [modelName '/Switch_Tau']);
set_param([modelName '/Switch_Tau'], 'Position', [380, 290, 410, 350]);

% Safety block: Constant 0 torque for HIL during pure simulation mode
add_block('simulink/Sources/Constant', [modelName '/Const_Zero']);
set_param([modelName '/Const_Zero'], 'Value', '0', 'Position', [300, 295, 330, 315]);

% 7. Re-route connections through the switches
disp('[*] Connecting lines...');

% Switch_Tau inputs (Top = 0 torque safety, Bottom = controller torque)
safe_add_line(modelName, 'Const_Zero/1', 'Switch_Tau/1');
safe_add_line(modelName, 'Attitude_Controller/1', 'Switch_Tau/2');
safe_add_line(modelName, 'Switch_Tau/1', 'HIL_Interface/1');

% Controller output torque always drives simulation dynamics too
safe_add_line(modelName, 'Attitude_Controller/1', 'Cubesat_Dynamics/1');

% Connect HIL outputs to Gains (HIL_Interface output 1 = fused_hdg, output 3 = gz)
safe_add_line(modelName, 'HIL_Interface/1', 'HIL_Theta_Gain/1');
safe_add_line(modelName, 'HIL_Interface/3', 'HIL_Omega_Gain/1');

% Switch_Theta inputs (Top = simulation theta, Bottom = HIL theta)
safe_add_line(modelName, 'Cubesat_Dynamics/1', 'Switch_Theta/1');
safe_add_line(modelName, 'HIL_Theta_Gain/1', 'Switch_Theta/2');

% Switch_Omega inputs (Top = simulation omega, Bottom = HIL omega)
safe_add_line(modelName, 'Cubesat_Dynamics/2', 'Switch_Omega/1');
safe_add_line(modelName, 'HIL_Omega_Gain/1', 'Switch_Omega/2');

% Connect Switch outputs to Controller Inputs & Workspace Sinks
safe_add_line(modelName, 'Switch_Theta/1', 'Attitude_Controller/2');
safe_add_line(modelName, 'Switch_Theta/1', 'Out_Theta/1');

safe_add_line(modelName, 'Switch_Omega/1', 'Attitude_Controller/3');
safe_add_line(modelName, 'Switch_Omega/1', 'Out_Omega/1');

% Out_Tau logging
safe_add_line(modelName, 'Attitude_Controller/1', 'Out_Tau/1');

% 8. Connect Switch outputs to the Live Scope
h1 = safe_add_line(modelName, 'Switch_Theta/1', 'Live_Scope/1');
set_param(h1, 'Name', 'Attitude Angle (rad)');

h2 = safe_add_line(modelName, 'Switch_Omega/1', 'Live_Scope/2');
set_param(h2, 'Name', 'Angular Velocity (rad/s)');

h3 = safe_add_line(modelName, 'Attitude_Controller/1', 'Live_Scope/3');
set_param(h3, 'Name', 'Control Torque (Nm)');

% 9. Configure Pacing, Save, and close
set_param(modelName, 'EnablePacing', 'on');
set_param(modelName, 'PacingRate', '1');
save_system(modelName);
close_system(modelName);

disp('[+] Cubesat_Control_PD.slx has been programmatically updated with dual Sim/HIL paths!');
disp('    Double-click the Manual Switch blocks to toggle between modes.');
