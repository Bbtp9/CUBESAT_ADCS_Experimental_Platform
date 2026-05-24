% build_simulink_model.m
% Automatically generates a structured Simulink model for Detumbling and Pointing

% clc; clear; % Do NOT clear workspace here!
modelName = 'Cubesat_Control_PD';

% Close if already open
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

% Create new model
new_system(modelName);
open_system(modelName);

% -------------------------------------------------------------------------
% 1. ROOT LEVEL BLOCKS
% -------------------------------------------------------------------------

% Constant: Theta_ref
add_block('simulink/Sources/Constant', [modelName '/Theta_ref']);
set_param([modelName '/Theta_ref'], 'Value', 'theta_ref', 'Position', [50, 100, 80, 130]);

% Subsystem: Controller
add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Attitude_Controller']);
set_param([modelName '/Attitude_Controller'], 'Position', [200, 100, 350, 200]);

% Subsystem: Dynamics
add_block('simulink/Ports & Subsystems/Subsystem', [modelName '/Cubesat_Dynamics']);
set_param([modelName '/Cubesat_Dynamics'], 'Position', [450, 100, 600, 200]);

% To Workspace blocks (for plotting)
add_block('simulink/Sinks/To Workspace', [modelName '/Out_Theta']);
set_param([modelName '/Out_Theta'], 'VariableName', 'theta_out', 'SaveFormat', 'Array', 'Position', [700, 60, 760, 90]);

add_block('simulink/Sinks/To Workspace', [modelName '/Out_Omega']);
set_param([modelName '/Out_Omega'], 'VariableName', 'omega_out', 'SaveFormat', 'Array', 'Position', [700, 135, 760, 165]);

add_block('simulink/Sinks/To Workspace', [modelName '/Out_Omega_w']);
set_param([modelName '/Out_Omega_w'], 'VariableName', 'omega_w_out', 'SaveFormat', 'Array', 'Position', [700, 210, 760, 240]);

add_block('simulink/Sinks/To Workspace', [modelName '/Out_Tau']);
set_param([modelName '/Out_Tau'], 'VariableName', 'tau_out', 'SaveFormat', 'Array', 'Position', [400, 40, 460, 70]);




% -------------------------------------------------------------------------
% 2. BUILD INNER DYNAMICS SUBSYSTEM (with 3 integrators)
% -------------------------------------------------------------------------
dynPath = [modelName '/Cubesat_Dynamics'];

% Remove default blocks
delete_block([dynPath '/In1']);
delete_block([dynPath '/Out1']);

% Add In/Out Ports
add_block('simulink/Sources/In1', [dynPath '/Tau_cmd'], 'Position', [20, 100, 50, 114]);
add_block('simulink/Sinks/Out1', [dynPath '/Theta'], 'Position', [500, 40, 530, 54]);
add_block('simulink/Sinks/Out1', [dynPath '/Omega'], 'Position', [500, 100, 530, 114]);
add_block('simulink/Sinks/Out1', [dynPath '/Omega_w'], 'Position', [500, 170, 530, 184]);

% Gain blocks
add_block('simulink/Math Operations/Gain', [dynPath '/Gain_1_Jw']);
set_param([dynPath '/Gain_1_Jw'], 'Gain', '1/Jw', 'Position', [150, 160, 200, 190]);

add_block('simulink/Math Operations/Gain', [dynPath '/Gain_1_J']);
set_param([dynPath '/Gain_1_J'], 'Gain', '-1/J', 'Position', [150, 90, 200, 120]);

% Integrators
add_block('simulink/Continuous/Integrator', [dynPath '/Int_Omega_w']);
set_param([dynPath '/Int_Omega_w'], 'InitialCondition', 'omega_w0', 'Position', [280, 160, 310, 190]);

add_block('simulink/Continuous/Integrator', [dynPath '/Int_Omega']);
set_param([dynPath '/Int_Omega'], 'InitialCondition', 'omega0', 'Position', [280, 90, 310, 120]);

add_block('simulink/Continuous/Integrator', [dynPath '/Int_Theta']);
set_param([dynPath '/Int_Theta'], 'InitialCondition', 'theta0', 'Position', [400, 30, 430, 60]);

% Routing inside Dynamics
add_line(dynPath, 'Tau_cmd/1', 'Gain_1_Jw/1', 'autorouting', 'on');
add_line(dynPath, 'Tau_cmd/1', 'Gain_1_J/1', 'autorouting', 'on');
add_line(dynPath, 'Gain_1_Jw/1', 'Int_Omega_w/1', 'autorouting', 'on');
add_line(dynPath, 'Gain_1_J/1', 'Int_Omega/1', 'autorouting', 'on');
add_line(dynPath, 'Int_Omega/1', 'Int_Theta/1', 'autorouting', 'on');
add_line(dynPath, 'Int_Theta/1', 'Theta/1', 'autorouting', 'on');
add_line(dynPath, 'Int_Omega/1', 'Omega/1', 'autorouting', 'on');
add_line(dynPath, 'Int_Omega_w/1', 'Omega_w/1', 'autorouting', 'on');


% -------------------------------------------------------------------------
% 3. BUILD INNER CONTROLLER SUBSYSTEM (Detumbling -> Pointing Logic)
% -------------------------------------------------------------------------
ctrlPath = [modelName '/Attitude_Controller'];

% Remove default blocks
delete_block([ctrlPath '/In1']);
delete_block([ctrlPath '/Out1']);

% Add In/Out Ports
add_block('simulink/Sources/In1', [ctrlPath '/Theta_ref'], 'Position', [20, 50, 50, 64]);
add_block('simulink/Sources/In1', [ctrlPath '/Theta'], 'Position', [20, 100, 50, 114]);
add_block('simulink/Sources/In1', [ctrlPath '/Omega'], 'Position', [20, 200, 50, 214]);
add_block('simulink/Sinks/Out1', [ctrlPath '/Tau'], 'Position', [900, 150, 930, 164]);

% Error calculation
add_block('simulink/Math Operations/Sum', [ctrlPath '/Sum_Error']);
set_param([ctrlPath '/Sum_Error'], 'Inputs', '+-', 'Position', [120, 65, 140, 85]);

% Pointing Controller (Kp * error - Kd * omega)
add_block('simulink/Math Operations/Gain', [ctrlPath '/Kp_Gain']);
set_param([ctrlPath '/Kp_Gain'], 'Gain', 'Kp', 'Position', [220, 60, 260, 90]);
add_block('simulink/Math Operations/Gain', [ctrlPath '/Kd_Gain']);
set_param([ctrlPath '/Kd_Gain'], 'Gain', 'Kd', 'Position', [220, 120, 260, 150]);
add_block('simulink/Math Operations/Sum', [ctrlPath '/Sum_PD']);
set_param([ctrlPath '/Sum_PD'], 'Inputs', '+-', 'Position', [350, 95, 370, 115]);

% Detumbling Controller (-Kd_detumble * omega)
add_block('simulink/Math Operations/Gain', [ctrlPath '/Kd_detumble']);
set_param([ctrlPath '/Kd_detumble'], 'Gain', '-Kd_detumble', 'Position', [250, 250, 310, 280]);

% Switch Logic (Stateflow or simple logic)
% We will use an absolute value of omega and a switch block.
add_block('simulink/Math Operations/Abs', [ctrlPath '/Abs_Omega']);
set_param([ctrlPath '/Abs_Omega'], 'Position', [150, 190, 180, 220]);

% Add Relay for Hysteresis
add_block('simulink/Discontinuities/Relay', [ctrlPath '/Hysteresis_Relay']);
set_param([ctrlPath '/Hysteresis_Relay'], 'OnSwitchValue', 'omega_th_high', 'OffSwitchValue', 'omega_th_low', 'OnOutputValue', '1', 'OffOutputValue', '0', 'Position', [250, 190, 280, 220]);

% Add a Switch
add_block('simulink/Signal Routing/Switch', [ctrlPath '/Mode_Switch']);
set_param([ctrlPath '/Mode_Switch'], 'Criteria', 'u2 > Threshold', 'Threshold', '0.5', 'Position', [550, 140, 600, 200]);
% Input 1 is the signal when Condition is TRUE. 
% Condition is Abs(Omega) >= threshold. So if TRUE, we are tumbling!
% We want port 1 = Detumble. Port 2 = Criteria. Port 3 = Pointing.

% Memory Block for Latch (Hysteresis) - a simple trick: if it falls below threshold it stays.
% Actually, a smooth switch or just instantaneous is fine for now. Let's do simple threshold switch.

% Saturation 
add_block('simulink/Discontinuities/Saturation', [ctrlPath '/Torque_Sat']);
set_param([ctrlPath '/Torque_Sat'], 'UpperLimit', 'tau_max', 'LowerLimit', '-tau_max', 'Position', [700, 155, 740, 185]);

% Routing inside Controller
add_line(ctrlPath, 'Theta_ref/1', 'Sum_Error/1', 'autorouting', 'on');
add_line(ctrlPath, 'Theta/1', 'Sum_Error/2', 'autorouting', 'on');
add_line(ctrlPath, 'Sum_Error/1', 'Kp_Gain/1', 'autorouting', 'on');

% Feed omega to pointing derivative, detumbling, and absolute
add_line(ctrlPath, 'Omega/1', 'Kd_Gain/1', 'autorouting', 'on');
add_line(ctrlPath, 'Omega/1', 'Kd_detumble/1', 'autorouting', 'on');
add_line(ctrlPath, 'Omega/1', 'Abs_Omega/1', 'autorouting', 'on');

add_line(ctrlPath, 'Kp_Gain/1', 'Sum_PD/1', 'autorouting', 'on');
add_line(ctrlPath, 'Kd_Gain/1', 'Sum_PD/2', 'autorouting', 'on');

% Gain -1 to convert Satellite Required Torque to Motor Requested Torque
add_block('simulink/Math Operations/Gain', [ctrlPath '/Motor_Inversion']);
set_param([ctrlPath '/Motor_Inversion'], 'Gain', '-1', 'Position', [630, 155, 670, 185]);

% Connect to Switch
% Switch uses Detumbling when Relay Output is 1 (ON)
add_line(ctrlPath, 'Kd_detumble/1', 'Mode_Switch/1', 'autorouting', 'on'); % Top pin (True, Detumbling)
add_line(ctrlPath, 'Abs_Omega/1', 'Hysteresis_Relay/1', 'autorouting', 'on'); % Abs_Omega into Relay
add_line(ctrlPath, 'Hysteresis_Relay/1', 'Mode_Switch/2', 'autorouting', 'on'); % Relay into Switch Control
add_line(ctrlPath, 'Sum_PD/1', 'Mode_Switch/3', 'autorouting', 'on'); % Bottom pin (False, Pointing)

add_line(ctrlPath, 'Mode_Switch/1', 'Motor_Inversion/1', 'autorouting', 'on');
add_line(ctrlPath, 'Motor_Inversion/1', 'Torque_Sat/1', 'autorouting', 'on');
add_line(ctrlPath, 'Torque_Sat/1', 'Tau/1', 'autorouting', 'on');

% -------------------------------------------------------------------------
% 4. ROOT LEVEL ROUTING 
% -------------------------------------------------------------------------
% Model routing at root
add_line(modelName, 'Theta_ref/1', 'Attitude_Controller/1', 'autorouting', 'on');
add_line(modelName, 'Attitude_Controller/1', 'Cubesat_Dynamics/1', 'autorouting', 'on');

% Connect Controller Output (Tau) to Workspace
add_line(modelName, 'Attitude_Controller/1', 'Out_Tau/1', 'autorouting', 'on');

% Connect Dynamics Outputs to Workspaces & Feedback
add_line(modelName, 'Cubesat_Dynamics/1', 'Out_Theta/1', 'autorouting', 'on');
add_line(modelName, 'Cubesat_Dynamics/2', 'Out_Omega/1', 'autorouting', 'on');
add_line(modelName, 'Cubesat_Dynamics/3', 'Out_Omega_w/1', 'autorouting', 'on');

% Feedback Routing (Theta and Omega go back to the controller)
add_line(modelName, 'Cubesat_Dynamics/1', 'Attitude_Controller/2', 'autorouting', 'on');
add_line(modelName, 'Cubesat_Dynamics/2', 'Attitude_Controller/3', 'autorouting', 'on');

% ADD SCOPE TO ROOT LEVEL
add_block('simulink/Sinks/Scope', [modelName '/Live_Scope']);
set_param([modelName '/Live_Scope'], 'NumInputPorts', '3', 'Position', [700, 280, 760, 360]);

% Route signals to scope and name them for legend clarity
h1 = add_line(modelName, 'Cubesat_Dynamics/1', 'Live_Scope/1', 'autorouting', 'on');
set_param(h1, 'Name', 'Attitude Angle (rad)');

h2 = add_line(modelName, 'Cubesat_Dynamics/2', 'Live_Scope/2', 'autorouting', 'on');
set_param(h2, 'Name', 'Angular Velocity (rad/s)');

h3 = add_line(modelName, 'Attitude_Controller/1', 'Live_Scope/3', 'autorouting', 'on');
set_param(h3, 'Name', 'Control Torque (Nm)');

% Update diagram and save
set_param(modelName, 'Solver', 'ode45', 'StopTime', 't_stop');
save_system(modelName, ['/Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/' modelName '.slx']);
close_system(modelName);

disp('Simulink PD model Cubesat_Control_PD.slx has been thoroughly generated!');
