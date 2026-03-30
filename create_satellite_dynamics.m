% MATLAB script to build missing Simulink integrators for Cubesat
modelName = 'Motor_Satellite_Dynamics';

% Close if already open
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

% Create new model
new_system(modelName);
open_system(modelName);

% Add Inport for Torque (M)
add_block('simulink/Sources/In1', [modelName '/Torque_M']);
set_param([modelName '/Torque_M'], 'Position', [50, 60, 80, 74]);

% ------------- MOTOR (ROTOR) DYNAMICS -------------
% Gain: 1 / J_rotor
add_block('simulink/Math Operations/Gain', [modelName '/Gain_1_over_Jrotor']);
set_param([modelName '/Gain_1_over_Jrotor'], 'Gain', '1/J_rotor', 'Position', [120, 50, 160, 80]);

% Integrator 1: dot_omega_rotor -> omega_rotor
add_block('simulink/Continuous/Integrator', [modelName '/Integrator_Rotor_Speed']);
set_param([modelName '/Integrator_Rotor_Speed'], 'Position', [200, 50, 230, 80]);

% Outport for omega_rotor
add_block('simulink/Sinks/Out1', [modelName '/Omega_rotor_Out']);
set_param([modelName '/Omega_rotor_Out'], 'Position', [280, 57, 310, 73]);

% Connect Rotor Dynamics
add_line(modelName, 'Torque_M/1', 'Gain_1_over_Jrotor/1');
add_line(modelName, 'Gain_1_over_Jrotor/1', 'Integrator_Rotor_Speed/1');
add_line(modelName, 'Integrator_Rotor_Speed/1', 'Omega_rotor_Out/1');


% ------------- SATELLITE DYNAMICS -------------
% Gain: -1 / J_satelit
add_block('simulink/Math Operations/Gain', [modelName '/Gain_minus_1_over_Jsatelit']);
set_param([modelName '/Gain_minus_1_over_Jsatelit'], 'Gain', '-1/J_satelit', 'Position', [120, 150, 180, 180]);

% Integrator 2: dot_omega_satelit -> omega_satelit (omega_s)
add_block('simulink/Continuous/Integrator', [modelName '/Integrator_Satellite_Speed']);
set_param([modelName '/Integrator_Satellite_Speed'], 'Position', [220, 150, 250, 180]);

% Integrator 3: omega_satelit -> Theta_satelit (P_s)
add_block('simulink/Continuous/Integrator', [modelName '/Integrator_Satellite_Angle']);
set_param([modelName '/Integrator_Satellite_Angle'], 'Position', [320, 150, 350, 180]);

% Outport for omega_satelit
add_block('simulink/Sinks/Out1', [modelName '/Omega_satelit_Out']);
set_param([modelName '/Omega_satelit_Out'], 'Position', [300, 217, 330, 233]);

% Outport for Theta_satelit (P_s)
add_block('simulink/Sinks/Out1', [modelName '/Theta_satelit_Out']);
set_param([modelName '/Theta_satelit_Out'], 'Position', [400, 157, 430, 173]);

% Connect Satellite Dynamics
add_line(modelName, 'Torque_M/1', 'Gain_minus_1_over_Jsatelit/1', 'autorouting', 'on');
add_line(modelName, 'Gain_minus_1_over_Jsatelit/1', 'Integrator_Satellite_Speed/1');
add_line(modelName, 'Integrator_Satellite_Speed/1', 'Integrator_Satellite_Angle/1');
add_line(modelName, 'Integrator_Satellite_Angle/1', 'Theta_satelit_Out/1');

% Branch from omega_s to outport
add_line(modelName, 'Integrator_Satellite_Speed/1', 'Omega_satelit_Out/1', 'autorouting', 'on');

% Save the model in the workspace
save_system(modelName, '/Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/Motor_Satellite_Dynamics.slx');
close_system(modelName);

disp('Simulink model Motor_Satellite_Dynamics.slx generated successfully.');
