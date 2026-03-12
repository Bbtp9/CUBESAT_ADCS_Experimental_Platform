% build_reaction_wheel_model.m
% Programmatically build a simple Simulink model for the single-axis reaction wheel
% Run this in MATLAB with Simulink available. The script creates and saves
% simulation/reaction_wheel_model.slx

model = 'reaction_wheel_model';
new_system(model);
open_system(model);

% Add blocks
add_block('simulink/Sources/In1',[model '/theta_ref'],'Position',[30 30 60 50]);
add_block('simulink/Sources/In1',[model '/theta_meas'],'Position',[30 120 60 140]);
add_block('simulink/Math Operations/Subtract',[model '/Error'],'Position',[120 70 180 110]);
add_block('simulink/Math Operations/Gain',[model '/Kp'],'Position',[230 50 270 90]);
add_block('simulink/Math Operations/Gain',[model '/-Kd'],'Position',[230 120 270 160]);
add_block('simulink/Discontinuities/Saturation',[model '/Saturation'],'Position',[350 80 410 120]);
add_block('simulink/Continuous/Integrator',[model '/Integrator_omega'],'Position',[520 80 560 120]);
add_block('simulink/Sinks/Out1',[model '/tau_out'],'Position',[700 90 730 110]);

% Connect lines
add_line(model,'theta_ref/1','Error/1');
add_line(model,'theta_meas/1','Error/2');
add_line(model,'Error/1','Kp/1');
add_line(model,'Error/1','-Kd/1');
add_line(model,'Kp/1','Saturation/1');
add_line(model,'-Kd/1','Saturation/2');
add_line(model,'Saturation/1','Integrator_omega/1');
add_line(model,'Integrator_omega/1','tau_out/1');

% Set block parameters (example gains)
set_param([model '/Kp'],'Gain','0.05');
set_param([model '/-Kd'],'Gain','-0.02');
set_param([model '/Saturation'],'UpperLimit','0.002','LowerLimit','-0.002');

% Save and close
save_system(model, fullfile(pwd,'simulation','reaction_wheel_model.slx'));
close_system(model);

fprintf('Simulink model built at: simulation/reaction_wheel_model.slx\n');
