% SIMULINK_InitialParameters.m
% Initial parameters for CubeSat reaction wheel Simulink model

clc;

%% Physical parameters
J  = 0.000634;   % spacecraft inertia [kg*m^2]
Jw = 4.607e-5;   % reaction wheel inertia [kg*m^2]

%% Control gains
Kp = 0.02;       % proportional gain for pointing
Kd = 0.01;       % derivative gain for pointing
Kd_detumble = 0.03;   % derivative gain for detumbling

%% Actuator limit
tau_max = 0.002; % maximum control torque [N*m]

%% Threshold for switching detumbling -> pointing
omega_th = deg2rad(0.5);   % [rad/s]

%% Initial conditions
theta0   = deg2rad(15);    % initial attitude angle [rad]
omega0   = deg2rad(8);     % initial body angular rate [rad/s] - viteza satelitului
omega_w0 = 0;              % initial wheel speed [rad/s]  - viteza reaction wheel ului

%% Ask user for reference angle
theta_ref_deg = input('Enter desired reference angle theta_ref [deg]: ');

while isempty(theta_ref_deg) || ~isnumeric(theta_ref_deg) || ~isscalar(theta_ref_deg)
    theta_ref_deg = input('Invalid input. Enter theta_ref as a scalar in degrees: ');
end

theta_ref = deg2rad(theta_ref_deg);

fprintf('theta_ref = %.2f deg = %.4f rad\n', theta_ref_deg, theta_ref);

%% Choose model name
model_name = 'POINTING_1';   % change this if your .slx has another name

%% Run Simulink model
open_system(model_name);
simOut = sim(model_name);


% -------------------------------------------------------------------------
% SIMULINK MODEL DESCRIPTION – CUBESAT ATTITUDE CONTROL USING REACTION WHEEL
%
% The Simulink model represents a simplified single-axis attitude control
% system of a CubeSat actuated by a reaction wheel.
%
% The signal flow in the model is structured as follows:
%
% 1. Reference input
%
%    theta_ref --> Error computation
%
%    The desired reference attitude (theta_ref) is compared with the
%    current spacecraft attitude (theta) in order to generate the
%    attitude error:
%
%        e = theta_ref - theta
%
%
% 2. Controller
%
%    Attitude error --> Kp --> \
%                                  --> Control law --> Torque saturation --> tau
%    Angular velocity --> -Kd --> /
%
%    The control torque is generated using a proportional-derivative
%    (PD) feedback controller based on the attitude error and the
%    spacecraft angular velocity:
%
%        tau = Kp(theta_ref - theta) - Kd*omega
%
%    A saturation block is included in order to represent actuator
%    limitations and restrict the maximum available control torque.
%
%
% 3. Spacecraft rotational dynamics
%
%    tau --> (-1/J) --> omega_dot --> Integrator --> omega --> Integrator --> theta
%
%    The spacecraft is modeled as a rigid body rotating around a single
%    axis. The applied torque generates angular acceleration, which is
%    integrated to obtain the spacecraft angular velocity and the
%    spacecraft attitude angle.
%
%
% 4. Reaction wheel dynamics
%
%    tau --> (1/Jw) --> omega_w_dot --> Integrator --> omega_w
%
%    The reaction wheel dynamics are modeled using the wheel moment of
%    inertia. The wheel accelerates in response to the applied torque,
%    generating the control moment acting on the spacecraft.
%
%
% 5. Simulation outputs
%
%    theta   --> spacecraft attitude angle
%    omega   --> spacecraft angular velocity
%    omega_w --> reaction wheel angular speed
%    tau     --> control torque
%
%    These signals are sent to a Scope block in order to visualize the
%    spacecraft attitude response and the actuator behavior during the
%    simulation.
%
% -------------------------------------------------------------------------