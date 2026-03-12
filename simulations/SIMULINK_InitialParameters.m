%% Parameters for the SIMULINK simulation

J = 0.002;          % spacecraft inertia [kg*m^2]
Jw = 1e-5;          % wheel inertia [kg*m^2]

Kp = 0.02;          % proportional gain
Kd = 0.01;          % derivative gain

tau_max = 0.002;    % max control torque [N*m]

theta_ref = 0;              % desired angle [rad]
theta0 = deg2rad(15);       % initial angle [rad]
omega0 = deg2rad(8);        % initial body rate [rad/s]
omega_w0 = 0;               % initial wheel speed [rad/s]


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