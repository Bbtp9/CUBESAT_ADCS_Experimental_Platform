function tau = control_rw(x, Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th)
% control_rw  PD pointing + derivative-only detumble controller
    theta = x(1);
    omega = x(2);

    if abs(omega) > omega_th
        tau_cmd = -Kd_detumble * omega;
    else
        err = wrapToPi(theta - theta_ref);
        tau_cmd = -Kp * err - Kd * omega;
    end

    tau = max(min(tau_cmd, tau_max), -tau_max);
end

function dx = dynamics_rw(~, x, J, Jw, Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th)
% dynamics_rw  single-axis spacecraft + reaction wheel dynamics
    theta   = x(1);
    omega   = x(2);
    omega_w = x(3);

    tau = control_rw([theta; omega], Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th);

    dtheta   = omega;
    domega   = -tau / J;    % torque on spacecraft
    domega_w =  tau / Jw;   % equal and opposite on wheel

    dx = [dtheta; domega; domega_w];
end
