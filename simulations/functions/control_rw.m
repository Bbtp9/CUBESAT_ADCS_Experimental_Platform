function tau = control_rw(x, Kp, Kd, tau_max, theta_ref, omega_th)
    theta = x(1);
    omega = x(2);

    % Mode switching: detumbling first, then pointing
    if abs(omega) > omega_th
        tau_cmd = -Kd * omega;
    else
        tau_cmd = -Kp * (theta - theta_ref) - Kd * omega;
    end

    % Torque saturation
    tau = min(max(tau_cmd, -tau_max), tau_max);
end