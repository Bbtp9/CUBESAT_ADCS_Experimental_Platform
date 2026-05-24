function [tau, mode] = control_rw(x, Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th, omega_th_low)
    theta = x(1);
    omega = x(2);

    if nargin < 8 || isempty(omega_th_low)
        omega_th_low = 0.5 * omega_th; % Hysteresis to avoid mode chattering
    end

    persistent mode_active;
    if isempty(mode_active)
        mode_active = 1; % detumble by default
    end

    if mode_active == 1
        if abs(omega) < omega_th_low
            mode_active = 2; % switch to pointing permanently
        end
    end
    mode = mode_active;

    if mode == 1
        tau_cmd = -Kd_detumble * omega;
    else
        err = wrapToPi(theta - theta_ref);
        tau_cmd = -Kp * err - Kd * omega;
    end

    tau = max(min(tau_cmd, tau_max), -tau_max);
end