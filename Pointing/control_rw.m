function [tau, mode] = control_rw(x, Kp, Kd, Kd_detumble, tau_max, theta_ref, omega_th, omega_th_low)
    theta = x(1);
    omega = x(2);

    if nargin < 8 || isempty(omega_th_low)
        omega_th_low = 0.5 * omega_th; % Hysteresis to avoid mode chattering
    end

    % Phase 1: detumble until body rate is low enough
    if abs(omega) > omega_th
        mode = 1; % detumble
    elseif abs(omega) < omega_th_low
        mode = 2; % pointing
    else
        mode = 1; % keep detumbling in the hysteresis band
    end

    if mode == 1
        tau_cmd = -Kd_detumble * omega;
    else
        err = wrapToPi(theta - theta_ref);
        tau_cmd = -Kp * err - Kd * omega;
    end

    tau = max(min(tau_cmd, tau_max), -tau_max);
end