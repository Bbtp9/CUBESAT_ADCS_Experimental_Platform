function dx = dynamics_rw(~, x, J, Jw, Kp, Kd, tau_max, theta_ref, omega_th)
    theta   = x(1);
    omega   = x(2);
    omega_w = x(3);

    tau = control_rw(x, Kp, Kd, tau_max, theta_ref, omega_th);

    dtheta   = omega;
    domega   = -tau / J;
    domega_w =  tau / Jw;

    dx = [dtheta; domega; domega_w];
end