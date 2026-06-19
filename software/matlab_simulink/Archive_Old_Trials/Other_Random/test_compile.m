% test_compile.m
omega0_deg = 50;
theta_ref_deg = 180;
J   = 0.000634;
Jw  = 4.607e-5;
tau = 2.3;
lambda = 1.0; 
Kp = 3 * J * (lambda^2);
Kd = J * (3 * lambda - tau);
Ki = J * (lambda^3);
Kd_detumble = 0.003;
tau_max  = 0.002;
omega_th_high = deg2rad(10000);
omega_th_low  = deg2rad(0.3);
theta0   = deg2rad(0);
theta_ref= deg2rad(theta_ref_deg);
omega0   = deg2rad(omega0_deg);
omega_w0 = 0;
t_stop = 20;

load_system('Cubesat_Control_PD.slx');
try
    set_param('Cubesat_Control_PD', 'SimulationCommand', 'update');
    disp('=== Compile successful! ===');
catch ME
    disp('=== Compile failed! ===');
    disp(ME.message);
    if ~isempty(ME.cause)
        for i=1:length(ME.cause)
            disp(ME.cause{i}.message);
            if ~isempty(ME.cause{i}.cause)
                for j=1:length(ME.cause{i}.cause)
                    disp(ME.cause{i}.cause{j}.message);
                end
            end
        end
    end
end
