clear; clc; close all;

data = readmatrix('slprj/mag1.txt');


Mx = data(:,1);
My = data(:,2);
Mz = data(:,3);

%calibration: centrează norul de puncte în 0
Mx0 = Mx - mean(Mx)+ 25;
My0 = My - mean(My);
Mz0 = Mz - mean(Mz);

disp(mean(Mx))
disp(mean(My))

figure;
plot(Mx, My, '.');
axis equal; grid on;
xlabel('Mx'); ylabel('My');
title('Raw magnetometer data');

figure;
plot(Mx0, My0, '.');
axis equal; grid on;
xlabel('Mx calibrated'); ylabel('My calibrated');
title('calibrated magnetometer data');

% Cerc ideal centrat în 0
R = mean(sqrt(Mx0.^2 + My0.^2));
theta = linspace(0, 2*pi, 500);

hold on;
plot(R*cos(theta), R*sin(theta), 'LineWidth', 2);
legend('Calibrated data', 'Reference circle');

