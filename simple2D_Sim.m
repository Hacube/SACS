clear; close all; clc;

% Constants
L = 0.6;            % length of thrust platform
m = 4;              % mass (kg) 
I = m * L^2 / 12;   % moment of inertia
P = 110*6894.76;    % convert max psi to Pa
r = 0.005 / 2;      % radius of "nozzle" (m)
A = pi * r^2;       % area of "nozzle"
F_max = P * A;      % max thrust, simplified
g = 9.81;
dt = 0.01;          % time step

% Initial state - simplifed by only doing 1axis 
y = 0; % see how controller reacts when changing these values 
v_y = 0;

% Desired position
y_desired = 1.0; 

% Set up PID controller
Kp = 10;
Ki = 6;
Kd = 10;

integral_error = 0; % set inital errors to zero
previous_error = y_desired - y;

% Storage for plotting
time = 0;
y_position = y;
F_total_data = 0;

for i = 1:1000
    % Calc the PID control
    error = y_desired - y;
    integral_error = integral_error + error * dt;
    derivative_error = (error - previous_error) / dt;
    F_total = Kp * error + Ki * integral_error + Kd * derivative_error;
    
    % Clamp the thrust output to stay within range
    F_total = max(0, min(F_total, F_max));
    F_total = 4 * F_total;   % simplifed
    
    % Update state w/ simplifed eqs
    y = y + v_y * dt;
    v_y = v_y + (1 / m) * F_total * dt - g * dt;

    % Update previous error
    previous_error = error;

    % Store the time & position data
    time = [time; i * dt];
    y_position = [y_position; y];
    F_total_data = [F_total_data; F_total];
end

% Plot Y-position 
figure;
hold on; grid on;
plot(time, y_position, time, y_desired * ones(size(time)), '--');
title('Altitude vs Time');
xlabel('Time (sec)'); ylabel('Altitude (m)');
legend('Actual Altitude', 'Desired Altitude');
