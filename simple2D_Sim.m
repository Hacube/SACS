clear; close all; clc;

% Constants
g = 9.81; 
m = 4;                % mass (kgs)
P_in = 110 * 6894.76; % maximum pressure (paa)
P_out = 101325;
rho_air = 1.225;
d_nozzle = 0.005;     % diameter of "nozzle"
A_nozzle = pi*(d_nozzle/2)^2; 

% IC's
x0 = 0;
y0 = 0;
vx0 = 0;
vy0 = 1;
derror = 0;

V_exit = sqrt((2*(P_in - P_out))/rho_air);       % Exit velocity
F_thrust_one = 2 * A_nozzle * rho_air * V_exit;
tspan = [0, 10];                                 % 10 seconds sim
z0 = [x0; y0; vx0; vy0; derror;];                 % Initial state vector

% ODE45 to solve the system
[t, z] = ode45(@(t, z) dynamics(t, z, m, g, F_thrust_one), tspan, z0);

% Get the position and velocity
x = z(:, 1);
y = z(:, 2);
vx = z(:, 3);
vy = z(:, 4);

% Plot the trajectory
figure;
grid on; hold on;
plot(t, y);
xlabel('Time (s)'); ylabel('Position (m)');
legend('Y Position');
title('Drone Altitude');


function zdot = dynamics(t, z, m, g, F_thrust_one)
    % Parse the state vector
    x = z(1);
    y = z(2);
    vx = z(3);
    vy = z(4);
    error = z(5);
    
    % Desired y position and velocity
    y_desired = 0;
    v_desired = 0;
    
    % PID controller gains - tune parameters
    Kp = 5;
    Ki = 5; 
    Kd = 10;
    
    % PID control for vertical position and velocity
    error = y_desired - y;
    derror = v_desired - vy;
    error = error + error; % update integral term
    u = Kp*error + Ki*error + Kd*derror; % control input
    
    % Calculate acceleration
    ax = 0;           % Assume no horizontal movement
    ay = (m*g + u)/m; % Acceleration in y direction
    
    % Return the derivative of the state
    zdot = [vx; vy; ax; ay; error];
end

