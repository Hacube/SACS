function pressure_out = pressureRegulator_V1(pressure_in, setpoint)
    % Parameters
    Kp = 1.0;  % Proportional gain (how fast it should react)
    Ti = 2.0;  % Integral time constant

    % Error calculation (difference between the desired setpoint and the
    % current process variable)
    error = setpoint - pressure_in;

    % Proportional control (calculates the control output)
    P_term = Kp * error;

    % Integral control using a simple discrete-time approximation
    persistent integral_sum;  % Keeps its value between function calls
    if isempty(integral_sum)
        integral_sum = 0;
    end
    integral_sum = integral_sum + error;

    % Calculation for the control output
    control_output = P_term + (Kp / Ti) * integral_sum;

    % Pressure output
    pressure_out = control_output;

    % Control output so it does not exceed certain limits
    if pressure_out > 100
        pressure_out = 100;
    elseif pressure_out < 0
        pressure_out = 0;
    end

end
