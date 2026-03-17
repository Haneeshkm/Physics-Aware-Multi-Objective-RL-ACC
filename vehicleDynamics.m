function [a_e, F_net] = vehicleDynamics(v_e, a_prev, action, dt, params, road)
    % Convert action to acceleration command
    if action == 1
        a_cmd = params.acc_accel;
    elseif action == -1
        a_cmd = params.acc_brake;
    else
        a_cmd = 0;
    end
    
    % Simulate acceleration with first-order lag
    a_e = a_prev + (a_cmd - a_prev) * (dt / (dt + params.tau));
    
    % Calculate net force (optional for simulation)
    F_drag = 0.5 * params.rho * params.Cd * params.A * v_e^2;
    F_grade = params.m * params.g * sin(road.grade);
    F_net = params.m * a_e + F_drag + F_grade;
end