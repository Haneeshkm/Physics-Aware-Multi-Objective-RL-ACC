classdef ACCSimulator < handle
    properties
        % Vehicle parameters
        params = struct(...
            'm', 1500, ...       % Mass (kg)
            'A', 2.2, ...        % Frontal area (m^2)
            'Cd', 0.3, ...       % Drag coefficient
            'g', 9.81, ...       % Gravity (m/s^2)
            'rho', 1.225, ...    % Air density (kg/m^3)
            'h', 1.5, ...        % Time headway (s)
            'd0', 5, ...         % Minimum gap (m)
            'tau', 0.5, ...     % Engine time constant (s)
            'acc_accel', 2, ...  % Max acceleration (m/s^2)
            'acc_brake', -3 ...  % Max braking (m/s^2)
        );
        
        % Simulation parameters
        dt = 0.1;                % Time step (s)
        max_steps = 200;         % Max steps per episode
        
        % Reward weights
        lambda = [0.1, 0.01, 0.05, 0.1, 10];  % [safety, speed, accel, jerk, collision]
    end
    
    methods
        function [state, lead_vehicle, road] = reset(obj, scenario)
            % Initialize road and lead vehicle from scenario
            road = scenario.get_road_conditions();
            lead_vehicle = scenario.get_lead_vehicle();
            
            % Initial state: [d_rel, v_e, a_e, x_e, x_lead]
            d_rel = 60;                          % Relative distance (m)
            v_e = 20 * (1000/3600);              % Ego velocity (m/s)
            a_e = 0;                             % Ego acceleration (m/s^2)
            x_e = 0;                             % Ego position (m)
            x_lead = d_rel + x_e;                % Lead position (m)
            
            state = [d_rel, v_e, a_e, x_e, x_lead];
        end
        
        function [next_state, reward, done] = step(obj, state, action, lead_vehicle, road)
            % Extract current state
            d_rel = state(1);
            v_e = state(2);
            a_e_prev = state(3);
            x_e = state(4);
            x_lead = state(5);
            
            % Ego vehicle dynamics (pass road to vehicleDynamics)
            [a_e, ~] = obj.vehicleDynamics(v_e, a_e_prev, action, road);
            v_e_next = v_e + a_e * obj.dt;
            x_e_next = x_e + v_e * obj.dt + 0.5 * a_e * obj.dt^2;
            
            % Update lead vehicle
            lead_vehicle.step(obj.dt);
            v_lead = lead_vehicle.get_velocity();
            x_lead_next = lead_vehicle.get_position();
            d_rel_next = x_lead_next - x_e_next;
            
            % Check collision
            done = d_rel_next < 0;
            
            % Jerk calculation
            jerk = (a_e - a_e_prev) / obj.dt;
            
            % Reward components
            d_des = obj.params.h * v_e + obj.params.d0;
            safety_penalty = obj.lambda(1) * abs(d_rel_next - d_des);
            speed_penalty = obj.lambda(2) * (v_e_next - v_lead)^2;
            acc_penalty = obj.lambda(3) * a_e^2;
            jerk_penalty = obj.lambda(4) * jerk^2;
            collision_penalty = obj.lambda(5) * (d_rel_next < 5);
            
            % Total reward
            reward = -(safety_penalty + speed_penalty + acc_penalty + jerk_penalty + collision_penalty);
            
            % Next state
            next_state = [d_rel_next, v_e_next, a_e, x_e_next, x_lead_next];
        end
        
        function [a_e, F_net] = vehicleDynamics(obj, v_e, a_prev, action, road)
            % Convert action to acceleration command
            if action == 1
                a_cmd = obj.params.acc_accel;
            elseif action == -1
                a_cmd = obj.params.acc_brake;
            else
                a_cmd = 0;
            end
            
            % Simulate first-order lag
            a_e = a_prev + (a_cmd - a_prev) * (obj.dt / (obj.dt + obj.params.tau));
            
            % Net force (include road grade effect)
            F_drag = 0.5 * obj.params.rho * obj.params.Cd * obj.params.A * v_e^2;
            F_grade = obj.params.m * obj.params.g * sin(road.grade);  % Now road is properly passed
            F_net = obj.params.m * a_e + F_drag + F_grade;
        end
    end
end