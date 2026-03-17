classdef EmergencyBrakeScenario
    properties
        time_points = [0, 10, 10.1, 15, 15.1, 20]; % s
        lead_velocity = [22, 22, 0, 0, 22, 22] .* (1000/3600); % m/s
        road = struct('Crr', 0.015, 'mu', 0.7, 'theta', 0);
    end
    
    methods
        function state = initialize(scenario)
            % [d_rel, v_e, a_e, μ, θ, v_lead]
            state = [60, 20*(1000/3600), 0, 0.7, 0, 22*(1000/3600)]; 
        end
        
        function v = get_velocity(scenario, t)
            v = interp1(scenario.time_points, scenario.lead_velocity, t, 'previous');
        end
        
        function road = get_road_conditions(scenario, t)
            road = scenario.road; % Constant for emergency brake
        end
    end
end