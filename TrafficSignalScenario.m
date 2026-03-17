classdef TrafficSignalScenario
    properties
        time_points = [0, 5, 15, 18, 22, 26]; % s
        lead_velocity = [22, 20, 20, 0, 0, 22] .* (1000/3600); % m/s
        road = struct('Crr', 0.015, 'mu', 0.7, 'theta', 0);
    end
    
    methods
        function state = initialize(scenario)
            % [d_rel, v_e, a_e, μ, θ, v_lead]
            state = [60, 20*(1000/3600), 0, 0.7, 0, 22*(1000/3600)]; 
        end
        
        function v = get_velocity(scenario, t)
            v = interp1(scenario.time_points, scenario.lead_velocity, t, 'linear');
        end
        
        function road = get_road_conditions(scenario, t)
            road = scenario.road; % Constant for traffic signal
        end
    end
end