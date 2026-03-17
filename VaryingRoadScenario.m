classdef VaryingRoadScenario
    properties
        time_points = [0, 10, 20, 30]; % s
        lead_velocity = [22, 22, 22, 22] .* (1000/3600); % m/s
    end
    
    methods
        function state = initialize(scenario)
            % [d_rel, v_e, a_e, μ, θ, v_lead]
            state = [60, 20*(1000/3600), 0, 0.7, 0, 22*(1000/3600)]; 
        end
        
        function v = get_velocity(scenario, t)
            v = interp1(scenario.time_points, scenario.lead_velocity, t, 'nearest');
        end
        
        function road = get_road_conditions(scenario, t)
            if t < 10
                % Dry road
                road = struct('Crr', 0.015, 'mu', 0.7, 'theta', 0);
            elseif t < 20
                % Wet road
                road = struct('Crr', 0.03, 'mu', 0.4, 'theta', 0);
            else
                % Icy road
                road = struct('Crr', 0.08, 'mu', 0.1, 'theta', 0);
            end
        end
    end
end