classdef HighwayScenario < handle
    methods
        function road = get_road_conditions(~)
            % Returns road parameters (e.g., grade, friction)
            road = struct('grade', 0);  % Flat road with 0% incline
        end
        
        function lead_vehicle = get_lead_vehicle(~)
            % Returns a lead vehicle object with initial conditions
            lead_vehicle = SimpleLeadVehicle(20, 100);  % 20 m/s, 100m ahead
        end
    end
end