classdef SimpleLeadVehicle < handle
    properties
        velocity;  % Current velocity (m/s)
        position;  % Current position (m)
    end
    
    methods
        function obj = SimpleLeadVehicle(initial_velocity, initial_position)
            % Constructor
            obj.velocity = initial_velocity;
            obj.position = initial_position;
        end
        
        function step(obj, dt)
            % Updates lead vehicle position based on constant velocity
            obj.position = obj.position + obj.velocity * dt;
        end
        
        function v = get_velocity(obj)
            % Returns current velocity
            v = obj.velocity;
        end
        
        function x = get_position(obj)
            % Returns current position
            x = obj.position;
        end
    end
end