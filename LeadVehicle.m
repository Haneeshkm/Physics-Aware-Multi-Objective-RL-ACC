classdef LeadVehicle < handle
    properties
        velocity_profile
        time_points
        position = 60; % initial position in meters
        current_time = 0;
    end
    
    methods
        function obj = LeadVehicle(velocity_profile, time_points)
            obj.velocity_profile = velocity_profile;
            obj.time_points = time_points;
        end
        
        function v = get_velocity(obj)
            % Interpolate velocity based on current time
            v = interp1(obj.time_points, obj.velocity_profile, obj.current_time, 'linear', 'extrap');
        end
        
        function x = get_position(obj)
            % Update position based on velocity and time step
            v = obj.get_velocity();
            obj.position = obj.position + v * 0.1; % assuming dt = 0.1
            obj.current_time = obj.current_time + 0.1;
            x = obj.position;
        end
    end
end
