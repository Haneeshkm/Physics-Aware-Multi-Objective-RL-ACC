%% ACCEnv.m - Physics-Aware Environment Class
classdef ACCEnv < rl.env.MATLABEnvironment
    properties
        % Physics parameters (from initialization)
        m
        Cd
        A
        rho
        Cr
        g
        Ts
        Tmax
        h
        d0
        
        % State variables
        egoPos = 0
        egoVel = 15
        egoAcc = 0
        leadPos = 50
        leadVel = 15
        time = 0
        prevAcc = 0
        minGap = 5
        
        % Environmental conditions
        mu = 0.8        % Friction coefficient
        theta = 0        % Road grade (radians)
        volatility = 1   % Lead vehicle volatility level
    end
    
    properties (Access = protected)
        IsDone = false
    end
    
    methods
        function this = ACCEnv(physicsParams)
            % Initialize with physics parameters
            this.m = physicsParams.m;
            this.Cd = physicsParams.Cd;
            this.A = physicsParams.A;
            this.rho = physicsParams.rho;
            this.Cr = physicsParams.Cr;
            this.g = physicsParams.g;
            this.Ts = physicsParams.Ts;
            this.Tmax = physicsParams.Tmax;
            this.h = physicsParams.h;
            this.d0 = physicsParams.d0;
            
            % Define observation and action spaces
            obsInfo = rlNumericSpec([9 1], ...
                'LowerLimit', [-inf; 0; -inf; 0; -inf; 0; -3; 0; -deg2rad(10)], ...
                'UpperLimit', inf(9,1));
            obsInfo.Name = 'observations';
            obsInfo.Description = ['egoPos, egoVel, leadPos, leadVel, ' ...
                'relVel, timeHeadway, prevAcc, mu, theta'];
            
            actInfo = rlNumericSpec([1 1], ...
                'LowerLimit', -3, 'UpperLimit', 3);
            actInfo.Name = 'acceleration';
            
            % Create environment
            this = this@rl.env.MATLABEnvironment(obsInfo, actInfo);
        end
        
        function [obs, reward, isDone, logged] = step(this, action)
            % Parse action
            cmdAcc = max(min(action, 3), -3);
            this.prevAcc = this.egoAcc;
            
            % === Physics-Based Vehicle Dynamics ===
            % Resistive forces
            F_drag = 0.5 * this.rho * this.Cd * this.A * this.egoVel^2;
            F_roll = this.m * this.g * this.Cr * cos(this.theta);
            F_gravity = this.m * this.g * sin(this.theta);
            
            % Actuator dynamics (1st-order lag)
            this.egoAcc = this.egoAcc + (cmdAcc - this.egoAcc) * (this.Ts / 0.2);
            
            % Net acceleration
            netAcc = this.egoAcc - (F_drag + F_roll + F_gravity)/this.m;
            
            % Update velocity and position
            this.egoVel = max(this.egoVel + netAcc * this.Ts, 0);
            this.egoPos = this.egoPos + this.egoVel * this.Ts;
            
            % === Lead Vehicle Behavior ===
            this.updateLeadVehicle();
            
            % === Environmental Changes ===
            this.updateEnvironment();
            
            % === Time Update ===
            this.time = this.time + this.Ts;
            
            % === Calculate Reward ===
            [reward, safetyViolation] = this.calculateReward();
            
            % === Check Termination ===
            if safetyViolation || this.time >= this.Tmax
                this.IsDone = true;
            end
            
            % === Observation ===
            obs = this.getObservation();
            isDone = this.IsDone;
            
            % === Logging ===
            logged = struct(...
                'gap', this.leadPos - this.egoPos, ...
                'reward', reward, ...
                'safety_violation', safetyViolation);
        end
        
        function initialObs = reset(this)
            % Reset state with realistic variations
            this.egoPos = 0;
            this.egoVel = 15 + 3*randn();  % 12-18 m/s
            this.egoAcc = 0;
            this.leadPos = 50 + 10*randn(); % 40-60m ahead
            this.leadVel = 15 + 5*randn();  % 10-20 m/s
            this.time = 0;
            this.prevAcc = 0;
            this.IsDone = false;
            
            % Reset environmental conditions
            this.mu = 0.7 + 0.3*rand();  % Friction coefficient (0.7-1.0)
            this.theta = deg2rad(5*(2*rand()-1));  % Road grade (-5° to +5°)
            
            % Random volatility (1-3 scale)
            this.volatility = randi(3);
            
            % Return initial observation
            initialObs = this.getObservation();
        end
    end
    
    methods (Access = private)
        function updateLeadVehicle(this)
            % Volatility-based behavior
            switch this.volatility
                case 1  % Calm driving
                    leadAcc = 0.1 * (randn() - 0.1);
                case 2  % Moderate driving
                    leadAcc = 0.3 * (randn() - 0.1);
                case 3  % Aggressive driving
                    if rand() < 0.05  % 5% chance of emergency braking
                        leadAcc = -4 + rand();
                    else
                        leadAcc = 0.5 * (randn() - 0.2);
                    end
            end
            
            % Apply speed limits
            this.leadVel = max(min(this.leadVel + leadAcc * this.Ts, 3), 35);
            this.leadPos = this.leadPos + this.leadVel * this.Ts;
        end
        
        function updateEnvironment(this)
            % Gradually change road conditions (10% chance per step)
            if rand() < 0.1
                this.mu = max(0.3, min(1.0, this.mu + 0.1*(randn())));
            end
            
            % Change road grade (5% chance per step)
            if rand() < 0.05
                this.theta = deg2rad(max(-10, min(10, rad2deg(this.theta) + 2*(randn()))));
            end
        end
        
        function [reward, safetyViolation] = calculateReward(this)
            % Calculate key metrics
            gap = this.leadPos - this.egoPos;
            relVel = this.leadVel - this.egoVel;
            desiredGap = this.h * this.egoVel + this.d0;
            timeHeadway = gap / max(this.egoVel, 1e-3);
            jerk = abs(this.egoAcc - this.prevAcc) / this.Ts;
            
            % Check safety violation
            safetyViolation = gap < this.minGap;
            
            % Base reward components
            gapReward = exp(-0.05 * abs(gap - desiredGap));
            velMatchReward = exp(-0.1 * abs(relVel));
            headwayReward = exp(-0.2 * abs(timeHeadway - 1.8));
            smoothPenalty = -0.01 * min(jerk^2, 100);
            energyPenalty = -1e-4 * abs(this.egoAcc) * this.m * this.egoVel;
            
            % Safety bonus/penalty
            if safetyViolation
                safetyComponent = -10 - 5*(this.minGap - gap);
            else
                safetyComponent = 0.1 * log(1 + gap - this.minGap);
            end
            
            % Composite reward
            reward = 3*gapReward + 2*velMatchReward + headwayReward + ...
                     safetyComponent + smoothPenalty + energyPenalty;
        end
        
        function obs = getObservation(this)
            % Construct observation vector
            gap = this.leadPos - this.egoPos;
            relVel = this.leadVel - this.egoVel;
            timeHeadway = gap / max(this.egoVel, 1e-3);
            
            obs = [this.egoPos; 
                   this.egoVel; 
                   this.leadPos; 
                   this.leadVel; 
                   relVel; 
                   timeHeadway; 
                   this.prevAcc;
                   this.mu;
                   this.theta];
        end
    end
end