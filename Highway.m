clc; clear; close all;
disp('🛣️ Testing RL-based ACC model under highway driving conditions...');

% === Load Trained Agent ===
load('bestAgent_ACC_TD3_Improved.mat','agent');

physicsParams = struct('m',1500,'Cd',0.3,'A',2.2,'rho',1.225,...
    'Cr',0.015,'g',9.81,'Ts',0.1,'Tmax',30,'h',1.5,'d0',5);
env = ACCEnv(physicsParams);

% === Custom Highway Scenario Setup ===
env = ACCEnv();
env.egoVel = 20 / 3.6;  % Convert km/h to m/s
env.leadVel = 22 / 3.6;
env.egoPos = 0;
env.leadPos = 0;
env.Cr = 0.015;  % Dry road rolling resistance
env.Tmax = 30;

% === Lead Vehicle Behavior ===
leadAccelProfile = [0.5*ones(1,50), -0.3*ones(1,50), 0.2*ones(1,100), -0.4*ones(1,100)];
leadAccelProfile = leadAccelProfile(1:300);  % Ensure 300 steps

% === Simulation ===
simOptions = rlSimulationOptions('MaxSteps', 300);
experience = sim(env, agent, simOptions);

% === Extract Data ===
ts = experience.Observation.observations;
timeVec = ts.Time;
dataMat = squeeze(ts.Data)';
egoVel = dataMat(:,2);
leadVel = dataMat(:,4);
followingDistance = dataMat(:,3) - dataMat(:,1);

% === Plot Speed Profiles ===
figure;
plot(timeVec, egoVel * 3.6, 'b', 'LineWidth', 1.5); hold on;
plot(timeVec, leadVel * 3.6, 'r--', 'LineWidth', 1.5);
title('Speed Profiles of Ego and Lead Vehicles');
xlabel('Time (s)');
ylabel('Speed (km/h)');
legend('Ego Vehicle', 'Lead Vehicle');
grid on;

% === Plot Following Distance ===
figure;
plot(timeVec, followingDistance, 'k', 'LineWidth', 1.5);
title('Following Distance Over Time');
xlabel('Time (s)');
ylabel('Distance (m)');
grid on;

disp('✅ Highway scenario test completed.');
