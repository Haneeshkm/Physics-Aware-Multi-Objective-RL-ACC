%% Main Training Script: Train_ACC_DQN.m
clc; clear; close all;
disp('🚀 High-Efficiency Physics-Aware ACC Training');

% Environment parameters
physicsParams = struct(...
    'm', 1500, ...          % Vehicle mass (kg)
    'Cd', 0.3, ...          % Drag coefficient
    'A', 2.2, ...           % Frontal area (m²)
    'rho', 1.225, ...       % Air density (kg/m³)
    'Cr', 0.015, ...        % Rolling resistance coefficient
    'g', 9.81, ...          % Gravity (m/s²)
    'Ts', 0.1, ...          % Sample time (s)
    'Tmax', 30, ...         % Max episode duration (s)
    'h', 1.5, ...           % Time headway (s)
    'd0', 5 ...             % Standstill spacing (m)
);

% Create environment
env = ACCEnv(physicsParams);
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);
disp('✅ Environment initialized');

% DDPG Agent Configuration
agent = createDDPGAgent(obsInfo, actInfo, physicsParams);
disp('🤖 DDPG Agent created');

% Training Configuration
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 2000, ...
    'MaxStepsPerEpisode', physicsParams.Tmax/physicsParams.Ts, ...
    'ScoreAveragingWindowLength', 100, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 15, ...
    'SaveAgentCriteria', 'EpisodeReward', ...
    'SaveAgentValue', 10, ...
    'Verbose', true, ...
    'Plots', 'training-progress', ...
    'UseParallel', false);

% Train Agent
disp('🔥 Training agent...');
trainingStats = train(agent, env, trainOpts);

% Save best agent
bestAgent = getBestAgent(trainingStats.SavedAgents);
save('efficient_acc_agent.mat', 'bestAgent');
disp('💾 Best agent saved');

% Simulate and analyze performance
simOpts = rlSimulationOptions('MaxSteps', physicsParams.Tmax/physicsParams.Ts);
exp = sim(env, bestAgent, simOpts);
analyzePerformance(exp, env);

%% Helper Functions
function agent = createDDPGAgent(obsInfo, actInfo, params)
    % Actor Network
    actorLayers = [
        featureInputLayer(obsInfo.Dimension(1), 'Name', 'state')
        fullyConnectedLayer(256, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        layerNormalizationLayer('Name', 'ln1')
        fullyConnectedLayer(256, 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'action')
        tanhLayer('Name', 'tanh')
        scalingLayer('Name', 'scale', 'Scale', 3, 'Bias', 0)
    ];
    actorOpts = rlRepresentationOptions(...
        'LearnRate', 1e-4, 'L2RegularizationFactor', 1e-5, ...
        'GradientThreshold', 1);
    actor = rlDeterministicActorRepresentation(...
        actorLayers, obsInfo, actInfo, 'Observation', {'state'}, actorOpts);

    % Critic Network
    statePath = [
        featureInputLayer(obsInfo.Dimension(1), 'Name', 'state')
        fullyConnectedLayer(256, 'Name', 's_fc1')
        reluLayer('Name', 's_relu')
    ];
    actionPath = [
        featureInputLayer(actInfo.Dimension(1), 'Name', 'action')
        fullyConnectedLayer(256, 'Name', 'a_fc1')
        reluLayer('Name', 'a_relu')
    ];
    commonPath = [
        concatenationLayer(1, 2, 'Name', 'concat')
        fullyConnectedLayer(256, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(1, 'Name', 'qval')
    ];
    
    criticNet = layerGraph();
    criticNet = addLayers(criticNet, statePath);
    criticNet = addLayers(criticNet, actionPath);
    criticNet = addLayers(criticNet, commonPath);
    criticNet = connectLayers(criticNet, 's_relu', 'concat/in1');
    criticNet = connectLayers(criticNet, 'a_relu', 'concat/in2');
    
    criticOpts = rlRepresentationOptions(...
        'LearnRate', 1e-3, 'L2RegularizationFactor', 1e-4, ...
        'GradientThreshold', 1);
    critic = rlQValueRepresentation(...
        criticNet, obsInfo, actInfo, ...
        'Observation', {'state'}, 'Action', {'action'}, criticOpts);

    % Agent Options
    agentOpts = rlDDPGAgentOptions(...
        'SampleTime', params.Ts, ...
        'TargetSmoothFactor', 1e-3, ...
        'DiscountFactor', 0.995, ...
        'MiniBatchSize', 256, ...
        'ExperienceBufferLength', 1e6, ...
        'NoiseOptions', rl.option.OrnsteinUhlenbeckActionNoise(...
            'Mean', 0, ...
            'MeanAttractionConstant', 0.15, ...
            'StandardDeviation', 0.3, ...
            'StandardDeviationDecayRate', 1e-5));
    
    agent = rlDDPGAgent(actor, critic, agentOpts);
end

function bestAgent = getBestAgent(savedAgents)
    bestReward = -inf;
    bestAgent = [];
    for i = 1:length(savedAgents)
        agentData = load(savedAgents{i});
        if agentData.saved_agent_info.EpisodeReward > bestReward
            bestReward = agentData.saved_agent_info.EpisodeReward;
            bestAgent = agentData.saved_agent;
        end
    end
end

function analyzePerformance(exp, env)
    ts = exp.Observation.observations;
    timeVec = ts.Time;
    data = squeeze(ts.Data)';
    
    % Extract states
    egoPos = data(:,1);
    egoVel = data(:,2);
    leadPos = data(:,3);
    leadVel = data(:,4);
    relVel = data(:,5);
    headway = data(:,6);
    accel = data(:,7);
    roadCond = data(:,8);
    theta = data(:,9);
    
    % Calculate metrics
    gap = leadPos - egoPos;
    jerk = [0; diff(accel)/env.Ts];
    energy = cumsum(abs(accel) .* env.m .* abs(egoVel) * env.Ts);
    
    % Create comprehensive performance figure
    figure('Position', [100, 100, 1200, 900], 'Name', 'ACC Performance Analysis');
    
    % Position plot
    subplot(5,1,1);
    plot(timeVec, egoPos, 'b', 'LineWidth', 1.5);
    hold on;
    plot(timeVec, leadPos, 'r', 'LineWidth', 1.5);
    ylabel('Position (m)');
    title('Vehicle Trajectories');
    legend('Ego', 'Lead', 'Location', 'best');
    grid on;
    
    % Velocity plot
    subplot(5,1,2);
    plot(timeVec, egoVel, 'b', 'LineWidth', 1.5);
    hold on;
    plot(timeVec, leadVel, 'r', 'LineWidth', 1.5);
    ylabel('Velocity (m/s)');
    title('Velocity Profiles');
    grid on;
    
    % Gap and safety
    subplot(5,1,3);
    plot(timeVec, gap, 'b', 'LineWidth', 1.5);
    hold on;
    desiredGap = env.h * egoVel + env.d0;
    plot(timeVec, desiredGap, 'g--', 'LineWidth', 1.5);
    plot([min(timeVec), max(timeVec)], [env.minGap, env.minGap], 'r--', 'LineWidth', 1.5);
    ylabel('Gap (m)');
    title('Following Distance');
    legend('Actual', 'Desired', 'Min Safe', 'Location', 'best');
    grid on;
    
    % Control inputs
    subplot(5,1,4);
    yyaxis left;
    plot(timeVec, accel, 'b', 'LineWidth', 1.5);
    ylabel('Accel (m/s²)');
    yyaxis right;
    plot(timeVec, jerk, 'm', 'LineWidth', 1.5);
    ylabel('Jerk (m/s³)');
    title('Control Inputs');
    grid on;
    
    % Energy and environment
    subplot(5,1,5);
    yyaxis left;
    plot(timeVec, energy, 'k', 'LineWidth', 1.5);
    ylabel('Energy (J)');
    yyaxis right;
    plot(timeVec, roadCond, 'c', 'LineWidth', 1.5);
    hold on;
    plot(timeVec, rad2deg(theta), 'g', 'LineWidth', 1.5);
    ylabel('Road Cond / Grade (°)');
    title('Energy Consumption & Road Conditions');
    legend('Energy', 'Friction Coeff', 'Road Grade', 'Location', 'best');
    grid on;
    
    % Safety analysis
    minGap = min(gap);
    maxJerk = max(abs(jerk));
    avgHeadway = mean(headway);
    collisions = sum(gap < env.minGap);
    
    fprintf('\n=== Safety and Performance Analysis ===\n');
    fprintf('Minimum gap: %.2f m (safe > %.1f m)\n', minGap, env.minGap);
    fprintf('Maximum jerk: %.2f m/s³ (comfortable < 10 m/s³)\n', maxJerk);
    fprintf('Average time headway: %.2f s (recommended 1.5-2.5s)\n', avgHeadway);
    fprintf('Collision events: %d\n', collisions);
    fprintf('Total energy consumption: %.2f kJ\n', energy(end)/1000);
    
    if minGap < env.minGap
        fprintf('⚠️ Safety violation detected!\n');
    else
        fprintf('✅ Safety maintained\n');
    end
end