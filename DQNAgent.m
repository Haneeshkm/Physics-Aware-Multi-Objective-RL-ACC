classdef DQNAgent < handle
    properties
        state_dim = 5;
        action_dim = 3;
        hidden_units = [128, 64];
        
        gamma = 0.995;
        lr = 0.0005;
        epsilon = 1.0;
        epsilon_decay = 0.003;
        epsilon_min = 0.01;
        batch_size = 64;
        memory_capacity = 10000;
        
        online_net;
        target_net;
        memory;
    end
    
    methods
        function agent = DQNAgent()
            agent.online_net = agent.buildNetwork();
            agent.target_net = agent.buildNetwork();
            agent.memory = replayMemory(agent.memory_capacity, agent.state_dim);
            agent.updateTargetNetwork();
        end
        
        function net = buildNetwork(agent)
            layers = [
                featureInputLayer(agent.state_dim, 'Name', 'input')
                fullyConnectedLayer(agent.hidden_units(1), 'Name', 'fc1')
                reluLayer('Name', 'relu1')
                fullyConnectedLayer(agent.hidden_units(2), 'Name', 'fc2')
                reluLayer('Name', 'relu2')
                fullyConnectedLayer(agent.action_dim, 'Name', 'output')
            ];
            net = dlnetwork(layers);
        end
        
        function action = choose_action(agent, state)
            if rand() < agent.epsilon
                action = randi([-1, 1]);
            else
                state_dl = dlarray(single(state), 'CB');
                q_values = predict(agent.online_net, state_dl);  % Changed to predict
                [~, idx] = max(extractdata(q_values));
                action = idx - 2;
            end
        end
        
        function store_experience(agent, state, action, reward, next_state, done)
            agent.memory.add(state, action, reward, next_state, done);
        end
        
        function learn(agent)
            if agent.memory.size < agent.batch_size
                return
            end
            
            [states, actions, rewards, next_states, dones] = ...
                agent.memory.sample(agent.batch_size);
            
            states_dl = dlarray(single(states'), 'CB');
            next_states_dl = dlarray(single(next_states'), 'CB');
            
            % Use predict instead of forward
            q_next = predict(agent.online_net, next_states_dl);
            [~, max_actions] = max(q_next, [], 1);
            q_target_next = predict(agent.target_net, next_states_dl);
            
            batch_size = size(rewards, 1);
            q_target = rewards + agent.gamma .* (1 - dones) .* ...
                extractdata(q_target_next(sub2ind(size(q_target_next), max_actions, 1:batch_size)))';
            
            q_values = predict(agent.online_net, states_dl);
            targets = q_values;
            for i = 1:batch_size
                action_idx = actions(i) + 2;
                targets(action_idx, i) = q_target(i);
            end
            
            [loss, gradients] = dlfeval(@modelLoss, agent.online_net, states_dl, targets);
            agent.online_net = dlupdate(@(w, g) w - agent.lr * g, agent.online_net.Learnables, gradients);
            
            agent.epsilon = max(agent.epsilon_min, ...
                agent.epsilon * exp(-agent.epsilon_decay));
        end
        
        function updateTargetNetwork(agent)
            % Manually copy learnable parameters
            targetLearnables = agent.target_net.Learnables;
            onlineLearnables = agent.online_net.Learnables;
            
            for i = 1:height(targetLearnables)
                targetLearnables.Value{i} = onlineLearnables.Value{i};
            end
            
            agent.target_net.Learnables = targetLearnables;
        end
    end
end

function [loss, gradients] = modelLoss(net, states_dl, targets)
    predictions = predict(net, states_dl);  % Changed to predict
    loss = mse(predictions, targets);
    gradients = dlgradient(loss, net.Learnables);
end