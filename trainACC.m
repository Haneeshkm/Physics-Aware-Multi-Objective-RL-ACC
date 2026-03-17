function trainACC()
    % Initialize environment and agent
    env = ACCSimulator();
    agent = DQNAgent();
    
    % Define all scenarios
    scenarios = {
        HighwayScenario(), ...  % Primary scenario
        % Add other scenarios here (e.g., LaneChangeScenario)
    };
    
    % Training parameters
    num_episodes = 2000;
    max_steps = 200;
    save_interval = 100;
    
    % Training loop
    for ep = 1:num_episodes
        % Select scenario
        scenario_idx = mod(ep-1, numel(scenarios)) + 1;
        scenario = scenarios{scenario_idx};
        
        % Reset environment
        [state, lead_vehicle, road] = env.reset(scenario);
        total_reward = 0;
        
        for step = 1:max_steps
            % Get action from agent
            action = agent.choose_action(state);
            
            % Simulate environment
            [next_state, reward, done] = env.step(state, action, lead_vehicle, road);
            
            % Store experience
            agent.store_experience(state, action, reward, next_state, done);
            
            % Train agent
            agent.learn();
            
            % Update state
            state = next_state;
            total_reward = total_reward + reward;
            
            % Check termination
            if done
                break;
            end
        end
        
        % Update target network periodically
        if mod(ep, 10) == 0
            agent.updateTargetNetwork();
        end
        
        % Save model periodically
        if mod(ep, save_interval) == 0
            save(sprintf('acc_model_ep%d.mat', ep), 'agent');
        end
        
        % Log progress
        fprintf('Episode %d/%d (Scenario %d): Reward = %.2f, Steps = %d, Epsilon = %.3f\n', ...
            ep, num_episodes, scenario_idx, total_reward, step, agent.epsilon);
    end
    
    % Save final model
    save('trained_ACC_final.mat', 'agent');
end