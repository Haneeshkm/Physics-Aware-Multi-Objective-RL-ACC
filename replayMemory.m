classdef replayMemory < handle
    properties
        capacity;
        state_dim;
        count;
        memory;
    end
    
    methods
        function obj = replayMemory(capacity, state_dim)
            obj.capacity = capacity;
            obj.state_dim = state_dim;
            obj.count = 0;
            obj.memory = struct(...
                'state', zeros(state_dim, capacity), ...
                'action', zeros(1, capacity), ...
                'reward', zeros(1, capacity), ...
                'next_state', zeros(state_dim, capacity), ...
                'done', zeros(1, capacity));
        end
        
        function add(obj, state, action, reward, next_state, done)
            idx = mod(obj.count, obj.capacity) + 1;
            obj.memory.state(:, idx) = state(:);
            obj.memory.action(idx) = action;
            obj.memory.reward(idx) = reward;
            obj.memory.next_state(:, idx) = next_state(:);
            obj.memory.done(idx) = done;
            obj.count = obj.count + 1;
        end
        
        function [states, actions, rewards, next_states, dones] = sample(obj, batch_size)
            max_mem = min(obj.count, obj.capacity);
            indices = randperm(max_mem, min(batch_size, max_mem));
            states = obj.memory.state(:, indices)';
            actions = obj.memory.action(indices)';
            rewards = obj.memory.reward(indices)';
            next_states = obj.memory.next_state(:, indices)';
            dones = obj.memory.done(indices)';
        end
        
        function s = size(obj)
            s = min(obj.count, obj.capacity);
        end
    end
end