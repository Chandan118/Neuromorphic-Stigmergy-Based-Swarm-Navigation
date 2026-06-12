classdef MARLController < handle
    % MARLController Multi-Agent Reinforcement Learning surrogate
    % Uses a simplified Q-learning heuristic to map states to actions.
    
    properties
        QTable
        Epsilon
        Alpha
        Gamma
        
        LastState
        LastActionIdx
    end
    
    methods
        function obj = MARLController()
            % Discretized state space: 
            % Lidar Left (2), Lidar Center (2), Lidar Right (2), Pheromone (2)
            % Total states: 2x2x2x2 = 16
            % Actions: 3 (Forward, Turn Left, Turn Right)
            obj.QTable = zeros(16, 3);
            
            obj.Epsilon = 0.1; % Exploration rate
            obj.Alpha = 0.1;   % Learning rate
            obj.Gamma = 0.9;   % Discount factor
            
            obj.LastState = 1;
            obj.LastActionIdx = 1;
        end
        
        function stateIdx = getState(obj, noisyLidar, pheroLevel)
            lLidar = mean(noisyLidar(1:3)) > 1.0; % 1 if clear, 0 if blocked
            cLidar = mean(noisyLidar(4:5)) > 1.0;
            rLidar = mean(noisyLidar(6:8)) > 1.0;
            pLevel = pheroLevel > 0.5; % 1 if high, 0 if low
            
            % Binary to decimal + 1 (1 to 16)
            stateIdx = lLidar*8 + cLidar*4 + rLidar*2 + pLevel*1 + 1;
        end
        
        function [tauL, tauR] = stepController(obj, noisyLidar, pheroLevel, dt)
            currentState = obj.getState(noisyLidar, pheroLevel);
            
            % Reward formulation
            reward = -0.1; % living penalty
            if mean(noisyLidar) < 0.5
                reward = -10.0; % collision penalty
            elseif pheroLevel > 0.5
                reward = 5.0; % finding trail reward
            end
            
            % Update Q-Table
            maxNextQ = max(obj.QTable(currentState, :));
            obj.QTable(obj.LastState, obj.LastActionIdx) = ...
                obj.QTable(obj.LastState, obj.LastActionIdx) + ...
                obj.Alpha * (reward + obj.Gamma * maxNextQ - obj.QTable(obj.LastState, obj.LastActionIdx));
            
            % Select Action (Epsilon-Greedy)
            if rand() < obj.Epsilon
                actionIdx = randi(3);
            else
                [~, actionIdx] = max(obj.QTable(currentState, :));
            end
            
            % Execute Action
            tauL = 0; tauR = 0;
            if actionIdx == 1 % Forward
                tauL = 0.05; tauR = 0.05;
            elseif actionIdx == 2 % Turn Left
                tauL = -0.02; tauR = 0.05;
            elseif actionIdx == 3 % Turn Right
                tauL = 0.05; tauR = -0.02;
            end
            
            % Save state
            obj.LastState = currentState;
            obj.LastActionIdx = actionIdx;
        end
    end
end
