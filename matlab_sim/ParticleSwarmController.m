classdef ParticleSwarmController < handle
    % ParticleSwarmController A traditional PSO-inspired swarm controller
    % Acts as a robust baseline surrogate for T-RO comparisons.
    
    properties
        PersonalBestPos
        PersonalBestVal
        
        c1 % Cognitive weight
        c2 % Social/Stigmergy weight
        w  % Inertia weight
        
        VelocityVector
    end
    
    methods
        function obj = ParticleSwarmController(initPos)
            if nargin < 1
                initPos = [0, 0];
            end
            obj.PersonalBestPos = initPos;
            obj.PersonalBestVal = -inf;
            
            obj.c1 = 1.5;
            obj.c2 = 2.0;
            obj.w = 0.7;
            
            obj.VelocityVector = [0, 0];
        end
        
        function [tauL, tauR] = stepController(obj, rPos, rHeading, noisyLidar, pheroLevel, pheroGradient, dt)
            % PSO logic merged with obstacle avoidance
            % pheroGradient is an estimated vector towards higher pheromones (simulating global best)
            
            % Update personal best
            if pheroLevel > obj.PersonalBestVal
                obj.PersonalBestVal = pheroLevel;
                obj.PersonalBestPos = rPos;
            end
            
            % PSO velocity update
            r1 = rand(); r2 = rand();
            cognitive = obj.c1 * r1 * (obj.PersonalBestPos - rPos);
            social = obj.c2 * r2 * pheroGradient;
            
            obj.VelocityVector = obj.w * obj.VelocityVector + cognitive + social;
            
            % Convert desired velocity vector to target heading
            targetHeading = atan2(obj.VelocityVector(2), obj.VelocityVector(1));
            
            % Obstacle Avoidance (Braitenberg) overrides PSO if close
            leftLidar = mean(noisyLidar(1:3));
            rightLidar = mean(noisyLidar(6:8));
            
            avoidanceTurn = 0;
            if leftLidar < 0.8
                avoidanceTurn = -1.0; % Turn right
            elseif rightLidar < 0.8
                avoidanceTurn = 1.0; % Turn left
            end
            
            % Compute steering
            headingError = wrapToPi(targetHeading - rHeading);
            if abs(avoidanceTurn) > 0
                steering = avoidanceTurn;
                speed = 0.01; % slow down
            else
                steering = headingError * 0.5;
                speed = norm(obj.VelocityVector) * 0.1;
                speed = min(0.05, max(0.01, speed));
            end
            
            % Differential drive mixing
            tauL = speed - steering * 0.02;
            tauR = speed + steering * 0.02;
            
            % Clamp
            tauL = max(-0.1, min(0.1, tauL));
            tauR = max(-0.1, min(0.1, tauR));
        end
    end
end
