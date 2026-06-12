classdef SensorSuite < handle
    % SensorSuite Simulates noisy IMU and Lidar sensors for T-RO rigor
    % Injects Gaussian noise and drift.
    
    properties
        LidarRange          % Maximum range of lidar (m)
        LidarNumBeams       % Number of simulated beams
        LidarNoiseStd       % Std deviation of lidar distance noise
        
        IMUHeadingNoiseStd  % Std deviation of IMU heading noise (rad)
        IMUDriftRate        % Drift rate of IMU (rad/s)
        
        CurrentDrift        % Accumulated drift error
    end
    
    methods
        function obj = SensorSuite()
            % Initialize with realistic noise parameters
            obj.LidarRange = 2.0; % 2 meters
            obj.LidarNumBeams = 8; % 8 directions (e.g. 45 deg intervals)
            obj.LidarNoiseStd = 0.05; % 5cm noise
            
            obj.IMUHeadingNoiseStd = 0.02; % ~1 degree noise
            obj.IMUDriftRate = 0.001; % small drift per second
            
            obj.CurrentDrift = 0;
        end
        
        function [noisyDistances] = simulateLidar(obj, robotPos, robotHeading, terrain)
            % Cast rays and find intersection with terrain, adding noise
            noisyDistances = zeros(1, obj.LidarNumBeams);
            angles = linspace(0, 2*pi, obj.LidarNumBeams+1);
            angles = angles(1:end-1) + robotHeading;
            
            for i = 1:obj.LidarNumBeams
                % Simple raycasting logic
                dist = obj.LidarRange;
                rayAng = angles(i);
                
                % Step through the grid
                stepSize = 1.0 / terrain.Resolution;
                for d = 0:stepSize:obj.LidarRange
                    checkPos = robotPos + [d*cos(rayAng), d*sin(rayAng)];
                    if terrain.checkCollision(checkPos, 0)
                        dist = d;
                        break;
                    end
                end
                
                % Add noise
                dist = dist + randn() * obj.LidarNoiseStd;
                dist = max(0, min(dist, obj.LidarRange)); % Clamp to realistic bounds
                
                noisyDistances(i) = dist;
            end
        end
        
        function [noisyHeading] = simulateIMU(obj, trueHeading, dt)
            % Adds Gaussian noise and drift to the heading
            obj.CurrentDrift = obj.CurrentDrift + randn() * obj.IMUDriftRate * dt;
            noise = randn() * obj.IMUHeadingNoiseStd;
            
            noisyHeading = wrapToPi(trueHeading + noise + obj.CurrentDrift);
        end
    end
end
