classdef RigidBodyRobot < handle
    % RigidBodyRobot Differential drive robot dynamics for swarm simulation
    % Incorporates mass, inertia, and non-holonomic constraints.
    
    properties
        ID                  % Robot ID
        Position            % [x, y] in meters
        Heading             % Theta in radians
        Velocity            % Linear velocity v (m/s)
        AngularVelocity     % Angular velocity omega (rad/s)
        
        % Physical parameters
        Mass                % Robot mass (kg)
        Inertia             % Moment of inertia (kg*m^2)
        WheelRadius         % Radius of the wheels (m)
        WheelBase           % Distance between wheels (m)
        
        MaxVelocity         % Maximum linear velocity
        MaxAngularVelocity  % Maximum angular velocity
        
        % Memory for SNN and previous states
        Trajectory
    end
    
    methods
        function obj = RigidBodyRobot(id, initPos, initHeading)
            if nargin < 1, id = 1; end
            if nargin < 2, initPos = [0, 0]; end
            if nargin < 3, initHeading = 0; end
            obj.ID = id;
            obj.Position = initPos;
            obj.Heading = initHeading;
            obj.Velocity = 0;
            obj.AngularVelocity = 0;
            
            % Default parameter values based on typical micro-swarm bots
            obj.Mass = 0.5; % 500g
            obj.Inertia = 0.005; 
            obj.WheelRadius = 0.03; % 3cm
            obj.WheelBase = 0.1; % 10cm
            
            obj.MaxVelocity = 0.5; % 0.5 m/s
            obj.MaxAngularVelocity = pi; % 180 deg/s
            
            obj.Trajectory = initPos;
        end
        
        function updateDynamics(obj, dt, force, torque)
            % Update robot dynamics based on applied force and torque
            % v_dot = F / m
            % omega_dot = Tau / I
            
            linAcc = force / obj.Mass;
            angAcc = torque / obj.Inertia;
            
            % Update velocities with basic Euler integration
            obj.Velocity = obj.Velocity + linAcc * dt;
            obj.AngularVelocity = obj.AngularVelocity + angAcc * dt;
            
            % Clamp velocities
            obj.Velocity = max(min(obj.Velocity, obj.MaxVelocity), -obj.MaxVelocity);
            obj.AngularVelocity = max(min(obj.AngularVelocity, obj.MaxAngularVelocity), -obj.MaxAngularVelocity);
            
            % Non-holonomic update (differential drive kinematics)
            % dx = v * cos(theta)
            % dy = v * sin(theta)
            % dtheta = omega
            
            obj.Heading = obj.Heading + obj.AngularVelocity * dt;
            % Wrap heading to [-pi, pi]
            obj.Heading = wrapToPi(obj.Heading);
            
            dx = obj.Velocity * cos(obj.Heading) * dt;
            dy = obj.Velocity * sin(obj.Heading) * dt;
            
            obj.Position = obj.Position + [dx, dy];
            obj.Trajectory(end+1, :) = obj.Position;
        end
        
        function [vL, vR] = computeWheelSpeeds(obj)
            % Convert linear and angular velocity to left/right wheel speeds (rad/s)
            vL = (obj.Velocity - (obj.AngularVelocity * obj.WheelBase / 2)) / obj.WheelRadius;
            vR = (obj.Velocity + (obj.AngularVelocity * obj.WheelBase / 2)) / obj.WheelRadius;
        end
        
        function applyWheelTorques(obj, dt, tauL, tauR)
            % Update dynamics given direct wheel torques
            force = (tauL + tauR) / obj.WheelRadius;
            torque = ((tauR - tauL) * obj.WheelBase) / (2 * obj.WheelRadius);
            obj.updateDynamics(dt, force, torque);
        end
        
    end
end
