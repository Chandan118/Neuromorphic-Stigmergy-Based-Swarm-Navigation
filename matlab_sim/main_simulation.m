function [results] = main_simulation(N, controllerType, useStigmergy, maxSteps, dt, visualize)
    % MAIN_SIMULATION Core loop for T-RO rigorous swarm simulation
    % N: Number of robots
    % controllerType: 'SNN', 'PSO', or 'MARL'
    % useStigmergy: Boolean, whether to enable PDE stigmergy
    % maxSteps: Maximum simulation steps
    % dt: Time step (seconds)
    % visualize: Boolean, true to plot animation
    
    if nargin < 1, N = 10; end
    if nargin < 2, controllerType = 'SNN'; end
    if nargin < 3, useStigmergy = true; end
    if nargin < 4, maxSteps = 500; end
    if nargin < 5, dt = 0.1; end
    if nargin < 6, visualize = true; end
    
    % Initialize Environment
    gridSize = [10, 10]; % 10x10 meters
    resolution = 20; % 20 cells per meter (5cm cells)
    diffRate = 0.01; % Diffusion D
    evapRate = 0.005; % Evaporation Gamma
    
    env = StigmergyEnvironment(gridSize, resolution, diffRate, evapRate);
    
    % Initialize Terrain (Maze)
    terrain = TerrainGenerator(gridSize, resolution);
    terrain.generateMaze();
    
    % Initialize Sensors
    sensors = SensorSuite();
    
    % Initialize Robots and Controllers
    robots = cell(1, N);
    controllers = cell(1, N);
    
    % Define start and goal regions
    startRegion = [1.0, 1.0];
    goalRegion = [9.0, 9.0];
    
    for i = 1:N
        % Randomize starting position slightly around startRegion
        pos = startRegion + (rand(1,2) - 0.5) * 1.0;
        heading = rand() * 2 * pi - pi;
        
        robots{i} = RigidBodyRobot(i, pos, heading);
        
        % Assign requested controller
        if strcmp(controllerType, 'SNN')
            controllers{i} = SNNController(9, 20, 2);
        elseif strcmp(controllerType, 'PSO')
            controllers{i} = ParticleSwarmController(pos);
        elseif strcmp(controllerType, 'MARL')
            controllers{i} = MARLController();
        else
            error('Invalid controllerType. Use SNN, PSO, or MARL.');
        end
    end
    
    % Metrics tracking
    metrics.completionTimes = NaN(1, N);
    metrics.totalSpikes = 0; % Only relevant for SNN
    
    % Visualization setup
    if visualize
        figure('Name', sprintf('Simulation: %s', controllerType), 'NumberTitle', 'off');
        colormap(flipud(gray));
    end
    
    % Main Physics Loop
    for step = 1:maxSteps
        % 1. Step the Stigmergy PDE
        if useStigmergy
            env.stepPDE(dt);
        end
        
        for i = 1:N
            r = robots{i};
            
            % Skip if already reached goal
            if ~isnan(metrics.completionTimes(i))
                continue;
            end
            
            % Check if goal reached
            distToGoal = norm(r.Position - goalRegion);
            if distToGoal < 0.5
                metrics.completionTimes(i) = step * dt;
                continue;
            end
            
            % 2. Read Sensors (injects noise)
            noisyLidar = sensors.simulateLidar(r.Position, r.Heading, terrain);
            noisyHeading = sensors.simulateIMU(r.Heading, dt);
            
            % 3. Read Stigmergy (Pheromones)
            pheroConcentration = 0;
            pheroGradient = [0, 0];
            if useStigmergy
                pheroConcentration = env.sensePheromone(r.Position);
                % Simple gradient estimation for PSO
                dx = env.sensePheromone(r.Position + [0.1, 0]) - env.sensePheromone(r.Position - [0.1, 0]);
                dy = env.sensePheromone(r.Position + [0, 0.1]) - env.sensePheromone(r.Position - [0, 0.1]);
                pheroGradient = [dx, dy];
                
                % Deposit pheromones
                env.depositPheromone(r.Position, 0.5 * dt);
            end
            
            % 4. Controller logic
            if strcmp(controllerType, 'SNN')
                inputRates = [max(0, 1 - noisyLidar / sensors.LidarRange)'; pheroConcentration];
                motorSpikes = controllers{i}.stepNetwork(inputRates, dt);
                [tauL, tauR] = controllers{i}.decodeMotorSpikes(motorSpikes);
                metrics.totalSpikes = metrics.totalSpikes + sum(motorSpikes);
                
            elseif strcmp(controllerType, 'PSO')
                [tauL, tauR] = controllers{i}.stepController(r.Position, r.Heading, noisyLidar, pheroConcentration, pheroGradient, dt);
                
            elseif strcmp(controllerType, 'MARL')
                [tauL, tauR] = controllers{i}.stepController(noisyLidar, pheroConcentration, dt);
            end
            
            % 5. Update Rigid Body Dynamics
            r.applyWheelTorques(dt, tauL, tauR);
            
            % Handle physical collisions
            if terrain.checkCollision(r.Position, r.WheelBase/2)
                r.Velocity = 0;
                % Revert position slightly
                if size(r.Trajectory, 1) > 1
                    r.Position = r.Trajectory(end-1, :);
                end
            end
        end
        
        % Visualization
        if visualize && mod(step, 10) == 0
            clf; hold on;
            imagesc(terrain.XGrid(1,:), terrain.YGrid(:,1)', terrain.OccupancyGrid);
            if useStigmergy
                pheroAlpha = min(1, env.PheromoneGrid / 2);
                h = imagesc(terrain.XGrid(1,:), terrain.YGrid(:,1)', env.PheromoneGrid);
                set(h, 'AlphaData', pheroAlpha);
            end
            
            plot(startRegion(1), startRegion(2), 'gx', 'MarkerSize', 15, 'LineWidth', 2);
            plot(goalRegion(1), goalRegion(2), 'rx', 'MarkerSize', 15, 'LineWidth', 2);
            
            for i = 1:N
                r = robots{i};
                plot(r.Position(1), r.Position(2), 'bo', 'MarkerFaceColor', 'b');
                plot([r.Position(1), r.Position(1) + 0.2*cos(r.Heading)], ...
                     [r.Position(2), r.Position(2) + 0.2*sin(r.Heading)], 'r-');
            end
            
            set(gca, 'YDir', 'normal');
            xlim([0, gridSize(1)]); ylim([0, gridSize(2)]);
            title(sprintf('Swarm Simulation [%s] - Step %d', controllerType, step));
            drawnow;
        end
    end
    
    results.metrics = metrics;
    results.robots = robots;
end
