classdef TerrainGenerator < handle
    % TerrainGenerator Creates and manages complex simulated environments
    % Supports creating random cluttered terrain or specific mazes (e.g. Martian landscape)
    
    properties
        GridSize        % [width, height] in meters
        Resolution      % Grid resolution (cells per meter)
        OccupancyGrid   % Binary grid: 1 is obstacle, 0 is free space
        
        XGrid
        YGrid
    end
    
    methods
        function obj = TerrainGenerator(size, res)
            if nargin < 1, size = [10, 10]; end
            if nargin < 2, res = 20; end
            obj.GridSize = size;
            obj.Resolution = res;
            
            nx = round(size(1) * res);
            ny = round(size(2) * res);
            
            obj.OccupancyGrid = zeros(ny, nx);
            [obj.XGrid, obj.YGrid] = meshgrid(linspace(0, size(1), nx), linspace(0, size(2), ny));
        end
        
        function generateCluttered(obj, numObstacles, maxRadius)
            % Generate a cluttered environment with random circular obstacles
            for i = 1:numObstacles
                cx = rand() * obj.GridSize(1);
                cy = rand() * obj.GridSize(2);
                r = rand() * maxRadius;
                
                % Mark occupancy
                distances = sqrt((obj.XGrid - cx).^2 + (obj.YGrid - cy).^2);
                obj.OccupancyGrid(distances <= r) = 1;
            end
            
            % Add walls along borders
            obj.OccupancyGrid(1,:) = 1;
            obj.OccupancyGrid(end,:) = 1;
            obj.OccupancyGrid(:,1) = 1;
            obj.OccupancyGrid(:,end) = 1;
        end
        
        function generateMaze(obj)
            % Generates a simple maze layout
            % Empty for now, will implement a basic structured layout
            nx = size(obj.OccupancyGrid, 2);
            ny = size(obj.OccupancyGrid, 1);
            
            % Borders
            obj.OccupancyGrid(1,:) = 1; obj.OccupancyGrid(end,:) = 1;
            obj.OccupancyGrid(:,1) = 1; obj.OccupancyGrid(:,end) = 1;
            
            % Internal walls
            obj.OccupancyGrid(round(ny/3):round(2*ny/3), round(nx/3)) = 1;
            obj.OccupancyGrid(round(ny/3), round(nx/3):round(2*nx/3)) = 1;
            obj.OccupancyGrid(round(2*ny/3), round(nx/2):round(5*nx/6)) = 1;
        end
        
        function isColliding = checkCollision(obj, position, robotRadius)
            % Check if a given circular robot is in collision with an obstacle
            if position(1) < 0 || position(1) > obj.GridSize(1) || ...
               position(2) < 0 || position(2) > obj.GridSize(2)
                isColliding = true;
                return;
            end
            
            idxX = round(position(1) * obj.Resolution) + 1;
            idxY = round(position(2) * obj.Resolution) + 1;
            
            % Simple boundary check
            idxX = max(1, min(size(obj.OccupancyGrid, 2), idxX));
            idxY = max(1, min(size(obj.OccupancyGrid, 1), idxY));
            
            % Strict grid occupancy check (no radius expansion for speed right now)
            isColliding = obj.OccupancyGrid(idxY, idxX) > 0.5;
        end
        
        function visualize(obj)
            % Plot the terrain
            colormap(flipud(gray));
            imagesc(obj.XGrid(1,:), obj.YGrid(:,1)', obj.OccupancyGrid);
            set(gca, 'YDir', 'normal');
            title('Terrain Occupancy Grid');
            xlabel('X (m)');
            ylabel('Y (m)');
        end
    end
end
