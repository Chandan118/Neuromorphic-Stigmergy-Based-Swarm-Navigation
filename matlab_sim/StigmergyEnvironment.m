classdef StigmergyEnvironment < handle
    % StigmergyEnvironment Models pheromone diffusion and evaporation using PDEs
    
    properties
        GridSize        % Physical size of the environment [width, height] in meters
        Resolution      % Grid resolution (cells per meter)
        PheromoneGrid   % 2D array storing pheromone concentration
        
        DiffusionRate   % D coefficient in PDE
        EvaporationRate % Gamma coefficient in PDE
        
        XGrid           % X coordinates
        YGrid           % Y coordinates
    end
    
    methods
        function obj = StigmergyEnvironment(size, res, diffRate, evapRate)
            if nargin < 1, size = [10, 10]; end
            if nargin < 2, res = 20; end
            if nargin < 3, diffRate = 0.01; end
            if nargin < 4, evapRate = 0.005; end
            obj.GridSize = size;
            obj.Resolution = res;
            
            nx = round(size(1) * res);
            ny = round(size(2) * res);
            
            obj.PheromoneGrid = zeros(ny, nx);
            
            [obj.XGrid, obj.YGrid] = meshgrid(linspace(0, size(1), nx), linspace(0, size(2), ny));
            
            obj.DiffusionRate = diffRate;
            obj.EvaporationRate = evapRate;
        end
        
        function stepPDE(obj, dt)
            % stepPDE updates the pheromone field using Finite Difference Method (FDM)
            % dC/dt = D * Laplacian(C) - gamma * C
            
            % Using del2 function which approximates 0.25 * Laplacian * h^2
            % Since del2(U) = (U(i+1,j) + U(i-1,j) + U(i,j+1) + U(i,j-1) - 4U(i,j))/4
            % Laplacian(U) approx 4 * del2(U) / h^2
            
            h = 1 / obj.Resolution; % Grid spacing
            
            % Approximate Laplacian
            L_C = 4 * del2(obj.PheromoneGrid) / (h^2);
            
            % PDE Update
            dC_dt = obj.DiffusionRate * L_C - obj.EvaporationRate * obj.PheromoneGrid;
            
            obj.PheromoneGrid = obj.PheromoneGrid + dC_dt * dt;
            
            % Ensure no negative pheromones due to numerical instability
            obj.PheromoneGrid = max(0, obj.PheromoneGrid);
        end
        
        function depositPheromone(obj, position, amount)
            % Deposit pheromone at a specific (x,y) location
            idxX = max(1, min(size(obj.PheromoneGrid, 2), round(position(1) * obj.Resolution) + 1));
            idxY = max(1, min(size(obj.PheromoneGrid, 1), round(position(2) * obj.Resolution) + 1));
            
            obj.PheromoneGrid(idxY, idxX) = obj.PheromoneGrid(idxY, idxX) + amount;
        end
        
        function val = sensePheromone(obj, position)
            % Sense pheromone concentration at (x,y)
            % Interpolates from the grid
            if position(1) < 0 || position(1) > obj.GridSize(1) || ...
               position(2) < 0 || position(2) > obj.GridSize(2)
                val = 0;
                return;
            end
            
            val = interp2(obj.XGrid, obj.YGrid, obj.PheromoneGrid, position(1), position(2), 'linear', 0);
        end
        
        function visualize(obj)
            % Visualize the current pheromone field
            imagesc(obj.XGrid(1,:), obj.YGrid(:,1)', obj.PheromoneGrid);
            set(gca, 'YDir', 'normal');
            colormap(hot);
            colorbar;
            title('Stigmergic Pheromone Concentration');
            xlabel('X (m)');
            ylabel('Y (m)');
        end
    end
end
