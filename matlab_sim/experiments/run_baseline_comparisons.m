% run_baseline_comparisons.m
% T-RO Required Experiment: Direct Comparisons
% Benchmarks the Neuromorphic approach against PSO and MARL baselines.

addpath('../');

N = 50; 
maxSteps = 1000;
dt = 0.1;

controllers = {'SNN', 'PSO', 'MARL'};
numControllers = length(controllers);

successRates = zeros(1, numControllers);
meanTimes = zeros(1, numControllers);

for i = 1:numControllers
    ctype = controllers{i};
    fprintf('Running %s Approach...\n', ctype);
    
    % Run simulation
    results = main_simulation(N, ctype, true, maxSteps, dt, false);
    
    % Calculate Metrics
    valid = results.metrics.completionTimes(~isnan(results.metrics.completionTimes));
    successRates(i) = length(valid) / N * 100;
    
    if isempty(valid)
        meanTimes(i) = maxSteps * dt;
    else
        meanTimes(i) = mean(valid);
    end
end

% Plot 3-Way Comparison
figure('Name', 'Baseline Comparisons (T-RO Rigor)', 'NumberTitle', 'off');

subplot(1, 2, 1);
bar(successRates);
set(gca, 'xticklabel', controllers);
ylabel('Success Rate (%)');
title('Comparison: Mission Success');
grid on;

subplot(1, 2, 2);
bar(meanTimes);
set(gca, 'xticklabel', controllers);
ylabel('Mean Completion Time (s)');
title('Comparison: Speed / Efficiency');
grid on;

fprintf('Baseline comparisons complete. The SNN should outperform or match MARL with lower overhead.\n');
