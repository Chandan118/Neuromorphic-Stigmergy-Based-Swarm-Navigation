% run_scalability_benchmarks.m
% T-RO Required Experiment: Scalability Benchmarks
% Runs the maze with N=10, 50, 250, and 500 robots.
% Measures mission completion time and computational overhead.

addpath('../'); % Ensure parent dir classes are available

swarmSizes = [10, 50, 250, 500];
numTrials = 1; % Set higher for actual paper data

avgCompletionTime = zeros(1, length(swarmSizes));
avgSpikeCount = zeros(1, length(swarmSizes));

maxSteps = 1000;
dt = 0.1;

for i = 1:length(swarmSizes)
    N = swarmSizes(i);
    fprintf('Running Scalability Benchmark for N = %d...\n', N);
    
    trialCompletionTimes = [];
    trialSpikeCounts = [];
    
    for t = 1:numTrials
        fprintf('  Trial %d/%d\n', t, numTrials);
        % Run with SNN and Stigmergy (true) without visualization (false)
        results = main_simulation(N, 'SNN', true, maxSteps, dt, false);
        
        % Calculate metrics
        validTimes = results.metrics.completionTimes(~isnan(results.metrics.completionTimes));
        if isempty(validTimes)
            trialCompletionTimes(end+1) = maxSteps * dt; % penalty if none reached
        else
            trialCompletionTimes(end+1) = mean(validTimes);
        end
        trialSpikeCounts(end+1) = results.metrics.totalSpikes;
    end
    
    avgCompletionTime(i) = mean(trialCompletionTimes);
    avgSpikeCount(i) = mean(trialSpikeCounts);
end

% Plot Scalability Results
figure('Name', 'Scalability Benchmark Results', 'NumberTitle', 'off');
subplot(1, 2, 1);
plot(swarmSizes, avgCompletionTime, '-o', 'LineWidth', 2);
xlabel('Swarm Size (N)');
ylabel('Avg Mission Completion Time (s)');
title('Scalability: Completion Time');
grid on;

subplot(1, 2, 2);
plot(swarmSizes, avgSpikeCount, '-o', 'LineWidth', 2);
xlabel('Swarm Size (N)');
ylabel('Total Computational Overhead (Spikes)');
title('Scalability: SNN Efficiency');
grid on;

fprintf('Scalability benchmarks complete.\n');
