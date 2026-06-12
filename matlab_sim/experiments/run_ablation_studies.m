% run_ablation_studies.m
% T-RO Required Experiment: Ablation Studies
% Compares performance with and without SNN, and with/without Stigmergy.

addpath('../');

N = 50; % Fixed medium swarm size
maxSteps = 1000;
dt = 0.1;

% Configurations: 
% 1. Full system (SNN + Stigmergy)
% 2. No SNN (PSO Baseline + Stigmergy)
% 3. No Stigmergy (SNN only)
% 4. Baseline (PSO Baseline, No Stigmergy)

configs = {
    'SNN', true,  'Full Neuromorphic Swarm';
    'PSO', true,  'PSO + Stigmergy (No SNN)';
    'SNN', false, 'SNN Only (No Stigmergy)';
    'PSO', false, 'Baseline PSO (No Stigmergy)'
};

completionRates = zeros(1, 4);
meanTimes = zeros(1, 4);

for i = 1:4
    ctype = configs{i, 1};
    useStigmergy = configs{i, 2};
    name = configs{i, 3};
    
    fprintf('Running Ablation: %s...\n', name);
    results = main_simulation(N, ctype, useStigmergy, maxSteps, dt, false);
    
    validTimes = results.metrics.completionTimes(~isnan(results.metrics.completionTimes));
    completionRates(i) = length(validTimes) / N * 100;
    
    if isempty(validTimes)
        meanTimes(i) = maxSteps * dt;
    else
        meanTimes(i) = mean(validTimes);
    end
end

% Plot Results
figure('Name', 'Ablation Study Results', 'NumberTitle', 'off');

subplot(1, 2, 1);
bar(completionRates);
set(gca, 'xticklabel', configs(:,3));
xtickangle(45);
ylabel('Mission Success Rate (%)');
title('Ablation: Success Rate');
grid on;

subplot(1, 2, 2);
bar(meanTimes);
set(gca, 'xticklabel', configs(:,3));
xtickangle(45);
ylabel('Mean Completion Time (s)');
title('Ablation: Efficiency');
grid on;

fprintf('Ablation studies complete.\n');
