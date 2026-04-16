%% MATLAB Project Demo
% Multi-UAV cooperative search and dynamic target tracking
% Author: ChatGPT

clear; clc; close all;

cfg = defaultMissionConfig();
scenario = createScenario(cfg);
results = simulateSwarmMission(cfg, scenario);
plotMissionSummary(cfg, scenario, results);

disp('Mission finished. Open the generated figures to inspect the results.');
