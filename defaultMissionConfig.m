function cfg = defaultMissionConfig()
%DEFAULTMISSIONCONFIG Configuration for the swarm search-and-track demo.
%
% This project showcases MATLAB engineering in four layers:
%   1) multi-agent dynamics
%   2) dynamic target motion
%   3) Kalman-filter-based target state estimation
%   4) real-time visualization + post-mission analysis

cfg.randomSeed = 42;

% World and time
cfg.worldSize = [120, 80];   % [width, height]
cfg.dt = 0.20;
cfg.totalTime = 60;
cfg.steps = round(cfg.totalTime / cfg.dt);

% Agents
cfg.numAgents = 8;
cfg.agentMaxSpeed = 5.0;
cfg.agentResponse = 2.8;
cfg.agentRepulsionRadius = 8.0;
cfg.agentRepulsionGain = 13.0;

% Targets
cfg.numTargets = 3;
cfg.targetSpeedRange = [1.1, 2.8];
cfg.trackersPerTarget = 2;
cfg.targetAttractionGain = 4.5;

% Perception / estimation
cfg.sensorRange = 16.0;
cfg.coverageSenseRadius = 10.0;
cfg.processNoise = 0.22;
cfg.measurementNoise = 1.10;

% Environment
cfg.numObstacles = 5;
cfg.obstacleBuffer = 5.5;
cfg.obstacleRepulsionGain = 22.0;
cfg.wallBuffer = 7.0;
cfg.wallRepulsionGain = 17.0;

% Coverage map
cfg.coverageGridSize = [60, 40];  % [Nx, Ny]
cfg.coverageAttractionGain = 3.6;

% Visualization
cfg.renderEvery = 1;
cfg.trailLength = 35;
cfg.makeVideo = false;
cfg.videoName = 'swarm_mission_demo.mp4';

% Misc
cfg.historyWindowForMetrics = 15;
end
