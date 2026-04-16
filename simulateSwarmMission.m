function results = simulateSwarmMission(cfg, scenario)
%SIMULATESWARMMISSION Run the search-and-track mission.

W = cfg.worldSize(1);
H = cfg.worldSize(2);
dt = cfg.dt;
steps = cfg.steps;

nA = cfg.numAgents;
nT = cfg.numTargets;
Nx = cfg.coverageGridSize(1);
Ny = cfg.coverageGridSize(2);

agentPos = scenario.agentPos0;
agentVel = scenario.agentVel0;
targets = scenario.targets0;
obstacles = scenario.obstacles;

coverageMap = zeros(Nx, Ny);

% Kalman filter setup for each target.
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Hk = [1 0 0 0; 0 1 0 0];
q = cfg.processNoise;
Q = q^2 * [dt^4/4 0 dt^3/2 0; 0 dt^4/4 0 dt^3/2; dt^3/2 0 dt^2 0; 0 dt^3/2 0 dt^2];
R = (cfg.measurementNoise ^ 2) * eye(2);

filters = repmat(struct('initialized', false, ...
                        'x', zeros(4,1), ...
                        'P', 1e3 * eye(4), ...
                        'missedCount', 999), nT, 1);

results = struct();
results.time = (0:steps-1)' * dt;
results.agentPos = zeros(nA, 2, steps);
results.targetPos = zeros(nT, 2, steps);
results.estimatePos = nan(nT, 2, steps);
results.targetObserved = false(nT, steps);
results.coverageRatio = zeros(steps, 1);
results.meanTrackError = nan(steps, 1);
results.numTrackedTargets = zeros(steps, 1);
results.assignedTargetIdx = zeros(nA, steps);
results.agentModes = strings(nA, steps);
results.finalCoverageMap = [];
results.config = cfg;

fig = figure('Name', 'Swarm Search + Tracking Mission', 'Color', 'w', 'Position', [80 80 1400 680]);
videoObj = [];
if cfg.makeVideo
    videoObj = VideoWriter(cfg.videoName, 'MPEG-4');
    videoObj.FrameRate = round(1 / dt);
    open(videoObj);
end

for k = 1:steps
    % Ground truth target dynamics.
    targets = updateTargets(targets, obstacles, cfg);

    % KF predict.
    for t = 1:nT
        if filters(t).initialized
            filters(t).x = A * filters(t).x;
            filters(t).P = A * filters(t).P * A' + Q;
            filters(t).missedCount = filters(t).missedCount + 1;
        end
    end

    % Measurement update: any nearby agent can observe a target.
    observedNow = false(nT, 1);
    for t = 1:nT
        pTrue = targets(t, 1:2);
        d = sqrt(sum((agentPos - pTrue) .^ 2, 2));
        visibleAgents = find(d <= cfg.sensorRange);
        if ~isempty(visibleAgents)
            observedNow(t) = true;
            z = pTrue(:) + cfg.measurementNoise * randn(2, 1);
            if ~filters(t).initialized
                filters(t).initialized = true;
                filters(t).x = [z; 0; 0];
                filters(t).P = diag([8, 8, 3, 3]);
                filters(t).missedCount = 0;
            else
                [filters(t).x, filters(t).P] = kfUpdate(filters(t).x, filters(t).P, z, Hk, R);
                filters(t).missedCount = 0;
            end
        end
    end

    % Task allocation.
    [goals, modes, assignedTargetIdx] = assignTargets(agentPos, filters, observedNow, coverageMap, cfg);

    % Agent motion update.
    for a = 1:nA
        agentVel(a, :) = computeAgentVelocity(a, agentPos, agentVel, goals(a, :), modes(a), obstacles, cfg);
        agentPos(a, :) = agentPos(a, :) + dt * agentVel(a, :);
        agentPos(a, :) = resolveCollisions(agentPos(a, :), obstacles, cfg);
    end

    % Update coverage map.
    coverageMap = updateCoverageMap(coverageMap, agentPos, cfg);

    % Metrics.
    estCount = 0;
    errAcc = 0;
    for t = 1:nT
        if filters(t).initialized
            estPos = filters(t).x(1:2)';
            errAcc = errAcc + norm(estPos - targets(t, 1:2));
            estCount = estCount + 1;
            results.estimatePos(t, :, k) = estPos;
        end
    end
    if estCount > 0
        results.meanTrackError(k) = errAcc / estCount;
    end

    results.coverageRatio(k) = nnz(coverageMap > 0) / numel(coverageMap);
    results.numTrackedTargets(k) = sum([filters.initialized]);
    results.agentPos(:, :, k) = agentPos;
    results.targetPos(:, :, k) = targets(:, 1:2);
    results.targetObserved(:, k) = observedNow;
    results.assignedTargetIdx(:, k) = assignedTargetIdx;
    results.agentModes(:, k) = modes;

    % Real-time render.
    if mod(k - 1, cfg.renderEvery) == 0 || k == steps
        renderMissionFrame(fig, cfg, scenario, agentPos, targets, filters, coverageMap, results, k);
        drawnow limitrate;
        if cfg.makeVideo
            writeVideo(videoObj, getframe(fig));
        end
    end
end

if cfg.makeVideo
    close(videoObj);
end

results.finalCoverageMap = coverageMap;
if isvalid(fig)
    close(fig);
end
end

function [x, P] = kfUpdate(xPred, PPred, z, H, R)
S = H * PPred * H' + R;
K = PPred * H' / S;
innovation = z - H * xPred;
x = xPred + K * innovation;
P = (eye(size(PPred)) - K * H) * PPred;
end

function p = resolveCollisions(p, obstacles, cfg)
W = cfg.worldSize(1);
H = cfg.worldSize(2);
p(1) = min(max(p(1), 1), W - 1);
p(2) = min(max(p(2), 1), H - 1);

for j = 1:size(obstacles, 1)
    c = obstacles(j, 1:2);
    r = obstacles(j, 3) + 0.9;
    dv = p - c;
    dist = norm(dv);
    if dist < r
        if dist < 1e-9
            n = [1, 0];
        else
            n = dv / dist;
        end
        p = c + n * (r + 0.15);
    end
end
end

function coverageMap = updateCoverageMap(coverageMap, agentPos, cfg)
Nx = cfg.coverageGridSize(1);
Ny = cfg.coverageGridSize(2);
W = cfg.worldSize(1);
H = cfg.worldSize(2);
r = cfg.coverageSenseRadius;

xCenters = linspace(W/(2*Nx), W - W/(2*Nx), Nx);
yCenters = linspace(H/(2*Ny), H - H/(2*Ny), Ny);
[Xc, Yc] = ndgrid(xCenters, yCenters);

for a = 1:size(agentPos, 1)
    d2 = (Xc - agentPos(a,1)) .^ 2 + (Yc - agentPos(a,2)) .^ 2;
    coverageMap(d2 <= r^2) = coverageMap(d2 <= r^2) + 1;
end
end
