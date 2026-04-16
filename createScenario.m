function scenario = createScenario(cfg)
%CREATESCENARIO Build obstacles, initial agents and dynamic targets.

rng(cfg.randomSeed);
W = cfg.worldSize(1);
H = cfg.worldSize(2);

% Obstacles: [x, y, radius]
obstacles = zeros(cfg.numObstacles, 3);
placed = 0;
tries = 0;
while placed < cfg.numObstacles && tries < 5000
    tries = tries + 1;
    r = 4 + 4 * rand();
    c = [12 + (W - 24) * rand(), 10 + (H - 20) * rand()];

    ok = true;
    for k = 1:placed
        if norm(c - obstacles(k, 1:2)) < r + obstacles(k, 3) + 10
            ok = false;
            break;
        end
    end

    if ok
        obstacles(placed + 1, :) = [c, r];
        placed = placed + 1;
    end
end
obstacles = obstacles(1:placed, :);

% Agents start on left / lower side to create visible exploration.
agentPos = sampleFreePoints(cfg.numAgents, cfg, obstacles, [5, 22], [5, H - 8], 2.0);
agentVel = zeros(cfg.numAgents, 2);

% Targets start deeper in the map.
targetPos = sampleFreePoints(cfg.numTargets, cfg, obstacles, [W * 0.35, W - 10], [8, H - 8], 3.0);
targetVel = zeros(cfg.numTargets, 2);
for i = 1:cfg.numTargets
    ang = 2 * pi * rand();
    spd = cfg.targetSpeedRange(1) + diff(cfg.targetSpeedRange) * rand();
    targetVel(i, :) = spd * [cos(ang), sin(ang)];
end

targets = [targetPos, targetVel];

scenario = struct();
scenario.obstacles = obstacles;
scenario.agentPos0 = agentPos;
scenario.agentVel0 = agentVel;
scenario.targets0 = targets;
scenario.worldSize = cfg.worldSize;
end

function pts = sampleFreePoints(n, cfg, obstacles, xRange, yRange, margin)
pts = zeros(n, 2);
count = 0;
tries = 0;
while count < n && tries < 10000
    tries = tries + 1;
    p = [xRange(1) + diff(xRange) * rand(), yRange(1) + diff(yRange) * rand()];

    if isInsideAnyObstacle(p, obstacles, margin)
        continue;
    end

    if count > 0
        d = sqrt(sum((pts(1:count, :) - p) .^ 2, 2));
        if any(d < 4)
            continue;
        end
    end

    if p(1) < 1 || p(1) > cfg.worldSize(1) - 1 || p(2) < 1 || p(2) > cfg.worldSize(2) - 1
        continue;
    end

    count = count + 1;
    pts(count, :) = p;
end

if count < n
    error('Unable to place all points in free space.');
end
end

function tf = isInsideAnyObstacle(p, obstacles, margin)
if isempty(obstacles)
    tf = false;
    return;
end

d = sqrt(sum((obstacles(:, 1:2) - p) .^ 2, 2));
tf = any(d <= (obstacles(:, 3) + margin));
end
