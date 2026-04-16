function desiredVel = computeAgentVelocity(idx, agentPos, agentVel, goal, mode, obstacles, cfg)
%COMPUTEAGENTVELOCITY Goal-seeking plus wall / obstacle / swarm avoidance.

p = agentPos(idx, :);
v = agentVel(idx, :);

% Goal attraction
vecGoal = goal - p;
dGoal = norm(vecGoal);
if dGoal < 1e-9
    dirGoal = [0, 0];
else
    dirGoal = vecGoal / dGoal;
end

goalGain = cfg.coverageAttractionGain;
if strcmp(mode, "track")
    goalGain = cfg.targetAttractionGain;
end
u = goalGain * dirGoal;

% Obstacle avoidance
for k = 1:size(obstacles, 1)
    c = obstacles(k, 1:2);
    r = obstacles(k, 3) + cfg.obstacleBuffer;
    dv = p - c;
    dist = norm(dv);
    if dist < r && dist > 1e-9
        n = dv / dist;
        u = u + cfg.obstacleRepulsionGain * (1 / max(dist, 0.25) - 1 / r) * n;
    end
end

% Inter-agent avoidance
for j = 1:size(agentPos, 1)
    if j == idx
        continue;
    end
    dv = p - agentPos(j, :);
    dist = norm(dv);
    if dist < cfg.agentRepulsionRadius && dist > 1e-9
        n = dv / dist;
        u = u + cfg.agentRepulsionGain * (1 / max(dist, 0.35) - 1 / cfg.agentRepulsionRadius) * n;
    end
end

% Wall avoidance
W = cfg.worldSize(1);
H = cfg.worldSize(2);
if p(1) < cfg.wallBuffer
    u = u + cfg.wallRepulsionGain * [1 / max(p(1), 0.35) - 1 / cfg.wallBuffer, 0];
end
if W - p(1) < cfg.wallBuffer
    u = u + cfg.wallRepulsionGain * [-(1 / max(W - p(1), 0.35) - 1 / cfg.wallBuffer), 0];
end
if p(2) < cfg.wallBuffer
    u = u + cfg.wallRepulsionGain * [0, 1 / max(p(2), 0.35) - 1 / cfg.wallBuffer];
end
if H - p(2) < cfg.wallBuffer
    u = u + cfg.wallRepulsionGain * [0, -(1 / max(H - p(2), 0.35) - 1 / cfg.wallBuffer)];
end

% First-order velocity response
vCmd = v + cfg.dt * cfg.agentResponse * (u - v);
spd = norm(vCmd);
if spd > cfg.agentMaxSpeed
    vCmd = cfg.agentMaxSpeed * vCmd / spd;
end

desiredVel = vCmd;
end
