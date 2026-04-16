function targets = updateTargets(targets, obstacles, cfg)
%UPDATETARGETS Propagate dynamic targets with bounce behavior.

dt = cfg.dt;
W = cfg.worldSize(1);
H = cfg.worldSize(2);

for i = 1:size(targets, 1)
    p = targets(i, 1:2);
    v = targets(i, 3:4);

    pNew = p + dt * v;

    % Wall bounce
    if pNew(1) < 1 || pNew(1) > W - 1
        v(1) = -v(1);
        pNew(1) = min(max(pNew(1), 1), W - 1);
    end
    if pNew(2) < 1 || pNew(2) > H - 1
        v(2) = -v(2);
        pNew(2) = min(max(pNew(2), 1), H - 1);
    end

    % Obstacle bounce / deflection
    for k = 1:size(obstacles, 1)
        c = obstacles(k, 1:2);
        r = obstacles(k, 3) + 1.5;
        diffVec = pNew - c;
        dist = norm(diffVec);
        if dist < r
            if dist < 1e-9
                n = [1, 0];
            else
                n = diffVec / dist;
            end
            v = v - 2 * dot(v, n) * n;
            pNew = c + n * (r + 0.25);
        end
    end

    % Mild random maneuvering to make motion less trivial.
    jitter = 0.12 * randn(1, 2);
    v = v + jitter;
    spd = norm(v);
    if spd > cfg.targetSpeedRange(2)
        v = (cfg.targetSpeedRange(2) / spd) * v;
    elseif spd < cfg.targetSpeedRange(1)
        if spd < 1e-9
            v = cfg.targetSpeedRange(1) * [1, 0];
        else
            v = (cfg.targetSpeedRange(1) / spd) * v;
        end
    end

    targets(i, :) = [pNew, v];
end
end
