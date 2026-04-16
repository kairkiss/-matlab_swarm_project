function waypoints = selectCoverageWaypoints(coverageMap, cfg, numWaypoints)
%SELECTCOVERAGEWAYPOINTS Pick exploration goals from low-coverage cells.

Nx = cfg.coverageGridSize(1);
Ny = cfg.coverageGridSize(2);
W = cfg.worldSize(1);
H = cfg.worldSize(2);

if numWaypoints <= 0
    waypoints = zeros(0, 2);
    return;
end

score = coverageMap;
score = score + 0.02 * randn(size(score)); % break ties visually

[~, order] = sort(score(:), 'ascend');

waypoints = zeros(numWaypoints, 2);
selected = 0;
minSeparation = 8.0;

for idx = 1:numel(order)
    [ix, iy] = ind2sub(size(score), order(idx));
    p = [((ix - 0.5) / Nx) * W, ((iy - 0.5) / Ny) * H];

    if selected > 0
        d = sqrt(sum((waypoints(1:selected, :) - p) .^ 2, 2));
        if any(d < minSeparation)
            continue;
        end
    end

    selected = selected + 1;
    waypoints(selected, :) = p;
    if selected >= numWaypoints
        break;
    end
end

if selected < numWaypoints
    % Fallback: fill with evenly spaced points.
    remaining = numWaypoints - selected;
    xs = linspace(0.15 * W, 0.90 * W, remaining + 2);
    ys = linspace(0.15 * H, 0.85 * H, remaining + 2);
    extra = [xs(2:end-1)', ys(end-1:-1:2)'];
    waypoints(selected + 1:end, :) = extra(1:remaining, :);
end
end
