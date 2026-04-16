function renderMissionFrame(fig, cfg, scenario, agentPos, targets, filters, coverageMap, results, k)
%RENDERMISSIONFRAME Draw live world state and mission metrics.

W = cfg.worldSize(1);
H = cfg.worldSize(2);
obstacles = scenario.obstacles;

figure(fig);
clf(fig);

t = tiledlayout(fig, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact'); %#ok<NASGU>

% --- Left panel: world and entities ---
nexttile(1);
imagesc([0 W], [0 H], coverageMap');
set(gca, 'YDir', 'normal');
hold on;
axis equal tight;
xlim([0 W]);
ylim([0 H]);
box on;
grid on;
colormap(parula);
colorbar;
title(sprintf('Mission World | t = %.1f s', results.time(k)), 'FontWeight', 'bold');
xlabel('X'); ylabel('Y');

for j = 1:size(obstacles, 1)
    c = obstacles(j, 1:2);
    r = obstacles(j, 3);
    rectangle('Position', [c(1)-r, c(2)-r, 2*r, 2*r], 'Curvature', [1 1], ...
              'FaceColor', [0.18 0.18 0.18], 'EdgeColor', [0 0 0], 'LineWidth', 1.3);
end

% Trails
trailLength = min(cfg.trailLength, k);
if trailLength >= 2
    for a = 1:cfg.numAgents
        traj = squeeze(results.agentPos(a, :, max(1, k-trailLength+1):k))';
        plot(traj(:,1), traj(:,2), '-', 'Color', [0.65 0.78 1.0], 'LineWidth', 1.0);
    end
end

% Sensor circles
th = linspace(0, 2*pi, 80);
for a = 1:cfg.numAgents
    xc = agentPos(a,1) + cfg.sensorRange * cos(th);
    yc = agentPos(a,2) + cfg.sensorRange * sin(th);
    plot(xc, yc, '--', 'Color', [0.65 0.85 1.0], 'LineWidth', 0.6);
end

% Targets (ground truth)
scatter(targets(:,1), targets(:,2), 120, 'r', 'filled', 'd', 'DisplayName', 'True targets');
quiver(targets(:,1), targets(:,2), targets(:,3), targets(:,4), 0, 'Color', [0.65 0 0], 'LineWidth', 1.2, 'MaxHeadSize', 1.5);

% Estimates
estPts = nan(cfg.numTargets, 2);
for t = 1:cfg.numTargets
    if filters(t).initialized
        estPts(t,:) = filters(t).x(1:2)';
    end
end
valid = all(~isnan(estPts), 2);
if any(valid)
    scatter(estPts(valid,1), estPts(valid,2), 110, 'y', 'x', 'LineWidth', 2.0, 'DisplayName', 'KF estimate');
end

% Agents, colored by current mode.
trackMask = results.agentModes(:, k) == "track";
searchMask = ~trackMask;
if any(searchMask)
    scatter(agentPos(searchMask,1), agentPos(searchMask,2), 95, [0 0.45 1], 'filled', 'o', 'DisplayName', 'Search agents');
end
if any(trackMask)
    scatter(agentPos(trackMask,1), agentPos(trackMask,2), 110, [0.2 0.85 0.4], 'filled', '^', 'DisplayName', 'Tracking agents');
end

% Assignment lines
for a = 1:cfg.numAgents
    tid = results.assignedTargetIdx(a, k);
    if tid > 0 && filters(tid).initialized
        ep = filters(tid).x(1:2)';
        plot([agentPos(a,1), ep(1)], [agentPos(a,2), ep(2)], '-', 'Color', [0.55 0.88 0.55], 'LineWidth', 1.0);
    end
end

legend('Location', 'southoutside', 'NumColumns', 2);

% --- Right panel: metrics dashboard ---
nexttile(2);
hold on;
grid on;
box on;
plot(results.time(1:k), results.coverageRatio(1:k), 'LineWidth', 2.0);
plot(results.time(1:k), normalizeMetric(results.meanTrackError(1:k)), 'LineWidth', 2.0);
plot(results.time(1:k), results.numTrackedTargets(1:k) / cfg.numTargets, 'LineWidth', 2.0);
ylim([0 1.05]);
xlim([0, results.time(end)]);
xlabel('Time (s)');
ylabel('Normalized value');
title('Mission Dashboard', 'FontWeight', 'bold');
legend({'Coverage ratio', 'Tracking quality (inverted error)', 'Tracked target ratio'}, 'Location', 'southoutside');

metricWindow = results.meanTrackError(max(1, k-cfg.historyWindowForMetrics+1):k);
summaryText = sprintf([ ...
    'Coverage: %5.1f%%\n' ...
    'Tracked targets: %d / %d\n' ...
    'Mean track error: %.2f\n' ...
    'Agents in tracking mode: %d / %d'], ...
    100 * results.coverageRatio(k), ...
    results.numTrackedTargets(k), cfg.numTargets, ...
    nanmeanSafe(metricWindow), ...
    nnz(trackMask), cfg.numAgents);
text(0.03 * results.time(end), 0.22, summaryText, 'FontName', 'Consolas', 'FontSize', 12, ...
     'BackgroundColor', [1 1 1], 'Margin', 10, 'EdgeColor', [0.75 0.75 0.75]);
end

function y = normalizeMetric(x)
if all(isnan(x))
    y = zeros(size(x));
    return;
end

xFilled = x;
idx = isnan(xFilled);
if any(~idx)
    xFilled(idx) = max(xFilled(~idx));
else
    xFilled(:) = 1;
end

y = 1 ./ (1 + xFilled);
end

function m = nanmeanSafe(x)
mask = ~isnan(x);
if any(mask)
    m = mean(x(mask));
else
    m = NaN;
end
end
