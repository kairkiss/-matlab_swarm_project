function plotMissionSummary(cfg, scenario, results)
%PLOTMISSIONSUMMARY Create a polished post-mission analysis figure.

W = cfg.worldSize(1);
H = cfg.worldSize(2);
obstacles = scenario.obstacles;

fig = figure('Name', 'Mission Summary', 'Color', 'w', 'Position', [100 100 1450 820]);
tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

% 1) Final coverage map
nexttile(1);
imagesc([0 W], [0 H], results.finalCoverageMap');
set(gca, 'YDir', 'normal');
hold on;
axis equal tight;
xlim([0 W]); ylim([0 H]);
box on; grid on;
colormap(parula);
colorbar;
title('Final Coverage Heatmap', 'FontWeight', 'bold');
xlabel('X'); ylabel('Y');
for j = 1:size(obstacles, 1)
    c = obstacles(j, 1:2);
    r = obstacles(j, 3);
    rectangle('Position', [c(1)-r, c(2)-r, 2*r, 2*r], 'Curvature', [1 1], ...
              'FaceColor', [0.12 0.12 0.12], 'EdgeColor', 'k', 'LineWidth', 1.2);
end
scatter(results.agentPos(:,1,end), results.agentPos(:,2,end), 95, [0 0.45 1], 'filled');
scatter(results.targetPos(:,1,end), results.targetPos(:,2,end), 110, 'r', 'filled', 'd');

% 2) Coverage curve
nexttile(2);
plot(results.time, 100 * results.coverageRatio, 'LineWidth', 2.2);
grid on; box on;
ylim([0 100]);
xlabel('Time (s)'); ylabel('Coverage (%)');
title('Coverage Growth', 'FontWeight', 'bold');

% 3) Tracking quality
nexttile(3);
plot(results.time, results.meanTrackError, 'LineWidth', 2.2);
grid on; box on;
xlabel('Time (s)'); ylabel('Mean tracking error');
title('Kalman Tracking Error', 'FontWeight', 'bold');

% 4) Observation / tracking raster
nexttile(4);
obsMap = double(results.targetObserved);
imagesc(results.time, 1:cfg.numTargets, obsMap);
set(gca, 'YDir', 'normal');
colormap(gca, gray);
colorbar;
caxis([0 1]);
box on;
xlabel('Time (s)'); ylabel('Target index');
title('Observation Timeline (1 = observed)', 'FontWeight', 'bold');

sgtitle({ ...
    'MATLAB Project: Multi-UAV Cooperative Search and Dynamic Target Tracking', ...
    sprintf('Final coverage = %.1f%% | Tracked targets = %d/%d | Final mean error = %.2f', ...
        100 * results.coverageRatio(end), results.numTrackedTargets(end), cfg.numTargets, lastValid(results.meanTrackError))}, ...
    'FontWeight', 'bold');
end

function v = lastValid(x)
idx = find(~isnan(x), 1, 'last');
if isempty(idx)
    v = NaN;
else
    v = x(idx);
end
end
