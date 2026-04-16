function [goals, modes, assignedTargetIdx] = assignTargets(agentPos, filters, observedNow, coverageMap, cfg)
%ASSIGNTARGETS Allocate agents between target-tracking and exploration.

nAgents = size(agentPos, 1);
nTargets = numel(filters);

goals = zeros(nAgents, 2);
modes = strings(nAgents, 1);
modes(:) = "search";
assignedTargetIdx = zeros(nAgents, 1);

freeAgents = 1:nAgents;

% Target priority: recently observed and lower covariance first.
priority = -inf(nTargets, 1);
targetPos = zeros(nTargets, 2);
for t = 1:nTargets
    if filters(t).initialized
        covPenalty = trace(filters(t).P(1:2, 1:2));
        recencyBonus = max(0, 6 - filters(t).missedCount);
        obsBonus = 20 * observedNow(t);
        priority(t) = obsBonus + recencyBonus - 0.4 * covPenalty;
        targetPos(t, :) = filters(t).x(1:2)';
    end
end

[~, targetOrder] = sort(priority, 'descend');

for k = 1:nTargets
    t = targetOrder(k);
    if ~filters(t).initialized || isempty(freeAgents)
        continue;
    end

    need = 1 + double(observedNow(t));
    need = min(need, cfg.trackersPerTarget);

    for q = 1:need
        if isempty(freeAgents)
            break;
        end
        d = sqrt(sum((agentPos(freeAgents, :) - targetPos(t, :)) .^ 2, 2));
        [~, idLocal] = min(d);
        a = freeAgents(idLocal);

        goals(a, :) = targetPos(t, :);
        modes(a) = "track";
        assignedTargetIdx(a) = t;

        freeAgents(idLocal) = [];
    end
end

% Remaining agents do active coverage.
if ~isempty(freeAgents)
    waypoints = selectCoverageWaypoints(coverageMap, cfg, numel(freeAgents));
    for i = 1:numel(freeAgents)
        a = freeAgents(i);
        goals(a, :) = waypoints(i, :);
        modes(a) = "search";
    end
end
end
