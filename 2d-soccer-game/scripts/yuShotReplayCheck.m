function [T, summary] = yuShotReplayCheck(sources)
%YUSHOTREPLAYCHECK (B-Y3) shot_open at the moment of every real shot in saved matches.
%   [T, summary] = yuShotReplayCheck({'build/yu-v12-neutral-neutral', ...})
%   Read-only. For every kick of at least 1.75 m/s towards the kicker's
%   attacking goal (the match counts kicks >= shotSpeed by ST/MF as shots):
%     before   world of the row BEFORE the kick (what a decision would see)
%     predOpen tdp.shotOpen(...) with the default thresholds
%     centreOpen  same check for the aim point the V1.2 coordinator really
%              uses (goal centre): the fair predicted-vs-observed pair
%     dirOpen  lane along the ACTUAL kick direction (noise already applied,
%              so noiseSigmas = 0) clear and inside the posts
%     outcome  what happened next, from the recorded rows:
%              "goal"     score rose and the ball rolled freely until then
%              "touched"  ball deviated from free rolling (contact) or was
%                         kicked again before reaching the line; touchedBy
%                         names the robot in contact (kicker / team-mate /
%                         opponent / keeper)
%              "out"      play stopped (whole ball over a line) without a goal
%              "short"    ball stopped in play
%   Robots MOVE during the shot here (real match), unlike shotRollout.
%   Free rolling is checked against tdp.ballModel to 1e-9 m.
if ischar(sources) || isstring(sources), sources = cellstr(sources); end
c = replayColumns();
rows = {};
for f = 1:numel(sources)
    rec = loadReplayRecording(sources{f});
    tr = rec.Trace;  P = rec.P;  Q = tdp.shotLaneParams(P);
    Q0 = Q;  Q0.noiseSigmas = 0;
    half = P.goalW/2;
    if P.rulesProfile == TdpRules.CourseSimplified, half = P.goalW/2 - P.ballR; end
    [~, label] = fileparts(regexprep(char(sources{f}), '[\\/]trace\.mat$', ''));
    for k = 2:size(tr, 1) - 1
        if tr(k, c.kicked) < 0.5, continue; end
        i = find(tr(k, c.cool) > tr(k-1, c.cool) + 1e-9, 1);
        if isempty(i), continue; end
        team = 1 + (i > 4);  s = P.dirSign(team);
        v = tr(k, [c.ballVx c.ballVy]);
        speed = norm(v) + P.ballDecel*P.dt;
        if speed < 1.75 || s*v(1) <= 0, continue; end
        b = tr(k-1, [c.ballX c.ballY]);
        pos = [tr(k-1, c.x)' tr(k-1, c.y)'];
        S = tdp.shotOpen(b, team, pos, true(1,8), i, P, Q);
        u = v/norm(v);  gx = s*(P.L/2 + P.ballR);
        Tline = b + u*((gx - b(1))/u(1));
        use = true(1,8);  use(i) = false;
        dirOpen = abs(Tline(2)) < half && tdp.laneCorridor(b, Tline, pos, use, P, Q0) >= 0;
        % the V1/V1.2 coordinator always aims at the goal centre
        dc = hypot(gx - b(1), b(2));
        [cs, cb] = tdp.laneCorridor(b, [gx 0], pos, use, P, Q);
        centreBy = "";
        if cb == 1 + 4*(2 - team), centreBy = "keeper";
        elseif cb > 0 && 1 + (cb > 4) ~= team, centreBy = "opponent";
        elseif cb > 0, centreBy = "team-mate"; end
        centreOpen = cs >= 0 && ...
            half - dc*tan(Q.noiseSigmas*P.kickNoise) >= 0 && S.inRange;
        [outcome, tEnd, who] = follow(tr, k, team, P, c);
        by = "";
        if who == i, by = "kicker"; elseif who > 0 && 1 + (who > 4) == team, by = "team-mate";
        elseif who > 0, by = "opponent"; if who == 1 + 4*(2 - team), by = "keeper"; end, end
        rows(end+1, :) = {string(label), tr(k-1, c.time), i, team, speed, ...
            S.open, S.reason, S.slack, S.keeperOut, centreOpen, centreBy, dirOpen, string(outcome), by, tEnd}; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'match','t','kicker','team','speed', ...
    'predOpen','reason','slack','keeperOut','centreOpen','centreBlockedBy','dirOpen','outcome','touchedBy','secondsToOutcome'});
goal = T.outcome == "goal";
summary = struct('shots', height(T), 'goals', sum(goal), ...
    'predOpen_goal', sum(T.predOpen & goal), 'predOpen_noGoal', sum(T.predOpen & ~goal), ...
    'predBlocked_goal', sum(~T.predOpen & goal), 'predBlocked_noGoal', sum(~T.predOpen & ~goal), ...
    'goalRateWhenOpen', mean(goal(T.predOpen)), 'goalRateWhenBlocked', mean(goal(~T.predOpen)), ...
    'centreOpen_goal', sum(T.centreOpen & goal), 'centreOpen_noGoal', sum(T.centreOpen & ~goal), ...
    'centreBlocked_goal', sum(~T.centreOpen & goal), 'centreBlocked_noGoal', sum(~T.centreOpen & ~goal), ...
    'dirOpen_goal', sum(T.dirOpen & goal), 'dirOpen_noGoal', sum(T.dirOpen & ~goal), ...
    'dirBlocked_goal', sum(~T.dirOpen & goal), 'dirBlocked_noGoal', sum(~T.dirOpen & ~goal));
end

function [outcome, tEnd, who] = follow(tr, k, team, P, c)
outcome = 'short';  tEnd = NaN;  who = 0;
for j = k+1:size(tr, 1)
    if tr(j, c.score(team)) > tr(j-1, c.score(team))
        outcome = 'goal';  tEnd = tr(j, c.time) - tr(k, c.time);  return
    end
    if tr(j, c.pauseUntil) > tr(j-1, c.pauseUntil) + 1e-9
        outcome = 'out';  tEnd = tr(j, c.time) - tr(k, c.time);  return
    end
    if tr(j, c.kicked) > 0.5
        outcome = 'touched';  tEnd = tr(j, c.time) - tr(k, c.time);
        w = find(tr(j, c.cool) > tr(j-1, c.cool) + 1e-9, 1);
        if ~isempty(w), who = w; end
        return
    end
    b = struct('x', tr(j-1, c.ballX), 'y', tr(j-1, c.ballY), 'vx', tr(j-1, c.ballVx), 'vy', tr(j-1, c.ballVy));
    if tr(j, c.time) >= tr(j-1, c.pauseUntil)     % as Plant2D: rolls only while playing
        b = tdp.ballModel(b, P);
    end
    if abs(b.x - tr(j, c.ballX)) > 1e-9 || abs(b.y - tr(j, c.ballY)) > 1e-9
        outcome = 'touched';  tEnd = tr(j, c.time) - tr(k, c.time);
        % the contact model leaves the toucher exactly robotR + ballR from the ball
        d = hypot(tr(j, c.x) - tr(j, c.ballX), tr(j, c.y) - tr(j, c.ballY));
        [dm, who] = min(abs(d - (P.robotR + P.ballR)));
        if dm > 1e-3, who = 0; end
        return
    end
    if b.vx == 0 && b.vy == 0
        outcome = 'short';  tEnd = tr(j, c.time) - tr(k, c.time);  return
    end
end
end
