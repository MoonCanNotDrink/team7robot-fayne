function files = yuTask2Figures(outDir)
%YUTASK2FIGURES (B-Y1..B-Y4) Figures for Yu's Task 2 chapter and Word report.
%   files = yuTask2Figures('docs/generated/yu_task2')
%   Needs the saved results of yuShotReplayCheck and yuHalfTask2 in
%   build/yu-task2/ (shot_replay.mat, half_reconcile.mat). Scenario and
%   threshold figures are recomputed (deterministic seeds).
if nargin < 1, outDir = fullfile('docs', 'generated', 'yu_task2'); end
if ~isfolder(outDir), mkdir(outDir); end
P = tdp.v12Config();  Q = tdp.shotLaneParams(P);
files = {};
files{end+1} = pitchFigure(P, outDir);
files{end+1} = corridorFigure(P, Q, outDir);
[T, sc] = yuShotScenarios(1000, 7);
writetable(T, fullfile(outDir, 'scenarios.csv'));
files{end+1} = scenarioFigure(P, Q, T, sc, outDir);
[U, C] = yuShotThreshold(400, 11);
writetable(U, fullfile(outDir, 'threshold_sweep.csv'));
writetable(C, fullfile(outDir, 'threshold_calibration.csv'));
files{end+1} = thresholdFigure(U, outDir);
S = load(fullfile('build', 'yu-task2', 'shot_replay.mat'));
files{end+1} = replayFigure(S.T, outDir);
H = load(fullfile('build', 'yu-task2', 'half_reconcile.mat'));
files{end+1} = halfFigure(H.T, outDir);
end

% ---------------------------------------------------------------- drawing
function drawPitch(ax, P, zones)
h = P.L/2;  w = P.W/2;  hold(ax, 'on');
fill(ax, [-h-P.border h+P.border h+P.border -h-P.border], ...
    [-w-P.border -w-P.border w+P.border w+P.border], [0.80 0.90 0.80], 'EdgeColor', 'none');
fill(ax, [-h h h -h], [-w -w w w], [0.55 0.78 0.55], 'EdgeColor', 'w', 'LineWidth', 1.5);
if zones
    Z = tdp.spatialZones(P);
    for x = Z.depthLines([4 6]), plot(ax, [x x], [-w w], ':', 'Color', [1 1 1]*0.95, 'LineWidth', 1); end
end
plot(ax, [0 0], [-w w], 'w', 'LineWidth', 1.5);
th = linspace(0, 2*pi, 80);
plot(ax, P.centreD/2*cos(th), P.centreD/2*sin(th), 'w', 'LineWidth', 1.2);
for s = [-1 1]
    rect(ax, s*h, s*(h - P.penAreaL), P.penAreaW/2, 'w');
    rect(ax, s*h, s*(h - P.goalAreaL), P.goalAreaW/2, 'w');
    plot(ax, s*[h h+P.goalD h+P.goalD h], [-1 -1 1 1]*P.goalW/2, 'Color', [0.3 0.3 0.3], 'LineWidth', 2);
end
axis(ax, 'equal');  xlim(ax, [-h-0.7 h+0.9]);  ylim(ax, [-w-0.5 w+0.5]);
set(ax, 'XTick', -4.5:1.5:4.5, 'YTick', -3:1:3, 'Layer', 'top', 'FontSize', 8);
end

function rect(ax, x0, x1, hw, col)
plot(ax, [x0 x1 x1 x0], [-hw -hw hw hw], 'Color', col, 'LineWidth', 1.2);
end

function disc(ax, c, r, face, edge)
th = linspace(0, 2*pi, 60);
fill(ax, c(1) + r*cos(th), c(2) + r*sin(th), face, 'EdgeColor', edge, 'LineWidth', 1);
end

function drawWorld(ax, P, ball, pos, valid, kicker, receiver)
for i = 1:8
    if ~valid(i), continue; end
    if i <= 4, col = [0.2 0.4 0.9]; else, col = [0.9 0.25 0.2]; end
    disc(ax, pos(i,:), P.robotR, col, 'k');
    lab = 'ST';  r = mod(i-1, 4) + 1;  names = {'GK','DF','MF','ST'};  lab = names{r};
    if i == kicker, lab = [lab '*']; end
    if i == receiver, lab = [lab ' (R)']; end
    text(ax, pos(i,1), pos(i,2) + 0.27, lab, 'HorizontalAlignment', 'center', 'FontSize', 6.5, 'Color', 'k', 'Clipping', 'on');
end
disc(ax, ball, P.ballR, [1 1 1], 'k');
end

function drawCorridor(ax, P, Q, a, b, col)
grow = tan(Q.noiseSigmas*P.kickNoise);
base = P.robotR + P.ballR + Q.margin;
u = (b - a)/norm(b - a);  nrm = [-u(2) u(1)];  L = norm(b - a);
pts = [a + base*nrm; b + (base + L*grow)*nrm; b - (base + L*grow)*nrm; a - base*nrm];
fill(ax, pts(:,1), pts(:,2), col, 'FaceAlpha', 0.25, 'EdgeColor', col);
plot(ax, [a(1) b(1)], [a(2) b(2)], '--', 'Color', col, 'LineWidth', 1);
end

function f = saveFig(fig, outDir, name)
f = fullfile(outDir, name);
exportgraphics(fig, f, 'Resolution', 200, 'BackgroundColor', 'white');
close(fig);
end

% ---------------------------------------------------------------- figures
function f = pitchFigure(P, outDir)
fig = newFig([0 0 1100 720]);
ax = axes(fig);  drawPitch(ax, P, true);
h = P.L/2;  w = P.W/2;
quiver(ax, -1.2, -w-0.3, 2.4, 0, 0, 'Color', [0.2 0.4 0.9], 'LineWidth', 1.5, 'MaxHeadSize', 0.6);
text(ax, 0, -w-0.15, 'blue (ids 1-4) attacks +x', 'Color', [0.2 0.4 0.9], 'HorizontalAlignment', 'center', 'FontSize', 8);
quiver(ax, 1.2, w+0.3, -2.4, 0, 0, 'Color', [0.9 0.25 0.2], 'LineWidth', 1.5, 'MaxHeadSize', 0.6);
text(ax, 0, w+0.15, 'red (ids 5-8) attacks -x', 'Color', [0.9 0.25 0.2], 'HorizontalAlignment', 'center', 'FontSize', 8);
text(ax, -h+0.1, -w+0.25, 'left half x<0 = blue defends', 'FontSize', 7.5, 'Color', 'w');
text(ax, 0.1, -w+0.25, 'right half x>0 = blue attacks', 'FontSize', 7.5, 'Color', 'w');
text(ax, h+0.02, -1.75, {sprintf('goal: |y| < %.2f', P.goalW/2), '(CourseSimplified:', '|y| + r_b < 1.30)'}, 'FontSize', 6.5);
text(ax, -h+0.05, w-0.25, ' penalty area 2.0 x 5.0', 'FontSize', 7, 'Color', 'w');
text(ax, -h+0.05, 1.3, ' goal area 1.0 x 3.0', 'FontSize', 7, 'Color', 'w');
text(ax, -1.5, -w+0.6, ' third line', 'FontSize', 7, 'Color', 'w');
% robot and ball model, enlarged inset at (2.5, 1.2)
c = [0.9 -1.7];
disc(ax, c, P.robotR, [0.2 0.4 0.9], 'k');
th = linspace(0, 2*pi, 60);
plot(ax, c(1) + P.kickRange*cos(th), c(2) + P.kickRange*sin(th), 'y--', 'LineWidth', 1);
plot(ax, c(1) + (P.robotR + P.ballR)*cos(th), c(2) + (P.robotR + P.ballR)*sin(th), 'w:', 'LineWidth', 1);
disc(ax, c + [P.robotR + P.ballR 0], P.ballR, [1 1 1], 'k');
text(ax, c(1) + 0.4, c(2) + 0.25, sprintf(['robot disc r=%.2f (robot-robot contact 2r=%.2f)\n' ...
    'ball r=%.2f, robot-ball contact %.2f (dotted)\nkick range %.2f (dashed)'], ...
    P.robotR, 2*P.robotR, P.ballR, P.robotR + P.ballR, P.kickRange), 'FontSize', 6.5, 'Color', 'w');
text(ax, h+0.05, -w-0.25, 'border strip 1 m', 'FontSize', 6.5);
title(ax, 'B-Y1  2D pitch, ball and robot model (world frame, metres)', 'FontSize', 10);
f = saveFig(fig, outDir, 'fig1_pitch_model.png');
end

function f = corridorFigure(P, Q, outDir)
fig = newFig([0 0 1100 520]);
ax = axes(fig);  drawPitch(ax, P, false);
xlim(ax, [0.8 5.2]);  ylim(ax, [-2.0 2.0]);
b = [1.5 0.3];  pos = repmat([-4 -2.9], 8, 1);  valid = false(1,8);
pos(5,:) = [4.15 0.15];  pos(6,:) = [2.9 -0.6];  valid([5 6]) = true;
S = tdp.shotOpen(b, 1, pos, valid, 0, P, Q);
for j = 1:numel(S.targetsY)
    col = [0.6 0.6 0.6];  if S.targetSlack(j) >= 0, col = [0.1 0.6 0.1]; end
    plot(ax, [b(1) P.L/2 + P.ballR], [b(2) S.targetsY(j)], ':', 'Color', col);
    plot(ax, P.L/2 + P.ballR, S.targetsY(j), 'o', 'MarkerSize', 4, 'MarkerFaceColor', col, 'MarkerEdgeColor', 'k');
end
drawCorridor(ax, P, Q, b, S.target, [0.1 0.6 0.1]);
drawWorld(ax, P, b, pos, valid, 0, 0);
text(ax, 1.0, 1.75, sprintf(['corridor half-width w(s) = r_robot + r_ball + margin + s tan(k\\sigma)\n' ...
    '= %.2f + %.2f + %.2f + s tan(%.0f deg), k = %.1f'], P.robotR, P.ballR, Q.margin, ...
    rad2deg(Q.noiseSigmas*P.kickNoise), Q.noiseSigmas), 'FontSize', 8);
text(ax, 1.0, -1.75, sprintf('%d aim points on x = L/2 + r_{ball}; green = usable (slack >= 0). Best: y = %.2f, slack %.2f m', ...
    numel(S.targetsY), S.target(2), S.slack), 'FontSize', 8);
title(ax, 'B-Y2  shot lane: swept ball disc widened by the kick-noise cone', 'FontSize', 10);
f = saveFig(fig, outDir, 'fig2_shot_corridor.png');
end

function f = scenarioFigure(P, Q, T, sc, outDir)
fig = newFig([0 0 1500 1150]);
tl = tiledlayout(fig, 4, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:numel(sc)
    ax = nexttile(tl);  drawPitch(ax, P, false);
    s = sc(k);
    if s.kind == "shot"
        S = s.S;  col = [0.85 0.2 0.2];  if S.open, col = [0.1 0.6 0.1]; end
        drawCorridor(ax, P, Q, s.ball, S.target, col);
        xlim(ax, [min(s.ball(1), 1.0) - 0.6, 5.2]);  ylim(ax, [-3.2 3.2]);
    else
        S = s.S;  col = [0.85 0.2 0.2];  if S.open, col = [0.1 0.6 0.1]; end
        drawCorridor(ax, P, Q, s.ball, s.pos(s.receiver,:), col);
        xlim(ax, [-0.6 5.2]);  ylim(ax, [-3.2 3.2]);
    end
    drawWorld(ax, P, s.ball, s.pos, s.valid, s.kicker, s.receiver);
    obs = sprintf('pMC %.2f', T.pMC(k));
    title(ax, {char(s.name), sprintf('pred %s (reason %d) | nominal %d, %s | agree %d', ...
        ternary(T.predOpen(k), 'OPEN', 'BLOCKED'), T.reason(k), T.nominal(k), obs, T.agree(k))}, ...
        'FontSize', 7.5, 'Interpreter', 'none');
end
title(tl, 'B-Y3  controlled scenarios: predicted (tdp.shotOpen / passOpen) vs simulated kicks', 'FontSize', 11);
f = saveFig(fig, outDir, 'fig3_scenarios.png');
end

function f = thresholdFigure(U, outDir)
fig = newFig([0 0 900 480]);
ax = axes(fig);  hold(ax, 'on');  grid(ax, 'on');
cols = lines(3);  s = unique(U.s);
for k = 1:numel(s)
    r = U.s == s(k);
    plot(ax, U.slack(r), U.pClean(r), '-o', 'Color', cols(k,:), 'MarkerSize', 3, ...
        'DisplayName', sprintf('blocker %.1f m along a 3 m shot', s(k)));
end
xline(ax, 0, 'k-', 'slack = 0 (corridor edge)', 'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
yline(ax, 0.8, 'k--', 'reliable shot 0.80', 'HandleVisibility', 'off');
xlabel(ax, 'predicted slack (m), 1-sigma corridor');  ylabel(ax, 'share of noisy kicks scoring untouched');
legend(ax, 'Location', 'southeast');  xlim(ax, [-0.45 0.45]);  ylim(ax, [0 1.02]);
title(ax, 'B-Y3  threshold calibration: one static blocker moved across the lane (400 kicks each)', 'FontSize', 10);
f = saveFig(fig, outDir, 'fig4_threshold.png');
end

function f = replayFigure(T, outDir)
fig = newFig([0 0 1100 430]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
ax = nexttile(tl);
cats = ["keeper" "opponent"];
v = [sum(T.centreBlockedBy == cats(1)) sum(T.centreBlockedBy == cats(2)) sum(T.centreOpen)];
bar(ax, categorical(["keeper", "opponent field player", "centre lane open"], ...
    ["keeper", "opponent field player", "centre lane open"]), v);
text(ax, 1:3, v + 4, string(v), 'HorizontalAlignment', 'center');  ylim(ax, [0 1.15*max(v)]);
ylabel(ax, 'shots');  title(ax, sprintf('V1.2 aims at the goal centre: what blocked that lane (n=%d)', height(T)), 'FontSize', 9);
ax = nexttile(tl);
g = T.outcome == "goal";
M = [sum(T.dirOpen & g) sum(T.dirOpen & ~g); sum(~T.dirOpen & g) sum(~T.dirOpen & ~g)];
bar(ax, categorical(["lane open", "lane blocked"], ["lane open", "lane blocked"]), M, 'stacked');
legend(ax, {'goal', 'no goal'}, 'Location', 'northwest');
text(ax, [1 2], sum(M, 2)' + 6, compose('%d goals / %d', M(:,1), sum(M,2)), 'HorizontalAlignment', 'center');
ylabel(ax, 'shots');  title(ax, 'lane along the ACTUAL kick direction vs outcome', 'FontSize', 9);
title(tl, 'B-Y3  replay check on 30 V1.2 matches x 300 s (robots moving)', 'FontSize', 10);
f = saveFig(fig, outDir, 'fig5_replay_shots.png');
end

function f = halfFigure(T, outDir)
pair = extractBefore(T.match, '-s');
groups = unique(pair, 'stable');
M = zeros(numel(groups), 4);
for k = 1:numel(groups)
    r = pair == groups(k);
    M(k,:) = [sum(T.qin_left(r)) sum(T.qin_centre(r)) sum(T.qin_right(r)) ...
        sum(T.qin_paused(r) + T.qin_restartTransition(r) + T.qin_outside(r))]/sum(T.recordedSeconds(r))*100;
end
fig = newFig([0 0 900 420]);
ax = axes(fig);
bh = barh(ax, categorical(groups, groups), M, 'stacked');
set(bh(1), 'FaceColor', [0.2 0.4 0.9]);  set(bh(2), 'FaceColor', [0.6 0.6 0.6]);
set(bh(3), 'FaceColor', [0.9 0.25 0.2]); set(bh(4), 'FaceColor', [0.95 0.85 0.3]);
legend(ax, {'left x<0 (blue defends)', 'centre x=0', 'right x>0 (blue attacks)', 'excluded (paused/reset/out)'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal');
xlabel(ax, '% of recorded time (10 seeds x 300 s per pairing, blue-red)');
title(ax, 'B-Y4  ball-in-half time, identical under Yu and Qin rules', 'FontSize', 10);
f = saveFig(fig, outDir, 'fig6_half_time.png');
end

function fig = newFig(position)
fig = figure('Visible', 'off', 'Position', position, 'Color', 'w');
try, theme(fig, 'light'); catch, end     % R2025a+: never inherit a dark theme
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
