function S = shotOpen(ball, team, pos, valid, kicker, P, Q)
%SHOTOPEN (B-Y2) Is there a clear shooting lane from the ball to the goal?
%   S = tdp.shotOpen(ball, team, pos, valid, kicker, P)
%   S = tdp.shotOpen(ball, team, pos, valid, kicker, P, Q)
%   ball    1x2 ball centre, world frame (metres).
%   team    1 blue (attacks +x) or 2 red (attacks -x).
%   pos     8x2 robot centres, ids 1-4 blue, 5-8 red; local role 1 of each
%           team (ids 1 and 5) is the goalkeeper, as in tdp.v1TeamPlans.
%   valid   1x8 logical, robot position known. An unknown robot is ignored.
%   kicker  id of the robot that would shoot (excluded from the blockers);
%           0 when nobody is chosen yet.
%   Q       thresholds, default tdp.shotLaneParams(P).
%
%   Geometry only, no tactics. Q.nTargets aim points are spread over the goal
%   mouth, on the line x = +-(L/2 + ballR) where the referee decides a goal. An aim point T is usable when
%     (1) the corridor ball -> T (tdp.laneCorridor) touches no robot, and
%     (2) the kick-noise cone still enters between the posts:
%         |T_y| + d*tan(noiseSigmas*kickNoise) <= goal half-width for a goal
%         (CourseSimplified: goalW/2 - ballR, ACompat: goalW/2).
%   S.open is true when at least one aim point is usable AND the ball is in
%   range: a shot of Q.shotSpeed still rolls at Q.minArrivalSpeed at the
%   goal line (v^2 = shotSpeed^2 - 2*ballDecel*d).
%
%   The goalkeeper is a blocker like any other robot. Whether it has LEFT its
%   goal is reported separately (S.keeperOut), so "lane clear" and "keeper
%   off its line" are never mixed up.
%
%   Fields
%     open            shot_open flag (lane usable and in range)
%     target          1x2 best aim point (largest slack); goal centre when
%                     no aim point is usable
%     slack           slack of that aim point, metres (>= 0 usable)
%     openFraction    share of aim points that are usable (0..1)
%     inRange, dist   range check and distance to the best aim point
%     arrivalSpeed    ball speed at the goal line for Q.shotSpeed (0 if short)
%     reason          0 open, 1 out of range, 2 keeper blocks (removing the
%                     keeper would open a lane), 3 opponent field player
%                     blocks, 4 team-mate blocks, 5 angle too narrow for the
%                     posts and kick noise
%     blocker         robot id limiting the best aim point (0 if none)
%     openWithoutKeeper  some aim point would be usable if the keeper were
%                     not there (keeper is the only obstacle)
%     keeperOut       opponent keeper known and outside its goal area plus
%                     Q.keeperBoxMargin
%     keeperKnown     opponent keeper position valid
%     targetsY, targetSlack  all aim points (y) and their slack (diagnostic)
if nargin < 7, Q = tdp.shotLaneParams(P); end
% Every field is defined first, in a fixed order, so the function can be
% compiled inside the Simulink match (called from tdp.v1TeamPlans).
n = 9;                           % aim points; fixed size for code generation
assert(Q.nTargets == n);
S = struct('open', false, 'target', [0 0], 'slack', -Inf, 'targetsY', zeros(1, n), ...
    'targetSlack', zeros(1, n), 'openFraction', 0, 'dist', 0, 'arrivalSpeed', 0, ...
    'inRange', false, 'blocker', 0, 'openWithoutKeeper', false, ...
    'keeperKnown', false, 'keeperOut', false, 'reason', 0);
s = P.dirSign(team);
gx = s*(P.L/2 + P.ballR);       % the referee scores once the WHOLE ball is over
half = P.goalW/2;
if P.rulesProfile == TdpRules.CourseSimplified, half = P.goalW/2 - P.ballR; end
grow = tan(Q.noiseSigmas*P.kickNoise);
yMax = half - Q.postMargin;
ys = linspace(-yMax, yMax, n);

opp = 1 + 4*(2 - team);          % opponent ids opp..opp+3, keeper = opp
mate = 1 + 4*(team - 1);
use = logical(valid(:));
if kicker >= 1 && kicker <= 8, use(kicker) = false; end
useNoKeeper = use;  useNoKeeper(opp) = false;

slackAll = zeros(1, n);  slackNoKeeper = zeros(1, n);
blockAll = zeros(1, n);  postSlack = zeros(1, n);
for j = 1:n
    T = [gx ys(j)];
    d = hypot(T(1) - ball(1), T(2) - ball(2));
    postSlack(j) = half - abs(ys(j)) - d*grow;
    [sl, bk] = tdp.laneCorridor(ball, T, pos, use, P, Q);
    [sk, ~] = tdp.laneCorridor(ball, T, pos, useNoKeeper, P, Q);
    slackAll(j) = min(sl, postSlack(j));
    slackNoKeeper(j) = min(sk, postSlack(j));
    if sl < postSlack(j), blockAll(j) = bk; end
end
[best, j] = max(slackAll);
% Equal slack: prefer the aim point nearest the goal centre (shorter, safer).
for jj = 1:n
    if abs(slackAll(jj) - best) < 1e-12 && abs(ys(jj)) < abs(ys(j))
        j = jj;
    end
end
S.target = [gx ys(j)];
S.slack = best;
S.targetsY = ys;
S.targetSlack = slackAll;
S.openFraction = mean(slackAll >= 0);
S.dist = hypot(S.target(1) - ball(1), S.target(2) - ball(2));
v2 = Q.shotSpeed^2 - 2*P.ballDecel*S.dist;
S.arrivalSpeed = sqrt(max(v2, 0));
S.inRange = v2 >= Q.minArrivalSpeed^2 && s*ball(1) < P.L/2;
S.blocker = blockAll(j);
S.openWithoutKeeper = any(slackNoKeeper >= 0);
S.keeperKnown = logical(valid(opp));
S.keeperOut = false;
if S.keeperKnown
    f = -s*pos(opp,1);           % keeper position in the keeper's team frame
    lat = abs(pos(opp,2));
    S.keeperOut = f > -P.L/2 + P.goalAreaL + Q.keeperBoxMargin || ...
        lat > P.goalAreaW/2 + Q.keeperBoxMargin;
end
lane = best >= 0;
S.open = lane && S.inRange;
if S.open
    S.reason = 0;
elseif ~S.inRange
    S.reason = 1;
elseif S.openWithoutKeeper || S.blocker == opp
    S.reason = 2;                % the keeper is the obstacle that decides
elseif S.blocker == 0
    S.reason = 5;
elseif S.blocker >= opp && S.blocker <= opp + 3
    S.reason = 3;
elseif S.blocker >= mate && S.blocker <= mate + 3
    S.reason = 4;
else
    S.reason = 3;
end
end
