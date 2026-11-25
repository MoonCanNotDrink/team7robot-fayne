function [T, sc] = yuShotScenarios(nNoise, seed)
%YUSHOTSCENARIOS (B-Y3) Controlled shooting/passing scenarios: predicted vs observed.
%   [T, sc] = yuShotScenarios()            % 1000 noisy kicks per scenario
%   [T, sc] = yuShotScenarios(nNoise, seed)
%   Every scenario is a frozen world (blue attacks +x, ids 1-4 blue, 5-8 red,
%   1 and 5 are the keepers). PREDICTED = tdp.shotOpen / tdp.passOpen.
%   OBSERVED = the match ball and contact models (shotRollout) with the
%   robots standing still:
%     nominal  one kick at the chosen target with no direction error
%     pMC      share of nNoise kicks with the match kick noise (P.kickNoise)
%              that score without touching a robot (shot), or that no robot
%              other than the receiver touches first (pass: lane clear)
%     pBest    shots: best pMC over all aim points (is ANY target good?)
%              passes: share of noisy passes that actually reach the
%              receiver (accuracy, not part of the lane check)
%   Observed open = nominal kick works AND pMC >= pReliable (0.80). The
%   corridor is 1 kick-noise sigma wide, i.e. designed for about 84 %
%   success at zero slack, so 0.80 is the matching observed threshold.
%   T is a table, sc the scenario structs (for figures).
if nargin < 1, nNoise = 1000; end
if nargin < 2, seed = 7; end
pReliable = 0.80;
P = tdp.v12Config();
Q = tdp.shotLaneParams(P);
rng(seed, 'twister');
noise = randn(nNoise, 1)*P.kickNoise;

sc = scenarios(P);
n = numel(sc);
name = strings(n,1); kind = strings(n,1); expected = strings(n,1);
predOpen = false(n,1); reason = zeros(n,1); keeperOut = false(n,1);
slack = zeros(n,1); openFrac = nan(n,1); nominal = false(n,1);
pMC = zeros(n,1); pBest = nan(n,1); agree = false(n,1);
for k = 1:n
    s = sc(k);
    name(k) = s.name;  kind(k) = s.kind;  expected(k) = s.expected;
    if s.kind == "shot"
        S = tdp.shotOpen(s.ball, 1, s.pos, s.valid, s.kicker, P, Q);
        predOpen(k) = S.open;  reason(k) = S.reason;  keeperOut(k) = S.keeperOut;
        slack(k) = S.slack;  openFrac(k) = S.openFraction;
        use = s.valid;  use(s.kicker) = false;
        aim = atan2(S.target(2) - s.ball(2), S.target(1) - s.ball(1));
        R = shotRollout(s.ball, aim, Q.shotSpeed, s.pos, use, 1, P);
        nominal(k) = R.clean;
        pMC(k) = mcShot(s.ball, aim, Q.shotSpeed, s.pos, use, noise, P);
        best = 0;
        for j = 1:numel(S.targetsY)
            a = atan2(S.targetsY(j) - s.ball(2), P.L/2 - s.ball(1));
            best = max(best, mcShot(s.ball, a, Q.shotSpeed, s.pos, use, noise(1:min(200,nNoise)), P));
        end
        pBest(k) = best;
        sc(k).S = S;
    else
        S = tdp.passOpen(s.ball, 1, s.pos, s.valid, s.kicker, s.receiver, P, Q);
        predOpen(k) = S.open;  reason(k) = S.reason;  slack(k) = S.slack;
        use = s.valid;  use(s.kicker) = false;
        T0 = s.pos(s.receiver,:);
        aim = atan2(T0(2) - s.ball(2), T0(1) - s.ball(1));
        R = shotRollout(s.ball, aim, S.speed, s.pos, use, 1, P);
        nominal(k) = R.touched == s.receiver;
        clear = 0;  reach = 0;
        for i = 1:nNoise
            R = shotRollout(s.ball, aim + noise(i), S.speed, s.pos, use, 1, P);
            clear = clear + (R.touched == 0 || R.touched == s.receiver);
            reach = reach + (R.touched == s.receiver);
        end
        pMC(k) = clear/nNoise;
        pBest(k) = reach/nNoise;
        sc(k).S = S;
    end
    % observed "open": the noise-free kick works AND most noisy kicks work
    agree(k) = predOpen(k) == (nominal(k) && pMC(k) >= pReliable);
end
T = table(name, kind, expected, predOpen, reason, keeperOut, slack, openFrac, ...
    nominal, pMC, pBest, agree);
end

function p = mcShot(ball, aim, speed, pos, use, noise, P)
hit = 0;
for i = 1:numel(noise)
    R = shotRollout(ball, aim + noise(i), speed, pos, use, 1, P);
    hit = hit + R.clean;
end
p = hit/numel(noise);
end

function sc = scenarios(P)
% Base world: blue GK/DF at home, red players placed away from the lanes.
kx = P.L/2 - P.v1KeeperInset;                 % red keeper line x = 4.15
keeperY = @(by) min(max(0.5*by, -P.goalW/2 + P.robotR), P.goalW/2 - P.robotR);
base = [-4.15 0; -2.70 0; 0.5 -2.2; 1.0 2.4; ...   % blue 1-4
        kx 0; -1.0 -2.4; -1.5 2.4; -2.0 0.0];       % red 5-8
valid = true(1,8);
sc = struct('name',{},'kind',{},'expected',{},'ball',{},'pos',{},'valid',{}, ...
    'kicker',{},'receiver',{},'note',{});

% S1 clear goal, keeper on its line (keeper follows half the ball y)
b = [2.0 0.6];  p = base;  p(4,:) = b - [0.25 0];  p(5,:) = [kx keeperY(b(2))];
sc(end+1) = mk("S1 clear goal, keeper on line", "shot", "open", b, p, valid, 4, 0, ...
    "Only the keeper is in front of the ball; it covers the middle, the far side is free.");

% S2 empty goal: keeper has come out
b = [2.0 0.0];  p = base;  p(4,:) = b - [0.25 0];  p(5,:) = [2.9 1.9];
sc(end+1) = mk("S2 keeper off its line", "shot", "open", b, p, valid, 4, 0, ...
    "Keeper is outside its goal area; the whole mouth is free.");

% S3 field-player blocker right in front of the ball
b = [2.0 0.0];  p = base;  p(4,:) = b - [0.25 0];  p(5,:) = [kx 0];  p(6,:) = [2.6 0.0];
sc(end+1) = mk("S3 field-player blocker", "shot", "blocked (field player)", b, p, valid, 4, 0, ...
    "Red defender 0.6 m in front of the ball shadows the whole goal mouth.");

% S4 keeper blocker: narrow angle from the side, keeper at the near post
b = [3.6 2.0];  p = base;  p(4,:) = b - [0 0.25];  p(5,:) = [kx keeperY(b(2))];
sc(end+1) = mk("S4 keeper blocks narrow angle", "shot", "blocked (keeper)", b, p, valid, 4, 0, ...
    "Ball near the corner of the goal area; the keeper at the near post covers the narrow mouth.");

% S5a/S5b threshold: one blocker just outside / just inside the corridor
b = [1.5 0.0];  p = base;  p(4,:) = b - [0.25 0];  p(5,:) = [4.4 -2.9]; % keeper far away (ignored lane)
valid5 = valid;  valid5(5) = false;                                     % keeper unknown
Q = tdp.shotLaneParams(P);
[yb, xb] = thresholdBlocker(b, P, Q);
p(6,:) = [xb yb + 0.03];
sc(end+1) = mk("S5a threshold, blocker 3 cm outside", "shot", "open (just)", b, p, valid5, 4, 0, ...
    "Defender wall across the goal except one gap; blocker 3 cm outside the corridor edge.");
sc(end).pos = wall(sc(end).pos, xb);
p(6,:) = [xb yb - 0.03];
sc(end+1) = mk("S5b threshold, blocker 3 cm inside", "shot", "blocked (just)", b, p, valid5, 4, 0, ...
    "Same gap, blocker moved 6 cm into the corridor.");
sc(end).pos = wall(sc(end).pos, xb);

% S6 lane clear but too far
b = [-0.5 0.0];  p = base;  p(4,:) = b - [0.25 0];  p(5,:) = [kx 0];
sc(end+1) = mk("S6 lane clear but out of range", "shot", "blocked (range)", b, p, valid, 4, 0, ...
    "5 m from goal: a 1.8 m/s shot stops before the line.");

% S7 keeper out but a defender blocks: keeperOut and shot_open differ
b = [2.0 0.0];  p = base;  p(4,:) = b - [0.25 0];  p(5,:) = [3.0 -1.8];  p(6,:) = [2.6 0.0];
sc(end+1) = mk("S7 keeper out, defender blocks", "shot", "blocked (field player), keeper out", b, p, valid, 4, 0, ...
    "Keeper has left the goal but the defender still shadows the mouth.");

% S8 pass, receiver available
b = [0.5 -1.0];  p = base;  p(3,:) = b - [0.25 0];  p(4,:) = [2.3 0.5];  p(5,:) = [kx 0];
sc(end+1) = mk("S8 pass, receiver available", "pass", "open", b, p, valid, 3, 4, ...
    "MF passes to the ST 2.3 m away; no opponent near the lane.");

% S9 pass blocked
p(7,:) = 0.5*(b + p(4,:));
sc(end+1) = mk("S9 pass blocked", "pass", "blocked (opponent)", b, p, valid, 3, 4, ...
    "Same pass with a red player in the middle of the lane.");
end

function s = mk(name, kind, expected, ball, pos, valid, kicker, receiver, note)
s = struct('name', name, 'kind', kind, 'expected', expected, 'ball', ball, 'pos', pos, ...
    'valid', valid, 'kicker', kicker, 'receiver', receiver, 'note', note);
end

function [y, x] = thresholdBlocker(b, P, Q)
% A blocker at x = b+1.2 m whose centre sits exactly on the corridor edge
% of the straight shot to the goal centre (y > 0 side).
x = b(1) + 1.2;
s = x - b(1);
y = P.robotR + P.ballR + Q.margin + s*tan(Q.noiseSigmas*P.kickNoise);
end

function p = wall(p, xb)
% Red players 7 and 8 close the rest of the mouth on the other side, so the
% only usable aim points pass next to blocker 6.
p(7,:) = [xb -0.45];
p(8,:) = [xb + 1.4 -0.9];
end
