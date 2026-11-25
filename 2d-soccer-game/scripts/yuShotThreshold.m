function [T, C] = yuShotThreshold(nNoise, seed)
%YUSHOTTHRESHOLD (B-Y3) Calibrate the corridor width against simulated kicks.
%   [T, C] = yuShotThreshold()   % 400 noisy kicks per blocker position
%   One shot from (1.5, 0) to the goal centre (3.0 m, 1.8 m/s) with ONE
%   static robot placed s = 0.6, 1.2 or 2.0 m along the path and moved
%   sideways from 0 to 0.80 m. For each position:
%     slack   tdp.laneCorridor slack with the default Q (1 sigma corridor)
%     pClean  share of kicks with the match kick noise that score without
%             touching the robot (shotRollout, robots static)
%   C compares corridor widths noiseSigmas = 0, 0.5, 1, 1.5, 2: for each,
%   how often "slack >= 0" agrees with "pClean >= 0.80" over all positions.
if nargin < 1, nNoise = 400; end
if nargin < 2, seed = 11; end
P = tdp.v12Config();
Q = tdp.shotLaneParams(P);
rng(seed, 'twister');
noise = randn(nNoise, 1)*P.kickNoise;
b = [1.5 0];  goal = [P.L/2 0];
pos0 = repmat([-4 -2.9], 8, 1);         % everybody else far away
use = false(1, 8);  use(6) = true;      % one red field player
along = [0.6 1.2 2.0];
offs = 0:0.02:0.80;
rows = numel(along)*numel(offs);
s = zeros(rows,1); y = s; slack = s; pClean = s;  r = 0;
for a = along
    for o = offs
        r = r + 1;
        pos = pos0;  pos(6,:) = [b(1) + a, o];
        [sl, ~] = tdp.laneCorridor(b, goal, pos, use, P, Q);
        hit = 0;
        for i = 1:nNoise
            R = shotRollout(b, noise(i), Q.shotSpeed, pos, use, 1, P);
            hit = hit + R.clean;
        end
        s(r) = a;  y(r) = o;  slack(r) = sl;  pClean(r) = hit/nNoise;
    end
end
T = table(s, y, slack, pClean);
sig = [0 0.5 1 1.5 2];
agree = zeros(size(sig));  falseOpen = agree;  falseBlocked = agree;
obs = pClean >= 0.80;
for k = 1:numel(sig)
    Qk = Q;  Qk.noiseSigmas = sig(k);
    pred = false(rows,1);
    for r = 1:rows
        pos = pos0;  pos(6,:) = [b(1) + s(r), y(r)];
        pred(r) = tdp.laneCorridor(b, goal, pos, use, P, Qk) >= 0;
    end
    agree(k) = mean(pred == obs);
    falseOpen(k) = sum(pred & ~obs);
    falseBlocked(k) = sum(~pred & obs);
end
C = table(sig', agree', falseOpen', falseBlocked', ...
    'VariableNames', {'noiseSigmas','agreement','predictedOpenButUnreliable','predictedBlockedButReliable'});
end
