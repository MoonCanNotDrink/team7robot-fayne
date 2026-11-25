function tests = test_shot_lanes
%TEST_SHOT_LANES (B-Y2/B-Y3) Shot-lane and pass-lane geometry.
% Run: tdpSetup; runtests('tests/test_shot_lanes.m')
tests = functiontests(localfunctions);
end

function setupOnce(t)
t.TestData.P = tdp.v12Config();
t.TestData.Q = tdp.shotLaneParams(t.TestData.P);
t.TestData.far = repmat([-4.2 -2.9], 8, 1);   % parking spot behind every ball
end

function testCorridorWidthFormula(t)
P = t.TestData.P;  Q = t.TestData.Q;
pos = t.TestData.far;  use = false(1,8);  use(6) = true;
a = [0 0];  b = [3 0];
for s = [0.5 1.0 2.0]
    for y = [0.1 0.3 0.6]
        pos(6,:) = [s y];
        sl = tdp.laneCorridor(a, b, pos, use, P, Q);
        want = y - (P.robotR + P.ballR + Q.margin + s*tan(Q.noiseSigmas*P.kickNoise));
        verifyEqual(t, sl, want, 'AbsTol', 1e-12);
    end
end
end

function testRobotBehindBallIgnoredAndBeyondEndMeasuredToEnd(t)
P = t.TestData.P;  Q = t.TestData.Q;
pos = t.TestData.far;  use = false(1,8);  use(6) = true;
pos(6,:) = [-0.1 0];                                  % touching the ball from behind
verifyEqual(t, tdp.laneCorridor([0 0], [3 0], pos, use, P, Q), Inf);
pos(6,:) = [3.5 0];                                   % 0.5 m beyond the end point
sl = tdp.laneCorridor([0 0], [3 0], pos, use, P, Q);
verifyEqual(t, sl, 0.5 - (P.robotR + P.ballR + Q.margin + 3*tan(Q.noiseSigmas*P.kickNoise)), 'AbsTol', 1e-12);
end

function testKickerIsNotABlocker(t)
P = t.TestData.P;
pos = t.TestData.far;  b = [2 0];
pos(4,:) = [2.2 0];          % kicker standing "in front" of the ball (dribble contact)
pos(5,:) = [4.4 -2.9];       % keeper parked away
verifyTrue(t, tdp.shotOpen(b, 1, pos, true(1,8), 4, P).open);
verifyFalse(t, tdp.shotOpen(b, 1, pos, true(1,8), 0, P).open);
end

function testRangeLimit(t)
P = t.TestData.P;  Q = t.TestData.Q;
pos = t.TestData.far;  pos(5,:) = [4.4 -2.9];
dMax = (Q.shotSpeed^2 - Q.minArrivalSpeed^2)/(2*P.ballDecel);         % 3.74 m
gx = P.L/2 + P.ballR;                                  % goal decided here
S = tdp.shotOpen([gx - dMax + 0.05, 0], 1, pos, true(1,8), 0, P);
verifyTrue(t, S.open);  verifyGreaterThanOrEqual(t, S.arrivalSpeed, Q.minArrivalSpeed);
S = tdp.shotOpen([gx - dMax - 0.05, 0], 1, pos, true(1,8), 0, P);
verifyFalse(t, S.open);  verifyEqual(t, S.reason, 1);
end

function testPostRuleUsesKickNoiseCone(t)
% No robot near the lanes. The 2D pitch has no posts, so from the field
% almost every position can reach the mouth; what limits a shot is the kick
% noise cone at the goal line. With a 6 deg sigma a 3 m shot is fine; with
% a 30 deg sigma no aim point keeps the cone between the posts.
P = t.TestData.P;
pos = t.TestData.far;  pos(5,:) = [-4.4 2.9];      % keeper parked at the other end
S = tdp.shotOpen([1.8 0], 1, pos, true(1,8), 0, P);
verifyTrue(t, S.open);
P.kickNoise = deg2rad(30);
S = tdp.shotOpen([1.8 0], 1, pos, true(1,8), 0, P);
verifyFalse(t, S.open);  verifyEqual(t, S.reason, 5);  verifyEqual(t, S.blocker, 0);
% each aim point: half-width - |y| - d*tan(sigma)
Q = t.TestData.Q;  half = P.goalW/2 - P.ballR;
d = hypot(P.L/2 + P.ballR - 1.8, S.targetsY);
verifyEqual(t, S.targetSlack, half - abs(S.targetsY) - d*tan(Q.noiseSigmas*P.kickNoise), 'AbsTol', 1e-12);
end

function testKeeperOutIsSeparateFromLane(t)
P = t.TestData.P;
pos = t.TestData.far;  b = [2 0];
pos(5,:) = [3.0 -1.8];                              % keeper out of its goal area
S = tdp.shotOpen(b, 1, pos, true(1,8), 0, P);
verifyTrue(t, S.keeperOut);  verifyTrue(t, S.open);
pos(6,:) = [2.6 0];                                 % now a defender blocks
S = tdp.shotOpen(b, 1, pos, true(1,8), 0, P);
verifyTrue(t, S.keeperOut);  verifyFalse(t, S.open);  verifyEqual(t, S.reason, 3);
pos(5,:) = [4.15 0];  pos(6,:) = t.TestData.far(6,:);   % keeper home, lane by the side
S = tdp.shotOpen(b, 1, pos, true(1,8), 0, P);
verifyFalse(t, S.keeperOut);  verifyTrue(t, S.open);
verifyGreaterThan(t, abs(S.target(2)), 0.3);         % aims beside the keeper
end

function testUnknownKeeperIsIgnoredButReported(t)
P = t.TestData.P;
pos = t.TestData.far;  pos(5,:) = [4.15 0];
valid = true(1,8);  valid(5) = false;
S = tdp.shotOpen([2 0], 1, pos, valid, 0, P);
verifyFalse(t, S.keeperKnown);  verifyFalse(t, S.keeperOut);  verifyTrue(t, S.open);
end

function testRedIsPointMirrorOfBlue(t)
P = t.TestData.P;
rng(3, 'twister');
for trial = 1:200
    pos = [-4.5 + 9*rand(8,1), -3 + 6*rand(8,1)];
    b = [-1 + 5*rand, -2.5 + 5*rand];
    kicker = randi(4);
    S1 = tdp.shotOpen(b, 1, pos, true(1,8), kicker, P);
    mpos = -pos([5:8 1:4], :);                       % red sees the mirrored world
    S2 = tdp.shotOpen(-b, 2, mpos, true(1,8), kicker + 4, P);
    verifyEqual(t, S2.open, S1.open);
    verifyEqual(t, S2.reason, S1.reason);
    verifyEqual(t, S2.slack, S1.slack, 'AbsTol', 1e-9);
    verifyEqual(t, S2.target, -S1.target, 'AbsTol', 1e-9);
    verifyEqual(t, S2.keeperOut, S1.keeperOut);
end
end

function testPassReceiverRules(t)
P = t.TestData.P;
pos = t.TestData.far;  b = [0.5 -1];
pos(3,:) = b - [0.25 0];  pos(4,:) = [2.3 0.5];  pos(5,:) = [4.15 0];
S = tdp.passOpen(b, 1, pos, true(1,8), 3, 4, P);
verifyTrue(t, S.open);
verifyEqual(t, S.speed, min(P.maxKickSpeed, sqrt(2*P.ballDecel*S.dist) + 0.2), 'AbsTol', 1e-12);
verifyFalse(t, tdp.passOpen(b, 1, pos, true(1,8), 3, 3, P).open);   % to itself
verifyFalse(t, tdp.passOpen(b, 1, pos, true(1,8), 3, 6, P).open);   % to an opponent
valid = true(1,8);  valid(4) = false;
verifyFalse(t, tdp.passOpen(b, 1, pos, valid, 3, 4, P).open);       % receiver unknown
pos(4,:) = b + [4.6 0];
S = tdp.passOpen(b, 1, pos, true(1,8), 3, 4, P);
verifyFalse(t, S.open);  verifyEqual(t, S.reason, 1);                % too far
pos(4,:) = [2.3 0.5];  pos(7,:) = 0.5*(b + pos(4,:));
S = tdp.passOpen(b, 1, pos, true(1,8), 3, 4, P);
verifyFalse(t, S.open);  verifyEqual(t, S.reason, 3);  verifyEqual(t, S.blocker, 7);
end

function testScenarioPredictionsMatchSimulation(t)
% B-Y3: every controlled scenario, predicted vs simulated (static robots).
T = yuShotScenarios(300, 7);
verifyEqual(t, height(T), 10);
verifyTrue(t, all(T.agree), strjoin(T.name(~T.agree), ', '));
end
