function [block, oldBlock, onLine] = goalSideBlock(ball, team, defDist, P)
%GOALSIDEBLOCK (F4/Yu) Defender block point that never passes the ball.
%   [block, oldBlock, onLine] = tdp.goalSideBlock(ball, team, defDist, P)
%   ball = [x y] world. oldBlock reproduces the DF point of v1TeamPlans.m
%   (V1 and V1.2, "block = ownGoal + defenderDist*threat/distance"): a point
%   defDist from the own goal centre on the goal->ball line. When the ball is
%   closer than defDist that point lies BEYOND the ball (on the opponent
%   side), the known V1 anomaly in handoff section 6.3.
%
%   block uses the distance min(defDist, |ball-goal| - gap), gap =
%   robotR + ballR + 0.05 m, so the defender stays between the ball and its
%   own goal.
%
%   Ball at the goal line (task Y4, e.g. 5 cm from the line): there is no
%   room for a robot behind the ball, so the point above would be pushed in
%   front of the ball by the "inside the pitch" clamp. In that case
%   (onLine = true) the defender stands on the goal line instead,
%   f = -L/2 + robotR, beside the ball on the goal-centre side:
%       lat = s * min(defDist, |lat_ball| - gap),  s = sign(lat_ball) (+1 at 0)
%   For a ball near the goal centre this is gap to the side of the ball;
%   for a wide ball it is the V1 point (within 1 mm). The rule is continuous
%   with the normal case and gives the V1 point whenever the ball is far.
%   All points get the same clamps as V1 (own half, inside the pitch by one
%   robot radius). Team frame: red is the point mirror of blue.

d = P.dirSign(team);
ownGoal = [-d*P.L/2 0];
threat = ball - ownGoal;
dist = hypot(threat(1), threat(2));
fallback = [d*(-P.L/2 + defDist) 0];
gap = P.robotR + P.ballR + 0.05;
onLine = false;
if dist > 1e-9
    oldBlock = ownGoal + defDist*threat/dist;
    block = ownGoal + min(defDist, max(dist - gap, 0))*threat/dist;
    if d*block(1) < -P.L/2 + P.robotR          % no room behind the ball
        onLine = true;
        latBall = d*ball(2);
        s = 1;
        if latBall < 0, s = -1; end
        block = [d*(-P.L/2 + P.robotR), d*s*min(defDist, abs(latBall) - gap)];
    end
else
    oldBlock = fallback;
    block = fallback;
end
oldBlock = clampV1(oldBlock, d, P);
block = clampV1(block, d, P);
end

function p = clampV1(p, d, P)
p(1) = d*min(max(d*p(1), -P.L/2 + P.robotR), -P.robotR);
p = [min(max(p(1), -P.L/2 + P.robotR), P.L/2 - P.robotR), ...
     min(max(p(2), -P.W/2 + P.robotR), P.W/2 - P.robotR)];
end
