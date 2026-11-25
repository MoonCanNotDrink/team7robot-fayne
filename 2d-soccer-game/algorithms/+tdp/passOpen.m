function S = passOpen(ball, team, pos, valid, kicker, receiver, P, Q)
%PASSOPEN (B-Y2) Is the pass lane from the ball to one team-mate clear?
%   S = tdp.passOpen(ball, team, pos, valid, kicker, receiver, P)
%   S = tdp.passOpen(ball, team, pos, valid, kicker, receiver, P, Q)
%   Same inputs as tdp.shotOpen; receiver is the id of the team-mate.
%   Geometry only: WHICH team-mate to choose is Fayne's B-F2 decision rule.
%
%   The pass is aimed at the receiver centre with the legacy actionSelection
%   speed min(maxKickSpeed, sqrt(2*ballDecel*d) + passExtraSpeed), so the ball
%   would just reach the receiver. The lane uses the same corridor as a shot
%   (tdp.laneCorridor), with the kicker and the receiver removed from the
%   blockers. S.open = lane clear AND receiver valid, a team-mate, not the
%   kicker, at most Q.maxPassDist away.
%
%   Fields: open, slack, blocker, dist, speed, pressured (an opponent within
%   Q.receiverClearance of the receiver; reported, not part of open),
%   reason 0 open, 1 receiver invalid/too far, 3 opponent blocks, 4 team-mate
%   blocks.
if nargin < 8, Q = tdp.shotLaneParams(P); end
mate = 1 + 4*(team - 1);
opp = 1 + 4*(2 - team);
S.open = false;  S.slack = -Inf;  S.blocker = 0;  S.dist = Inf;  S.speed = 0;
S.pressured = false;  S.reason = 1;
okReceiver = receiver >= mate && receiver <= mate + 3 && receiver ~= kicker && ...
    logical(valid(receiver));
if ~okReceiver, return; end
T = pos(receiver, :);
S.dist = hypot(T(1) - ball(1), T(2) - ball(2));
S.speed = min(P.maxKickSpeed, sqrt(2*P.ballDecel*S.dist) + Q.passExtraSpeed);
use = logical(valid(:));
use(receiver) = false;
if kicker >= 1 && kicker <= 8, use(kicker) = false; end
[S.slack, S.blocker] = tdp.laneCorridor(ball, T, pos, use, P, Q);
for i = opp:opp + 3
    if valid(i) && hypot(pos(i,1) - T(1), pos(i,2) - T(2)) <= Q.receiverClearance
        S.pressured = true;
    end
end
if S.dist > Q.maxPassDist
    S.reason = 1;
elseif S.slack >= 0
    S.open = true;  S.reason = 0;
elseif S.blocker >= opp && S.blocker <= opp + 3
    S.reason = 3;
else
    S.reason = 4;
end
end
