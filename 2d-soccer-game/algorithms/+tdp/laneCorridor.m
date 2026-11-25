function [slack, blocker, along] = laneCorridor(a, b, pos, use, P, Q)
%LANECORRIDOR (B-Y2) Clearance of a ball path a -> b from robot discs.
%   [slack, blocker, along] = tdp.laneCorridor(a, b, pos, use, P, Q)
%   a, b   1x2 start (ball centre) and end point, world frame, metres.
%   pos    Nx2 robot centres; use Nx1/1xN logical, robots to test.
%   slack  smallest (distance - required width) over the tested robots;
%          >= 0 means no robot touches the corridor, Inf when none is ahead.
%   blocker index (into pos) of the robot with the smallest slack when
%          slack < 0, else 0.
%   along  distance from a to that robot's closest point on the path.
%
%   Required width at distance s along the path:
%       w(s) = robotR + ballR + Q.margin + s*tan(Q.noiseSigmas*P.kickNoise)
%   Robots behind the ball (projection t <= 0) are ignored: the ball moves
%   away from them. Robots beyond b are measured to the end point b.
%   Robots are assumed not to move while the ball travels (see the chapter,
%   "omitted dynamics").
slack = Inf;  blocker = 0;  along = 0;
ab = b - a;  L2 = ab(1)^2 + ab(2)^2;
if L2 < 1e-12, return; end
len = sqrt(L2);
grow = tan(Q.noiseSigmas*P.kickNoise);
base = P.robotR + P.ballR + Q.margin;
minSlack = Inf;
for k = 1:size(pos, 1)
    if ~use(k), continue; end
    t = ((pos(k,1) - a(1))*ab(1) + (pos(k,2) - a(2))*ab(2))/L2;
    if t <= 0, continue; end
    t = min(t, 1);
    c = a + t*ab;
    d = hypot(pos(k,1) - c(1), pos(k,2) - c(2));
    s = t*len;
    sl = d - (base + s*grow);
    if sl < minSlack
        minSlack = sl;
        along = s;
        if sl < 0, blocker = k; else, blocker = 0; end
    end
end
slack = minSlack;
end
