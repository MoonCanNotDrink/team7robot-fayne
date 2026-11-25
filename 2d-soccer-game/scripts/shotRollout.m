function R = shotRollout(ball, angle, speed, pos, use, team, P)
%SHOTROLLOUT (B-Y3) Roll one kicked ball through STATIC robots, physics only.
%   R = shotRollout(ball, angle, speed, pos, use, team, P)
%   Uses the match ball model (tdp.ballModel) and the match contact model
%   (tdp.collisions) every P.dt; robots in pos (8x2) with use=true stand
%   still. team is the attacking team (its goal is at dirSign(team)*L/2).
%   Stops at the first of: goal, whole ball over a line (out), ball stopped,
%   20 s.
%
%   R.outcome  1 goal, 2 out over a line, 3 stopped in play
%   R.touched  id of the FIRST robot the ball touched (0 = none)
%   R.clean    goal without touching any robot (what shot_open predicts)
%   R.time     seconds until the outcome; R.path  Kx2 ball centres
b = struct('x', ball(1), 'y', ball(2), 'vx', speed*cos(angle), 'vy', speed*sin(angle));
robots = repmat(struct('x', 0, 'y', 0, 'th', 0, 'vxw', 0, 'vyw', 0, 'team', 0), 1, 0);
ids = zeros(1, 0);
for i = 1:size(pos, 1)
    if use(i)
        robots(end+1) = struct('x', pos(i,1), 'y', pos(i,2), 'th', 0, 'vxw', 0, 'vyw', 0, ...
            'team', 1 + (i > 4)); %#ok<AGROW>
        ids(end+1) = i; %#ok<AGROW>
    end
end
half = P.goalW/2;
if P.rulesProfile == TdpRules.CourseSimplified, half = P.goalW/2 - P.ballR; end
gx = P.dirSign(team)*P.L/2;
R.touched = 0;  R.outcome = 3;  R.time = 0;
path = zeros(ceil(20/P.dt) + 1, 2);  path(1,:) = ball;  k = 1;
state = struct('lastTouch', 0);
for step = 1:ceil(20/P.dt)
    b = tdp.ballModel(b, P);
    if ~isempty(robots)
        before = [b.x b.y];
        [~, b, ~] = tdp.collisions(robots, b, state, true, P);
        if R.touched == 0 && any(abs([b.x b.y] - before) > 0)
            % which robot: the one now exactly in contact
            d = hypot([robots.x] - b.x, [robots.y] - b.y);
            [~, m] = min(d);
            R.touched = ids(m);
        end
    end
    k = k + 1;  path(k,:) = [b.x b.y];
    R.time = step*P.dt;
    if abs(b.x) > P.L/2 + P.ballR
        if sign(b.x) == sign(gx) && abs(b.y) < half
            R.outcome = 1;
        else
            R.outcome = 2;
        end
        break
    elseif abs(b.y) > P.W/2 + P.ballR
        R.outcome = 2;
        break
    elseif b.vx == 0 && b.vy == 0
        R.outcome = 3;
        break
    end
end
R.path = path(1:k, :);
R.clean = R.outcome == 1 && R.touched == 0;
end
