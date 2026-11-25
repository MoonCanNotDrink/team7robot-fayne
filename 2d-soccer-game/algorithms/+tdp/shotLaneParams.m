function Q = shotLaneParams(P)
%SHOTLANEPARAMS (B-Y2) Thresholds of the shot-lane / pass-lane geometry.
%   Q = tdp.shotLaneParams(P). Every value is in metres or m/s and is a
%   provisional engineering choice, documented in
%   docs/chapters/yu_task2_shot_pass_geometry.md. Fayne may copy them into P
%   once the Test 2 decision rule is agreed; until then nothing in the match
%   reads them.
%
%   The ball path is a swept disc of radius ballR. A robot (disc radius
%   robotR) blocks the path when its centre is closer to the path than
%       robotR + ballR + margin + s*tan(noiseSigmas*kickNoise)
%   where s is the distance travelled along the path. The last term widens
%   the corridor with the kick direction error of the actuator (P.kickNoise,
%   1 standard deviation = 6 deg).
Q.margin = 0.02;          % extra clearance on top of robot + ball radius
Q.noiseSigmas = 1.0;      % corridor widening in kick-noise standard deviations
Q.nTargets = 9;           % aim points spread across the goal mouth
Q.postMargin = 0.05;      % outermost aim point stays this far inside the post rule
Q.minArrivalSpeed = 0.5;  % a shot must still roll this fast at the goal line
Q.shotSpeed = P.shotSpeed;
Q.keeperBoxMargin = 0.10; % keeper counts as "off its line" outside goal area + this
Q.maxPassDist = 4.5;      % same limit as the legacy actionSelection pass
Q.passExtraSpeed = 0.2;   % legacy pass speed sqrt(2*a*d) + 0.2 m/s
Q.receiverClearance = P.kickRange;   % opponent this close to the receiver = pressured
end
