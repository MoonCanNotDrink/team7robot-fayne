function k = v12PolicyParams(name,P)
%V12POLICYPARAMS One team's V1.2 tactical parameters for a named policy.
%   k = tdp.v12PolicyParams(name,P) with name 'neutral', 'aggressive' or
%   'defensive'. P supplies the pitch and the approved V1 values that the
%   neutral policy reuses. All lengths are metres in the TEAM frame: x grows
%   towards the opponent goal, the own goal line is at -P.L/2. Role columns
%   are [GK DF MF ST].
%
%   Every value is a provisional engineering choice for the 9 x 6 m pitch and
%   the 0.25 m/s walk. The zone-like values (depth, contestDepth, defenderDist,
%   screenDepth, counterDepth) are placeholders until the pitch work package
%   confirms the zone boundaries.
%
%   id            numeric policy tag stored in P.v12Policy (0, 1, 2).
%   roles         which roles may become the Defend interceptor. GK never.
%   range         outside the must-contest zone a role only starts a challenge
%                 when it is at most this far from the ball.
%   depth         a role only challenges while the ball is at or behind this x.
%   contestDepth  must-contest line: with the ball at or behind this x the
%                 range limit and the opponent veto are waived, so a slow ball
%                 near the own goal always gets a challenger.
%   clearance     outside the must-contest zone an opponent this close to the
%                 ball vetoes a new challenge (0 = never veto).
%   speed         a new challenge only starts when the ball is at most this fast.
%   margin        hysteresis: the current interceptor keeps the job unless a
%                 team-mate is closer by more than this, and its own limits
%                 are relaxed by the same amount.
%   clearSpeed    kick speed of a Defend interceptor.
%   defenderDist  DF distance from the own goal centre on the ball-goal line.
%   screenDepth   MF screening depth inside the own half while defending.
%   counterDepth  ST (and idle MF) depth inside the own half.
%   cover         MF takes the DF blocking point while the DF challenges.
far = P.L/2;                 % "no zone limit": every ball position qualifies
switch name
    case 'neutral'
        k.id = 0;
        k.roles = [false true true false];
        % DF 1.0 m: four seconds of walking, so it stays near its blocking
        % line. MF 2.0 m: from the screening point (1.4 m deep) this reaches
        % the whole middle third of the own half, hypot(1.4,1.5) = 2.05 m.
        k.range = [0 1.0 2.0 0];
        k.depth = [-far far far -far];
        k.contestDepth = -P.L/6;                 % own third of the pitch
        % No opponent veto. With the approved V1 value (0.60 m) a 60 s trial
        % still left the defending team without a chaser for 72 % of Defend,
        % with its midfielder a median 0.32 m from the dribbled ball.
        k.clearance = 0;
        k.speed = P.v1LooseSpeed;                % approved V1 value, 0.35 m/s
        k.margin = P.roleHyst;                   % existing 0.40 m role hysteresis
        % Below shotSpeed so a clearance is never counted as a shot; 1.5 m/s
        % rolls 1.5^2/(2*ballDecel) = 2.8 m, from the own third to midfield.
        k.clearSpeed = 1.5;
        k.defenderDist = P.defDist;
        k.screenDepth = P.v1ScreenDepth;
        k.counterDepth = P.v1CounterDepth;
        k.cover = true;
    case 'aggressive'
        k.id = 1;
        k.roles = [false true true true];        % the striker also defends
        k.range = [0 2.0 2.5 3.0];               % only used in the midline band
        k.depth = [-far far far far];
        k.contestDepth = 0;                      % contest anywhere in the own half
        k.clearance = 0;                         % challenge a dribbling opponent
        k.speed = P.dribbleSpeed;                % also chase a dribbled ball
        k.margin = 0.2;                          % switch to a closer player sooner
        k.clearSpeed = 1.5;
        k.defenderDist = 2.6;                    % higher defensive line
        k.screenDepth = 0.8;                     % midfielder screens higher
        k.counterDepth = P.v1CounterDepth;
        k.cover = false;                         % nobody drops into the gap
    case 'defensive'
        k.id = 2;
        k.roles = [false true true false];
        k.range = [0 0.8 1.0 0];                 % short excursions only
        k.depth = [-far -1.0 0 -far];            % DF acts 1 m or more inside the own half
        k.contestDepth = -(P.L/2-P.penAreaL);    % penalty-area line
        % In front of that line an opponent within kicking range has the ball
        % and is not challenged; a loose touch is.
        k.clearance = P.kickRange;
        k.speed = P.v1LooseSpeed;
        k.margin = 0.6;                          % fewer interceptor changes
        k.clearSpeed = 1.5;
        k.defenderDist = 1.5;                    % deeper defender
        k.screenDepth = 2.0;                     % midfielder at the penalty-area line
        k.counterDepth = 1.0;                    % striker drops into the own half
        k.cover = true;
    otherwise
        error('TDP:V12Policy','Unknown policy "%s".',name);
end
end
