function P = v12TeamConfig(bluePolicy,redPolicy)
%V12TEAMCONFIG V1.2 match configuration with one named policy per team.
%   P = tdp.v12TeamConfig('defensive','aggressive') gives blue (team 1) the
%   defensive and red (team 2) the aggressive policy. Swap the two names for
%   the side-swapped match. Policies are 'neutral', 'aggressive', 'defensive'
%   (see tdp.v12PolicyParams).
%
%   Both teams share one coordinator (tdp.v1TeamPlans). The V1.2 fields only
%   carry numbers: row 1 or column 1 is blue, row 2 or column 2 is red, and
%   role columns are [GK DF MF ST]. P.v12Policy records the policy tags.
%
%   The V1.1 striker fallback is the common takeover rule for both teams, so
%   one failed kick no longer hands the attack to the midfielder for good.
P = tdp.v11FallbackConfig();
names = {bluePolicy,redPolicy};
P.v12Policy = zeros(1,2);
P.v12InterceptRole = false(2,4);
P.v12InterceptRange = zeros(2,4);
P.v12InterceptDepth = zeros(2,4);
P.v12ContestDepth = zeros(1,2);
P.v12OpponentClearance = zeros(1,2);
P.v12InterceptSpeed = zeros(1,2);
P.v12SwitchMargin = zeros(1,2);
P.v12ClearSpeed = zeros(1,2);
P.v12DefenderDist = zeros(1,2);
P.v12ScreenDepth = zeros(1,2);
P.v12CounterDepth = zeros(1,2);
P.v12CoverDefender = false(1,2);
for team = 1:2
    k = tdp.v12PolicyParams(names{team},P);
    P.v12Policy(team) = k.id;
    P.v12InterceptRole(team,:) = k.roles;
    P.v12InterceptRange(team,:) = k.range;
    P.v12InterceptDepth(team,:) = k.depth;
    P.v12ContestDepth(team) = k.contestDepth;
    P.v12OpponentClearance(team) = k.clearance;
    P.v12InterceptSpeed(team) = k.speed;
    P.v12SwitchMargin(team) = k.margin;
    P.v12ClearSpeed(team) = k.clearSpeed;
    P.v12DefenderDist(team) = k.defenderDist;
    P.v12ScreenDepth(team) = k.screenDepth;
    P.v12CounterDepth(team) = k.counterDepth;
    P.v12CoverDefender(team) = k.cover;
end
end
