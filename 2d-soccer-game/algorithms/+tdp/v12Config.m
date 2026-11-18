function P = v12Config()
%V12CONFIG Common V1.2 baseline: both teams use the neutral policy.
% Adds the shared Defend interception rule to the approved V1 coordinator and
% uses the V1.1 striker fallback as the takeover rule. Without the v12 fields
% (tdp.v1Config, tdp.v11FallbackConfig) the coordinator behaves as before.
P = tdp.v12TeamConfig('neutral','neutral');
end
