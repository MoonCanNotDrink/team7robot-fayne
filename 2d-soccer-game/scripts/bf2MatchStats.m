function S=bf2MatchStats(trace,tactics,bf,P)
%BF2MATCHSTATS (B-F3) Per-team attack statistics of one B-F2 match, for the A/B
% comparison of Test 2 (pass, shot and goal counts; B-F2-09 finding that "shot and
% pass both feasible" is rare is reported by bothFresh).
% Reads only logged data: trace = recordRows(out.records) (98 columns,
% replayColumns), tactics = recordRows(out.tacticsRecords) (171 columns,
% v1TraceColumns), bf = recordRows(out.bf2Records) (23 columns, bf2TraceColumns),
% and P (bf2Priority, dirSign). The audit result is not computed here: take
% report.passed and violations from v1MatchMetrics(trace,tactics,P).
%   P = tdp.bf2Config('shoot','pass'); P.seed = 1; P.Tmatch = 120;
%   rng(P.seed,'twister'); [in,~] = matchSimulationInput(P); out = sim(in);
%   S = bf2MatchStats(recordRows(out.records),recordRows(out.tacticsRecords), ...
%       recordRows(out.bf2Records),P);
% Fields, one struct element per team (S(1) blue, S(2) red):
%   team        1 blue, 2 red
%   priority    0 off, 1 Shoot-first, 2 Pass-first (P.bf2Priority)
%   goals       final score of the team
%   shots       referee shot count (kicks at shotSpeed or above by MF/ST)
%   kicks       kicks by the team's robots
%   passCommits fresh decisions that committed to a pass
%   shotCommits fresh decisions that committed to a shot
%   passKicked  pass kick confirmed (release 9)
%   received    pass received (release 11)
%   intercepted pass intercepted (release 12)
%   receiveTimeout receive timeout (release 13)
%   commitTimeout  commitment timeout (release 5)
%   softRelease    soft invalidation (release 7)
%   bothFresh   fresh decisions with shot and pass both feasible
%   bothFollowed of those, the action the team's priority asks for
%   possessionX mean ball x in the team's attacking direction while the team
%               is in Attack (m, positive = toward the opponent goal)
r=replayColumns();c=v1TraceColumns();b=bf2TraceColumns();
S=struct([]);
kicker=tactics(:,c.kicker);
for team=1:2
    pr=0;if isfield(P,'bf2Priority'),pr=P.bf2Priority(team);end
    want=2;if pr==2,want=3;end
    fresh=bf(:,b.fresh(team))==1;act=bf(:,b.action(team));rel=bf(:,b.release(team));
    both=fresh & bf(:,b.both(team))==1;
    ids=(1:4)+4*(team-1);
    attack=tactics(:,c.phase(team))==double(TdpTeamPhase.Attack);
    n=min(size(trace,1),size(tactics,1));
    x=P.dirSign(team)*trace(1:n,r.ballX);
    s=struct('team',team,'priority',pr, ...
        'goals',trace(end,r.score(team)),'shots',trace(end,r.shots(team)), ...
        'kicks',sum(ismember(kicker,ids)), ...
        'passCommits',sum(fresh & act==3),'shotCommits',sum(fresh & act==2), ...
        'passKicked',sum(rel==9),'received',sum(rel==11),'intercepted',sum(rel==12), ...
        'receiveTimeout',sum(rel==13),'commitTimeout',sum(rel==5),'softRelease',sum(rel==7), ...
        'bothFresh',sum(both),'bothFollowed',sum(both & act==want), ...
        'possessionX',mean(x(attack(1:n))));
    S=[S s]; %#ok<AGROW>
end
end
