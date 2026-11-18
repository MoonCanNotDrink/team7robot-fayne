function tests=test_v12_policies()
%TEST_V12_POLICIES Defend interceptor and per-team policies (V1.2).
% Function-level tests call tdp.v1TeamPlans directly on constructed
% observations. Model-level tests run tdpV1Match, so they also exercise the
% unchanged Stateflow controllers, the authorisation layer and the plant.
tests=functiontests(localfunctions);
end

% ---- configuration ----------------------------------------------------------

function testConfigValidation(t)
P=tdp.v12Config();
verifyEqual(t,tdp.validateV1Config(P),P);
verifyEqual(t,P.v12Policy,[0 0]);
verifyTrue(t,P.v11Fallback,'The V1.1 fallback is the common takeover rule.');
verifyFalse(t,isfield(tdp.v1Config(),'v12Policy'));
verifyFalse(t,isfield(tdp.v11FallbackConfig(),'v12Policy'));
Q=tdp.v12TeamConfig('defensive','aggressive');
verifyEqual(t,tdp.validateV1Config(Q),Q);
verifyEqual(t,Q.v12Policy,[2 1]);
verifyError(t,@()tdp.v12TeamConfig('neutral','reckless'),'TDP:V12Policy');
bad=P;bad.v12InterceptRole(2,1)=true;
verifyError(t,@()tdp.validateV1Config(bad),'TDP:V12Config','A goalkeeper may not intercept.');
bad=P;bad.v12ClearSpeed(1)=P.shotSpeed;
verifyError(t,@()tdp.validateV1Config(bad),'TDP:V12Config','A clearance must not count as a shot.');
bad=P;bad.v12InterceptRange=zeros(1,4);
verifyError(t,@()tdp.validateV1Config(bad),'TDP:V12Config');
bad=P;bad.v12SwitchMargin(2)=-0.1;
verifyError(t,@()tdp.validateV1Config(bad),'TDP:V12Config');
bad=P;bad.v12ContestDepth(1)=-P.L;
verifyError(t,@()tdp.validateV1Config(bad),'TDP:V12Config');
bad=P;bad.v12Policy(1)=7;
verifyError(t,@()tdp.validateV1Config(bad),'TDP:V12Config');
bad=P;bad.v1AutoRecovery=true;bad.v1RecoveryHealthySeconds=.5;bad.v1HealthDropWindow=zeros(8,2);
verifyError(t,@()tdp.validateV1Config(bad),'TDP:V11Config');
end

function testNeutralAttackMatchesV1(t)
% The neutral policy only changes the Defend phase.
[P,o,q,f,m]=scenario('neutral','neutral');
o=ballAt(o,[2 .5],[0 0]);
for team=1:2
    [want,wantM]=tdp.v1TeamPlans(o,team,m,q,f,false,tdp.v1Config());
    [got,gotM]=tdp.v1TeamPlans(o,team,m,q,f,false,P);
    if wantM.phase==TdpTeamPhase.Attack
        verifyEqual(t,got,want);verifyEqual(t,gotM,wantM);
    end
end
end

% ---- defender eligibility and selection ------------------------------------

function testDefenderChallengesContestedBallInOwnThird(t)
[P,o,q,f,m]=scenario('neutral','neutral');
o=ballAt(o,[-2.6 .5],[0 0]);
o=place(o,8,[-2.3 .5]);            % red striker at the ball
[old,oldM]=tdp.v1TeamPlans(o,1,m,q,f,false,tdp.v1Config());
verifyEqual(t,oldM.chaser,0,'Approved V1 leaves this ball unchallenged.');
[p,n]=tdp.v1TeamPlans(o,1,m,q,f,false,P);
verifyEqual(t,n.phase,TdpTeamPhase.Defend);
verifyEqual(t,n.chaser,2,'The defender is closest and must challenge.');
verifyEqual(t,[p.type],[TdpAction.Goto TdpAction.Kick TdpAction.Goto TdpAction.Goto]);
verifyEqual(t,[p.role],[TdpRole.GK TdpRole.DF TdpRole.MF TdpRole.ST]);
verifyEqual(t,p(2).target,[P.L/2 0]);
verifyEqual(t,p(2).speed,P.v12ClearSpeed(1));
verifyLessThan(t,p(2).speed,P.shotSpeed);
verifyEqual(t,p(1).target,old(1).target,'The goalkeeper keeps its V1 task.');
% The midfielder covers goal-side of the ball, on the goal-to-ball line.
goal=[-P.L/2 0];line=[-2.6 .5]-goal;cover=p(3).target-goal;
verifyEqual(t,cover(1)*line(2)-cover(2)*line(1),0,'AbsTol',1e-12);
verifyEqual(t,norm(cover),min(P.v12DefenderDist(1),norm(line)/2),'AbsTol',1e-12);
verifyLessThan(t,p(3).target(1),-2.6,'The cover must not stand in the clearance path.');
end

function testClosestEligibleAndTieBreak(t)
[P,o,q,f,m]=scenario('neutral','neutral');
o=place(o,2,[-2.6 .9]);o=place(o,3,[-2.6 -.9]);
[~,n]=tdp.v1TeamPlans(ballAt(o,[-2.6 0],[0 0]),1,m,q,f,false,P);
verifyEqual(t,n.chaser,2,'An exact tie goes to the lower role number.');
[~,n]=tdp.v1TeamPlans(ballAt(o,[-2.6 -.1],[0 0]),1,m,q,f,false,P);
verifyEqual(t,n.chaser,3,'The midfielder is closer.');
[~,n]=tdp.v1TeamPlans(ballAt(o,[-2.6 .1],[0 0]),1,m,q,f,false,P);
verifyEqual(t,n.chaser,2,'The defender is closer.');
% The same decisions are deterministic and mirrored for red.
o=place(o,6,[2.6 -.9]);o=place(o,7,[2.6 .9]);
[~,r]=tdp.v1TeamPlans(ballAt(o,[2.6 0],[0 0]),2,m,q,f,false,P);
[~,again]=tdp.v1TeamPlans(ballAt(o,[2.6 0],[0 0]),2,m,q,f,false,P);
verifyEqual(t,r.chaser,6);verifyEqual(t,again,r);
end

function testNoEligiblePlayer(t)
[P,o,q,f,m]=scenario('neutral','neutral');
o=place(o,2,[-2.6 .9]);o=place(o,3,[-2.6 -.9]);
% (a) A fast ball does not start a challenge, even near the own goal.
fast=ballAt(o,[-2.6 .5],[P.v12InterceptSpeed(1)+.01 0]);
[p,n]=tdp.v1TeamPlans(fast,1,m,q,f,false,P);
verifyEqual(t,n.chaser,0);verifyEqual(t,[p.type],repmat(TdpAction.Goto,1,4));
% (b) In front of the must-contest line a distant ball is left alone.
[~,n]=tdp.v1TeamPlans(ballAt(o,[-.8 2.5],[0 0]),1,m,q,f,false,P);
verifyEqual(t,n.chaser,0);
% (c) In front of the line an opponent inside the clearance vetoes a new
% challenge. The neutral policy has no veto; switch one on to test the rule.
near=place(ballAt(o,[-1.2 .9],[0 0]),2,[-1.9 .9]);
contested=place(near,8,[-1.0 .9]);
[~,n]=tdp.v1TeamPlans(near,1,m,q,f,false,P);
verifyEqual(t,n.chaser,2,'0.7 m away, inside the 1.0 m defender range.');
[~,n]=tdp.v1TeamPlans(contested,1,m,q,f,false,P);
verifyEqual(t,n.chaser,2,'Neutral: an opponent at the ball does not stop the challenge.');
veto=P;veto.v12OpponentClearance(1)=.3;
[~,n]=tdp.v1TeamPlans(contested,1,m,q,f,false,veto);
verifyEqual(t,n.chaser,0,'Opponent 0.2 m from the ball, inside a 0.3 m clearance.');
veto.v12OpponentClearance(1)=.15;
[~,n]=tdp.v1TeamPlans(contested,1,m,q,f,false,veto);
verifyEqual(t,n.chaser,2);
unknown=contested;for i=1:8,unknown(i).robotValid(7)=false;end
veto.v12OpponentClearance(1)=.15;
[~,n]=tdp.v1TeamPlans(unknown,1,m,q,f,false,veto);
verifyEqual(t,n.chaser,0,'An opponent of unknown position counts as being at the ball.');
% (d) No allowed role is available: nobody challenges, the keeper stays.
deep=ballAt(o,[-4.0 .2],[0 0]);
deep(2).selfValid=false;deep(3).selfValid=false;
[p,n]=tdp.v1TeamPlans(deep,1,m,q,f,false,P);
verifyEqual(t,n.chaser,0);
verifyEqual(t,[p.type],[TdpAction.Goto TdpAction.Stop TdpAction.Stop TdpAction.Goto]);
end

function testGoalkeeperNeverIntercepts(t)
for policy={'neutral','aggressive','defensive'}
    [P,o,q,f,m]=scenario(policy{1},policy{1});
    o=ballAt(o,[-4.1 .3],[0 0]);    % 0.3 m from the blue goalkeeper
    [p,n]=tdp.v1TeamPlans(o,1,m,q,f,false,P);
    verifyNotEqual(t,n.chaser,1);verifyNotEqual(t,n.chaser,0);
    verifyEqual(t,p(1).type,TdpAction.Goto);
    verifyEqual(t,sum([p.type]==TdpAction.Kick),1);
end
end

% ---- switching and hysteresis ----------------------------------------------

function testIncumbentKeepsTheJob(t)
[P,o,q,f,m]=scenario('neutral','neutral');
o=place(o,2,[-2.6 .9]);o=place(o,3,[-2.6 -.9]);
margin=P.v12SwitchMargin(1);
held=m;held.chaser=3;held.phase=TdpTeamPhase.Defend;held.basePhase=TdpTeamPhase.Defend;
% Defender closer by less than the margin: the midfielder keeps the job.
[~,n]=tdp.v1TeamPlans(ballAt(o,[-2.6 margin/2-.01],[0 0]),1,held,q,f,false,P);
verifyEqual(t,n.chaser,3);
% Defender closer by more than the margin: the job changes hands.
[~,n]=tdp.v1TeamPlans(ballAt(o,[-2.6 margin/2+.01],[0 0]),1,held,q,f,false,P);
verifyEqual(t,n.chaser,2);
% A ball that speeds up does not strip the current interceptor.
quick=ballAt(o,[-2.6 -.5],[P.dribbleSpeed 0]);
[~,n]=tdp.v1TeamPlans(quick,1,held,q,f,false,P);
verifyEqual(t,n.chaser,3);
[~,n]=tdp.v1TeamPlans(quick,1,m,q,f,false,P);
verifyEqual(t,n.chaser,0,'Without the job nobody starts on a fast ball.');
% In front of the line the range is relaxed by the margin for the holder only.
out=place(ballAt(o,[-1.0 .9],[0 0]),2,[-2.3 .9]);     % defender 1.3 m away
held.chaser=2;
[~,n]=tdp.v1TeamPlans(out,1,held,q,f,false,P);
verifyEqual(t,n.chaser,2);
[~,n]=tdp.v1TeamPlans(out,1,m,q,f,false,P);
verifyEqual(t,n.chaser,0);
far=place(out,2,[-2.45 .9]);                         % 1.45 m > 1.0 + 0.4
[~,n]=tdp.v1TeamPlans(far,1,held,q,f,false,P);
verifyEqual(t,n.chaser,0,'Beyond range plus margin the defender returns.');
end

function testJitterDoesNotSwitchInterceptor(t)
% The ball wobbles 2 cm either side of the equidistant line for 40 ticks.
[P,o,q,f,m]=scenario('neutral','neutral');
o=place(o,2,[-2.6 .9]);o=place(o,3,[-2.6 -.9]);
owner=zeros(1,40);
for k=1:40
    [~,m]=tdp.v1TeamPlans(ballAt(o,[-2.6 .02*(-1)^k],[0 0]),1,m,q,f,false,P);
    owner(k)=m.chaser;
end
verifyEqual(t,nnz(diff(owner)),0,'Hysteresis must absorb the wobble.');
verifyTrue(t,all(owner>0));
P.v12SwitchMargin(:)=0;m=tdp.v1TeamMemory();
for k=1:40
    [~,m]=tdp.v1TeamPlans(ballAt(o,[-2.6 .02*(-1)^k],[0 0]),1,m,q,f,false,P);
    owner(k)=m.chaser;
end
verifyEqual(t,nnz(diff(owner)),39,'Without the margin the same input flips every tick.');
end

% ---- striker fallback and hand-back ----------------------------------------

function testTakeoverAndInterceptorDoNotConflict(t)
[P,o,q,f,m]=scenario('neutral','neutral');
o=ballAt(o,[-2.6 .5],[0 0]);
s=repmat(double(TdpPhase.Approach),1,8);
v=tdp.v11FallbackMemory();
taken=m;taken.takeover=true;taken.failed=true;taken.chaser=3;
v(2)=o(1).tSim;                     % striker stable since now
[p,n,v]=tdp.v1TeamPlans(o,1,taken,q,f,false,P,s,v);
verifyTrue(t,n.takeover);verifyEqual(t,n.phase,TdpTeamPhase.Fallback);
verifyEqual(t,n.chaser,3,'During takeover the midfielder alone holds the grant.');
verifyEqual(t,[p.type],[TdpAction.Goto TdpAction.Goto TdpAction.Kick TdpAction.Stop]);
% After T_recover the striker is released and Defend picks its interceptor.
for i=1:8
    o(i).tSim=o(i).tSim+P.v11RecoverSeconds;
    o(i).robotStamp(:)=o(i).tSim;o(i).ballStamp=o(i).tSim;
end
[p,n,~]=tdp.v1TeamPlans(o,1,n,q,f,false,P,s,v);
verifyFalse(t,n.takeover);verifyFalse(t,n.failed);
verifyEqual(t,n.phase,TdpTeamPhase.Defend);
verifyEqual(t,n.chaser,2);
verifyEqual(t,[p.type],[TdpAction.Goto TdpAction.Kick TdpAction.Goto TdpAction.Goto]);
end

% ---- team-specific policies -------------------------------------------------

function testPoliciesDifferOnTheSameSituation(t)
% Ball 0.8 m inside blue's half with the red striker 0.2 m from it.
names={'neutral','aggressive','defensive'};
chaser=zeros(1,3);df=zeros(3,2);mf=zeros(3,2);st=zeros(3,2);deep=zeros(1,3);
for k=1:3
    [P,o,q,f,m]=scenario(names{k},'neutral');
    o=place(o,2,[-2.0 0]);o=place(o,3,[-1.6 .5]);o=place(o,4,[-.4 .8]);
    o=place(o,8,[-.6 0]);
    [p,n]=tdp.v1TeamPlans(ballAt(o,[-.8 0],[0 0]),1,m,q,f,false,P);
    verifyEqual(t,n.phase,TdpTeamPhase.Defend);
    verifyLessThanOrEqual(t,sum([p.type]==TdpAction.Kick),1);
    chaser(k)=n.chaser;
    % A shot-speed ball starts no challenge, so every player shows its position.
    [p,n]=tdp.v1TeamPlans(ballAt(o,[-.8 0],[P.shotSpeed 0]),1,m,q,f,false,P);
    verifyEqual(t,n.chaser,0);
    df(k,:)=p(2).target;mf(k,:)=p(3).target;st(k,:)=p(4).target;
    [~,n]=tdp.v1TeamPlans(ballAt(o,[-3.0 0],[0 0]),1,m,q,f,false,P);
    deep(k)=n.chaser;
end
verifyEqual(t,chaser,[3 4 0], ...
    'Neutral sends its midfielder, aggressive its closer striker, defensive holds.');
verifyEqual(t,deep,[2 2 2],'Every policy challenges a slow ball at its own penalty area.');
verifyGreaterThan(t,df(2,1),df(1,1));verifyLessThan(t,df(3,1),df(1,1));
verifyGreaterThan(t,mf(2,1),mf(1,1));verifyLessThan(t,mf(3,1),mf(1,1));
verifyLessThan(t,st(3,1),st(1,1),'The defensive striker drops deeper.');
end

function testEachTeamUsesItsOwnPolicy(t)
% Mirrored worlds: each team sees the ball 0.8 m inside its own half.
for swap=[false true]
    if swap
        [P,o,q,f,m]=scenario('aggressive','defensive');want=[4 0];
    else
        [P,o,q,f,m]=scenario('defensive','aggressive');want=[0 8];
    end
    o=place(o,2,[-2.0 0]);o=place(o,3,[-1.6 .5]);o=place(o,4,[-.4 .8]);
    o=place(o,6,[2.0 0]);o=place(o,7,[1.6 -.5]);o=place(o,8,[.4 -.8]);
    blue=place(ballAt(o,[-.8 0],[0 0]),8,[-.6 0]);
    red=place(ballAt(o,[.8 0],[0 0]),4,[.6 0]);
    [~,b]=tdp.v1TeamPlans(blue,1,m,q,f,false,P);
    [~,r]=tdp.v1TeamPlans(red,2,m,q,f,false,P);
    verifyEqual(t,[b.chaser r.chaser],want);
end
end

function testMirrorAndSingleKickAcrossPitch(t)
% With equal policies the two teams stay mirror images, as in V1.
saved=rng;cleanup=onCleanup(@()rng(saved)); %#ok<NASGU>
rng(1203,'twister');
for policy={'neutral','aggressive','defensive'}
    [P,o,q,f,m]=scenario(policy{1},policy{1});
    for sample=1:300
        ball=([P.L P.W]-2*P.ballR).*(rand(1,2)-.5);
        velocity=rand(1,2)-.5;
        blue=ballAt(o,ball,velocity);red=ballAt(o,-ball,-velocity);
        for idx=1:8
            xy=([P.L P.W]-2*P.robotR).*(rand(1,2)-.5);
            blue=place(blue,idx,xy);red=place(red,mod(idx+3,8)+1,-xy);
        end
        [p,b]=tdp.v1TeamPlans(blue,1,m,q,f,false,P);
        [r,d]=tdp.v1TeamPlans(red,2,m,q,f,false,P);
        verifyEqual(t,b.phase,d.phase);verifyEqual(t,[p.type],[r.type]);
        verifyEqual(t,[p.target],-[r.target],'AbsTol',1e-12);
        verifyLessThanOrEqual(t,sum([p.type]==TdpAction.Kick),1);
        verifyNotEqual(t,b.chaser,1);
        if b.chaser~=0,verifyEqual(t,d.chaser,b.chaser+4);else,verifyEqual(t,d.chaser,0);end
    end
end
end

% ---- model level: unchanged controllers, authorisation and plant -----------

function testModelDefenderWinsBallAndReturns(t)
% A still ball in blue's own third with every opponent far away.
c=v1TraceColumns();
for dt=[.05 .04]
    P=tdp.v12Config();P.dt=dt;P.Tmatch=40;P.kickNoise=0;
    % Red is switched off so the clearance rolls freely.
    P.v1UnavailableWindow(5:8,:)=repmat([0 P.Tmatch+P.dt],4,1);
    T=tdpTemplates(P);world=T.World;
    world.ball.x=-3.0;world.ball.y=1.0;
    [q,trace]=runFixture(P,world);
    first=find(q(:,c.kicker)>0,1);
    assertNotEmpty(t,first,'Somebody must reach the ball.');
    verifyEqual(t,q(first,c.kicker),2,'The blue defender clears the ball.');
    verifyEqual(t,q(first,c.speed(2)),P.v12ClearSpeed(1));
    before=q(1:first,c.skill(2));
    verifyTrue(t,all(ismember(double([TdpPhase.Approach TdpPhase.Align TdpPhase.Kick]),before)), ...
        'The defender uses the existing Approach, Align and Kick stages.');
    verifyTrue(t,all(q(1:first,c.chaser(1))==2 | q(1:first,c.chaser(1))==0));
    % Defensive recovery: once released the defender is sent back to a
    % blocking point in its own half and no longer asks to kick.
    released=find(q(first:end,c.chaser(1))~=2,1)+first-1;
    assertNotEmpty(t,released,'The defender must be released after the clearance.');
    verifyEqual(t,q(released,c.intent(2)),double(TdpAction.Goto));
    verifyLessThan(t,q(released,c.targetX(2)),0);
    verifyGreaterThan(t,trace(released,57),-P.L/6,'The clearance left the own third.');
    home=hypot(trace(end,2)-q(end,c.targetX(2)),trace(end,10)-q(end,c.targetY(2)));
    verifyLessThan(t,home,.3,'The defender is back on its blocking point at the end.');
    checkInvariants(t,q,c);
end
end

function testModelMatchesKeepEveryInvariant(t)
% Short natural matches: common baseline and both side assignments.
pairs={'neutral','neutral';'defensive','aggressive';'aggressive','defensive'};
c=v1TraceColumns();
for k=1:size(pairs,1)
    P=tdp.v12TeamConfig(pairs{k,1},pairs{k,2});P.Tmatch=60;
    [trace,q]=runMatch(P);
    [report,violations]=v1MatchMetrics(trace,q,P);
    verifyTrue(t,report.passed,sprintf('%s v %s: %s',pairs{k,1},pairs{k,2},strjoin({violations.name},',')));
    verifyFalse(t,any(ismember(q(:,c.chaser),[1 5]),'all'),'A goalkeeper never holds the grant.');
    checkInvariants(t,q,c);
    if k==1
        [again,q2]=runMatch(P);
        verifyEqual(t,again,trace);verifyEqual(t,q2,q);
    end
end
end

function testModelDefendingTeamNowChallenges(t)
% Seed 1, 60 s: under V1 and V1.1 red defends 85 % of the time and has no
% chaser in 99 % of those ticks. The common baseline must change that.
c=v1TraceColumns();
share=zeros(1,2);
configs={tdp.v11FallbackConfig(),tdp.v12Config()};
for k=1:2
    P=configs{k};P.Tmatch=60;
    [~,q]=runMatch(P);
    defend=q(:,c.phase)==double(TdpTeamPhase.Defend) & logical(q(:,c.running));
    share(k)=nnz(defend & q(:,c.chaser)==0)/max(1,nnz(defend));
end
verifyGreaterThan(t,share(1),.9);
verifyLessThan(t,share(2),.5);
end

function testModelForcedFailuresStillHandBack(t)
% The V1.1 takeover and hand-back remain reachable with the interceptor on.
c=v1TraceColumns();
P=tdp.v12Config();P.Tmatch=14;P.kickNoise=0;
P.v1FailWindow(4,:)=[0 P.Tmatch+P.dt];
% Red is switched off so that only the forced blue failures shape the run.
P.v1UnavailableWindow(5:8,:)=repmat([0 P.Tmatch+P.dt],4,1);
T=tdpTemplates(P);world=T.World;
world.ball.x=2;world.ball.y=0;
world.robots(4).x=world.ball.x-.245;world.robots(4).y=0;world.robots(4).th=0;
q=runFixture(P,world);
entry=find(diff(q(:,c.takeover(1)))==1,1)+1;
assertNotEmpty(t,entry,'Three forced striker failures must take over.');
leave=find(diff(q(entry:end,c.takeover(1)))==-1,1)+entry;
assertNotEmpty(t,leave,'The striker must be handed back.');
verifyGreaterThanOrEqual(t,q(leave,c.time)-q(entry,c.time),P.v11RecoverSeconds-1e-9);
verifyEqual(t,q(entry,c.chaser(1)),3);
checkInvariants(t,q,c);
end

% ---- helpers ----------------------------------------------------------------

function [P,o,q,f,m]=scenario(bluePolicy,redPolicy)
P=tdp.v12TeamConfig(bluePolicy,redPolicy);T=tdpTemplates(P);w=T.World;
w.referee.t=1;
o=repmat(T.Observation,1,8);
for i=1:8,o(i)=Observation2D(w,i,1,P);end
q=repmat(T.Request,1,8);
f=repmat(T.Feedback,1,8);
m=tdp.v1TeamMemory();
end

function o=ballAt(o,xy,velocity)
for i=1:8
    o(i).ball.x=xy(1);o(i).ball.y=xy(2);
    o(i).ball.vx=velocity(1);o(i).ball.vy=velocity(2);
end
end

function o=place(o,robot,xy)
for i=1:8
    o(i).robots(robot).x=xy(1);o(i).robots(robot).y=xy(2);
end
end

function [tactics,trace]=runFixture(P,world)
[input,model]=matchSimulationInput(P);
assert(strcmp(model,'tdpV1Match'));
input=input.setVariable('world0',world,'Workspace',model);
output=sim(input);tactics=recordRows(output.tacticsRecords);trace=recordRows(output.records);
end

function checkInvariants(t,q,c)
for team=1:2
    ids=(1:4)+4*(team-1);
    kicks=ismember(q(:,c.kind(ids)),double([TdpAction.Kick TdpAction.Tap]));
    verifyLessThanOrEqual(t,max(sum(kicks,2)),1,'At most one kick request per team.');
    owner=q(:,c.chaser(team));
    for k=1:4
        verifyFalse(t,any(kicks(:,k) & owner~=ids(k)),'Only the chaser may request a kick.');
    end
    verifyFalse(t,any(q(:,c.kicker)==ids(1)),'The goalkeeper never kicks.');
end
end
