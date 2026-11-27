function tests = test_bf2_priority
%TEST_BF2_PRIORITY (B-F3) Shoot-first / Pass-first attack priority, B-F2-01..08.
% Function-level tests call tdp.v1TeamPlans on frozen observations, like Yu's
% B-Y3 scenarios: every decision of the B-F2 design has a deterministic test.
% Model-level tests run tdpV1Match (rebuild it first: tdpSetup; build_v1_match)
% and so also exercise the Stateflow skill, the authorisation layer and the plant.
% Run: tdpSetup; assertSuccess(runtests('tests/test_bf2_priority.m'));
tests = functiontests(localfunctions);
end

% ---- B-F2-01 configuration and the off switch ------------------------------------

function testConfigAndValidation(t)
P = tdp.bf2Config('shoot','pass');
verifyEqual(t,tdp.validateV1Config(P),P);
verifyEqual(t,P.bf2Priority,[1 2]);
verifyEqual(t,[P.bf2PassMin P.bf2PassMax],[0.5 2.0]);
verifyEqual(t,P.bf2SoftTicks,3);
verifyEqual(t,P.bf2CommitTimeout,2.0);
verifyEqual(t,P.bf2ReceiveTimeout,3.0);
verifyEqual(t,P.bf2PassMaxTurn,pi/2);
verifyEqual(t,P.bf2ContestRadius,0.5);
verifyEqual(t,tdp.bf2Config('pass','off').bf2Priority,[2 0]);
verifyFalse(t,isfield(tdp.v12Config(),'bf2Priority'),'B-F2 is opt-in.');
verifyError(t,@()tdp.bf2Config('shoot','dribble'),'TDP:BF2Priority');
bad = P; bad.bf2Priority = [1 3];
verifyError(t,@()tdp.validateV1Config(bad),'TDP:BF2Config');
bad = P; bad.bf2PassMin = 3;
verifyError(t,@()tdp.validateV1Config(bad),'TDP:BF2Config');
bad = P; bad.bf2SoftTicks = 0;
verifyError(t,@()tdp.validateV1Config(bad),'TDP:BF2Config');
bad = P; bad.bf2ReceiveTimeout = -1;
verifyError(t,@()tdp.validateV1Config(bad),'TDP:BF2Config');
% bf2PassMaxTurn must be a finite real scalar in (0, pi]
for value = {0,-0.1,pi+0.01,[1 1]}
    bad = P; bad.bf2PassMaxTurn = value{1};
    verifyError(t,@()tdp.validateV1Config(bad),'TDP:BF2Config');
end
% non-finite values are already refused by the generic check of tdp.validateConfig
for value = {Inf,NaN}
    bad = P; bad.bf2PassMaxTurn = value{1};
    verifyError(t,@()tdp.validateV1Config(bad),'TDP:UnknownParameter');
end
ok = P; ok.bf2PassMaxTurn = pi;
verifyEqual(t,tdp.validateV1Config(ok),ok);
% bf2ContestRadius must be a positive real scalar
for value = {0,-0.5,[0.5 0.5]}
    bad = P; bad.bf2ContestRadius = value{1};
    verifyError(t,@()tdp.validateV1Config(bad),'TDP:BF2Config');
end
end

function testOffMeansBaselinePlans(t)
% No bf2 fields, or a zero priority for the team: the plans and the memory are
% those of tdp.v12Config, whatever the ball and the robots do.
base = tdp.v12Config();
off = tdp.bf2Config('off','off');
half = tdp.bf2Config('off','shoot');
for ball = {[2 0],[0.5 0.4],[-1.5 1],[3.5 -0.8]}
    [P,o,q,f,m,s,v] = scenario(off,1,ball{1},[2.0 1.2]);
    [want,wantM] = tdp.v1TeamPlans(o,1,tdp.v1TeamMemory(),q,f,false,base,s,v);
    [got,gotM] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
    verifyEqual(t,got,want);
    verifyEqual(t,stripBf(gotM),wantM);
    verifyEqual(t,gotM.bfAction,0);
    % Red is switched on in "half" but blue is off: blue stays at baseline.
    [P2,o2,q2,f2,m2,s2,v2] = scenario(half,1,ball{1},[2.0 1.2]);
    [got2,gotM2] = tdp.v1TeamPlans(o2,1,m2,q2,f2,false,P2,s2,v2);
    verifyEqual(t,got2,want);
    verifyEqual(t,stripBf(gotM2),wantM);
end
end

% ---- B-F2-01 priority table ------------------------------------------------------

function testPriorityTable(t)
% rows: shot feasible, pass feasible, expected action for [shoot-first pass-first]
cases = {true,true,[2 3]; true,false,[2 2]; false,true,[3 3]; false,false,[1 1]};
for team = 1:2
    for k = 1:size(cases,1)
        for policy = 1:2
            P = tdp.bf2Config(names(policy),names(policy));
            [ball,mf] = geometry(cases{k,1},cases{k,2});
            [P,o,q,f,m,s,v] = scenario(P,team,ball,mf);
            [p,n] = tdp.v1TeamPlans(o,team,m,q,f,false,P,s,v);
            c = chaserLocal(team);
            msg = sprintf('team %d shot %d pass %d policy %d',team,cases{k,1},cases{k,2},policy);
            verifyEqual(t,n.bfShot,cases{k,1},msg);
            verifyEqual(t,n.bfPass~=0,cases{k,2},msg);
            verifyEqual(t,n.bfBoth,cases{k,1} && cases{k,2},msg);
            verifyEqual(t,n.bfAction,cases{k,3}(policy),msg);
            verifyTrue(t,n.bfFresh,msg);
            verifyEqual(t,p(c).type,TdpAction.Kick,msg);
        end
    end
end
end

function testActionIntents(t)
% Shot: live S.target at shotSpeed. Pass: the receiver centre at the pass speed.
% Dribble: the goal centre at dribbleSpeed (B-F2-04).
P0 = tdp.bf2Config('shoot','shoot');
dirs = [1 -1];
for team = 1:2
    d = dirs(team);
    [ball,mf] = geometry(true,true);
    [P,o,q,f,m,s,v] = scenario(P0,team,ball,mf);
    [p,n] = tdp.v1TeamPlans(o,team,m,q,f,false,P,s,v);
    c = chaserLocal(team);
    verifyEqual(t,p(c).speed,P.shotSpeed);
    verifyEqual(t,p(c).target(1),d*(P.L/2+P.ballR),'AbsTol',1e-12);
    verifyEqual(t,n.bfCommit,2);
    % A keeper standing on the centre line moves the aim point: no fixed goal centre.
    blocked = o;
    blocked = placeBlue(blocked,team,'opp',1,[4.15 0]);
    [p2,~] = tdp.v1TeamPlans(blocked,team,m,q,f,false,P,s,v);
    verifyNotEqual(t,p2(c).target(2),0,'The shot follows the best aim point (S.target).');

    P = tdp.bf2Config('pass','pass');
    [P,o,q,f,m,s,v] = scenario(P,team,ball,mf);
    [p,n] = tdp.v1TeamPlans(o,team,m,q,f,false,P,s,v);
    receiver = firstId(team)+2;
    want = d*mf;
    verifyEqual(t,p(c).target,want,'AbsTol',1e-12);
    dist = hypot(mf(1)-ball(1),mf(2)-ball(2));
    verifyEqual(t,p(c).speed,min(P.maxKickSpeed,sqrt(2*P.ballDecel*dist)+0.2),'AbsTol',1e-12);
    verifyLessThan(t,p(c).speed,P.shotSpeed,'A pass must not count as a shot.');
    verifyEqual(t,n.bfPass,receiver);
    verifyEqual(t,n.bfPassTo,receiver);

    [ball,mf] = geometry(false,false);
    [P,o,q,f,m,s,v] = scenario(P0,team,ball,mf);
    [p,n] = tdp.v1TeamPlans(o,team,m,q,f,false,P,s,v);
    verifyEqual(t,p(c).target,[d*P.L/2 0],'AbsTol',1e-12);
    verifyEqual(t,p(c).speed,P.dribbleSpeed);
    verifyEqual(t,n.bfCommit,0,'A dribble is not a commitment.');
end
end

% ---- B-F2-02 receiver candidates -------------------------------------------------

function testReceiverFilters(t)
P0 = tdp.bf2Config('pass','pass');
[ball,~] = geometry(true,true);
% distance window 0.5 .. 2.0 m
for k = 1:6
    dist = [0.49 0.5 0.51 1.99 2.0 2.01];
    expect = [false true true true true false];
    [P,o,q,f,m,s,v] = scenario(P0,1,ball,[ball(1) ball(2)+dist(k)]);
    [~,n] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
    verifyEqual(t,n.bfPass==3,expect(k),sprintf('distance %.2f',dist(k)));
end
% No walk round the ball (bf2PassMaxTurn, B-F2-09), only when the ball is
% contested. With the striker behind the ball, a receiver behind it (turn
% passer->ball->receiver about 163 deg) is a candidate while no opponent is near
% the ball ...
[P,o,q,f,m,s,v] = scenario(P0,1,ball,[ball(1)-1.0 ball(2)+0.3]);
[~,n] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,3,'Uncontested, a back pass round the ball is allowed.');
% ... but not once a visible opponent is within bf2ContestRadius of the ball
% (here 0.40 m, clear of the lane and of the receiver) ...
contested = placeBlue(o,1,'opp',4,[ball(1)+0.2 ball(2)-0.35]);
[~,n] = tdp.v1TeamPlans(contested,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,0,'Contested, a pass that needs a walk round the ball is not chosen.');
verifyNotEqual(t,n.bfAction,3);
% ... the radius is inclusive and an unseen opponent does not count ...
far = placeBlue(o,1,'opp',4,[ball(1)+0.3 ball(2)-0.41]);   % 0.508 m
[~,n] = tdp.v1TeamPlans(far,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,3,'An opponent just outside the radius does not contest.');
unseen = contested;
for i = 1:8, unseen(i).robotValid(8) = false; end
[~,n] = tdp.v1TeamPlans(unseen,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,3,'An unseen opponent does not contest the ball.');
% ... and the field direction is not limited: from beside the ball (turn about
% 73 deg) the same receiver, diagonally behind, is a candidate when contested.
oo = placeBlue(contested,1,'own',4,[ball(1) ball(2)-0.4]);
[~,n] = tdp.v1TeamPlans(oo,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,3,'A diagonal pass backwards from beside the ball is allowed.');
% pressured receiver: an opponent within kickRange of the receiver (in this
% geometry it also touches the lane end; the pressure rule is explicit anyway)
[P,o,q,f,m,s,v] = scenario(P0,1,ball,[ball(1) ball(2)+1.2]);
oo = placeBlue(o,1,'opp',2,[ball(1) ball(2)+1.2+P.kickRange-0.01]);
[~,n] = tdp.v1TeamPlans(oo,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,0,'A pressured receiver is not a candidate.');
oo = placeBlue(o,1,'opp',2,[ball(1) ball(2)+1.2+P.kickRange+0.2]);
[~,n] = tdp.v1TeamPlans(oo,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,3,'An opponent clear of the receiver and the lane does not press.');
% blocked lane
oo = placeBlue(o,1,'opp',2,[ball(1) ball(2)+0.6]);
[~,n] = tdp.v1TeamPlans(oo,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,0,'An opponent on the lane closes it.');
% the goalkeeper is never a receiver, even when close and open
oo = placeBlue(o,1,'own',1,[ball(1) ball(2)-1.0]);
oo = placeBlue(oo,1,'own',3,[-4 -2.9]);
[~,n] = tdp.v1TeamPlans(oo,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,0,'The goalkeeper is excluded.');
% unavailable or unobserved receiver
oo = o;
for i = 1:8, oo(i).robotValid(3) = false; end
[~,n] = tdp.v1TeamPlans(oo,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,0,'An unobserved team-mate is excluded.');
oo = o;
for i = 1:8, oo(i).robotStamp(3) = oo(i).tSim-P.maxObservationAge-0.01; end
[~,n] = tdp.v1TeamPlans(oo,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,0,'A stale observation is excluded.');
% nearest wins; an exact tie goes to the lower role number
[P,o,q,f,m,s,v] = scenario(P0,1,ball,[ball(1) ball(2)+1.5]);
oo = placeBlue(o,1,'own',2,[ball(1)+1.0 ball(2)]);
[~,n] = tdp.v1TeamPlans(oo,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,2,'The nearer defender wins.');
oo = placeBlue(o,1,'own',2,[ball(1)+1.5 ball(2)]);
[~,n] = tdp.v1TeamPlans(oo,1,m,q,f,false,P,s,v);
verifyEqual(t,n.bfPass,2,'An exact tie goes to the lower role number.');
end

function testKickOffStandOffDribbles(t)
% Kick-off stand-off: both strikers at the ball, each midfielder behind its own
% striker (turn about 150 deg), the other striker 0.25 m from the ball, so the
% ball is contested. Without the no-walk-round filter the striker
% committed to that back pass and walked round the ball into the other striker;
% with it, the striker dribbles and nothing is committed.
for team = 1:2
    P0 = tdp.bf2Config('pass','pass');
    [P,o,q,f,m,s,v] = scenario(P0,team,[0 0],[-1.0 0.6]);
    o = placeBlue(o,team,'own',4,[-0.25 0]);
    o = placeBlue(o,team,'opp',4,[0.25 0]);
    o = placeBlue(o,team,'opp',3,[1.0 -0.6]);
    [~,n] = tdp.v1TeamPlans(o,team,m,q,f,false,P,s,v);
    verifyEqual(t,n.phase,TdpTeamPhase.Attack,sprintf('team %d attacks',team));
    verifyEqual(t,n.bfAction,1,sprintf('team %d dribbles',team));
    verifyEqual(t,n.bfCommit,0,sprintf('team %d commits to nothing',team));
    verifyFalse(t,n.bfShot);
    verifyEqual(t,n.bfPass,0);
    % The same frozen world without the turn limit is a feasible back pass, so
    % the filter is what decides here.
    P.bf2PassMaxTurn = pi;
    [~,n] = tdp.v1TeamPlans(o,team,m,q,f,false,P,s,v);
    verifyEqual(t,n.bfAction,3,sprintf('team %d: without the limit it passes back',team));
end
end

% ---- B-F2-05 commitment ---------------------------------------------------------

function testCommitmentFollowsLiveTargets(t)
% The commitment is the action type and the receiver, never coordinates.
P0 = tdp.bf2Config('pass','pass');
[ball,mf] = geometry(true,true);
[P,o,q,f,m,s,v] = scenario(P0,1,ball,mf);
[p,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,[m.bfCommit m.bfPassTo],[3 3]);
mf2 = mf+[0.2 0.3];
o = placeBlue(at(o,1.05),1,'own',3,mf2);
[p,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,p(4).target,mf2,'AbsTol',1e-12,'The pass follows the receiver.');
verifyFalse(t,m.bfFresh,'No new decision while committed.');
verifyEqual(t,m.bfAction,3);
% A shot follows the live aim point.
P0 = tdp.bf2Config('shoot','shoot');
[P,o,q,f,m,s,v] = scenario(P0,1,ball,mf);
[p1,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
o = placeBlue(at(o,1.05),1,'opp',1,[4.15 0]);
[p2,m,~] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfCommit,2);
verifyNotEqual(t,p2(4).target(2),p1(4).target(2),'The shot follows S.target.');
end

function testNoSwapButUpgradeFromDribble(t)
% Pass-first commits to a shot when only the shot is feasible; a pass becoming
% feasible later does not swap it. A dribble upgrades at once.
P0 = tdp.bf2Config('pass','pass');
[ball,~] = geometry(true,false);
[P,o,q,f,m,s,v] = scenario(P0,1,ball,[-3 -2.5]);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfAction,2);
o = placeBlue(at(o,1.05),1,'own',3,[ball(1) ball(2)+1.2]);
[p,m,~] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfAction,2,'No shoot<->pass swap.');
verifyEqual(t,p(4).speed,P.shotSpeed);
verifyFalse(t,m.bfFresh);
% Dribble -> shoot, immediately.
[ball2,~] = geometry(false,false);
[P,o,q,f,m,s,v] = scenario(P0,1,ball2,[-3 -2.5]);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfAction,1);
o = at(o,1.05);
for i = 1:8, o(i).ball.x = ball(1); o(i).ball.y = ball(2); end
[~,m,~] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfAction,2,'A dribble upgrades to a shot at once.');
verifyEqual(t,m.bfCommit,2);
end

function testSoftInvalidationNeedsThreeTicksInApproachOrAlign(t)
P0 = tdp.bf2Config('shoot','shoot');
[ball,mf] = geometry(true,false);
closed = [0.5 0];                              % out of range: the lane is closed
for phase = [TdpPhase.Approach TdpPhase.Align]
    [P,o,q,f,m,s,v] = scenario(P0,1,ball,mf);
    s(:) = double(phase);
    [~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
    verifyEqual(t,m.bfCommit,2);
    o = moveBall(at(o,1.05),closed);
    [~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
    verifyEqual(t,[m.bfCommit m.bfBad],[2 1]);
    o = moveBall(at(o,1.10),ball);              % one good tick resets the count
    [~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
    verifyEqual(t,[m.bfCommit m.bfBad],[2 0]);
    for k = 1:2
        o = moveBall(at(o,1.10+0.05*k),closed);
        [~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
        verifyEqual(t,[m.bfCommit m.bfBad],[2 k]);
    end
    o = moveBall(at(o,1.25),closed);
    [~,m,~] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
    verifyEqual(t,m.bfRelease,7,'The third closed tick releases.');
    verifyEqual(t,m.bfAction,1,'And the re-choice finds nothing: dribble.');
    verifyEqual(t,m.bfCommit,0);
end
% In Search (not Approach/Align) the count does not run.
[P,o,q,f,m,s,v] = scenario(P0,1,ball,mf);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
s(:) = double(TdpPhase.Search);
for k = 1:5
    o = moveBall(at(o,1+0.05*k),closed);
    [~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
end
verifyEqual(t,[m.bfCommit m.bfBad],[2 0]);
end

function testKickPreparationIsFrozen(t)
% Once the skill is in Kick no geometric revoke may follow (a revoke costs a
% 0.5 s Recover): the plan stays a Kick and the commitment stays.
P0 = tdp.bf2Config('pass','pass');
[ball,mf] = geometry(true,true);
[P,o,q,f,m,s,v] = scenario(P0,1,ball,mf);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfCommit,3);
s(4) = double(TdpPhase.Kick);
q(4).kind = TdpAction.Kick;q(4).requestId = uint32(31);q(4).stamp = 1;q(4).validUntil = 2;
for k = 1:5
    o = moveBall(at(o,1+0.05*k),[0.5 0]);                 % geometry collapses
    for i = 1:8, o(i).robotValid(3) = true; end
    o = pressure(o,3,P);
    [p,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
    verifyEqual(t,p(4).type,TdpAction.Kick);
    verifyEqual(t,[m.bfCommit m.bfRelease],[3 0]);
    verifyEqual(t,m.bfKickId,uint32(31));
end
end

function testCommitmentTimeout(t)
P0 = tdp.bf2Config('shoot','shoot');
[ball,mf] = geometry(true,false);
[P,o,q,f,m,s,v] = scenario(P0,1,ball,mf);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfStart,1);
o = at(o,2.95);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfRelease,0);
o = at(o,3.0);
[p,m,~] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfRelease,5,'2.0 s without entering Kick forces a re-choice.');
verifyTrue(t,m.bfFresh);
verifyEqual(t,m.bfStart,3);
verifyEqual(t,p(4).type,TdpAction.Kick);
end

function testReleaseOnChaserChangeUnavailabilityAndTerminalFeedback(t)
P0 = tdp.bf2Config('shoot','shoot');
[ball,mf] = geometry(true,false);
% chaser changed (takeover hands the attack to the midfielder)
[P,o,q,f,m,s,v] = scenario(P0,1,ball,mf);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
m.takeover = true;v(2) = 1;
[~,m,~] = tdp.v1TeamPlans(at(o,1.05),1,m,q,f,false,P,s,v);
verifyEqual(t,m.chaser,3);
verifyEqual(t,m.bfRelease,2);
verifyEqual(t,m.bfPasser,3,'The new chaser makes its own decision.');
% striker unavailable: V1.1 takes over at once, the chaser changes
[P,o,q,f,m,s,v] = scenario(P0,1,ball,mf);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
o = at(o,1.05);
for i = 1:8, o(i).robotValid(4) = false; end
[~,m,~] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,[m.chaser m.bfRelease],[3 2]);
% terminal feedback of the committed shot
[P,o,q,f,m,s,v] = scenario(P0,1,ball,mf);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
s(4) = double(TdpPhase.Kick);
q(4).kind = TdpAction.Kick;q(4).requestId = uint32(7);q(4).stamp = 1;q(4).validUntil = 2;
[~,m,v] = tdp.v1TeamPlans(at(o,1.05),1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfKickId,uint32(7));
f(4) = struct('requestId',uint32(8),'status',TdpStatus.Done,'stamp',1.05);
[~,m,v] = tdp.v1TeamPlans(at(o,1.10),1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfCommit,2,'A result with another request ID is not ours.');
f(4) = struct('requestId',uint32(7),'status',TdpStatus.Failed,'stamp',1.10);
[~,m,~] = tdp.v1TeamPlans(at(o,1.15),1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfRelease,1);
verifyEqual(t,m.bfKickId,uint32(0));
end

function testDribbleKickFeedbackIsNotAPassConfirmation(t)
% A dribble kick that is still being prepared must not be mistaken for the pass.
P0 = tdp.bf2Config('pass','pass');
[ball,mf] = geometry(false,false);
[P,o,q,f,m,s,v] = scenario(P0,1,ball,[-3 -2.5]);
s(4) = double(TdpPhase.Kick);
q(4).kind = TdpAction.Kick;q(4).requestId = uint32(5);q(4).stamp = 1;q(4).validUntil = 2;
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfAction,1);
% Even if a pass becomes feasible while the dribble kick is prepared.
[ball,mf] = geometry(true,true);
o = placeBlue(moveBall(at(o,1.05),ball),1,'own',3,mf);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfCommit,0,'No commitment while a kick is being prepared.');
f(4) = struct('requestId',uint32(5),'status',TdpStatus.Done,'stamp',1.10);
[~,m,~] = tdp.v1TeamPlans(at(o,1.10),1,m,q,f,false,P,s,v);
verifyEqual(t,m.bfLock,0);
end

% ---- B-F2-03 / B-F2-06 pass hand-over and receive lock -----------------------------

function testPassConfirmedStartsReceiveLock(t)
for team = 1:2
    P0 = tdp.bf2Config('pass','pass');
    [ball,mf] = geometry(true,true);
    [P,o,q,f,m,s,v] = scenario(P0,team,ball,mf);
    own = firstId(team)-1;
    [~,m,v] = tdp.v1TeamPlans(o,team,m,q,f,false,P,s,v);
    verifyEqual(t,m.bfCommit,3);
    passer = own+4;receiver = own+3;
    s(passer) = double(TdpPhase.Kick);
    q(passer).kind = TdpAction.Kick;q(passer).requestId = uint32(21);q(passer).stamp = 1;q(passer).validUntil = 2;
    [~,m,v] = tdp.v1TeamPlans(at(o,1.05),team,m,q,f,false,P,s,v);
    verifyEqual(t,m.bfKickId,uint32(21));
    verifyEqual(t,m.bfLock,0,'Kick not confirmed yet.');
    f(passer) = struct('requestId',uint32(21),'status',TdpStatus.Done,'stamp',1.2);
    [p,m,~] = tdp.v1TeamPlans(at(o,1.25),team,m,q,f,false,P,s,v);
    verifyEqual(t,[m.bfLock m.bfReceiver],[1 receiver]);
    verifyEqual(t,m.bfRelease,9);
    verifyEqual(t,m.bfCommit,0);
    verifyEqual(t,m.chaser,receiver,'The chaser role passes to the receiver.');
    verifyEqual(t,p(3).type,TdpAction.Kick);
    verifyEqual(t,p(4).type,TdpAction.Goto,'The passer no longer asks for a kick.');
    verifyEqual(t,m.bfAction,4);
end
end

function testFailedOrRejectedPassStartsNoLock(t)
P0 = tdp.bf2Config('pass','pass');
[ball,mf] = geometry(true,true);
for status = [TdpStatus.Failed TdpStatus.Rejected]
    [P,o,q,f,m,s,v] = scenario(P0,1,ball,mf);
    [~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
    s(4) = double(TdpPhase.Kick);
    q(4).kind = TdpAction.Kick;q(4).requestId = uint32(21);q(4).stamp = 1;q(4).validUntil = 2;
    [~,m,v] = tdp.v1TeamPlans(at(o,1.05),1,m,q,f,false,P,s,v);
    f(4) = struct('requestId',uint32(21),'status',status,'stamp',1.2);
    [~,m,~] = tdp.v1TeamPlans(at(o,1.25),1,m,q,f,false,P,s,v);
    verifyEqual(t,[m.bfLock m.chaser],[0 4]);
    verifyEqual(t,m.bfRelease,1);
end
end

function testReceiveLockEvents(t)
P0 = tdp.bf2Config('pass','pass');
% received: within alignEnter of the ball, no ball-speed condition
[P,o,q,f,m] = lockScenario(P0,[2.0 0],[2.0 0.31]);
[p,m,~] = lockStep(P,o,q,f,m,1.3);
verifyEqual(t,[m.bfLock m.bfHold m.bfRelease],[0 3 11]);
verifyEqual(t,m.chaser,3,'The receiver keeps the role after receiving.');
% not yet: just outside alignEnter
[P,o,q,f,m] = lockScenario(P0,[2.0 0],[2.0 0.36]);
[~,m,~] = lockStep(P,o,q,f,m,1.3);
verifyEqual(t,[m.bfLock m.bfHold m.bfRelease],[1 0 0]);
% intercepted: a visible opponent at the ball and nearer than the receiver
[P,o,q,f,m] = lockScenario(P0,[2.3 0],[2.0 1.2]);
o = placeBlue(o,1,'opp',2,[2.3+P.kickRange-0.02 0]);
[~,m,~] = lockStep(P,o,q,f,m,1.3,o);
verifyEqual(t,[m.bfLock m.bfRelease m.chaser],[0 12 4]);
% an opponent nearer than the receiver but outside kickRange does not intercept
[P,o,q,f,m] = lockScenario(P0,[2.3 0],[2.0 1.2]);
o = placeBlue(o,1,'opp',2,[2.3+P.kickRange+0.05 0]);
[~,m,~] = lockStep(P,o,q,f,m,1.3,o);
verifyEqual(t,m.bfLock,1);
% an unseen opponent does not intercept
[P,o,q,f,m] = lockScenario(P0,[2.3 0],[2.0 1.2]);
o = placeBlue(o,1,'opp',2,[2.3+0.1 0]);
for i = 1:8, o(i).robotValid(6) = false; end
[~,m,~] = lockStep(P,o,q,f,m,1.3,o);
verifyEqual(t,m.bfLock,1);
% timeout 3 s after the confirmation
[P,o,q,f,m] = lockScenario(P0,[2.3 0],[2.0 1.2]);
[~,m,~] = lockStep(P,o,q,f,m,1.25+P.bf2ReceiveTimeout-0.05);
verifyEqual(t,m.bfLock,1);
[~,m,~] = lockStep(P,o,q,f,m,1.25+P.bf2ReceiveTimeout);
verifyEqual(t,[m.bfLock m.bfRelease m.chaser],[0 13 4]);
% receiver unavailable
[P,o,q,f,m] = lockScenario(P0,[2.3 0],[2.0 1.2]);
for i = 1:8, o(i).robotValid(3) = false; end
[~,m,~] = lockStep(P,o,q,f,m,1.3,o);
verifyEqual(t,[m.bfLock m.bfRelease],[0 6]);
% ball observation lost
[P,o,q,f,m] = lockScenario(P0,[2.3 0],[2.0 1.2]);
for i = 1:8, o(i).ballValid = false; end
[~,m,~] = lockStep(P,o,q,f,m,1.3,o);
verifyEqual(t,[m.bfLock m.bfRelease],[0 4]);
% game paused
[P,o,q,f,m] = lockScenario(P0,[2.3 0],[2.0 1.2]);
for i = 1:8, o(i).gameRunning = false; end
[~,m,~] = lockStep(P,o,q,f,m,1.3,o);
verifyEqual(t,[m.bfLock m.bfRelease],[0 8]);
verifyEqual(t,m.phase,TdpTeamPhase.Hold);
end

function testPhaseChangeDoesNotCancelTheLock(t)
% Test 2 keeps the half-based phase rule: the ball crossing the midline changes
% the team phase but the receive lock stays.
P0 = tdp.bf2Config('pass','pass');
[P,o,q,f,m] = lockScenario(P0,[0.5 0],[0.8 1.2]);
[~,m,~] = lockStep(P,o,q,f,m,1.3);
verifyEqual(t,m.bfLock,1);
o2 = moveBall(at(o,1.35),[-1.5 0]);
[p,m,~] = lockStep(P,o2,q,f,m,1.35,o2);
verifyEqual(t,m.phase,TdpTeamPhase.Defend);
verifyEqual(t,[m.bfLock m.chaser],[1 3]);
verifyEqual(t,p(3).type,TdpAction.Kick);
end

function testHoldKeepsRoleUntilItsKickEnds(t)
P0 = tdp.bf2Config('pass','pass');
[P,o,q,f,m] = lockScenario(P0,[2.0 0],[2.0 0.31]);
[~,m,v] = lockStep(P,o,q,f,m,1.3);
verifyEqual(t,m.bfHold,3);
% The receiver makes its own decision as chaser (alone: dribble).
for i = 1:8, o(i).ball.x = 2.3;o(i).ball.y = 0; end
[p,m,v] = lockStep(P,o,q,f,m,1.35,o,v);
verifyEqual(t,[m.chaser m.bfHold],[3 3]);
verifyEqual(t,p(3).type,TdpAction.Kick);
% Its kick ends with a terminal result: the striker takes the role back.
q(3).kind = TdpAction.Kick;q(3).requestId = uint32(40);q(3).stamp = 1.35;q(3).validUntil = 2.3;
f(3) = struct('requestId',uint32(40),'status',TdpStatus.Done,'stamp',1.5);
[~,m,~] = lockStep(P,o,q,f,m,1.55,o,v);
verifyEqual(t,[m.bfHold m.bfRelease m.chaser],[0 1 4],'The terminal result is the logged reason.');
end

% ---- model level -------------------------------------------------------------------

function testModelOffMatchesBaselineConfig(t)
% In the same model, a zero priority is bit-identical to the config without B-F2.
base = tdp.v12Config();base.Tmatch = 20;
off = tdp.bf2Config('off','off');off.Tmatch = 20;
[qa,ta] = runMatchRows(base);
[qb,tb] = runMatchRows(off);
verifyEqual(t,ta,tb);
verifyEqual(t,qa,qb);
end

function testModelPassIsKickedHandedOverAndReceived(t)
% Pass-first, nothing blocking, noise off: the striker commits to a pass to the
% midfielder, kicks it, the midfielder is chaser while the ball rolls and
% receives it. Both audits (v1MatchMetrics) must still hold.
c = v1TraceColumns();b = bf2TraceColumns();
P = tdp.bf2Config('pass','off');P.Tmatch = 14;P.kickNoise = 0;
world = fixtureWorld(P,[0.5 0],[0.1 0],[0.5 1.2]);
[q,tr,bf] = runFixture(P,world);
verifyTrue(t,any(bf(:,b.commit(1))==3),'The striker commits to a pass.');
confirmed = find(bf(:,b.release(1))==9,1);
assertNotEmpty(t,confirmed,'The kick must be confirmed by the Done feedback.');
verifyEqual(t,bf(confirmed,b.lock(1)),1);
verifyEqual(t,q(confirmed,c.chaser(1)),3,'The receiver is the chaser from the confirmation on.');
verifyTrue(t,any(bf(confirmed:end,b.hold(1))==3) || any(bf(confirmed:end,b.release(1))==11), ...
    'The midfielder receives the ball.');
[report,violations] = v1MatchMetrics(tr,q,P);
verifyEmpty(t,violations,'No audit violation.');
verifyTrue(t,report.passed);
end

function testModelShootFirstShootsPassFirstPasses(t)
% The same frozen start, both actions feasible: only the priority differs.
c = v1TraceColumns();b = bf2TraceColumns();
for policy = {'shoot','pass'}
    P = tdp.bf2Config(policy{1},'off');P.Tmatch = 8;P.kickNoise = 0;
    world = fixtureWorld(P,[2.0 0],[1.6 0],[2.0 1.2]);
    [q,tr,bf] = runFixture(P,world);
    fresh = find(bf(:,b.fresh(1))==1 & bf(:,b.both(1))==1,1);
    assertNotEmpty(t,fresh,'Both actions are feasible at the first decision.');
    want = 2;if strcmp(policy{1},'pass'),want = 3;end
    verifyEqual(t,bf(fresh,b.action(1)),want,policy{1});
    [~,violations] = v1MatchMetrics(tr,q,P);
    verifyEmpty(t,violations);
    verifyGreaterThan(t,max(q(:,c.kicker)),0);
end
end

function testModelFullMatchesKeepTheAudits(t)
% Test 2 in miniature: A (blue shoot, red pass) and B (swapped), 40 s each. The
% audit passes and, at every fresh decision with both actions feasible, the
% team's priority decides.
b = bf2TraceColumns();
for side = 1:2
    names = {'shoot','pass'};
    if side==2, names = {'pass','shoot'}; end
    P = tdp.bf2Config(names{1},names{2});P.Tmatch = 40;
    [q,tr,bf] = runMatchRows(P);
    [report,violations] = v1MatchMetrics(tr,q,P);
    verifyEmpty(t,violations);
    verifyTrue(t,report.passed);
    for team = 1:2
        both = bf(:,b.fresh(team))==1 & bf(:,b.both(team))==1;
        want = 2;if P.bf2Priority(team)==2,want = 3;end
        verifyTrue(t,all(bf(both,b.action(team))==want),'The priority decides.');
    end
end
end

% ---- helpers -----------------------------------------------------------------------

function n = names(policy)
list = {'shoot','pass'};
n = list{policy};
end

function id = firstId(team)
id = 1+4*(team-1);
end

function c = chaserLocal(~)
c = 4;
end

function [ball,mf] = geometry(shotFeasible,passFeasible)
% Blue frame. The shot is feasible from x=2 (3.5 m, in range) and not from x=0.5
% (4.1 m, out of range). The pass is feasible with the midfielder 1.2 m beside
% the ball, and not with the midfielder far away (-3, -2.5).
ball = [0.5 0];
if shotFeasible, ball = [2.0 0]; end
mf = [-3 -2.5];
if passFeasible, mf = [ball(1) ball(2)+1.2]; end
end

function [P,o,q,f,m,s,v] = scenario(P,team,ballBlue,mfBlue)
% Frozen world seen from team 1 or 2 (team 2 is the 180 degree rotation). Own
% GK/DF/MF/ST at their kick-off roles except MF at mfBlue and ST 0.4 m behind
% the ball; the opponent keeper stands off the lane, the other opponents far away.
T = tdpTemplates(P);w = T.World;w.referee.t = 1;
d = [1 -1];d = d(team);
own = [-4.15 0; -2.5 1.5; mfBlue; ballBlue(1)-0.4 ballBlue(2)];
opp = [4.15 2.6; -4.2 -2.9; -4.2 2.9; -3.9 -2.9];
first = 1+4*(team-1);second = 1+4*(2-team);
for k = 1:4
    w.robots(first+k-1).x = d*own(k,1);w.robots(first+k-1).y = d*own(k,2);
    w.robots(second+k-1).x = d*opp(k,1);w.robots(second+k-1).y = d*opp(k,2);
    w.robots(first+k-1).th = 0;w.robots(second+k-1).th = 0;
end
w.ball.x = d*ballBlue(1);w.ball.y = d*ballBlue(2);w.ball.vx = 0;w.ball.vy = 0;
o = repmat(T.Observation,1,8);
for i = 1:8, o(i) = Observation2D(w,i,1,P);end
o = at(o,1);
q = repmat(T.Request,1,8);
f = repmat(T.Feedback,1,8);
m = tdp.bf2TeamMemory();
s = repmat(double(TdpPhase.Approach),1,8);
v = tdp.v11FallbackMemory();
end

function o = at(o,tt)
for i = 1:8
    o(i).tSim = tt;o(i).robotStamp(:) = tt;o(i).ballStamp = tt;
end
end

function o = moveBall(o,blueXY)
for i = 1:8
    o(i).ball.x = blueXY(1);o(i).ball.y = blueXY(2);
end
end

function o = placeBlue(o,team,who,k,blueXY)
% Move own (who='own') or opposing (who='opp') robot k of 1..4 to a blue-frame point.
d = [1 -1];d = d(team);
if strcmp(who,'own'),id = firstId(team)+k-1;else,id = firstId(3-team)+k-1;end
for i = 1:8
    o(i).robots(id).x = d*blueXY(1);o(i).robots(id).y = d*blueXY(2);
end
end

function o = pressure(o,receiverId,P)
% An opponent right on the receiver (blue frame, team 1 only).
for i = 1:8
    r = o(i).robots(receiverId);
    o(i).robots(6).x = r.x+0.5*P.kickRange;o(i).robots(6).y = r.y;
end
end

function [P,o,q,f,m,s,v] = lockScenario(P0,ballBlue,mfBlue)
% A confirmed pass: ST kicked at t=1.2 (Done), the lock starts at 1.25.
[P,o,q,f,m,s,v] = scenario(P0,1,[2.0 0],[2.0 1.2]);
[~,m,v] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
s(4) = double(TdpPhase.Kick);
q(4).kind = TdpAction.Kick;q(4).requestId = uint32(21);q(4).stamp = 1;q(4).validUntil = 2;
[~,m,v] = tdp.v1TeamPlans(at(o,1.05),1,m,q,f,false,P,s,v);
f(4) = struct('requestId',uint32(21),'status',TdpStatus.Done,'stamp',1.2);
o = moveBall(placeBlue(at(o,1.25),1,'own',3,mfBlue),ballBlue);
[~,m,~] = tdp.v1TeamPlans(o,1,m,q,f,false,P,s,v);
assert(m.bfLock==1,'fixture: the pass confirmation must start the lock');
q(4).kind = TdpAction.Stop;q(4).requestId = uint32(0);
s(:) = double(TdpPhase.Approach);
end

function [p,m,v] = lockStep(P,o,q,f,m,tt,oIn,v)
if nargin<7 || isempty(oIn),oIn = o;end
if nargin<8,v = tdp.v11FallbackMemory();end
[p,m,v] = tdp.v1TeamPlans(at(oIn,tt),1,m,q,f,false,P,repmat(double(TdpPhase.Approach),1,8),v);
end

function m = stripBf(m)
m = rmfield(m,{'bfCommit','bfPasser','bfPassTo','bfStart','bfBad','bfKickId', ...
    'bfLock','bfReceiver','bfLockStart','bfHold','bfAction','bfFresh','bfShot', ...
    'bfPass','bfBoth','bfRelease'});
end

function world = fixtureWorld(P,ballXY,stXY,mfXY)
T = tdpTemplates(P);world = T.World;
world.ball.x = ballXY(1);world.ball.y = ballXY(2);
world.robots(4).x = stXY(1);world.robots(4).y = stXY(2);world.robots(4).th = 0;
world.robots(3).x = mfXY(1);world.robots(3).y = mfXY(2);world.robots(3).th = 0;
world.robots(2).x = -2.5;world.robots(2).y = 1.5;
% red parked far from every lane
world.robots(5).x = 4.15;world.robots(5).y = 2.6;
world.robots(6).x = -4.2;world.robots(6).y = -2.9;
world.robots(7).x = -4.2;world.robots(7).y = 2.9;
world.robots(8).x = -3.9;world.robots(8).y = -2.9;
end

function [q,tr,bf] = runFixture(P,world)
[input,model] = matchSimulationInput(P);
assert(strcmp(model,'tdpV1Match'));
input = input.setVariable('world0',world,'Workspace',model);
saved = rng;cleanup = onCleanup(@()rng(saved)); %#ok<NASGU>
rng(P.seed,'twister');   % kick noise uses the global stream (as in runMatch)
output = sim(input);
q = recordRows(output.tacticsRecords);tr = recordRows(output.records);bf = recordRows(output.bf2Records);
end

function [q,tr,bf] = runMatchRows(P)
[input,model] = matchSimulationInput(P);
saved = rng;cleanup = onCleanup(@()rng(saved)); %#ok<NASGU>
rng(P.seed,'twister');   % kick noise uses the global stream (as in runMatch)
output = sim(input);
q = recordRows(output.tacticsRecords);tr = recordRows(output.records);bf = recordRows(output.bf2Records);
end
