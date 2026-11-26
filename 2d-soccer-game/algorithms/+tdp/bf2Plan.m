function [m,use,target,speed] = bf2Plan(m,source,team,first,available,ball,requests,feedback,skills,t,P)
%BF2PLAN (B-F3) Attack-action choice of one team: shoot, pass or dribble.
% Called by tdp.v1TeamPlans after the baseline chaser has been chosen, only when
% the team's P.bf2Priority is non-zero. It owns the B-F2 state of tdp.bf2TeamMemory.
%   use     true: the chaser's Kick intent is (target, speed). false: keep the
%           baseline plan (Defend phase, no chaser, chaser unavailable).
%   m       memory with the chaser possibly handed to a receiver (receive lock
%           or hold) and the per-tick log fields filled in.
%
% Decisions implemented (Notion B-F2-01..06, source: team7robot main 5eaa413):
%  01 Shoot-first / Pass-first only change the order of the two feasible actions.
%     Neither feasible: dribble toward the goal with P.dribbleSpeed.
%  02 Receiver: not the goalkeeper, not the passer, available and valid, distance
%     in [bf2PassMin bf2PassMax], passOpen.open, not pressured; nearest to the ball.
%     B-F3 addition: at a fresh decision the turn passer->ball->receiver is at
%     most bf2PassMaxTurn (the passer never walks round the ball).
%  03 After the passer's Done feedback the chaser role goes to the receiver.
%  05 Commit to the action type and the receiver ID only. Soft invalidation
%     (bf2SoftTicks closed ticks) only in Approach/Align. No shoot<->pass swap.
%     A dribble upgrades to shoot or pass at once. Kick preparation is frozen.
%     bf2CommitTimeout without entering Kick forces a re-choice.
%  06 Kick confirmed = passer's matching Done. Received = receiver within
%     alignEnter of the ball. Intercepted = a visible opponent within kickRange of
%     the ball and nearer than the receiver. Timeout bf2ReceiveTimeout.
%
% Release reasons (m.bfRelease): 1 terminal feedback, 2 chaser changed or none,
% 3 left Attack, 4 ball lost, 5 commitment timeout, 6 receiver or chaser
% unavailable, 7 soft invalidation, 8 game paused, 9 pass kick confirmed (hand
% over), 11 received, 12 intercepted, 13 receive timeout.
% Action codes (m.bfAction): 0 none, 1 dribble, 2 shoot, 3 pass, 4 receive.
use = false;
target = [0 0];
speed = 0;
direction = P.dirSign(team);
goal = [direction*P.L/2 0];

pos = zeros(8,2);
valid = false(1,8);
for i = 1:8
    pos(i,:) = [source.robots(i).x source.robots(i).y];
    valid(i) = validRobot(source,i,t,P);
end

% ---- receive lock: ball in flight to the receiver ------------------------------
if m.bfLock==1
    r = m.bfReceiver;
    lr = r-first+1;
    reason = 0;
    if lr<2 || lr>4
        reason = 6;
    elseif ~available(lr) || ~valid(r)
        reason = 6;
    else
        rb = hypot(pos(r,1)-ball(1),pos(r,2)-ball(2));
        if intercepted(pos,valid,team,rb,ball,P)
            reason = 12;
        elseif rb<=P.alignEnter+1e-12
            reason = 11;
        elseif t-m.bfLockStart>=P.bf2ReceiveTimeout-1e-9
            reason = 13;
        end
    end
    if reason~=0
        m.bfRelease = reason;
        m.bfLock = 0;
        if reason==11
            m.bfHold = r;
        end
        m.bfReceiver = 0;
    end
end

% ---- hold: the receiver keeps the role until its next kick ends -----------------
if m.bfHold~=0
    h = m.bfHold;
    lh = h-first+1;
    reason = 0;
    if lh<2 || lh>4
        reason = 6;
    elseif ~available(lh) || ~valid(h) || (lh==4 && m.takeover)
        reason = 6;
    elseif m.phase==TdpTeamPhase.Defend
        reason = 3;
    elseif finishedKick(requests(h),feedback(h),t)
        reason = 1;
    end
    if reason~=0
        m.bfRelease = reason;
        m.bfHold = 0;
    end
end

% The receiver keeps the chaser role while the lock or the hold is active.
if m.bfLock==1
    m.chaser = m.bfReceiver;
elseif m.bfHold~=0
    m.chaser = m.bfHold;
end
c = m.chaser;

if m.bfLock==1
    % Receiving: chase the ball with the ordinary dribble kick, no new decision.
    if m.bfCommit~=0
        m = dropCommit(m,2);
    end
    use = true;
    target = goal;
    speed = P.dribbleSpeed;
    m.bfAction = 4;
    return
end
if m.phase==TdpTeamPhase.Defend
    if m.bfCommit~=0
        m = dropCommit(m,3);
    end
    return
end
if c==0
    if m.bfCommit~=0
        m = dropCommit(m,2);
    end
    return
end
lc = c-first+1;
if ~available(lc)
    if m.bfCommit~=0
        m = dropCommit(m,6);
    end
    return
end

kickNow = skills(c)==double(TdpPhase.Kick);
Q = tdp.shotLaneParams(P);
Q.maxPassDist = P.bf2PassMax;

% ---- commitment bookkeeping -----------------------------------------------------
if m.bfCommit~=0
    if m.bfPasser~=c
        m = dropCommit(m,2);
    else
        if m.bfKickId==uint32(0) && kickNow && kickRequest(requests(c))
            m.bfKickId = requests(c).requestId;
        end
        if m.bfKickId~=uint32(0) && finishedKick(requests(c),feedback(c),t) && ...
                feedback(c).requestId==m.bfKickId
            if m.bfCommit==3 && feedback(c).status==TdpStatus.Done
                m = handOver(m,first,available,valid,t);
            else
                m = dropCommit(m,1);
            end
        elseif m.bfCommit==3 && ~receiverUsable(m.bfPassTo,first,c,available,valid,m.takeover)
            m = dropCommit(m,6);
        end
    end
end
if m.bfLock==1
    % The pass has just been confirmed: the receiver is the chaser from now on.
    m.chaser = m.bfReceiver;
    use = true;
    target = goal;
    speed = P.dribbleSpeed;
    m.bfAction = 4;
    return
end

% Geometry of the committed action. Frozen once the skill is in Kick.
if m.bfCommit~=0 && ~kickNow
    [laneOk,aim,aimSpeed] = committedLane(m,pos,valid,ball,team,c,P,Q);
    phaseNow = skills(c);
    counting = phaseNow==double(TdpPhase.Approach) || phaseNow==double(TdpPhase.Align);
    if counting
        if laneOk
            m.bfBad = 0;
        else
            m.bfBad = m.bfBad+1;
        end
    end
    if t-m.bfStart>=P.bf2CommitTimeout-1e-9
        m = dropCommit(m,5);
    elseif m.bfBad>=P.bf2SoftTicks
        m = dropCommit(m,7);
    else
        use = true;
        target = aim;
        speed = aimSpeed;
        m.bfAction = m.bfCommit;
        return
    end
end

if m.bfCommit~=0
    % Committed and inside the kick preparation: nothing may change.
    use = true;
    m.bfAction = m.bfCommit;
    if m.bfCommit==2
        target = goal;
        speed = P.shotSpeed;
    else
        rcv = m.bfPassTo;
        if valid(rcv)
            target = pos(rcv,:);
        else
            target = goal;
        end
        speed = min(P.maxKickSpeed,sqrt(2*P.ballDecel*hypot(target(1)-ball(1),target(2)-ball(2)))+Q.passExtraSpeed);
    end
    return
end

% ---- no commitment: dribble, or a fresh choice ---------------------------------
use = true;
if kickNow
    % A dribble kick is being prepared: a new commitment now would be confused
    % with that kick's feedback, so the decision waits for the next kick.
    target = goal;
    speed = P.dribbleSpeed;
    m.bfAction = 1;
    return
end

S = tdp.shotOpen(ball,team,pos,valid,c,P,Q);
best = 0;
bestDist = Inf;
bestSpeed = 0;
for local = 2:4
    id = first+local-1;
    if ~receiverUsable(id,first,c,available,valid,m.takeover), continue; end
    po = tdp.passOpen(ball,team,pos,valid,c,id,P,Q);
    if po.open && ~po.pressured && po.dist>=P.bf2PassMin && po.dist<bestDist && ...
            noWalkAround(pos(c,:),ball,pos(id,:),P)
        best = id;
        bestDist = po.dist;
        bestSpeed = po.speed;
    end
end
shotOk = S.open;
passOk = best~=0;
m.bfFresh = true;
m.bfShot = shotOk;
m.bfPass = best;
m.bfBoth = shotOk && passOk;
action = 1;
if shotOk && passOk
    if P.bf2Priority(team)==1
        action = 2;
    else
        action = 3;
    end
elseif shotOk
    action = 2;
elseif passOk
    action = 3;
end
m.bfAction = action;
if action==2
    target = S.target;
    speed = P.shotSpeed;
    m = makeCommit(m,2,c,0,t);
elseif action==3
    target = pos(best,:);
    speed = bestSpeed;
    m = makeCommit(m,3,c,best,t);
else
    target = goal;
    speed = P.dribbleSpeed;
end
end

% ---- helpers --------------------------------------------------------------------

function m = makeCommit(m,action,passer,receiver,t)
m.bfCommit = action;
m.bfPasser = passer;
m.bfPassTo = receiver;
m.bfStart = t;
m.bfBad = 0;
m.bfKickId = uint32(0);
end

function m = dropCommit(m,reason)
% Clear the commitment and log why. The first reason of a tick is kept: it is
% the root cause (a hold ended by terminal feedback also changes the chaser, and
% that must not be logged as a second, different release).
m.bfCommit = 0;
m.bfPasser = 0;
m.bfPassTo = 0;
m.bfBad = 0;
m.bfKickId = uint32(0);
if m.bfRelease==0
    m.bfRelease = reason;
end
end

function m = handOver(m,first,available,valid,t)
% The pass was kicked: start the receive lock for the committed receiver.
r = m.bfPassTo;
lr = r-first+1;
m = dropCommit(m,9);
if lr>=2 && lr<=4 && available(lr) && valid(r)
    m.bfLock = 1;
    m.bfReceiver = r;
    m.bfLockStart = t;
else
    m.bfRelease = 6;
end
end

function yes = noWalkAround(passer,ball,receiver,P)
% The passer need not walk round the ball: the angle between passer->ball and
% ball->receiver is at most P.bf2PassMaxTurn. Used for fresh choices only; a
% committed pass is not released by it (committedLane does not use it). A passer
% on the ball (|u| about 0) passes the test.
yes = true;
if ~isfield(P,'bf2PassMaxTurn'), return; end
u = ball(:)'-passer(:)';
w = receiver(:)'-ball(:)';
if norm(u)<1e-6 || norm(w)<1e-6, return; end
cosTurn = max(-1,min(1,dot(u,w)/(norm(u)*norm(w))));
yes = acos(cosTurn)<=P.bf2PassMaxTurn+1e-12;
end

function yes = receiverUsable(id,first,chaser,available,valid,takeover)
% Not the goalkeeper or the passer; available; the striker is stopped in takeover.
local = id-first+1;
yes = local>=2 && local<=4 && id~=chaser && available(local) && valid(id) && ~(local==4 && takeover);
end

function [laneOk,aim,aimSpeed] = committedLane(m,pos,valid,ball,team,c,P,Q)
% Live geometry of the committed action: shot -> S.target, pass -> receiver now.
if m.bfCommit==2
    S = tdp.shotOpen(ball,team,pos,valid,c,P,Q);
    laneOk = S.open;
    aim = S.target;
    aimSpeed = P.shotSpeed;
else
    rcv = m.bfPassTo;
    po = tdp.passOpen(ball,team,pos,valid,c,rcv,P,Q);
    laneOk = po.open && ~po.pressured && po.dist>=P.bf2PassMin;
    aim = pos(rcv,:);
    aimSpeed = po.speed;
end
end

function yes = intercepted(pos,valid,team,receiverDist,ball,P)
% A visible opponent at the ball and nearer to it than the receiver.
yes = false;
opponentFirst = 1+4*(2-team);
for i = opponentFirst:opponentFirst+3
    if valid(i)
        d = hypot(pos(i,1)-ball(1),pos(i,2)-ball(2));
        if d<=P.kickRange && d<receiverDist
            yes = true;
        end
    end
end
end

function yes = finishedKick(request,fb,t)
% The previous tick's kick request has a terminal result (Done, Failed, Rejected).
yes = kickRequest(request) && fb.requestId==request.requestId && ...
    (fb.status==TdpStatus.Done || fb.status==TdpStatus.Failed || fb.status==TdpStatus.Rejected) && ...
    fb.stamp>=request.stamp && fb.stamp<=t;
end

function yes = kickRequest(request)
yes = (request.kind==TdpAction.Kick || request.kind==TdpAction.Tap) && request.requestId~=0;
end

function yes = validRobot(ob,id,t,P)
p = ob.robots(id);
age = t-ob.robotStamp(id);
yes = ob.robotValid(id) && isfinite(age) && age>=0 && age<=P.maxObservationAge && ...
    isfinite(p.x) && isfinite(p.y) && isfinite(p.th);
end
