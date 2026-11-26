function m = bf2TeamMemory()
%BF2TEAMMEMORY Team memory of tdp.v1TeamMemory plus the B-F2 attack-action state.
% Only the V1 match model (tdpV1Match) uses this memory. The recovery models keep
% tdp.v1TeamMemory, so adding B-F2 state does not change their bus. All fields
% are scalars so the memory fits a Simulink bus.
%
% Commitment (B-F2-05): the action type and the receiver ID, never coordinates.
%   bfCommit    0 none, 2 shoot, 3 pass (same codes as bfAction)
%   bfPasser    robot ID that made the commitment (the chaser at that time)
%   bfPassTo    receiver ID of a committed pass
%   bfStart     tSim when the commitment was made
%   bfBad       consecutive ticks the committed lane was closed / receiver pressured
%   bfKickId    request ID of the commitment's kick once the skill entered Kick
% Receive lock (B-F2-03, B-F2-06):
%   bfLock      0 none, 1 receiving (kick confirmed, ball in flight)
%   bfReceiver  receiver ID while bfLock==1
%   bfLockStart tSim of the kick confirmation
%   bfHold      receiver ID that keeps the chaser role after a successful receive
% Per-tick log, rewritten every tick (see scripts/bf2TraceColumns.m):
%   bfAction    0 none, 1 dribble, 2 shoot, 3 pass, 4 receive
%   bfFresh     a new, uncommitted decision was made this tick
%   bfShot      shotOpen.open at the fresh decision
%   bfPass      ID of the nearest feasible receiver at the fresh decision, 0 none
%   bfBoth      shot and pass were both feasible at the fresh decision
%   bfRelease   release reason of this tick, 0 none (codes: see tdp.bf2Plan)
m = tdp.v1TeamMemory();
m.bfCommit = 0;
m.bfPasser = 0;
m.bfPassTo = 0;
m.bfStart = 0;
m.bfBad = 0;
m.bfKickId = uint32(0);
m.bfLock = 0;
m.bfReceiver = 0;
m.bfLockStart = 0;
m.bfHold = 0;
m.bfAction = 0;
m.bfFresh = false;
m.bfShot = false;
m.bfPass = 0;
m.bfBoth = false;
m.bfRelease = 0;
end
