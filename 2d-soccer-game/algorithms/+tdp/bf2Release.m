function m = bf2Release(m,reason)
%BF2RELEASE Drop the commitment, the receive lock and the hold in one go.
% Used when the whole team situation ends: ball lost (4) or game paused (8).
if m.bfCommit~=0 || m.bfLock~=0 || m.bfHold~=0
    m.bfRelease = reason;
end
m.bfCommit = 0;
m.bfPasser = 0;
m.bfPassTo = 0;
m.bfBad = 0;
m.bfKickId = uint32(0);
m.bfLock = 0;
m.bfReceiver = 0;
m.bfHold = 0;
end
