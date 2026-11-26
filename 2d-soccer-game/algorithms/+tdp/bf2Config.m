function P = bf2Config(bluePriority,redPriority)
%BF2CONFIG Test 2 configuration: V1.2 neutral baseline plus the B-F2 attack priority.
%   P = tdp.bf2Config('shoot','pass') gives blue (team 1) Shoot-first and red
%   (team 2) Pass-first. Priorities are 'shoot', 'pass' or 'off'; 'off' leaves
%   that team on the unchanged V1.2 attack. Swap the names for the side-swapped
%   match. Everything else is tdp.v12Config (neutral defence, V1.1 fallback), so
%   the attack priority is the only variable (B-F2-07).
%
%   The numbers are the initial engineering values of B-F2 (not tuned):
%     bf2PassMin / bf2PassMax  receiver distance window, m           (B-F2-02)
%     bf2SoftTicks             N ticks of a closed lane before a re-choice (B-F2-05)
%     bf2CommitTimeout         s without entering Kick before a re-choice  (B-F2-05)
%     bf2ReceiveTimeout        s after the confirmed kick to receive      (B-F2-06)
%     bf2PassMaxTurn           rad, largest angle between the directions passer->ball
%                              and ball->receiver for a receiver to be a candidate.
%                              Used only to filter receivers at a fresh decision, so
%                              the passer never has to walk round the ball (changes
%                              B-F2-02 in part). 90 deg is a trial value, not tuned.
P = tdp.v12Config();
P.bf2Priority = [priorityCode(bluePriority) priorityCode(redPriority)];
P.bf2PassMin = 0.5;
P.bf2PassMax = 2.0;
P.bf2SoftTicks = 3;
P.bf2CommitTimeout = 2.0;
P.bf2ReceiveTimeout = 3.0;
P.bf2PassMaxTurn = pi/2;   % 90 deg
end

function code = priorityCode(name)
switch lower(char(name))
    case 'off'
        code = 0;
    case 'shoot'
        code = 1;
    case 'pass'
        code = 2;
    otherwise
        error('TDP:BF2Priority','Unknown attack priority "%s" (use shoot, pass or off).',char(name));
end
end
