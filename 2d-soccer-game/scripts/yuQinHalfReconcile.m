function R = yuQinHalfReconcile(source)
%YUQINHALFRECONCILE (Y3) Compare Yu's and Qin's half-time rules on ONE log.
%   R = yuQinHalfReconcile(folderOrTraceMat)
%   Reads a saved recording (trace, tactics, P), classifies every row with
%   both rules and reports where and why they differ. Read-only: it runs no
%   simulation and changes no file.
%
%   Yu  : ballHalfTrace / tdp.ballHalf. Stoppage when t < pauseUntil of the
%         SAME row; Midline only at x = 0 exactly; Goal / OutOfPlay when the
%         whole ball is over a line.
%   Qin : qinBehaviourMetrics (H1). running from the tactics log (the
%         controller reads the PREVIOUS referee state); restartTransition =
%         running row in which pauseUntil increased (goal/out reset written
%         at the end of that step); outside = running, no reset, ball centre
%         beyond line + ballR; centre = |x| <= 1e-9.
%   Both rules are mapped onto Qin's six labels:
%     left, centre, right, paused, restartTransition, outside
%   Yu has no restartTransition label; those rows are Yu "Stoppage".
%
%   Fields: seconds (2x6, row 1 Yu, row 2 Qin), labels, crossTab (6x6 rows,
%   Yu label x Qin label), mismatchRows, agreeInHalves (true when left /
%   centre / right seconds are identical), attackHalfSeconds (Qin, [blue
%   red]) and the same from Yu.

rec = loadReplayRecording(source);
tr = rec.Trace;  P = rec.P;  n = size(tr, 1);
assert(rec.HasTactics, 'Yu:Reconcile', 'The recording needs its tactics log.');
ta = rec.Tactics;
c = replayColumns();  q = v1TraceColumns();
labels = {'left','centre','right','paused','restartTransition','outside'};

% ---- Qin, row by row (same expressions as qinBehaviourMetrics) ----
t = tr(:, c.time);  tol = max(1e-9, P.dt*1e-6);  midTol = 1e-9;
running = logical(ta(:, q.running));
reset = tr(:, c.pauseUntil) > [0; tr(1:end-1, c.pauseUntil)] + tol;
outside = abs(tr(:, c.ballX)) > P.L/2 + P.ballR + tol | abs(tr(:, c.ballY)) > P.W/2 + P.ballR + tol;
eligible = running & ~reset & ~outside;
qin = zeros(n, 1);
qin(eligible & tr(:, c.ballX) < -midTol) = 1;
qin(eligible & abs(tr(:, c.ballX)) <= midTol) = 2;
qin(eligible & tr(:, c.ballX) > midTol) = 3;
qin(~running) = 4;
qin(running & reset) = 5;
qin(running & ~reset & outside) = 6;
assert(all(qin > 0), 'Yu:Reconcile', 'Qin partition incomplete.');
Mq = qinBehaviourMetrics(tr, ta, P);
assert(max(abs(Mq.halfSeconds - accumarray(qin, P.dt, [6 1])')) < 1e-9, ...
    'Yu:Reconcile', 'Row-level Qin copy disagrees with qinBehaviourMetrics.');

% ---- Yu, row by row ----
Y = ballHalfTrace(tr, P);
C = tdp.halfCodes();
yu = zeros(n, 1);
yu(Y.code == C.BlueHalf) = 1;
yu(Y.code == C.Midline) = 2;
yu(Y.code == C.RedHalf) = 3;
yu(Y.code == C.Stoppage) = 4;
yu(Y.code == C.Goal | Y.code == C.OutOfPlay) = 6;
yu(Y.code == C.Unknown) = 6;
% Yu has no restartTransition: show the reset rows separately so the
% table says exactly where the extra Qin label comes from.
yu(yu == 4 & reset) = 5;

R.labels = labels;
R.seconds = [accumarray(yu, P.dt, [6 1])'; accumarray(qin, P.dt, [6 1])'];
R.crossTab = accumarray([yu qin], 1, [6 6]);
R.mismatchRows = find(yu ~= qin);
R.agreeInHalves = max(abs(R.seconds(1, 1:3) - R.seconds(2, 1:3))) < 1e-9;
R.attackHalfSecondsQin = Mq.attackHalfSeconds;
R.attackHalfSecondsYu = R.seconds(1, [3 1]);
R.excludedSeconds = [sum(R.seconds(1, 4:6)) sum(R.seconds(2, 4:6))];
R.totalSeconds = n*P.dt;
R.file = rec.File;
end
