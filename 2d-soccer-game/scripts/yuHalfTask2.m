function [T, totals] = yuHalfTask2(sources)
%YUHALFTASK2 (B-Y4) Half-time reconciliation, Yu vs Qin, over many saved matches.
%   [T, totals] = yuHalfTask2(sources)     sources: cell array of run folders
%   One row per match: seconds per label for both rules (left, centre, right,
%   paused, restartTransition, outside), whether left/centre/right agree
%   exactly, the attacking-half seconds of blue and red (blue attacks +x, so
%   the RIGHT half x > 0 is blue's attacking half and red's defensive half),
%   the excluded seconds and the switch-band diagnostic (|x| <= 0.15 m).
%   Read-only; uses yuQinHalfReconcile per match.
labels = {'left','centre','right','paused','restartTransition','outside'};
n = numel(sources);
match = strings(n,1);  agree = false(n,1);
yu = zeros(n,6);  qin = zeros(n,6);  att = zeros(n,2);  band = zeros(n,1);  total = zeros(n,1);
for k = 1:n
    R = yuQinHalfReconcile(sources{k});
    [~, match(k)] = fileparts(sources{k});
    yu(k,:) = R.seconds(1,:);  qin(k,:) = R.seconds(2,:);
    agree(k) = R.agreeInHalves;
    att(k,:) = R.attackHalfSecondsQin;
    rec = loadReplayRecording(sources{k});
    Y = ballHalfTrace(rec.Trace, rec.P);
    band(k) = Y.switchBandSeconds;
    total(k) = R.totalSeconds;
end
T = table(match, total, agree, att(:,1), att(:,2), band, ...
    'VariableNames', {'match','recordedSeconds','halvesAgree','blueAttackHalfSeconds', ...
    'redAttackHalfSeconds','switchBandSeconds'});
for j = 1:6
    T.(['yu_' labels{j}]) = yu(:,j);
    T.(['qin_' labels{j}]) = qin(:,j);
end
totals = struct('matches', n, 'allHalvesAgree', all(agree), ...
    'yu', array2table(sum(yu,1), 'VariableNames', labels), ...
    'qin', array2table(sum(qin,1), 'VariableNames', labels), ...
    'recordedSeconds', sum(total), 'switchBandSeconds', sum(band), ...
    'blueAttackShare', sum(att(:,1))/sum(qin(:,1:3), 'all'), ...
    'redAttackShare', sum(att(:,2))/sum(qin(:,1:3), 'all'));
end
