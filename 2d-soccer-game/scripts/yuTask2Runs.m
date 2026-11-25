function dirs = yuTask2Runs(seeds, Tmatch)
%YUTASK2RUNS (B-Y3/B-Y4) Validation matches for Yu's geometry checks.
%   dirs = yuTask2Runs(1:4, 300)
%   V1.2 pairings neutral/neutral, aggressive/defensive and the side swap,
%   one match per seed, saved with tdpRun under build/yu-task2/. These runs
%   only feed the shot-lane replay check and the half-time reconciliation;
%   they are NOT the Test 1 experiment batch (that protocol is Qin's B-Q3).
%   Existing folders are skipped, never overwritten.
if nargin < 1, seeds = 1:4; end
if nargin < 2, Tmatch = 300; end
pairs = {'neutral','neutral'; 'aggressive','defensive'; 'defensive','aggressive'};
dirs = {};
for p = 1:size(pairs, 1)
    for seed = seeds
        out = fullfile('build', 'yu-task2', sprintf('%s-%s-s%d', pairs{p,1}, pairs{p,2}, seed));
        dirs{end+1} = out; %#ok<AGROW>
        if isfolder(out), continue; end
        if ~isfolder(fileparts(out)), mkdir(fileparts(out)); end
        P = tdp.v12TeamConfig(pairs{p,1}, pairs{p,2});
        P.seed = seed;  P.Tmatch = Tmatch;
        tdpRun(P, out);
        fprintf('%s done\n', out);
    end
end
end
