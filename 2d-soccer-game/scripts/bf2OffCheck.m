function result=bf2OffCheck(mode,file,seeds,Tmatch)
%BF2OFFCHECK (B-F2-08) With B-F2 off the match records must equal main's, row for row.
%   bf2OffCheck('record',file,seeds,Tmatch)   run in a checkout of the BASELINE (team
%       main, commit 5eaa413 or later): the V1.2 neutral match (tdp.v12Config) for each
%       seed; the physical trace, the 171-column tactics log and the report are saved.
%   result=bf2OffCheck('compare',file,seeds,Tmatch)   run in the B-F3 checkout. Each seed
%       is run twice, with tdp.v12Config (no B-F2 fields) and with
%       tdp.bf2Config('off','off') (zero priority), and both are compared exactly
%       with the recording. result.passed is true only if every row of every
%       column is identical.
%   Use a separate MATLAB process (and TDP_CACHE_ID) for each checkout: one process
%   cannot hold two copies of the tdpV1Match model.
%   Defaults: seeds 1:10, Tmatch 300 s (B-F2-07). A short check: seeds 1:2, Tmatch 60.
if nargin<3,seeds=1:10;end
if nargin<4,Tmatch=300;end
tdpSetup;
switch mode
    case 'record'
        records=struct('seed',{},'trace',{},'tactics',{},'passed',{});
        for seed=seeds
            P=tdp.v12Config();P.seed=seed;P.Tmatch=Tmatch;
            [trace,tactics,passed]=runOne(P);
            records(end+1)=struct('seed',seed,'trace',trace,'tactics',tactics,'passed',passed); %#ok<AGROW>
            fprintf('recorded seed %d: %d rows, report.passed=%d\n',seed,size(trace,1),passed);
        end
        save(file,'records','Tmatch','-v7.3');
        result=records;
    case 'compare'
        data=load(file);
        assert(data.Tmatch==Tmatch,'The recording used Tmatch=%g.',data.Tmatch);
        result=struct('passed',true,'runs',struct('seed',{},'config',{},'identical',{},'firstRow',{}));
        for k=1:numel(data.records)
            rec=data.records(k);
            if ~any(seeds==rec.seed),continue;end
            configs={'v12Config',tdp.v12Config();'bf2Config(off,off)',tdp.bf2Config('off','off')};
            for c=1:2
                P=configs{c,2};P.seed=rec.seed;P.Tmatch=Tmatch;
                [trace,tactics,~]=runOne(P);
                same=isequaln(trace,rec.trace) && isequaln(tactics,rec.tactics);
                first=0;
                if ~same
                    first=min([firstDifference(trace,rec.trace) firstDifference(tactics,rec.tactics)]);
                end
                result.runs(end+1)=struct('seed',rec.seed,'config',configs{c,1},'identical',same,'firstRow',first);
                result.passed=result.passed && same;
                fprintf('seed %d %-20s identical=%d\n',rec.seed,configs{c,1},same);
            end
        end
    otherwise
        error('TDP:BF2OffCheck','Mode must be record or compare.');
end
end

function [trace,tactics,passed]=runOne(P)
[input,~]=matchSimulationInput(P);
output=sim(input);
trace=recordRows(output.records);tactics=recordRows(output.tacticsRecords);
[report,~]=v1MatchMetrics(trace,tactics,P);
passed=report.passed;
end

function row=firstDifference(a,b)
if ~isequal(size(a),size(b)),row=1;return;end
bad=any(a~=b & ~(isnan(a) & isnan(b)),2);
row=find(bad,1);
if isempty(row),row=Inf;end
end
