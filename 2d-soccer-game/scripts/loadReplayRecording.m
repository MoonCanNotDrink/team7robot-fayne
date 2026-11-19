function rec=loadReplayRecording(source)
%LOADREPLAYRECORDING Read and validate one saved match without altering it.
% source is a trace.mat path or a build/animation-* folder. The file is only
% opened for reading. rec fields:
%   File, P, Trace (N-by-98), Tactics (N-by-171, or [] when absent/unusable),
%   Times (N-by-1 seconds), Dt, HasTactics, Warnings (cellstr)
% A damaged or unusable file raises TDP:ReplayLoad with the reason.
file=char(source);
if isfolder(file),file=fullfile(file,'trace.mat');end
if ~isfile(file)
    error('TDP:ReplayLoad','Match recording file not found: %s',file);
end
try
    info=whos('-file',file);
catch problem
    error('TDP:ReplayLoad','Cannot read MAT file %s: %s',file,problem.message);
end
names={info.name};
for needed={'P','trace'}
    if ~ismember(needed{1},names)
        error('TDP:ReplayLoad','Recording %s is missing variable %s.',file,needed{1});
    end
end
wanted={'P','trace'};
if ismember('tactics',names),wanted{end+1}='tactics';end
try
    saved=load(file,wanted{:});
catch problem
    error('TDP:ReplayLoad','Cannot load %s: %s',file,problem.message);
end
trace=saved.trace;
if ~(isnumeric(trace) && isreal(trace) && ismatrix(trace) && size(trace,2)==98 && size(trace,1)>=1)
    error('TDP:ReplayLoad','The trace in %s must be a real N-by-98 matrix with N >= 1.',file);
end
if ~all(isfinite(trace),'all')
    error('TDP:ReplayLoad','The trace in %s contains NaN or Inf.',file);
end
c=replayColumns();
times=trace(:,c.time);
if any(diff(times)<=0)
    error('TDP:ReplayLoad','Match times in %s do not strictly increase from frame to frame.',file);
end
if ~(isstruct(saved.P) && isscalar(saved.P) && isfield(saved.P,'dt') && isfield(saved.P,'Tmatch'))
    error('TDP:ReplayLoad','Parameters P in %s are incomplete (dt and Tmatch are required).',file);
end
rec=struct('File',file,'P',saved.P,'Trace',trace,'Tactics',[],'Times',times, ...
    'Dt',saved.P.dt,'HasTactics',false,'Warnings',{{}});
if isfield(saved,'tactics') && ~isempty(saved.tactics)
    k=saved.tactics;
    n=size(trace,1);
    if ~(isnumeric(k) && isreal(k) && isequal(size(k),[n 171]))
        rec.Warnings{end+1}=sprintf('tactics size %s does not match the %d trace rows; tactics information ignored.',mat2str(size(k)),n);
    elseif ~all(isfinite(k),'all')
        rec.Warnings{end+1}='tactics contains NaN or Inf; tactics information ignored.';
    elseif max(abs(k(:,1)-times))>1e-9
        rec.Warnings{end+1}='tactics time column does not match the trace; tactics information ignored.';
    else
        rec.Tactics=k;rec.HasTactics=true;
    end
end
end
