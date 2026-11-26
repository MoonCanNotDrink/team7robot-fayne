function c=bf2TraceColumns()
%BF2TRACECOLUMNS Column numbers of the bf2Records signal (B-F3 action log).
% One row per tick, 23 columns; two-element ranges are [blue red].
%   action   0 none, 1 dribble, 2 shoot, 3 pass, 4 receive
%   fresh    a new uncommitted decision was made this tick
%   shot     shotOpen.open at the fresh decision
%   pass     ID of the nearest feasible receiver at the fresh decision (0 none)
%   both     shot and pass both feasible at the fresh decision
%   commit   committed action (0 none, 2 shoot, 3 pass), receiver = commit receiver ID
%   lock     1 while the pass is in flight to the receiver
%   hold     receiver ID that keeps the chaser role after a successful receive
%   release  release reason of this tick (codes: help tdp.bf2Plan)
%   bad      consecutive ticks the committed lane was closed
c=struct('time',1,'action',2:3,'fresh',4:5,'shot',6:7,'pass',8:9,'both',10:11, ...
    'commit',12:13,'receiver',14:15,'lock',16:17,'hold',18:19,'release',20:21,'bad',22:23);
end
