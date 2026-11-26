function m = bf2Reset(m)
%BF2RESET Clear the per-tick B-F2 log fields before a new decision.
m.bfAction = 0;
m.bfFresh = false;
m.bfShot = false;
m.bfPass = 0;
m.bfBoth = false;
m.bfRelease = 0;
end
