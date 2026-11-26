function row=V1Bf2Record(blue,red,tSim)
%V1BF2RECORD B-F2 attack-action log of both teams (columns: bf2TraceColumns).
c=bf2TraceColumns();
row=zeros(1,23);
row(c.time)=tSim;
row(c.action)=[double(blue.bfAction) double(red.bfAction)];
row(c.fresh)=[double(blue.bfFresh) double(red.bfFresh)];
row(c.shot)=[double(blue.bfShot) double(red.bfShot)];
row(c.pass)=[double(blue.bfPass) double(red.bfPass)];
row(c.both)=[double(blue.bfBoth) double(red.bfBoth)];
row(c.commit)=[double(blue.bfCommit) double(red.bfCommit)];
row(c.receiver)=[double(blue.bfPassTo) double(red.bfPassTo)];
row(c.lock)=[double(blue.bfLock) double(red.bfLock)];
row(c.hold)=[double(blue.bfHold) double(red.bfHold)];
row(c.release)=[double(blue.bfRelease) double(red.bfRelease)];
row(c.bad)=[double(blue.bfBad) double(red.bfBad)];
end
