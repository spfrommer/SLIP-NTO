addpath('analysis');
addpath('cost');
addpath('ntofuns');
addpath('sim');
addpath('test');
addpath('utils');
addpath('visualization');
rng('shuffle');

for i = Resource.instance().numDataFiles()+1:300
    [slipPatch, stateI, finX] = slipPatchSetup();
    results = runSim(slipPatch, stateI, finX);
    writeSim(results, i);
end