addpath('analysis');
addpath('cost');
addpath('ntofuns');
addpath('sim');
addpath('test');
addpath('utils');
addpath('visualization');
rng('shuffle');

for i = 1:300
    [slipPatch, stateI, finX] = initialConditions();
    results = runSim(slipPatch, stateI, finX);
    writeSim(results, i);
end