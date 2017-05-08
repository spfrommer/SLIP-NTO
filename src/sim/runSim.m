function [ results ] = runSim( slipPatch, stateI, finX )
    disp('-------------NEW SIMULATION-----------------');
    disp('--------OPTIMIZING BACKWARDS STEP-----------');
    
    spBack = SimParams(['sli'; 'stl'; 'str'], slipPatch, stateI, finX);
    [ optimalB, costB, flagB ] = optimize(spBack);
    
    disp('--------OPTIMIZING FORWARDS STEP------------');
    
    spFor = SimParams(['sli'; 'str'], slipPatch, stateI, finX);
    [ optimalF, costF, flagF ] = optimize(spFor);
    
    results = SimResults();
    results.spBack = spBack;
    results.optimalBack = optimalB;
    results.costBack = costB;
    results.flagBack = flagB;
    results.spFor = spFor;
    results.optimalFor = optimalF;
    results.costFor = costF;
    results.flagFor = flagF;
end

