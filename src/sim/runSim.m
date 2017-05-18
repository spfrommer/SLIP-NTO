function [ results ] = runSim( slipPatch, stateI, finX )
    disp('-------------NEW SIMULATION-----------------');
    disp('--------OPTIMIZING BACKWARDS STEP-----------');
    
    paramsBack = NTOParams(['sli'; 'stl'; 'str'], slipPatch, stateI, finX);
    [ optimalB, costB, flagB ] = optimize(paramsBack);
    
    disp('--------OPTIMIZING FORWARDS STEP------------');
    
    paramsFor = NTOParams(['sli'; 'str'], slipPatch, stateI, finX);
    [ optimalF, costF, flagF ] = optimize(paramsFor);
    
    results = NTOResults();
    results.paramsBack = paramsBack;
    results.optimalBack = optimalB;
    results.costBack = costB;
    results.flagBack = flagB;
    results.paramsFor = paramsFor;
    results.optimalFor = optimalF;
    results.costFor = costF;
    results.flagFor = flagF;
end

