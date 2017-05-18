function [ results ] = readSim( simNum )
    results = NTOResults();
        
    fid = fopen(Resource.instance().getDataPath(simNum), 'r');
    slipPatch = sscanf(fgetl(fid), '%f,')';
    stateI = sscanf(fgetl(fid), '%f,');
    finX = sscanf(fgetl(fid), '%f');
    
    results.optimalBack = sscanf(fgetl(fid), '%f,');
    results.costBack = sscanf(fgetl(fid), '%f');
    results.flagBack = sscanf(fgetl(fid), '%f');

    results.optimalFor = sscanf(fgetl(fid), '%f,');
    results.costFor = sscanf(fgetl(fid), '%f');
    results.flagFor = sscanf(fgetl(fid), '%f');
    
    results.paramsBack = NTOParams(['sli'; 'stl'; 'str'], slipPatch, stateI, finX);
    results.paramsFor = NTOParams(['sli'; 'str'], slipPatch, stateI, finX);
    fclose(fid);
end