function [] = writeSim( results, simNum )
    fid = fopen(Resource.instance().getDataPath(num2str(simNum)), 'w');
    fprintf(fid, '%.12f,', results.spFor.slipPatch);
    fprintf(fid, '\n');
    fprintf(fid, '%.12f,', results.spFor.initialState);
    fprintf(fid, '\n');
    fprintf(fid, '%.12f\n', results.spFor.finalProfileX);
    
    fprintf(fid, '%.12f,', results.optimalBack);
    fprintf(fid, '\n');
    fprintf(fid, '%.12f\n', results.costBack);
    fprintf(fid, '%d\n', results.flagBack);
    
    fprintf(fid, '%.12f,', results.optimalFor);
    fprintf(fid, '\n');
    fprintf(fid, '%.12f\n', results.costFor);
    fprintf(fid, '%d\n', results.flagFor);
    fclose(fid);
end

