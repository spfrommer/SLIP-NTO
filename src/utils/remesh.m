function [ mesh ] = remesh( newGridn, funParams, ntoParams )
%REMESH reconstructs the mesh by changing the number of gridpoints per
%phase
    [ stanceT, flightT, xtoe, xtoedot, x, xdot, y, ydot, ...
           ra, radot, raddot, torque] = unpack( funParams, ntoParams );
    oldGridn = ntoParams.gridn;
       
    phaseN = size(ntoParams.phases, 1);
    oldMeshTimes = zeros(phaseN, oldGridn);
    newMeshTimes = zeros(phaseN, newGridn);

    for i = 1:phaseN
        oldMeshTimes(i, :) = linspace(0, stanceT(i), oldGridn);
        newMeshTimes(i, :) = linspace(0, stanceT(i), newGridn);
    end

    function iv = interpVar(var)
        phaseVars = reshape(var, [oldGridn, phaseN])';
        iv = zeros(phaseN, newGridn);
        for j = 1:phaseN
            iv(j, :) = interp1(oldMeshTimes(j, :), phaseVars(j, :), ...
                               newMeshTimes(j, :), 'linear');
        end
        iv = reshape(iv', [phaseN * newGridn, 1]);
    end
    
    mesh = [stanceT; flightT; interpVar(xtoe); interpVar(xtoedot); ...
                              interpVar(x); interpVar(xdot); ...
                              interpVar(y); interpVar(ydot); ...                              
                              interpVar(ra); interpVar(radot); ...                              
                              interpVar(raddot); interpVar(torque)];
end

