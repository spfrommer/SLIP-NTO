function [ cost ] = actworkcost( funParams, ntoParams )
    % Unpack the vector
    [stanceT, ~, xtoe, ~, x, xdot, y, ydot, ra, radot, ~, torque] = ...
        unpack(funParams, ntoParams);
    phaseN = size(ntoParams.phases, 1);
    
    r = sqrt((x - xtoe).^2 + y.^2);
    rdot = ((x-xtoe).*(xdot)+y.*ydot)./(r);
    fs = ntoParams.spring * (ra - r) + ntoParams.damp * (radot - rdot);
    
    startRemInd = 1 : ntoParams.gridn : ntoParams.gridn * phaseN;
    endRemInd = ntoParams.gridn : ntoParams.gridn : ntoParams.gridn * phaseN;
    
    [fsA, fsB] = deal(fs);
    fsA(startRemInd) = [];
    fsB(endRemInd) = [];
    fsCombined = (fsA + fsB) .* 0.5;
    
    [radotA, radotB] = deal(radot);
    radotA(startRemInd) = [];
    radotB(endRemInd) = [];
    radotCombined = (radotA + radotB) .* 0.5;
    
    epsilon = ntoParams.sqrtSmooth;
    workRa = (sqrt((fsCombined.*radotCombined).^2 + epsilon^2) - epsilon);
    workRa = workRa .* kron(stanceT./(ntoParams.gridn-1), ones(ntoParams.gridn - 1, 1));
    
    angles = atan2(y, x - xtoe);
    angleShift = [angles(1); angles(1 : end-1)];
    angleDeltas = angles - angleShift;
    angleDeltas(startRemInd) = [];
    [torqueA,torqueB] = deal(torque);
    torqueA(startRemInd) = [];
    torqueB(endRemInd) = [];
    torqueCombined = (torqueA + torqueB) .* 0.5;
    
    workAng = sqrt((torqueCombined.*angleDeltas).^2 + epsilon^2) - epsilon;
    cost = sum(workRa) + sum(workAng);
    
    % Add small regularization term to force unique solution
    %cost = cost + 1e-3 * actsqrcost(funParams, ntoParams);
    cost = cost + 1e-2 * actsqrcost(funParams, ntoParams);
end