function [ cost ] = actworkcost( funparams, params )
    % Unpack the vector
    [stanceT, ~, xtoe, ~, x, xdot, y, ydot, ra, radot, ~, torque] = ...
        unpack(funparams, params);
    phaseN = size(params.phases, 1);
    
    r = sqrt((x - xtoe).^2 + y.^2);
    rdot = ((x-xtoe).*(xdot)+y.*ydot)./(r);
    fs = params.spring * (ra - r) + params.damp * (radot - rdot);
    
    startRemInd = 1 : params.gridn : params.gridn * phaseN;
    endRemInd = params.gridn : params.gridn : params.gridn * phaseN;
    
    [fsA, fsB] = deal(fs);
    fsA(startRemInd) = [];
    fsB(endRemInd) = [];
    fsCombined = (fsA + fsB) .* 0.5;
    
    [radotA, radotB] = deal(radot);
    radotA(startRemInd) = [];
    radotB(endRemInd) = [];
    radotCombined = (radotA + radotB) .* 0.5;
    
    epsilon = 0.001;
    workRa = (sqrt((fsCombined.*radotCombined).^2 + epsilon^2) - epsilon);
    workRa = workRa .* kron(stanceT./params.gridn, ones(params.gridn - 1, 1));
    
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
end