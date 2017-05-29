function [ cost ] = actsqrcost( funParams, ntoParams )
    % Unpack the vector
    [stanceT, ~, ~, ~, ~, ~, ~, ~, ~, ~, raddot, torque] = ...
        unpack(funParams, ntoParams);
    phasen = size(ntoParams.phases, 1);
    
    dts = kron(stanceT./(ntoParams.gridn-1), ones(ntoParams.gridn-1, 1));
    
    startRemInd = 1 : ntoParams.gridn : ntoParams.gridn * phasen;
    endRemInd = ntoParams.gridn : ntoParams.gridn : ntoParams.gridn * phasen;
    
    [torqueA, torqueB] = deal(torque);
    torqueA(startRemInd) = [];
    torqueB(endRemInd) = [];
    torqueCost = sum(0.5 .* dts .* (torqueA.^2 + torqueB.^2));

    [raddotA, raddotB] = deal(raddot);
    raddotA(startRemInd) = [];
    raddotB(endRemInd) = [];
    raddotCost = sum(0.5 .* dts .* (raddotA.^2 + raddotB.^2));

    cost = torqueCost + raddotCost;
end

