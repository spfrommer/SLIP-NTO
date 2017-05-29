function [ cost ] = coalesceActsqrcost( stanceT, raddot, torque, ntoParams )
    phasen = size(ntoParams.phases, 1);
    
    dts = repmat(stanceT./(ntoParams.gridn-1), 1, ntoParams.gridn-1)';
    dts = dts(:)';
    
    startRemInd = 1 : ntoParams.gridn : ntoParams.gridn * phasen;
    endRemInd = ntoParams.gridn : ntoParams.gridn : ntoParams.gridn * phasen;

    startInds = 1:length(torque);
    startInds(startRemInd) = [];
    endInds = 1:length(torque);
    endInds(endRemInd) = [];

    [torqueA, torqueB] = deal(torque);
    torqueA = torqueA(startInds);
    torqueB = torqueB(endInds);
    torqueCosts = 0.5 .* dts .* (torqueA.^2 + torqueB.^2);
    
    torqueCost = torqueCosts(1);
    for i = 2:length(torqueCosts)
        torqueCost = torqueCost + torqueCosts(i);
    end

    [raddotA, raddotB] = deal(raddot);
    raddotA = raddotA(startInds);
    raddotB = raddotB(endInds);
    raddotCosts = 0.5 .* dts .* (raddotA.^2 + raddotB.^2);

    raddotCost = raddotCosts(1);
    for i = 2:length(raddotCosts)
        raddotCost = raddotCost + raddotCosts(i);
    end

    cost = torqueCost + raddotCost;
end

