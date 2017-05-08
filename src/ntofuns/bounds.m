function [ lb, ub ] = bounds(sp)
    stanceTimeMin      = sp.minStanceTime;
    stanceTimeMax      = sp.maxStanceTime;
    flightTimeMin      = sp.minFlightTime;
    flightTimeMax      = sp.maxFlightTime;
    minXtoedot         = -Inf;
    maxXtoedot         = Inf;
    minX               = -Inf;
    maxX               = Inf;
    minXdot            = -Inf;
    maxXdot            = Inf;
    minY               = 0;
    maxY               = Inf;
    minYdot            = -Inf;
    maxYdot            = Inf;
    minRa              = -Inf;
    maxRa              = Inf;
    minRadot           = -Inf;
    maxRadot           = Inf;
    minRaddot          = sp.minraddot;
    maxRaddot          = sp.maxraddot;
    minTorque          = sp.mintorque;
    maxTorque          = sp.maxtorque;

    minXtoe = [];
    maxXtoe = [];

    for p=1:size(sp.phases, 1)
        phase = sp.phases(p, :);
        if strcmp(phase, 'sli')
            minXtoe = [minXtoe; ones(sp.gridn, 1) * sp.slipPatch(1)];
            maxXtoe = [maxXtoe; ones(sp.gridn, 1) * sp.slipPatch(2)];
        elseif strcmp(phase, 'stl')
            minXtoe = [minXtoe; ones(sp.gridn, 1) * -Inf];
            maxXtoe = [maxXtoe; ones(sp.gridn, 1) * sp.slipPatch(1)];
        elseif strcmp(phase, 'str')
            minXtoe = [minXtoe; ones(sp.gridn, 1) * sp.slipPatch(2)];
            maxXtoe = [maxXtoe; ones(sp.gridn, 1) * Inf];
        end
    end

    stanceTimeVars = ones(size(sp.phases, 1), 1);
    flightTimeVars = ones(size(sp.phases, 1) - 1, 1);
    gridVars = ones(sp.gridn * size(sp.phases, 1), 1);

    lb = [stanceTimeVars * stanceTimeMin; flightTimeVars * flightTimeMin;
          minXtoe;                  gridVars * minXtoedot;
          gridVars * minX;          gridVars * minXdot;
          gridVars * minY;          gridVars * minYdot;
          gridVars * minRa;         gridVars * minRadot;
          gridVars * minRaddot;     gridVars * minTorque];

    ub = [stanceTimeVars * stanceTimeMax; flightTimeVars * flightTimeMax;
          maxXtoe;                  gridVars * maxXtoedot;
          gridVars * maxX;          gridVars * maxXdot;
          gridVars * maxY;          gridVars * maxYdot;
          gridVars * maxRa;         gridVars * maxRadot;
          gridVars * maxRaddot;     gridVars * maxTorque];
end

