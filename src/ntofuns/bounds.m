function [ lb, ub ] = bounds(params)
    stanceTimeMin      = params.minStanceTime;
    stanceTimeMax      = params.maxStanceTime;
    flightTimeMin      = params.minFlightTime;
    flightTimeMax      = params.maxFlightTime;
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
    minRaddot          = params.minraddot;
    maxRaddot          = params.maxraddot;
    minTorque          = params.mintorque;
    maxTorque          = params.maxtorque;

    minXtoe = [];
    maxXtoe = [];

    for p=1:size(params.phases, 1)
        phase = params.phases(p, :);
        if strcmp(phase, 'sli')
            minXtoe = [minXtoe; ones(params.gridn, 1) * params.slipPatch(1)];
            maxXtoe = [maxXtoe; ones(params.gridn, 1) * params.slipPatch(2)];
        elseif strcmp(phase, 'stl')
            minXtoe = [minXtoe; ones(params.gridn, 1) * -Inf];
            maxXtoe = [maxXtoe; ones(params.gridn, 1) * params.slipPatch(1)];
        elseif strcmp(phase, 'str')
            minXtoe = [minXtoe; ones(params.gridn, 1) * params.slipPatch(2)];
            maxXtoe = [maxXtoe; ones(params.gridn, 1) * Inf];
        end
    end

    stanceTimeVars = ones(size(params.phases, 1), 1);
    flightTimeVars = ones(size(params.phases, 1) - 1, 1);
    gridVars = ones(params.gridn * size(params.phases, 1), 1);

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

