function [ type, gridPoint ] = getEqConstraint( index, ntoParams )
%GETEQCONSTRAINT Summary of this function goes here
%   Detailed explanation goes here
    type = 'none';
    gridPoint = NaN;
    
    phaseConstN = 8*(ntoParams.gridn-1)*size(ntoParams.phases, 1);
    transConstN = 8*(size(ntoParams.phases, 1)-1);

    if index <= phaseConstN
        gridPoint = ceil(index / 8) + 0.5; % 0.5 indicates the constraint is actually between two gridpoints
        switch mod(index, 8)
            case 0 % take into account wrap around
            type = 'Phase: radot';
            case 1
            type = 'Phase: xtoe';
            case 2
            type = 'Phase: xtoedot';
            case 3
            type = 'Phase: x';
            case 4
            type = 'Phase: xdot';
            case 5
            type = 'Phase: y';
            case 6
            type = 'Phase: ydot';
            case 7
            type = 'Phase: ra';
            otherwise
            type = 'Phase: unknown index';
        end
    elseif index <= phaseConstN + transConstN
        index = index - 8*(ntoParams.gridn-1)*size(ntoParams.phases, 1);
        % No gridpoint applicable
        switch mod(index, 8)
            case 0
            type = 'Trans: GRF to zero';
            case 1
            type = 'Trans: xtoedot';
            case 2
            type = 'Trans: x';
            case 3
            type = 'Trans: xdot';
            case 4
            type = 'Trans: y';
            case 5
            type = 'Trans: ydot';
            case 6
            type = 'Trans: ra to r';
            case 7
            type = 'Trans: radot';
            otherwise
            type = 'Trans: unknown index';
        end
    else
        type = 'Miscellaneous constraint';
    end
end

