function [ xtoedotland, xland, xdotland, yland, ydotland ] = ...
        ballisticDynamics( toState, flightTime, landPhaseStr, sp )

    % Expand the important information from the takeoff state
    stateCell = num2cell(toState');
    [~, ~, x, xdot, y, ydot, ~, ~] = deal(stateCell{:});

    % If landing on slippery surface, initial toe velocity should be equal
    % to hip velocity. Otherwise, the toe should stick to the ground with
    % zero velocity
    if strcmp(landPhaseStr, 'sli')
        xtoedotland = xdot;
    else
        xtoedotland = 0;
    end
    xland = x + xdot * flightTime;
    yland = y + ydot * flightTime - 0.5 * sp.gravity * flightTime^2;
    xdotland = xdot;
    ydotland = ydot - sp.gravity * flightTime;
end

