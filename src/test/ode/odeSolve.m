function [ times, states, transT ] = odeSolve( funParams, ntoParams )
%ODESOLVE Runs an ODE solver to verify the nto output
    
    % Unpack the parameter vector
    [ stanceT, ~, xtoe, ~, x, ~, y, ydot, ...
           ~, ~, raddot, torque] = unpack(funParams, ntoParams);
    
    % starting state at the beginning of the phase
    stateI = ntoParams.initialState;
    times = [];
    states = [];
    
    cumT = 0;
    transT = [];

    for p = 1:size(ntoParams.phases, 1)
        phaseStr = ntoParams.phases(p, :);
        
        stanceStartTime = cumT;
        stanceEndTime = cumT + stanceT(p);
        transT = [transT; stanceStartTime; stanceEndTime];
        gridTimes = linspace(stanceStartTime, stanceEndTime, ntoParams.gridn);
        raddotPhase = raddot((p-1) * ntoParams.gridn + 1 : p * ntoParams.gridn);
        torquePhase = torque((p-1) * ntoParams.gridn + 1 : p * ntoParams.gridn);
    
        raddotFun = @(xs) interp1(gridTimes, raddotPhase, xs, 'linear');
        torqueFun = @(xs) interp1(gridTimes, torquePhase, xs, 'linear');

        [t, odeStates] = ode45(@(t, state) stanceDynamics(state, raddotFun(t), ...
                                    torqueFun(t), ntoParams, phaseStr), ...
                                    [stanceStartTime, stanceEndTime], stateI);
        times = [times; t];
        states = [states; odeStates];
        
        cumT = stanceEndTime;
        if p < size(ntoParams.phases, 1)
            yland = y(p * ntoParams.gridn + 1);
            ballisticT = max(roots([-0.5 * ntoParams.gravity, ...
                ydot(p * ntoParams.gridn), y(p * ntoParams.gridn) - yland]));
            landTime = cumT + ballisticT;
            
            [ xtoedotland, xland, xdotland, yland, ydotland ] = ...
                ballisticDynamics(states(end, :), ballisticT, ntoParams.phases(p+1, :), ntoParams);
            
            xtoeland = xland + xtoe(p * ntoParams.gridn + 1) - x(p * ntoParams.gridn + 1);
            raland = sqrt(yland^2 + (xtoeland-xland)^2);
            radotland = 0;
            stateland = [xtoeland, xtoedotland, xland, xdotland, ...
                         yland, ydotland, raland, radotland];
            %times = [times; landTime];
            %states = [states; stateland];
            stateI = stateland;
            
            cumT = landTime;
        end
    end
end

