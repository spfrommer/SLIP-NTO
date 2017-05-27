function [ solved ] = odeSolve( funParams, ntoParams )
%ODESOLVE Runs an ODE solver to verify the nto output
    
    % Unpack the parameter vector
    [ stanceT, flightT, ~, ~, ~, ~, ~, ~, ...
           ~, ~, raddot, torque] = unpack(funParams, ntoParams);
    
    flightTAug = [0; flightT];
    cumT = [flightTAug'; stanceT'];
    cumT = cumsum(cumT(:));
    
    % starting state at the beginning of the phase
    stateI = ntoParams.initialState;
    times = [];
    states = [];

    for p = 1:size(ntoParams.phases, 1)
        phaseStr = ntoParams.phases(p, :);
        
        stanceStartTime = cumT((p-1) * 2 + 1);
        stanceEndTime = stanceStartTime + stanceT(p);
        times = linspace(stanceStartTime, stanceEndTime, ntoParams.gridn);
        raddotPhase = raddot((p-1) * ntoParams.gridn + 1 : p * ntoParams.gridn);
        torquePhase = torque((p-1) * ntoParams.gridn + 1 : p * ntoParams.gridn);
    
        raddotFun = @(x) interp1(times, raddotPhase, x);
        torqueFun = @(x) interp1(times, torquePhase, x);

        %[t, states] = ode45(@(t, y) odeDynamics(t, y, stanceT, flightT, ...
        %                                    raddot, torque, ntoParams), ...
        %                [fligh], ntoParams.initialState);
        [t, states] = ode45(@(t, y) stanceDynamics(y, raddotFun(t), ...
                                    torqueFun(t), ntoParams, phaseStr), ...
                            [stanceStartTime, stanceEndTime], stateI);
        times = [times, t];
        
    end
    
    
    
end

