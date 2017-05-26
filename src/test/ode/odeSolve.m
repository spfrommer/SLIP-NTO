function [ solved ] = odeSolve( funParams, ntoParams )
%ODESOLVE Runs an ODE solver to verify the nto output
    
    % Unpack the parameter vector
    [ stanceT, flightT, ~, ~, ~, ~, ~, ~, ...
           ~, ~, raddot, torque] = unpack(funParams, ntoParams);
    
    flightTAug = [flightT, Inf];
    combT = [stanceT'; flightTAug'];
    combT = combT(:);
    combT = [0; combT];
    
    flightT = [0; flightT];
    for i = 1:ntoParams.phaseN - 1
        [t, states] = ode45(@(t, y) odeDynamics(t, y, stanceT, flightT, ...
                                            raddot, torque, ntoParams), ...
                        [fligh], ntoParams.initialState);
                    
    end
    
    
    
end

