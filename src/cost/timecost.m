function [ time ] = timecost( funparams, params )
    % Unpack the vector
    [stanceT, flightT, ~, ~, ~, ~, ~, ~, ~, ~] = ...
        unpack(funparams, params);
    
    time = sum(stanceT) + sum(flightT);
end

