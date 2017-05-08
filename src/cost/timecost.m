function [ time ] = timecost( funparams, sp )
    % Unpack the vector
    [stanceT, flightT, ~, ~, ~, ~, ~, ~, ~, ~] = ...
        unpack(funparams, sp);
    
    time = sum(stanceT) + sum(flightT);
end

