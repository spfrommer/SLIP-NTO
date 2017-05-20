function [ time ] = timecost( funParams, ntoParams )
    % Unpack the vector
    [stanceT, flightT, ~, ~, ~, ~, ~, ~, ~, ~] = ...
        unpack(funParams, ntoParams);
    
    time = sum(stanceT) + sum(flightT);
end

