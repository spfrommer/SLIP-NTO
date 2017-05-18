function [ cost ] = actsqrcost( funparams, params )
    % Unpack the vector
    [stanceT, ~, ~, ~, ~, ~, ~, ~, ~, ~, raddot, torque] = ...
        unpack(funparams, params);

    dts = kron(stanceT./params.gridn, ones(params.gridn, 1));
    cost = sum((raddot .* dts) .^ 2) + sum((torque .* dts) .^ 2);
end

