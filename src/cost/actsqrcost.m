function [ cost ] = actsqrcost( funparams, sp )
    % Unpack the vector
    [stanceT, ~, ~, ~, ~, ~, ~, ~, ~, ~, raddot, torque] = ...
        unpack(funparams, sp);

    dts = kron(stanceT./sp.gridn, ones(sp.gridn, 1));
    cost = sum((raddot .* dts) .^ 2) + sum((torque .* dts) .^ 2);
end

