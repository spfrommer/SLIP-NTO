function [ output_args ] = odeDynamics( t, state, stanceT, flightT, ...
                                        raddot, torque, ntoParams )
%ODEDYNAMICS Summary of this function goes here
%   Detailed explanation goes here
    flightTAug = [flightT, Inf];
    combT = [stanceT'; flightTAug'];
    combT = combT(:);
    combT = [0; combT];
    if t == 0
        dts = linspace(combT(1), combT(2), ntoParams.gridn);
    else
        notComplete = find(combT>t);
        phase = notComplete(1);
        dts = linspace(combT(phase-1), combT(phase), ntoParams.gridn);
    end
    
    
end

