function [ output_args ] = odeDynamics( t, state, stanceT, flightT, ...
                                        raddot, torque, phaseStr, ntoParams )
%ODEDYNAMICS Computes the stance-phase dynamics for the ode
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

