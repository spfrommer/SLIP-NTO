function [ stanceT, flightT, xtoe, xtoedot, x, xdot, y, ydot, ...
           ra, radot, raddot, torque] = unpack( funParams, ntoParams )
    p = size(ntoParams.phases, 1);
    cnt = ntoParams.gridn * p;
    
    stanceT       = funParams(1               : p);
    flightT       = funParams(p + 1           : p * 2 - 1);
    xtoe          = funParams(p * 2           : p * 2 - 1 + cnt);
    xtoedot       = funParams(p * 2 + cnt     : p * 2 - 1 + cnt * 2);
    x             = funParams(p * 2 + cnt * 2 : p * 2 - 1 + cnt * 3);
    xdot          = funParams(p * 2 + cnt * 3 : p * 2 - 1 + cnt * 4);
    y             = funParams(p * 2 + cnt * 4 : p * 2 - 1 + cnt * 5);
    ydot          = funParams(p * 2 + cnt * 5 : p * 2 - 1 + cnt * 6);
    ra            = funParams(p * 2 + cnt * 6 : p * 2 - 1 + cnt * 7);
    radot         = funParams(p * 2 + cnt * 7 : p * 2 - 1 + cnt * 8);
    raddot        = funParams(p * 2 + cnt * 8 : p * 2 - 1 + cnt * 9);
    torque        = funParams(p * 2 + cnt * 9 : p * 2 - 1 + cnt * 10);
end

