function [ stanceT, flightT, xtoe, xtoedot, x, xdot, y, ydot, ...
           ra, radot, raddot, torque] = unpack( funparams, sp )
    p = size(sp.phases, 1);
    cnt = sp.gridn * p;
    
    stanceT       = funparams(1               : p);
    flightT       = funparams(p + 1           : p * 2 - 1);
    xtoe          = funparams(p * 2           : p * 2 - 1 + cnt);
    xtoedot       = funparams(p * 2 + cnt     : p * 2 - 1 + cnt * 2);
    x             = funparams(p * 2 + cnt * 2 : p * 2 - 1 + cnt * 3);
    xdot          = funparams(p * 2 + cnt * 3 : p * 2 - 1 + cnt * 4);
    y             = funparams(p * 2 + cnt * 4 : p * 2 - 1 + cnt * 5);
    ydot          = funparams(p * 2 + cnt * 5 : p * 2 - 1 + cnt * 6);
    ra            = funparams(p * 2 + cnt * 6 : p * 2 - 1 + cnt * 7);
    radot         = funparams(p * 2 + cnt * 7 : p * 2 - 1 + cnt * 8);
    raddot        = funparams(p * 2 + cnt * 8 : p * 2 - 1 + cnt * 9);
    torque        = funparams(p * 2 + cnt * 9 : p * 2 - 1 + cnt * 10);
end

