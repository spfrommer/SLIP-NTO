function [ statedot, auxvars ] = stanceDynamics( state, raddot, ...
                                                 torque, params, phaseStr )
    stateCell = num2cell(state');
    [xtoe, xtoedot, x, xdot, y, ydot, ra, radot] = deal(stateCell{:});
    
    r = sqrt((x - xtoe)^2 + y^2);
    rdot = ((x-xtoe)*(xdot-xtoedot)+y*ydot)/(r);
    cphi = (x-xtoe) / r;
    sphi = y / r;
    fs = params.spring * (ra - r) + params.damp * (radot - rdot);
    ft = torque / r;
    fg = params.masship * params.gravity;

    xddot = (1/params.masship) * (fs*cphi + ft * sphi);
    yddot = (1/params.masship) * (fs*sphi + ft * (-cphi) - fg);

    grf = fs * sphi - ft * cphi + params.masstoe * params.gravity;

    % Check if the phase should be slipping
    if strcmp(phaseStr, 'sli')
        ff = -params.friction * tanh(xtoedot * params.tanhSmooth) * grf;
        xtoeddot = (1/params.masstoe) * (-fs*cphi - ft*sphi + ff);
    else
        xtoeddot = 0;
    end

    statedot = [xtoedot; xtoeddot; xdot; xddot; ydot; yddot; ...
                radot; raddot];

    auxvars.r = r;
    auxvars.grf = grf;
end

