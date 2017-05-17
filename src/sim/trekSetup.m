function [ slipPatch, state, finX ] = trekSetup(  )
    slipPatch = [0, 0];
    finX = 2;
    
    sp = SimParams();
    x = 0;
    xdot = 0.5;
    xtoe = 0;
    xtoedot = 0;
    
    y = sp.maxlen;
    ydot = 0;
    
    % Actuated length at max length, and initial actuated length derivative
    % is zero
    ra = sp.maxlen;
    radot = 0;
    
    state = [xtoe; xtoedot; x; xdot; y; ydot; ra; radot];
end

