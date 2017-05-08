function [ slipPatch, state, finX ] = initialConditions(  )
    slipPatch = [0, 0.8 + rand() * 1];
    finX = slipPatch(2);
    
    sp = SimParams();
    % Generate random initial hip x with 0.3 meter margin from patch edges
    x = rand() * (slipPatch(2) - 0.6) + 0.3;
    % Random initial hip x vel must be positive
    xdot = rand() * 0.5;
    % Random toe x within 0.2 forward of hip x
    xtoe = x + rand() * 0.2;
    % Toe x vel equal to hip x vel
    xtoedot = xdot;
    
    % Y calculated assuming rtouchdown = maxlen
    y = sqrt(sp.maxlen ^ 2 - (x - xtoe) ^ 2);
    % Y vel some random downward number
    ydot = -rand() * 0.4;
    
    % Actuated length at max length, and initial actuated length derivative
    % is zero
    ra = sp.maxlen;
    radot = 0;
    
    state = [xtoe; xtoedot; x; xdot; y; ydot; ra; radot];
end

