function [ c, ceq ] = constraints( funparams, params )
    % Phase inequality constraints
    phaseIC = sym('pic', [1, 4*params.gridn*size(params.phases, 1)])';
     
    % Phase equality constraints
    phaseEC = sym('pec', [1, 8*(params.gridn-1)*size(params.phases, 1)])';
    % Phase transition equality constraints
    transEC = sym('tec', [1, 8*(size(params.phases, 1)-1)])';
    
    % Unpack the parameter vector
    [ stanceT, flightT, xtoe, xtoedot, x, xdot, y, ydot, ...
           ra, radot, raddot, torque] = unpack(funparams, params);
    
    % Iterate over all the phases
    for p = 1 : size(params.phases, 1)
        phaseStr = params.phases(p, :);
        % The index of the first dynamics variable for the current phase
        ps = (p - 1) * params.gridn + 1;
        
        % Calculate the timestep for that specific phase
        dt = stanceT(p) / params.gridn;
        
        % Take off state at the end of the last phase
        if p > 1
            toState = stateN;
            toAuxvars = auxvarsN;
        end
        
        stateN = [xtoe(ps); xtoedot(ps); x(ps);  xdot(ps);   ...
                  y(ps);    ydot(ps);    ra(ps); radot(ps)];
        
        % Link ballistic trajectory from end of last phase to this phase
        if p > 1
            rland = sqrt((x(ps) - xtoe(ps))^2 + y(ps)^2);
            
            [xtoedotland, xland, xdotland, yland, ydotland] = ...
                ballisticDynamics(toState, flightT(p-1), phaseStr, params);
            % Grf must equal zero at takeoff
            transEC((p-2)*8+1 : (p-1)*8) = ...
                    [xtoedotland-xtoedot(ps); xland-x(ps); ...
                    xdotland-xdot(ps); yland-y(ps); ydotland-ydot(ps); ...
                    ra(ps) - rland; radot(ps); toAuxvars.grf];
        end
            
        % Offset in the equality parameter vector due to phase
        pecOffset = 8 * (params.gridn - 1) * (p - 1);
        % Offset in the inequality parameter vector due to phase
        picOffset = 4 * (params.gridn) * (p - 1);
        
        [statedotN, auxvarsN] = stanceDynamics(stateN, raddot(ps), ...
                                          torque(ps), params, phaseStr);
        for i = 1 : params.gridn - 1
            % The state at the beginning of the time interval
            stateI = stateN;
            % What the state should be at the end of the time interval
            stateN = [xtoe(ps+i); xtoedot(ps+i); x(ps+i);  xdot(ps+i); ...
                      y(ps+i);    ydot(ps+i);    ra(ps+i); radot(ps+i)];
            % The state derivative at the beginning of the time interval
            statedotI = statedotN;
            % Some calculated variables at the beginning of the interval
            auxvarsI = auxvarsN;
            % The state derivative at the end of the time interval
            [statedotN, auxvarsN] = stanceDynamics(stateN, ...
                                raddot(ps+i), torque(ps+i), params, phaseStr);

            % The end position of the time interval calculated using quadrature
            endState = stateI + dt * (statedotI + statedotN) / 2;
            
            % Constrain the end state of the current time interval to be
            % equal to the starting state of the next time interval
            phaseEC(pecOffset+(i-1)*8+1:pecOffset+i*8) = stateN - endState;
            % Constrain the length of the leg, grf, and body y pos
            phaseIC(picOffset+(i-1)*4+1 : picOffset+i*4) = ...
                    [auxvarsI.r - params.maxlen; params.minlen - auxvarsI.r; ...
                     -auxvarsI.grf; auxvarsI.grf - params.maxgrf];
        end
        
        if p == size(params.phases, 1)
            % Constrain the length of the leg at the end position
            % Since it's the end of the last phase, add grf constraint
            phaseIC(picOffset+(params.gridn-1)*4+1:picOffset+params.gridn*4) = ...
                [auxvarsN.r - params.maxlen; params.minlen - auxvarsN.r; ...
                -auxvarsN.grf; auxvarsN.grf - params.maxgrf];
        else 
            % Constrain the length of the leg at the end position
            % No ground reaction force constraint (this will be handled in
            % transition equality constraints)
            phaseIC(picOffset+(params.gridn-1)*4+1:picOffset+params.gridn*4) = ...
                [auxvarsN.r - params.maxlen; params.minlen - auxvarsN.r; -1; -1];
        end
    end
    
    c = phaseIC;
    ceq = [phaseEC; transEC];
    
    initialState = [xtoe(1); xtoedot(1); x(1);  xdot(1);   ...
                  y(1);    ydot(1);    ra(1); radot(1)];
              
    % Add first phase start constraints
    ceq = [ceq; initialState - params.initialState];
    % Add last phase end constraints
    ceq = [ceq; x(end) - params.finalProfileX; xtoe(end) - params.finalProfileX];
end