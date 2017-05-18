function [ optimal, cost, flag ] = optimize( params )
    disp('Slip patch: ');
    disp(params.slipPatch);
    disp('Initial state: ');
    disp(params.initialState);
    disp('Final x: ');
    disp(params.finalProfileX);

    % Options for optimizing the constant cost function
    cOptions = optimoptions(@fmincon, 'TolFun', 0.00000001, ...
                       'MaxIterations', 100000, ...
                       'MaxFunEvals', 50000, ...
                       'Display', 'iter', 'Algorithm', 'sqp', ...
                       'StepTolerance', 1e-13, ...
                       'SpecifyConstraintGradient', true, ...
                       'SpecifyObjectiveGradient', true, ...
                       'ConstraintTolerance', 1e-6, ...
                       'CheckGradients', false, ...
                       'HonorBounds', true, ...
                       'FiniteDifferenceType', 'forward');
    % Options for optimizing the actual cost function
    aOptions = optimoptions(@fmincon, 'TolFun', 0.00000001, ...
                       'MaxIterations', 1000, ...
                       'MaxFunEvals', 1000000, ...
                       'Display', 'iter', 'Algorithm', 'sqp', ...
                       'StepTolerance', 1e-13, ...
                       'SpecifyConstraintGradient', true, ...
                       'SpecifyObjectiveGradient', true, ...
                       'ConstraintTolerance', 1e-6, ...
                       'CheckGradients', false, ...
                       'HonorBounds', true, ...
                       'FiniteDifferenceType', 'forward');
    % No linear inequality or equality constraints
    A = [];
    B = [];
    Aeq = [];
    Beq = [];
    % Set up the bounds
    [lb, ub] = bounds(params);

    numVars = size(params.phases, 1) * 2 - 1 + params.gridn * size(params.phases, 1) * 10;
    funparams = conj(sym('x', [1 numVars], 'real')');
    
    fprintf('Generating constraints...\n');
    [c, ceq] = constraints(funparams, params);
    cjac = jacobian(c, funparams).';
    ceqjac = jacobian(ceq, funparams).';
    fprintf('Generating constraints function...\n');
    constraintsFun = matlabFunction(c, ceq, cjac, ceqjac, 'Vars', {funparams});
    
    fprintf('Generating costs...\n');
    ccost = constcost(funparams, params);
    ccostjac = jacobian(ccost, funparams).';
    ccostFun = matlabFunction(ccost, ccostjac, 'Vars', {funparams});

    acost = actsqrcost(funparams, params);
    acostjac = jacobian(acost, funparams).';
    acostFun = matlabFunction(acost, acostjac, 'Vars', {funparams});
    
    awcost = actworkcost(funparams, params);
    awcostjac = jacobian(awcost, funparams).';
    awcostFun = matlabFunction(awcost, awcostjac, 'Vars', {funparams});
    
    flag = -1;
    tryCount = 0;
    while flag < 0 && tryCount < 3
        fprintf('Generating initial possible trajectory (%d)...\n', tryCount+1);
        % Generate initial guess
        x0 = MinMaxCheck(lb, ub, ones(numVars, 1) * rand() * 2);
        [ci, ceqi, cjaci, ceqjaci] = constraintsFun(x0);
        while any(imag(ci))  || any(imag(ceqi))  || any(any(imag(cjaci)))  || any(any(imag(ceqjaci)))  || ...
              any(isnan(ci)) || any(isnan(ceqi)) || any(any(isnan(cjaci))) || any(any(isnan(ceqjaci))) || ...
              any(isinf(ci)) || any(isinf(ceqi)) || any(any(isinf(cjaci))) || any(any(isinf(ceqjaci)))
            fprintf('Regenerating initial guess...\n');
            x0 = MinMaxCheck(lb, ub, ones(numVars, 1) * rand());
            [ci, ceqi, cjaci, ceqjaci] = constraintsFun(x0);
        end
    
        fprintf('########## Finding feasible trajectory ##########\n');
        % Find any feasible trajectory
        [feasible, ~, flag, ~] = ...
            fmincon(@(x) call(ccostFun, x, 2),x0,A,B,Aeq,Beq,lb,ub, ...
                    @(x) call(constraintsFun, x, 4), cOptions);
        tryCount = tryCount + 1;
    end

    if flag > 0
        fprintf('########## Optimizing for torque/raddot squared ##########\n');
        % Optimize for torque/raddot squared cost
        [optimal, ~, ~, ~] = ...
            fmincon(@(x) call(acostFun, x, 2),feasible,A,B, ...
                    Aeq,Beq,lb,ub,@(x) call(constraintsFun,x,4),aOptions);
        fprintf('########## Optimizing for work ##########\n');
        % Optimize for minimum work
        [optimal, cost, flag, ~] = ...
            fmincon(@(x) call(awcostFun, x, 2),optimal,A,B, ...
                    Aeq,Beq,lb,ub,@(x) call(constraintsFun,x,4),aOptions);
        fprintf('Done optimizing\n');
    else
        optimal = [];
        cost = 0;
        fprintf('Exited with non-positive flag: %d\n', flag);
    end            
end

