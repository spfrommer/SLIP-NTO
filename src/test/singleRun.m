GEN_CONSTRAINTS = true;
GEN_COSTS = true;

sp = SimParams(['str'], [0,1], [1.1; 0; 1.1-cosd(45); 0; ...
                                sind(45); 0; 1; 0], 1.1);

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
[lb, ub] = bounds(sp);

tic

numVars = size(sp.phases, 1) * 2 - 1 + sp.gridn * size(sp.phases, 1) * 10;
funparams = conj(sym('x', [1 numVars], 'real')');

if GEN_CONSTRAINTS
    fprintf('Generating constraints...\n');
    [c, ceq] = constraints(funparams, sp);
    cjac = jacobian(c, funparams).';
    ceqjac = jacobian(ceq, funparams).';
    constraintsFun = matlabFunction(c, ceq, cjac, ceqjac, 'Vars', {funparams});
    fprintf('Done generating constraints...\n');
end

if GEN_COSTS
    fprintf('Generating costs...\n');
    ccost = constcost(funparams, sp);
    ccostjac = jacobian(ccost, funparams).';
    ccostFun = matlabFunction(ccost, ccostjac, 'Vars', {funparams});

    acost = actsqrcost(funparams, sp);
    acostjac = jacobian(acost, funparams).';
    acostFun = matlabFunction(acost, acostjac, 'Vars', {funparams});
    
    awcost = actworkcost(funparams, sp);
    awcostjac = jacobian(awcost, funparams).';
    awcostFun = matlabFunction(awcost, awcostjac, 'Vars', {funparams});
    fprintf('Done generating costs...\n');
end

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

    % Find any feasible trajectory
    [feasible, ~, flag, ~] = ...
        fmincon(@(x) call(ccostFun, x, 2),x0,A,B,Aeq,Beq,lb,ub, ...
                @(x) call(constraintsFun, x, 4), cOptions);
    tryCount = tryCount + 1;
end

if flag > 0
    fprintf('Optimizing for torque/raddot squared...\n');
    % Optimize for torque/raddot squared cost
    [optimal, ~, ~, ~] = ...
        fmincon(@(x) call(acostFun, x, 2),feasible,A,B, ...
                Aeq,Beq,lb,ub,@(x) call(constraintsFun,x,4),aOptions);
    fprintf('Optimizing for work...\n');
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

fprintf('Finished optimizing in %f seconds\n', toc);
visualize