GEN_CONSTRAINTS = true;
GEN_COSTS = true;

% A trek setup
%params = NTOParams(['str'; 'str'; 'str'; 'str'; 'str'], [-.1,-.1], ...
%                   [0; 0; 0; 0.5; 1; 0; 1; 0], 8);
%params = NTOParams(['str'], [-0.1,-0.1], ...
%                          [0; 0; -cos(pi/4); 0; sin(pi/4); 0; 1; 0], ...
%                          0);
params = NTOParams(['str'; 'str'], [-0.1,-0.1], ...
                          [0; 0; 0; 0; 1; 0; 1; 0], ...
                          1);

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
                   'HonorBounds', true);
% Options for optimizing the actual cost function
aOptions = optimoptions(@fmincon, 'TolFun', 0.00000001, ...
                   'MaxIterations', 1000, ...
                   'MaxFunEvals', 1000000, ...
                   'Display', 'iter', ...
                   'Algorithm', 'sqp', ...
                   'StepTolerance', 1e-13, ...
                   'OptimalityTolerance', 1e-6, ...
                   'SpecifyConstraintGradient', true, ...
                   'SpecifyObjectiveGradient', true, ...
                   'ConstraintTolerance', 1e-8, ...
                   'CheckGradients', false, ...
                   'Diagnostics', 'on', ...
                   'HonorBounds', true, ...
                   'FiniteDifferenceType', 'central', ...
                   'PlotFcn', [], ...
                   'FunValCheck', 'on');
% No linear inequality or equality constraints
A = [];
B = [];
Aeq = [];
Beq = [];
% Set up the bounds
[lb, ub] = bounds(params);

numVars = size(params.phases, 1) * 2 - 1 + params.gridn * size(params.phases, 1) * 10;
funparams = conj(sym('x', [1 numVars], 'real')');

if GEN_CONSTRAINTS
    fprintf('Generating constraints...\n');
    [c, ceq] = constraints(funparams, params);
    cjac = jacobian(c, funparams).';
    ceqjac = jacobian(ceq, funparams).';
    fprintf('Generating constraints function...\n');
    constraintsFun = matlabFunction(c, ceq, cjac, ceqjac, 'Vars', {funparams});
end

if GEN_COSTS
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
end

tic

flag = 1;
%flag = -1;
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

    %{
    [optimal, ~, ~, ~] = ...
        fmincon(@(x) call(acostFun, x, 2),feasible,A,B, ...
                Aeq,Beq,lb,ub,@(x) call(constraintsFun,x,4),aOptions);

    [optimal, cost, flag, ~] = ...
        fmincon(@(x) call(acostFun, x, 2),feasible,A,B, ...
                Aeq,Beq,lb,ub,@(x) call(constraintsFun,x,4),aOptions);
    %}
    
    if true
        [optimal, ~, ~, ~] = ...
            fmincon(@(x) call(acostFun, x, 2), ...
                   MinMaxCheck(lb, ub, ones(numVars, 1) * rand() * 2), ...
                   A, B, Aeq, Beq, lb, ub, ...
                   @(x) call(constraintsFun,x,4),aOptions);
    else
        [optimal, ~, ~, ~] = ...
            fmincon(@(x) call(acostFun, x, 2), ...
                    optimal, ...
                    A, B, Aeq, Beq, lb, ub, ...
                    @(x) call(constraintsFun,x,4),aOptions);
    end
    [ stanceT, flightT, xtoe, xtoedot, x, xdot, y, ydot, ...
             ra, radot, raddot, torque] = unpack(optimal, params);

    figure();
    plot(torque, '*');
    visualize(optimal, params, VisParams());

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
    
fprintf('Finished optimizing in %f seconds\n', toc);

[ stanceT, flightT, xtoe, xtoedot, x, xdot, y, ydot, ...
             ra, radot, raddot, torque] = unpack(optimal, params);

r = sqrt((x - xtoe).^2 + y.^2);
rdot = ((x-xtoe).*(xdot)+y.*ydot)./(r);
cphi = (x-xtoe) ./ r;
sphi = y ./ r;
fs = params.spring .* (ra - r) + params.damp .* (radot - rdot);
ft = torque ./ r;
fg = params.masship .* params.gravity;

xddot = (1./params.masship) .* (fs.*cphi + ft .* sphi);
yddot = (1./params.masship) .* (fs.*sphi + ft .* (-cphi) - fg);

grf = fs .* sphi - ft .* cphi + params.masstoe * params.gravity;
    
visualize(optimal, params, VisParams());
figure();
plot(torque, '*');