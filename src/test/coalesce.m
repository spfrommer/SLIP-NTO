addpath(genpath('../DRL/Coalesce'));
addpath(genpath('../../src'));

rng('shuffle');

clear all; close all; clc;

ntoParams = NTOParams(['str'; 'str'], [-0.1,-0.1], ...
                          [0; 0; -cos(pi/4); 0; sin(pi/4); 0; 1; 0], ...
                          1);

dc = DirectCollocation([], 'Name', 'Test Optimization');
%{
[xtoe, xtoedot, x, xdot, y, ydot, ra, radot, raddot, torque] = deal([]);
for p = 1 : size(ntoParams.phases, 1)
    phase = dc.addPhase(ntoParams.gridn, 0.01, Inf);
    
end
%}
phase1 = dc.addPhase(ntoParams.gridn, 0.1, Inf);
phase2 = dc.addPhase(ntoParams.gridn, 0.1, Inf);

stanceT = [phase1.duration, phase2.duration];
flightT = dc.addVariable(0, 0.8, Inf);

xtoe = [phase1.addState(1, -Inf, Inf).expand(), phase2.addState(1, -Inf, Inf).expand()];
xtoedot = [phase1.addState(1, -Inf, Inf).expand(), phase2.addState(1, -Inf, Inf).expand()];
x = [phase1.addState(1, -Inf, Inf).expand(), phase2.addState(1, -Inf, Inf).expand()];
xdot = [phase1.addState(1, -Inf, Inf).expand(), phase2.addState(1, -Inf, Inf).expand()];
y = [phase1.addState(1, 0, Inf).expand(), phase2.addState(1, 0, Inf).expand()];
ydot = [phase1.addState(1, -Inf, Inf).expand(), phase2.addState(1, -Inf, Inf).expand()];
ra = [phase1.addState(1, -Inf, Inf).expand(), phase2.addState(1, -Inf, Inf).expand()];
radot = [phase1.addState(1, -Inf, Inf).expand(), phase2.addState(1, -Inf, Inf).expand()];

raddot = [phase1.addInput(1, -1, 1).expand(), phase2.addInput(1, -1, 1).expand()];
torque = [phase1.addInput(1, -1, 1).expand(), phase2.addInput(1, -1, 1).expand()];

funParams = [stanceT, flightT, xtoe, xtoedot, x, xdot, y, ydot, ...
                                    ra, radot, raddot, torque];

[c, ceq] = coalesceConstraints(funParams, ntoParams);

cost = coalesceActsqrcost(stanceT', raddot, torque, ntoParams);

for i = 1:length(ceq)
    dc.addConstraint(0, ceq(i), 0);
end

for i = 1:length(c)
    dc.addConstraint(-Inf, c(i), 0);
end

dc.addObjective(cost, 'Description', 'Effort Squared');

optim = Fmincon(dc);
optim.setOptions(optimoptions(@fmincon, 'TolFun', 0.0000000001, ...
                 'MaxIterations', 1000, ...
                 'MaxFunEvals', 1000000, ...
                 'Display', 'iter', ...
                 'Algorithm', 'sqp', ...
                 'StepTolerance', 1e-13, ...
                 'SpecifyConstraintGradient', true, ...
                 'SpecifyObjectiveGradient', true, ...
                 'ConstraintTolerance', 1e-6, ...
                 'CheckGradients', false, ...
                 'HonorBounds', true, ...
                 'FiniteDifferenceType', 'central', ...
                 'UseParallel', true));
optim.export;
optim.solve;

stanceT = [eval(phase1.duration); eval(phase2.duration)];
flightT = eval(flightT);

xtoe = squeeze(eval(xtoe))';
xtoedot = squeeze(eval(xtoedot))';

x = squeeze(eval(x))';
xdot = squeeze(eval(xdot))';

y = squeeze(eval(y))';
ydot = squeeze(eval(ydot))';

ra = squeeze(eval(ra))';
radot = squeeze(eval(radot))';
raddot = squeeze(eval(raddot))';

torque = squeeze(eval(torque))';

optimal = [stanceT; flightT; xtoe; xtoedot; x; xdot; y; ydot; ...
           ra; radot; raddot; torque];

visualize(optimal, ntoParams, VisParams());