addpath(genpath('../DRL/Coalesce'));
addpath(genpath('../../src'));

rng('shuffle');

clear all; close all; clc;

ntoParams = NTOParams('str', [-0.1,-0.1], ...
                          [0; 0; -cos(pi/4); 0; sin(pi/4); 0; 1; 0], ...
                          0);

dc = DirectCollocation([], 'Name', 'Test Optimization');

phase = dc.addPhase(ntoParams.gridn);
xtoe = phase.addState(1, -Inf, Inf).expand();
xtoedot = phase.addState(1, -Inf, Inf).expand();
x = phase.addState(1, -Inf, Inf).expand();
xdot = phase.addState(1, -Inf, Inf).expand();
y = phase.addState(1, 0, Inf).expand();
ydot = phase.addState(1, -Inf, Inf).expand();
ra = phase.addState(1, -Inf, Inf).expand();
radot = phase.addState(1, -Inf, Inf).expand();

raddot = phase.addInput(1, -1, 1).expand();
torque = phase.addInput(1, -1, 1).expand();

funParams = [phase.duration, xtoe, xtoedot, x, xdot, y, ydot, ...
                                    ra, radot, raddot, torque];

[c, ceq] = coalesceConstraints(funParams, ntoParams);

cost = coalesceActsqrcost(phase.duration, raddot, torque, ntoParams);

for i = 1:length(ceq)
    dc.addConstraint(0, ceq(i), 0);
end

for i = 1:length(c)
    dc.addConstraint(-Inf, c(i), 0);
end

dc.addObjective(cost, 'Description', 'Effort Squared');

optim = Fmincon(dc);
optim.export;
optim.solve;

stanceT = eval(phase.duration);

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

optimal = [stanceT; xtoe; xtoedot; x; xdot; y; ydot; ...
           ra; radot; raddot; torque];

visualize(optimal, ntoParams, VisParams());