% A kinda tough 3-phase problem
% Has local optima for the backwards sequence
params = NTOParams(['str'], [-0.1,-0.1], ...
                          [0; 0; -cos(pi/4); 0; sin(pi/4); 0; 1; 0], ...
                          0);
[optimal, cost, flag] = optimize(params);

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