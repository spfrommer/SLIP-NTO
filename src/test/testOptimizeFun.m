slipPatch = [0, 1];
dy = 1 - sind(45);
dpe = 1 * 1 * dy;
vi = sqrt(dpe * 2);

stateI = [1.1; 0; 1.1 - cosd(45); vi * sind(45); sind(45); vi * cosd(45); 1; 0];
finX = 1.1;

params = NTOParams(['str'], slipPatch, stateI, finX);
[optimal, cost, flag] = optimize(params);

visualize(optimal, params, VisParams());