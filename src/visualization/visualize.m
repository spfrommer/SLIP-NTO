function [] = visualize( funparams, sp, vp )
    % Unpack the vector
    [ stanceT, flightT, xtoe, ~, x, xdot, y, ydot, ...
               ~, ~, ~, ~] = unpack(funparams, sp);

    % Make vector of times of each phase transition
    transT = [];
    transT(1:2:length(stanceT)*2) = stanceT;
    transT(2:2:length(flightT)*2) = flightT;
    transT = [0, cumsum(transT)];

    % Calculate leg lengths
    r = sqrt((x - xtoe).^2 + y.^2);

    % Discretize the times for the first phase
    times = 0 : stanceT(1) / sp.gridn : stanceT(1);
    times = times(1 : end-1);

    i = sp.gridn + 1;
    for p = 1 : size(sp.phases, 1) - 1
        % Interpolate ballistic flight
        time = 0;
        while time < flightT(p) - vp.dt
            xtoe = [xtoe(1:i-1); xtoe(i-1); xtoe(i:end)];
            x = [x(1:i-1); (x(i-1)+xdot(i-1)*vp.dt); x(i:end)];
            xdot = [xdot(1:i-1); xdot(i-1); xdot(i:end)];
            y = [y(1:i-1); (y(i-1)+ydot(i-1)*vp.dt); y(i:end)];
            ydot = [ydot(1:i-1); (ydot(i-1)-sp.gravity*vp.dt); ydot(i:end)];

            r = [r(1:i-1); 0; r(i:end)];
            i = i + 1;
            time = time + vp.dt;
            times = [times times(end)+vp.dt];
        end
        times = [times times(end)+vp.dt];
        times = [times(1 : end-1) times(end) : stanceT(p+1) / sp.gridn ...
                                             : (times(end) + stanceT(p+1))];
        times = times(1:end-1);
        i = i + sp.gridn;
    end


    if vp.interpolate
        xtoe  = interp1(times, xtoe, 0:vp.dt:times(end), 'linear');
        x     = interp1(times, x,    0:vp.dt:times(end), 'linear');
        y     = interp1(times, y,    0:vp.dt:times(end), 'linear');
        % TODO: fix r interpolation between phase transitions
        r     = interp1(times, r,    0:vp.dt:times(end), 'linear');
        times = 0:vp.dt:times(end);
    end
    phi = atan2(y, x - xtoe);

    fig = figure();

    clf;
    set(fig, 'Position', [100, 200, 600, 550]);
    toolbar = uitoolbar(fig);
    % Prevent clf from clearing the toolbar
    toolbar.HandleVisibility = 'off';
    % Read an image
    [img,map] = imread('rewind.gif');
    p = uipushtool(toolbar, 'TooltipString', 'Replay animation', ...
          'ClickedCallback', @(im, e) animate(fig, times, xtoe, x, y, ...
                                              phi, r, transT, sp, vp));
    icon = ind2rgb(img, map);
    p.CData = icon;
    animate(fig, times, xtoe, x, y, phi, r, transT, sp, vp);
end