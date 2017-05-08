function [] = animate( fig, times, xtoes, xs, ys, phis, lens, ...
                       transTs, sp, vp )
    % Initialize the figure
    figure(fig);
    clf;
    % Make all the plots draw to the same figure
    hold on;
    % Draw ground
    patch([-100 100 100 -100], [0 0 -5 -5], [0 0.5 0], ...
          'FaceColor', [0.05 0.2 0.05]);
    
    % Draw ice patch
    patch([sp.slipPatch, fliplr(sp.slipPatch)], ...
          [0 0 -0.1 -0.1], [0 0 0.5], 'FaceColor', [0.5 0.5 1]);
    
    % Initialize the spring plot
    springPlot = plot(0, 0);
    % Draw the path of the hip
    if vp.showPath
        if ~vp.interpolate
            fprintf('Warning: to show path must interpolate first');
        end
        
        selTimes = times(1:vp.pathSel:end);
        selXs = xs(1:vp.pathSel:end);
        selYs = ys(1:vp.pathSel:end);
        selXtoes = xtoes(1:vp.pathSel:end);
        
        % Plot paths
        for i = 1:(length(transTs)-1)
            indices = find(selTimes >= transTs(i) & selTimes < transTs(i+1));
            
            % Get color for current phase
            if mod(i,2)
                color = vp.phaseColor(ceil(i / 2), :);
            else
                color = vp.flightColor;
            end
            
            if isempty(indices)
                % If last phase, plot end position for clarity
                if i == length(transTs) - 1
                    plot(xs(end), ys(end), 'o', ...
                        'MarkerSize', vp.markerSize, ...
                        'MarkerEdgeColor', 'none', ...
                        'MarkerFaceColor', color);
                else
                    continue
                end
            end
            
            % Plot path of toe during slipping phase
            if i == 1
                plot(selXtoes(indices), zeros(length(indices)), 'o', ...
                 'MarkerSize', vp.markerSize, ...
                 'MarkerEdgeColor', 'none', ...
                 'MarkerFaceColor', vp.toeColor);
                plot(selXtoes(1), zeros(1), 'o', ...
                 'MarkerSize', vp.markerSize+1, ...
                 'MarkerEdgeColor', vp.toeStartColor);
                plot(selXtoes(1), zeros(1), 'o', ...
                 'MarkerSize', vp.markerSize+2, ...
                 'MarkerEdgeColor', vp.toeStartColor);
                plot(selXtoes(indices(end)), zeros(1), 'o', ...
                 'MarkerSize', vp.markerSize+1, ...
                 'MarkerEdgeColor', vp.toeEndColor);
                plot(selXtoes(indices(end)), zeros(1), 'o', ...
                 'MarkerSize', vp.markerSize+2, ...
                 'MarkerEdgeColor', vp.toeEndColor);
            end
            
            % Plot path of hip
            plot(selXs(indices), selYs(indices), 'o', ...
                 'MarkerSize', vp.markerSize, ...
                 'MarkerEdgeColor', 'none', ...
                 'MarkerFaceColor', color);
        end
    end
    % Initialize the hip circle fill
    hipCircle = [];
    
    time = -vp.dt;
    if vp.drawText
        % Draw debug text
        timeText = text(0, 1.8, sprintf('Simulation time: %f', time), 'FontSize', 13);
        lenText = text(0, 1.6, sprintf('len: %f', 0), 'FontSize', 13);
    end

    has_saved = false;
    while time <= times(end)
        time = time + vp.dt;
        tgreater = find(times >= time);
        if ~isempty(tgreater)
            ti = tgreater(1);
        else
            ti = length(lens);
        end
        
        ncoils = 10;
        coilres = 4;
        springY = linspace(0, -lens(ti), 4*coilres*ncoils + 1);
        springX = sin(springY*2*pi*ncoils / lens(ti)) * vp.springWidth;

        % Rotate springX and springY
        springMat = [springX; springY];
        center = repmat([0; 0], 1, length(springX));
        R = [cos(phis(ti)-pi/2) -sin(phis(ti)-pi/2); ...
             sin(phis(ti)-pi/2) cos(phis(ti)-pi/2)];
        rotMat = R * (springMat - center) + center;
        springX = rotMat(1,:) + xs(ti);
        springY = rotMat(2,:) + ys(ti);

         % Clear the hip circle
        if ~isempty(hipCircle)
            delete(hipCircle);
        end

        % Draw spring
        springPlot.XData = springX;
        springPlot.YData = springY;
        springPlot.Color = 'k';

        % Draw hip/pelvis
        [X,Y] = pol2cart(linspace(0, 2 * pi, 100), ...
                         ones(1, 100) * vp.hipDiam);
        X = X + xs(ti);
        Y = Y + ys(ti);
        hipCircle = fill(X, Y, vp.hipColor);
        alpha(vp.hipAlpha);
        
        if vp.drawText
            timeText.String = sprintf('Simulation time: %f', time);
            lenText.String = sprintf('len: %f', lens(ti));
        end

        camWidth = sp.slipPatch(2) - sp.slipPatch(1) + vp.camMargin;
        camArea = [sp.slipPatch(1) - vp.camMargin / 2, ...
                   sp.slipPatch(2) + vp.camMargin / 2, ...
                   sp.maxlen / 2 - camWidth / 2, ...
                   sp.maxlen / 2 + camWidth / 2];
        axis(camArea);
        set(gca,'visible','off');
        axis square;
        axis manual;
        pause(vp.dt * vp.pauseFactor);
        
        if ~has_saved && ~strcmp(vp.picPath, 'none')
            saveas(fig, vp.picPath);
            has_saved = true;
        end
    end
end

