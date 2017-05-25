function [] = animate( fig, times, xtoes, xs, ys, phis, lens, ...
                       transTs, ntoParams, visParams )
    % Initialize the figure
    figure(fig);
    clf;
    % Make all the plots draw to the same figure
    hold on;
    % Draw ground
    patch([-100 100 100 -100], [0 0 -5 -5], [0 0.5 0], ...
          'FaceColor', [0.05 0.2 0.05]);
    
    % Draw ice patch
    patch([ntoParams.slipPatch, fliplr(ntoParams.slipPatch)], ...
          [0 0 -0.1 -0.1], [0 0 0.5], 'FaceColor', [0.5 0.5 1]);
    
    % Initialize the spring plot
    springPlot = plot(0, 0);
    % Draw the path of the hip
    if visParams.showPath
        if ~visParams.interpolate
            fprintf('Warning: to show path must interpolate first');
        end
        
        selTimes = times(1:visParams.pathSel:end);
        selXs = xs(1:visParams.pathSel:end);
        selYs = ys(1:visParams.pathSel:end);
        selXtoes = xtoes(1:visParams.pathSel:end);
        
        % Plot paths
        for i = 1:(length(transTs)-1)
            indices = find(selTimes >= transTs(i) & selTimes < transTs(i+1));
            
            % Get color for current phase
            if mod(i,2)
                color = visParams.phaseMColor(ceil(i / 2), :);
            else
                color = visParams.flightMColor;
            end
            
            if isempty(indices)
                % If last phase, plot end position for clarity
                if i == length(transTs) - 1
                    plot(xs(end), ys(end), 'o', ...
                        'MarkerSize', visParams.markerSize, ...
                        'MarkerEdgeColor', 'none', ...
                        'MarkerFaceColor', color);
                else
                    continue
                end
            end
            
            % Plot path of toe during slipping phase
            if i == 1
                plot(selXtoes(indices), zeros(length(indices)), 'o', ...
                 'MarkerSize', visParams.markerSize, ...
                 'MarkerEdgeColor', 'none', ...
                 'MarkerFaceColor', visParams.toeMColor);
                plot(selXtoes(1), zeros(1), 'o', ...
                 'MarkerSize', visParams.markerSize+1, ...
                 'MarkerEdgeColor', visParams.toeMStartColor);
                plot(selXtoes(1), zeros(1), 'o', ...
                 'MarkerSize', visParams.markerSize+2, ...
                 'MarkerEdgeColor', visParams.toeMStartColor);
                plot(selXtoes(indices(end)), zeros(1), 'o', ...
                 'MarkerSize', visParams.markerSize+1, ...
                 'MarkerEdgeColor', visParams.toeMEndColor);
                plot(selXtoes(indices(end)), zeros(1), 'o', ...
                 'MarkerSize', visParams.markerSize+2, ...
                 'MarkerEdgeColor', visParams.toeMEndColor);
            end
            
            % Plot path of hip
            plot(selXs(indices), selYs(indices), 'o', ...
                 'MarkerSize', visParams.markerSize, ...
                 'MarkerEdgeColor', 'none', ...
                 'MarkerFaceColor', color);
        end
    end
    % Initialize the hip circle fill
    hipCircle = [];
    % Initialize the toe circle fill
    toeCircle = [];
    
    time = -visParams.dt;
    if visParams.drawText
        % Draw debug text
        timeText = text(0, 1.8, sprintf('Simulation time: %f', time), 'FontSize', 13);
        lenText = text(0, 1.6, sprintf('len: %f', 0), 'FontSize', 13);
    end

    has_saved = false;
    while time <= times(end)
        time = time + visParams.dt;
        tgreater = find(times >= time);
        if ~isempty(tgreater)
            ti = tgreater(1);
        else
            ti = length(lens);
        end
        
        ncoils = 10;
        coilres = 4;
        springY = linspace(0, -lens(ti), 4*coilres*ncoils + 1);
        springX = sin(springY*2*pi*ncoils / lens(ti)) * visParams.springWidth;

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
         % Clear the toe circle
        if ~isempty(toeCircle)
            delete(toeCircle);
        end

        % Draw spring
        springPlot.XData = springX;
        springPlot.YData = springY;
        springPlot.Color = 'k';

        % Draw hip
        [hipX,hipY] = pol2cart(linspace(0, 2 * pi, 100), ...
                         ones(1, 100) * visParams.hipDiam);
        hipX = hipX + xs(ti);
        hipY = hipY + ys(ti);
        hipCircle = fill(hipX, hipY, visParams.hipColor);
        alpha(visParams.hipAlpha);
        
        % Draw toe
        if ~isnan(xtoes(ti))
            [toeX,toeY] = pol2cart(linspace(0, 2 * pi, 100), ...
                             ones(1, 100) * visParams.toeDiam);
            toeX = toeX + xtoes(ti);
            toeY = toeY + 0;
            toeCircle = fill(toeX, toeY, visParams.toeColor);
            alpha(visParams.toeAlpha);
        end
        
        if visParams.drawText
            timeText.String = sprintf('Simulation time: %f', time);
            lenText.String = sprintf('len: %f', lens(ti));
        end
    
        %leftCapture = ntoParams.slipPatch(1);
        %rightCapture = ntoParams.slipPatch(2);
        leftCapture = -1;
        rightCapture = 1;
        camWidth = rightCapture - leftCapture + visParams.camMargin;
        camArea = [leftCapture - visParams.camMargin / 2, ...
                   rightCapture + visParams.camMargin / 2, ...
                   ntoParams.maxlen / 2 - camWidth / 2, ...
                   ntoParams.maxlen / 2 + camWidth / 2];
        
        axis(camArea);
        set(gca,'visible','off');
        axis square;
        axis manual;
        pause(visParams.dt * visParams.pauseFactor);
        
        if ~has_saved && ~strcmp(visParams.picPath, 'none')
            saveas(fig, visParams.picPath);
            has_saved = true;
        end
    end
end

