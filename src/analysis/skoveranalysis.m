results = readSim(14);
% 14 is interesting
af = results.analyzeFor();

lineWidth = 4;

times = linspace(0, af.stanceT(1), results.paramsFor.gridn);
xtoe = af.xtoe(1:10) - af.xtoe(1);
xtoedot = af.xtoedot(1:10);

tQuery = 0 : 0.01 : times(end);

hEndTime = times(end);
htQuery = 0 : 0.01 : hEndTime;

hxtoeTime = [0 0.1 0.2 0.3 0.4 0.45 0.5 0.55 0.6 0.7 0.8];
hxtoe = [0 0.06 0.12 0.18 0.24 0.27 0.29 0.3 0.31 0.305 0.28];
hxtoeTime = hxtoeTime * hEndTime / hxtoeTime(end);
[xDataPrep, yDataPrep] = prepareCurveData( hxtoeTime, hxtoe );
[fitResult, ~] = fit( xDataPrep, yDataPrep, fittype('smoothingspline'));
hxtoeQuery = feval(fitResult, htQuery);

hxtoedotTime = [0 0.1 0.15 0.2 0.3 0.4 0.45 0.5 0.6 0.7 0.8];
hxtoedot = [0 0.38 0.5 0.52 0.5 0.45 0.49 0.52 0.35 0 -0.7];
hxtoedotTime = hxtoedotTime * hEndTime / hxtoedotTime(end);
[xDataPrep, yDataPrep] = prepareCurveData( hxtoedotTime, hxtoedot );
[fitResult, ~] = fit( xDataPrep, yDataPrep, fittype('smoothingspline'));
hxtoedotQuery = feval(fitResult, htQuery);

xtoeQuery = interp1(times, xtoe, tQuery, 'spline');
xtoedotQuery = interp1(times, xtoedot, tQuery, 'spline');

fig = figure();
set(fig, 'defaultAxesColorOrder', [0.96 0.56 0.26; 0.26 0.41 0.96]);
title('Skateover: Foot Position / Velocity vs Time');

xlabel('Time (s, adjusted)');
xticks(0:0.2:1.5);

yyaxis left;
hold on;
plot(htQuery, hxtoeQuery, '--', 'LineWidth', lineWidth);
plot(tQuery, xtoeQuery, 'LineWidth', lineWidth);
ylabel('Foot Position (m)');
ylim([-0.5, 0.5]);

yyaxis right;
plot(htQuery, hxtoedotQuery, '--', 'LineWidth', lineWidth);
plot(tQuery, xtoedotQuery, 'LineWidth', lineWidth);
ylabel('Foot Velocity (m/s)');
ylim([-1.1, 1.1]);

legend({'Human Foot Position', 'SLIP Foot Position', 'Human Foot Velocity', 'SLIP Foot Velocity'}, 'Location', 'southwest');
set(fig.CurrentAxes, 'TickDir', 'out');
set(findall(fig,'-property','FontSize'), 'FontSize', 30)