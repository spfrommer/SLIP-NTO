results = readSim(30);
% 30 is maybe good, same with 34
af = results.analyzeFor();

times = linspace(0, af.stanceT(1), results.paramsFor.gridn);
xtoe = af.xtoe(1:10) - af.xtoe(1);
xtoedot = af.xtoedot(1:10);

tQuery = 0 : 0.01 : times(end);

hEndTime = times(end);
htQuery = 0 : 0.01 : hEndTime;

hxtoeTime = [0 0.1 0.2 0.3 0.4 0.45 0.5 0.55 0.6 0.7 0.73];
hxtoe = [0 0.027 0.047 0.063 0.061 0.055 0.045 0.02 0.005 0 0];
hxtoeTime = hxtoeTime * hEndTime / hxtoeTime(end);
[xDataPrep, yDataPrep] = prepareCurveData( hxtoeTime, hxtoe );
[fitResult, ~] = fit( xDataPrep, yDataPrep, fittype('smoothingspline'));
hxtoeQuery = feval(fitResult, htQuery);

%hxtoePoly = polyfit(hxtoeTime(1:end-2), hxtoe(1:end-2), 4);
%hxtoeQuery = polyval(hxtoePoly, htQuery(htQuery <= hxtoeTime(end-2)));
%hxtoeQuery = [hxtoeQuery, zeros(1, length(htQuery) - length(hxtoeQuery))];

hxtoedotTime = [0 0.05 0.1 0.14 0.17 0.19 0.23 0.28 0.3 0.35 0.4 0.5 0.55 0.58 0.59 0.6 0.61 0.62 0.65 0.7 0.73];
hxtoedot = [0 0.2 0.26 0.29 0.3 0.28 0.24 0.12 0.05 -0.03 -0.08 -0.13 -0.22 -0.5 -0.52 -0.55 -0.52 -0.5 -0.07 -0.03 0];
hxtoedotTime = hxtoedotTime * hEndTime / hxtoedotTime(end);
[xDataPrep, yDataPrep] = prepareCurveData( hxtoedotTime, hxtoedot );
[fitResult, ~] = fit( xDataPrep, yDataPrep, fittype('smoothingspline'));
hxtoedotQuery = feval(fitResult, htQuery);

xtoeQuery = interp1(times, xtoe, tQuery, 'spline');
xtoedotQuery = interp1(times, xtoedot, tQuery, 'spline');

fig = figure();
set(fig, 'defaultAxesColorOrder', [0 0.5 0; 0 0 0.5]);
title('Walkover: Toe Position / Velocity vs Time');

%set(gca,'XTick',[])
xlabel('Time (s, adjusted)');

yyaxis left;
hold on;
plot(htQuery, hxtoeQuery, '--');
plot(tQuery, xtoeQuery);
ylabel('Toe Position (m)');
ylim([-0.09, 0.09]);

yyaxis right;
plot(htQuery, hxtoedotQuery, '--');
plot(tQuery, xtoedotQuery);
ylabel('Toe Velocity (m/s)');
ylim([-0.9, 0.9]);

legend('Human Toe Position', 'SLIP Toe Position', 'Human Toe Velocity', 'SLIP Toe Velocity');