numSims = Resource.instance().numDataFiles();

xdiffs = zeros(numSims, 1);
xdots = zeros(numSims, 1);
features = zeros(numSims, 4);

forStrats = zeros(numSims, 1); % -1 is none, 0 is leap, 1 is walk-over, 2 is slide
backStrats = zeros(numSims, 1); % -1 is none, 0 is normal, 1 is backslide
bestStrats = zeros(numSims, 1);

for i = 1 : numSims
    fprintf('Analyzing sim: %d\n', i);
    results = readSim(i);
    af = results.analyzeFor();
    ab = results.analyzeBack();
    % Whether the forwards strategy used the leap sequence
    forLeap = true;
    % Whether the backwards strategy used the back slide sequence
    backSlide = false;
    
    if af.exists
        if af.xtoe(10) - af.xtoe(1) > 0.15 && ...
           sum(af.xtoe(1:10) >= af.xtoe(1)) == 10
           forStrats(i) = 2; % slide
        else
           if af.flightT(1) < 0.1
               forStrats(i) = 1; % walk-over
           else
               forStrats(i) = 0; % leap
           end
        end
    else
        forStrats(i) = -1; % none
    end
    
    if ab.exists
        if ab.xtoe(1) - ab.xtoe(10) > 0.15 && ...
           sum(ab.xtoe(1:10) > ab.xtoe(1)) == 0
           backStrats(i) = 1; % backslide
        else
           backStrats(i) = 0; % normal
        end
    else
        backStrats(i) = -1;
    end
    
    features(i, 1) = results.paramsFor.finalProfileX - results.paramsFor.iss().x;
    features(i, 2) = results.paramsFor.iss().x - results.paramsFor.slipPatch(1);
    features(i, 3) = results.paramsFor.iss().xdot;
    features(i, 4) = results.paramsFor.iss().xtoe - results.paramsFor.iss().x;
    
    xdiffs(i) = (results.paramsFor.finalProfileX - results.paramsFor.iss().x) - ...
            (results.paramsFor.iss().x - results.paramsFor.slipPatch(1));
    xdots(i) = results.paramsFor.iss().xdot;
    %xdots(i) = er.paramsFor.iss().xtoe - er.paramsFor.iss().x;
    %xdiffs(i) = (er.paramsFor.finalProfileX - er.paramsFor.iss().x);
    %xdots(i) = (er.paramsFor.iss().x - er.paramsFor.slipPatch(1));
    
    if results.getControl() == 1
        bestStrats(i) = forStrats(i);
    else
        bestStrats(i) = backStrats(i) + 2;
    end
end

fprintf('Visualizing analysis results\n');

forRemove = find(forStrats == -1);
backRemove = find(backStrats == -1);

colorFor = forStrats;
colorBack = backStrats;
colorBest = bestStrats;

Mdl = fitcdiscr(features, bestStrats);
predicted = predict(Mdl, features);
matching = predicted == bestStrats;

figure(1);
%colormap jet
scatter(xdiffs, xdots, [], colorBest);

a = (1:numSims)';
b = num2str(a);
c = cellstr(b);
text(xdiffs + rand(numSims, 1) * 0.02, xdots + rand(numSims, 1) * 0.02, c);
%text(xdiffs + 0.01, xdots + 0.01, c);