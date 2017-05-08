numSims = 157;
gridN = 10;

xdiffs = zeros(numSims, 1);
xdots = zeros(numSims, 1);
features = zeros(numSims, 4);

forStrats = zeros(numSims, 1); % -1 is none, 0 is leap, 1 is walk-over, 2 is slide
backStrats = zeros(numSims, 1); % -1 is none, 0 is normal, 1 is backslide
bestStrats = zeros(numSims, 1);

for i = 1 : numSims
    er = examineSim(i, false);
    af = er.analyzeFor();
    ab = er.analyzeBack();
    % Whether the forwards strategy used the leap sequence
    forLeap = true;
    % Whether the backwards strategy used the back slide sequence
    backSlide = false;
    
    if af.exists
        if af.xtoe(10) - af.xtoe(1) > 0.15 && ...
           sum(af.xtoe(1:10) >= af.xtoe(1)) == 10
            forStrats(i) = 2;
        else
            if af.flightT(1) < 0.1
                forStrats(i) = 1;
            else
                forStrats(i) = 0;
            end
        end
    else
        forStrats(i) = -1;
    end
    
    if ab.exists
        if ab.xtoe(1) - ab.xtoe(10) > 0.15 && ...
           sum(ab.xtoe(1:10) > ab.xtoe(1)) == 0
            backStrats(i) = 1;
        else
            backStrats(i) = 0;
        end
    else
        backStrats(i) = -1;
    end
    
    features(i, 1) = er.spFor.finalProfileX - er.spFor.iss().x;
    features(i, 2) = er.spFor.iss().x - er.spFor.slipPatch(1);
    features(i, 3) = er.spFor.iss().xdot;
    features(i, 4) = er.spFor.iss().xtoe - er.spFor.iss().x;
    
    xdiffs(i) = (er.spFor.finalProfileX - er.spFor.iss().x) - ...
            (er.spFor.iss().x - er.spFor.slipPatch(1));
    xdots(i) = er.spFor.iss().xdot;
    %xdots(i) = er.spFor.iss().xtoe - er.spFor.iss().x;
    %xdiffs(i) = (er.spFor.finalProfileX - er.spFor.iss().x);
    %xdots(i) = (er.spFor.iss().x - er.spFor.slipPatch(1));
    
    if er.control == 1
        bestStrats(i) = forStrats(i);
    else
        bestStrats(i) = backStrats(i) + 2;
    end
end

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

a = [1:numSims]';
b = num2str(a);
c = cellstr(b);
text(xdiffs + rand(numSims, 1) * 0.02, xdots + rand(numSims, 1) * 0.02, c);
%text(xdiffs + 0.01, xdots + 0.01, c);