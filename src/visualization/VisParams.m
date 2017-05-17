classdef VisParams < matlab.mixin.Copyable
    properties
        camMargin = 0.5;
        drawText = false;
        dt = 0.03;
        pauseFactor = 0.5;
        
        springWidth = 0.05;
        hipDiam = 0.07;
        hipColor = [1 .5 0];
        hipAlpha = 0.5;
        
        toeDiam = 0.025;
        toeColor = [1 .5 0];
        toeAlpha = 0.5;
        
        phaseMColor = [0.2510, 0.3059, 0.4863;
                      0.3647, 0.5804, 0.4549;
                      0.7608, 0.3412, 0.3020;
                      0.9569, 0.7137, 0.2588];
        flightMColor = [0.2431, 0.5725, 0.8000];
        toeMColor = [0.3, 0.3, 0.3];
        toeMStartColor = [0 1 0];
        toeMEndColor = [0.05 0.8 1];
        markerSize = 8;
        interpolate = true;
        showPath = true;
        % Take every nth element from time-interpolate hip positions
        pathSel = 6;
        
        % Filename where to save starting pic, if at all
        picPath = 'none';
    end
end

