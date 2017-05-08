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
        
        phaseColor = [0.2510, 0.3059, 0.4863;
                      0.3647, 0.5804, 0.4549;
                      0.7608, 0.3412, 0.3020];
        flightColor = [0.2431, 0.5725, 0.8000];
        toeColor = [0.3, 0.3, 0.3];
        toeStartColor = [0 1 0];
        toeEndColor = [0.05 0.8 1];
        markerSize = 8;
        interpolate = true;
        showPath = true;
        % Take every nth element from time-interpolate hip positions
        pathSel = 6;
        
        % Filename where to save starting pic, if at all
        picPath = 'none';
    end
end

