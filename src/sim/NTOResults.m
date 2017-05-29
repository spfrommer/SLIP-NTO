classdef NTOResults < handle
    properties
        paramsFor
        paramsBack
        costFor
        costBack
        % The raw optimizer outputs
        optimalFor
        optimalBack
        flagFor
        flagBack
    end
    
    methods (Access = private)
        function [r, rdot, radotCombined, raddotCombined, fs, angleDeltas, torqueCombined, ...
                workAng, workRa] = process(~, optimal, params)
            [stanceT, ~, xtoe, ~, x, xdot, y, ydot, ra, radot, raddot, torque] = ...
                unpack(optimal, params);
           phaseN = size(params.phases, 1);

           r = sqrt((x - xtoe).^2 + y.^2);
           rdot = ((x-xtoe).*(xdot)+y.*ydot)./(r);
           fs = params.spring * (ra - r) + params.damp * (radot - rdot);

           startRemInd = 1 : params.gridn : params.gridn * phaseN;
           endRemInd = params.gridn : params.gridn : params.gridn * phaseN;

           [fsA, fsB] = deal(fs);
           fsA(startRemInd) = [];
           fsB(endRemInd) = [];
           fsCombined = (fsA + fsB) .* 0.5;

           [radotA, radotB] = deal(radot);
           radotA(startRemInd) = [];
           radotB(endRemInd) = [];
           radotCombined = (radotA + radotB) .* 0.5;
           
           [raddotA, raddotB] = deal(raddot);
           raddotA(startRemInd) = [];
           raddotB(endRemInd) = [];
           raddotCombined = (raddotA + raddotB) .* 0.5;
           
           angles = atan2(y, x - xtoe);
           angleShift = [angles(1); angles(1 : end-1)];
           angleDeltas = angles - angleShift;
           angleDeltas(startRemInd) = [];
           [torqueA,torqueB] = deal(torque);
           torqueA(startRemInd) = [];
           torqueB(endRemInd) = [];
           torqueCombined = (torqueA + torqueB) .* 0.5;

           workAng = torqueCombined .* angleDeltas;
           
           workRa = fsCombined .* radotCombined;
           workRa = workRa .* kron(stanceT./params.gridn, ones(params.gridn - 1, 1));
        end
    end
    
    methods
       function [] = render( obj, vp )
           if nargin < 2
              vp = VisParams();
           end
           
           if obj.flagBack > 0
              visualize(obj.optimalBack, obj.paramsBack, vp);
           end
        
           if obj.flagFor > 0
              visualize(obj.optimalFor, obj.paramsFor, vp);
           end
       end
       
       function [] = saveRenderImage( obj, simNum, vp )
           % Saves a snapshot of the starting state
           if nargin < 3
              vp = VisParams();
           end
           vp2 = copy(vp);
           
           [pathb, pathf] = Resource.instance().getFigurePaths(simNum);
           vp.picPath = pathb;
           vp2.picPath = pathf;
           
           if obj.flagFor > 0
              visualize(obj.optimalFor, obj.paramsFor, vp);
           end
           if obj.flagBack > 0
              visualize(obj.optimalBack, obj.paramsBack, vp2);
           end
       end
       
       function [] = saveRenderVideo( obj, simNum, vp )
           % Saves a video of the trajectory
           if nargin < 3
              vp = VisParams();
           end
           vp2 = copy(vp);
           
           [pathb, pathf] = Resource.instance().getVideoPaths(simNum);
           vp.vidPath = pathb;
           vp2.vidPath = pathf;
           
           if obj.flagFor > 0
              visualize(obj.optimalFor, obj.paramsFor, vp);
           end
           if obj.flagBack > 0
              visualize(obj.optimalBack, obj.paramsBack, vp2);
           end
       end
       
       function control = getControl( obj )
           if obj.flagBack > 0 && obj.flagFor <= 0
               control = -1;
           elseif obj.flagFor > 0 && obj.flagBack <= 0
               control = 1;
           else
               % If back has greater cost, control should be one (true/forward)
               control = obj.costBack > obj.costFor;
           end
       end
       
       function af = analyzeFor( obj )
           af = struct();
           
           if isempty(obj.optimalFor)
               af.exists = false;
               return
           else
               af.exists = true;
           end
           
           [ stanceT, flightT, xtoe, xtoedot, x, xdot, y, ydot, ...
             ra, radot, raddot, torque] = unpack(obj.optimalFor, obj.paramsFor);
           af.stanceT = stanceT;
           af.flightT = flightT;
           af.xtoe = xtoe;
           af.xtoedot = xtoedot;
           af.x = x;
           af.xdot = xdot;
           af.y = y;
           af.ydot = ydot;
           af.ra = ra;
           af.radot = radot;
           af.raddot = raddot;
           af.torque = torque;
           
           [af.r, af.rdot, af.radotCombined, af.raddotCombined, af.fs, ...
             af.angleDeltas, af.torqueCombined, af.workAng, af.workRa] ...
               = obj.process(obj.optimalFor, obj.paramsFor);
       end
       
       function ab = analyzeBack( obj )
           ab = struct();
           
           if isempty(obj.optimalBack)
               ab.exists = false;
               return
           else
               ab.exists = true;
           end
           
           [ stanceT, flightT, xtoe, xtoedot, x, xdot, y, ydot, ...
             ra, radot, raddot, torque] = unpack(obj.optimalBack, obj.paramsBack);
           ab.stanceT = stanceT;
           ab.flightT = flightT;
           ab.xtoe = xtoe;
           ab.xtoedot = xtoedot;
           ab.x = x;
           ab.xdot = xdot;
           ab.y = y;
           ab.ydot = ydot;
           ab.ra = ra;
           ab.radot = radot;
           ab.raddot = raddot;
           ab.torque = torque;
           
           [ab.r, ab.rdot, ab.radotCombined, ab.raddotCombined, ab.fs, ...
             ab.angleDeltas, ab.torqueCombined, ab.workAng, ab.workRa] ...
               = obj.process(obj.optimalBack, obj.paramsBack);
       end
    end
end

