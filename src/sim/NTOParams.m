classdef NTOParams < handle
    properties
        % Order of phases (sli=slip, stl=stick left side of patch, str =
        % stick right side of patch
        phases
        % The region of slippery terrain
        slipPatch
        % The initial state of the SLIP over the ice
        initialState
        % The final x position of the toe and the hip (if NaN ignore)
        finalProfileX

        gridn = 10;             % Number of grid points during stance phase
        masship = 1;           % Mass of body in kilograms
        masstoe = 0.1;         % Mass of the toe in kilograms
        spring = 20;           % Spring coefficient
        damp = 0.5;            % Damping coefficient
        gravity = 1;           % Gravity
        % Friction coefficient between toe and ground on slippery surfaces
        friction = 0.05;
        
        minStanceTime = 0.1;   % Minimum stance phase time
        maxStanceTime = 1000;  % Maximum stance phase time
        minFlightTime = 0;     % Minimum flight phase time
        maxFlightTime = 1000;  % Maximum flight phase time
        minraddot = -1;        % Minimum second deriv of actuated length
        maxraddot = 1;         % Maximum second deriv of actuated length
        mintorque = -1;        % Minimum torque at hip
        maxtorque = 1;         % Maximum torque at hip
        minlen = 0.5;          % Minimum length of the leg
        maxlen = 1;            % Maximum length of the leg
        
        mingrf = 0;            % Minimum ground reaction force
        maxgrf = 1000;         % Maximum ground reaction force
        
        sqrtSmooth = 0.001;    % Smoothing factor for the work function
        tanhSmooth = 50;       % Smoothing factor for the dynamics
    end
    
    methods
        function obj = NTOParams(phases, slipPatch, initialState, finProfX)
            if nargin == 0
                phases = [];
                slipPatch = [];
                initialState = [];
                finProfX = 0;
            end
            obj.phases = phases;
            obj.slipPatch = slipPatch;
            obj.initialState = initialState;
            obj.finalProfileX = finProfX;
        end
        
        function iss = iss(obj)
            iss = struct();
            iss.xtoe = obj.initialState(1);
            iss.xtoedot = obj.initialState(2);
            iss.x = obj.initialState(3);
            iss.xdot = obj.initialState(4);
            iss.y = obj.initialState(5);
            iss.ydot = obj.initialState(6);
            iss.ra = obj.initialState(7);
            iss.radot = obj.initialState(8);
        end
    end
end

