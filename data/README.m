300 simulation results with following configurations:
trapezoidal quadrature, 3-step optimization, no work + torque^2 regularization term, just end position constraint, no end velocity

function [ slipPatch, state, finX ] = slipPatchSetup(  )
    slipPatch = [0, 0.8 + rand() * 1];
    finX = slipPatch(2);
    
    params = NTOParams();
    % Generate random initial hip x with 0.3 meter margin from patch edges
    x = rand() * (slipPatch(2) - 0.6) + 0.3;
    % Random initial hip x vel must be positive
    xdot = rand() * 0.5;
    % Random toe x within 0.2 forward of hip x
    xtoe = x + rand() * 0.2;
    % Toe x vel equal to hip x vel
    xtoedot = xdot;
    
    % Y calculated assuming rtouchdown = maxlen
    y = sqrt(params.maxlen ^ 2 - (x - xtoe) ^ 2);
    % Y vel some random downward number
    ydot = -rand() * 0.4;
    
    % Actuated length at max length, and initial actuated length derivative
    % is zero
    ra = params.maxlen;
    radot = 0;
    
    state = [xtoe; xtoedot; x; xdot; y; ydot; ra; radot];
end

gridn = 10;            % Number of grid points during stance phase
masship = 1;           % Mass of body in kilograms
masstoe = 0.1;         % Mass of the toe in kilograms
spring = 20;           % Spring coefficient
damp = 0.5;            % Damping coefficient
gravity = 1;           % Gravity
% Friction coefficient between toe and ground on slippery surfaces
friction = 0.05;
        
minStanceTime = 0.1;   % Minimum stance phase time
maxStanceTime = 2;     % Maximum stance phase time
minFlightTime = 0;     % Minimum flight phase time
maxFlightTime = 10;    % Maximum flight phase time
minraddot = -1;        % Minimum second deriv of actuated length
maxraddot = 1;         % Maximum second deriv of actuated length
mintorque = -1;        % Minimum torque at hip
maxtorque = 1;         % Maximum torque at hip
minlen = 0.5;          % Minimum length of the leg
maxlen = 1;            % Maximum length of the leg
        
maxgrf = 3;            % Maximum ground reaction force