%MASSE_INITIALIZEHINF Initialize Masse et al.'s H_inf controller
%   Controller structure from:
%   Masse, Catherine, Gougeon, Olivier, Nguyen, Duc-Tien and Saussie, David.
%   �Modeling and Control of a Quadcopter Flying in a Wind Field: A
%   Comparison Between LQR and Structured Hinf Control Techniques.� 2018
%   International Conference on Unmanned Aircraft Systems (ICUAS) (2018):
%   1408-1417.
%   Written by:    J.X.J. Bannwarth, 2018/11/26
%	Last modified: J.X.J. Bannwarth, 2019/01/14

clearvars
project = simulinkproject; projectRoot = project.RootFolder;

%% Configuration
uX = 5;
uY = 0;
useWind = true;
layout = 'octo';

%% Trim using Euler model
model = 'MultirotorSimLin';
load_system( model )

% Get wind data and aero parameters
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );
load( fullfile( projectRoot, 'data_wind', 'blwt', 'turbsim_35_01' ) );

switch layout 
    case 'octo' 
        [Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( true );
    case 'quad'
        [Uav, Motor, Aero, Initial] = InitializeParametersQuadcopter( );
    otherwise
        [Uav, Motor, Aero, Initial] = InitializeParametersQuadcopter( );
end
Simulation = InitializeModel( model, Initial );

if useWind
    UMean = mean( windInput );
    uX = UMean(1);
    uY = 0;
end

% Select submodels
set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelAIAAv3' );

% Set trimming parameters
opspec = operspec( model );
% States:  1 - omega, 2 - eta, 3 - nuBody, 4 - xiDot, 5 - xi
% Inputs:  1 - w, 2 - u_m
% Outputs: 1 - y
opspec.States(1).Min   = zeros(1,Uav.N_ROTORS);
opspec.States(2).Known = [0 0 1];
opspec.States(3).Known = [1 1 1];
opspec.States(4).Known = [1 1 1];
opspec.States(5).Known = [1 1 1];

opspec.Inputs(1).u     = [uX uY 0];
opspec.Inputs(1).Known = [1 1 1];
opspec.Inputs(2).Min   = zeros(1,Uav.N_ROTORS);

op = findop( model, opspec );

% Set initial states
Initial.OMEGA   = op.states(1).x;
Initial.ETA     = op.states(2).x;
Initial.Q       = EulerToQuat(Initial.ETA')';
Initial.NU_BODY = op.states(3).x;
Initial.XI_DOT  = op.states(4).x;
Initial.XI      = op.states(5).x;

Initial.U_M     = op.Inputs(2).u;

Initial.Y1      = [ Initial.XI_DOT([2 1]); Initial.XI([2 1]) ];
Initial.Y2      = [ Initial.XI_DOT(3); Initial.NU_BODY; Initial.XI(3); Initial.ETA ];
Initial.Y       = [ Initial.Y1; Initial.Y2 ];

% Get linear model (not used by controller)
linsys = linearize( model, op );

%% Run on real model
model = 'Masse_MultirotorSimHinf';
load_system( model )

if useWind
    set_param( [model '/Varying wind input'], 'VariableName', 'windInput' );
    UseWindProfile( model, true );
else
    windVelString = sprintf('[%f; %f; 0]', uX, uY);
    set_param( [model '/Fixed wind input'], 'value', windVelString );
    UseWindProfile( model, false );
end

% Get new simulation structure
Simulation = InitializeModel( model, Initial, 30 );

% Cols: thrust, roll, pitch, yaw
Upsilon = [ -0.25, -0.25,  0.25, -0.25 ;
            -0.25, -0.25, -0.25,  0.25 ;
            -0.25,  0.25, -0.25, -0.25 ;
            -0.25,  0.25,  0.25,  0.25 ];

% Use PX4-style weigthing - rotors further apart from the axis of rotation
% are given a higher control command (likely because of the higher moment
% sensitivity)
c = cosd(22.5) ./ (4*cosd(22.5)+4*sind(22.5));
s = sind(22.5) ./ (4*cosd(22.5)+4*sind(22.5));
Upsilon = [ -1/8, -c,  s, -1/8 ;
            -1/8, -s,  c,  1/8 ;
            -1/8, -s, -c, -1/8 ;
            -1/8, -c, -s,  1/8 ;
            -1/8,  c, -s, -1/8 ;
            -1/8,  s, -c,  1/8 ;
            -1/8,  s,  c, -1/8 ;
            -1/8,  c,  s,  1/8 ];

% Initialize all matrices (not used)
SOF2     = [eye(4), eye(4)];
SOF2Free = [eye(4), eye(4)];

SOF1 = [ 0 0 0 0 ;
         1 0 1 0 ;
         0 1 0 1 ;
         0 0 0 0 ];
SOF1Free = SOF1;

K_p = [ 0 0 1 0 ;
        0 1 0 0 ;
        1 0 0 0 ;
        0 0 0 1 ];
K_pFree = K_p;

K_i = [ 0 0 1 0 ;
        0 1 0 0 ;
        1 0 0 0 ;
        0 0 0 1 ];
K_iFree = K_i;

% Reset
if false
    sof2Blocks = find_system( model, 'SearchDepth', '2', 'regexp', 'on', 'Name', 'SOF2_');
    sof1Blocks = find_system( model, 'SearchDepth', '2', 'regexp', 'on', 'Name', 'SOF1_');
    kpBlocks = find_system( model, 'SearchDepth', '2', 'regexp', 'on', 'Name', 'K_p_');
    kiBlocks = find_system( model, 'SearchDepth', '2', 'regexp', 'on', 'Name', 'K_i_');
    blocks = [ sof2Blocks; sof1Blocks; kpBlocks; kiBlocks ];
    for i = 1:length(blocks)
        set_param( blocks{i},  'Gain', '1' );
    end
end