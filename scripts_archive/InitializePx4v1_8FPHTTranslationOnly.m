%INITIALIZEPX4V1_8FPHTTRANSLATIONONLY Initialise simplified FPHT controller
%   Written by:    J.X.J. Bannwarth, 2019/07/09
%	Last modified: J.X.J. Bannwarth, 2019/07/09

project = simulinkproject; projectRoot = project.RootFolder;

%% Simulation parameters
% Model
model = 'MultirotorSimPx4v1_8FPHTTranslationOnly';
if ~exist( 'tEnd', 'var')
    tEnd = 20;
end

%% Load parameters
load_system( model )

% Get wind data and aero parameters
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );
load( fullfile( projectRoot, 'data_wind', 'TurbSimOC', 'turbsim_35_01' ) );

% Coment out what might cause issues
set_param( [model '/Sinusoidal input'], 'Commented', 'on' )
set_param( [model '/Varying wind input'], 'Commented', 'on' )
set_param( [model '/Step wind input'], 'Commented', 'on' )

% Load UAV parameters
[Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( true );
Simulation = InitializeModel( model, Initial, tEnd );
Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;
Ctrl.BYPASS_ROTATION = 1;

%% Find operating point
toLoad = { 'posOnly', 'velD' };
FindOpPx4

Ctrl.BYPASS_ROTATION = 0;