%INITIALIZEPX4V1_8CONT Initialise the continuous PX4 implementation
%   Written by:    J.X.J. Bannwarth, 2019/05/15
%	Last modified: J.X.J. Bannwarth, 2019/05/16

project = simulinkproject; projectRoot = project.RootFolder;

%% Simulation parameters
% Model
model = 'MultirotorSimPx4v1_8Cont';
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
if ~isfield( Simulation, 'T_END' )
    Simulation.T_END = 20;
end

%% Load parameters
load_system( model )

% Get wind data and aero parameters
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );
load( fullfile( projectRoot, 'data_wind', 'TurbSimOC', 'TurbSim_40_01' ) );

% Select submodels
set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelZJChen' );

% Coment out what might cause issues
set_param( [model '/Sinusoidal input'], 'Commented', 'on' )
set_param( [model '/Varying wind input'], 'Commented', 'on' )
set_param( [model '/Step wind input'], 'Commented', 'on' )

% Load UAV parameters
loadBuses = false;
InitializeParametersOctocopterCanted
InitializeModel
Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;

%% Find operating point
toLoad = { 'attRatePID', 'velPID' };
FindOpPx4v1_8Cont