%INITIALIZEPX4V1_8FPHT Initialise full position horizontal thrust controller
%   Written by:    J.X.J. Bannwarth, 2019/05/16
%	Last modified: J.X.J. Bannwarth, 2019/09/02

project = simulinkproject; projectRoot = project.RootFolder;

%% Simulation parameters
% Model
model = 'MultirotorSimPx4v1_8FPHT';
if ~exist( 'tEnd', 'var')
    tEnd = 20;
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
[Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( true );
Simulation = InitializeModel( model, Initial, tEnd );
Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;
Ctrl.BYPASS_ROTATION = 1;

%% Find operating point
toLoad = { 'attRatePID', 'velD' };
FindOpPx4

%% Load real model
% modelLin = model;
% model = 'MultirotorSimPx4v1_8FPHT';
% load_system( model )
% 
% % Coment out what might cause issues
% set_param( [model '/Sinusoidal input'], 'Commented', 'on' )
% set_param( [model '/Varying wind input'], 'Commented', 'on' )
% set_param( [model '/Step wind input'], 'Commented', 'on' )
% 
% % Adjust operating point
% for i = 1:length(ops)
%     ind = ops(i).getInputIndex([modelLin '/thrustDes']);
%     Ctrl.THROTTLE_HOVER = ops(i).Inputs(ind(1)).u;
%     ops(i).Inputs(1:4) = []; 
%     ops(i).Inputs(1).Block = strrep( ops(i).Inputs(1).Block, modelLin, model );
%     for j = 1:length(ops(i).States)
%         ops(i).States(j).Block = strrep( ops(i).States(j).Block, modelLin, model );
%     end
% end
% op = ops(1);
% 
% % Select submodels
% set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
% set_param( [model '/Motor model'], 'ModelName', 'MotorModelZJChen' );
% 
% % Set initial states
% set_param( model, 'LoadInitialState', 'on' );
% set_param( model, 'InitialState', 'getstatestruct(op)' );
% 
% % Set controller lin point
% ind = op.getStateIndex('eta');
% Ctrl.ETA_LIN = op.States(ind(1)).x;

Ctrl.BYPASS_ROTATION = 0;