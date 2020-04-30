%INITIALIZEPX4V1_8FPHTFULLGAIN Initialise FPHT controller with full gain K
%   Written by:    J.X.J. Bannwarth, 2020/04/23
%	Last modified: J.X.J. Bannwarth, 2020/04/23

project = simulinkproject; projectRoot = project.RootFolder;

%% Simulation parameters
% Model
model = 'MultirotorSimPx4v1_8FPHTFullGainMatrix';
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
if ~isfield( Simulation, 'T_END' )
    Simulation.T_END = 20;
end

%% Load parameters
load_system( model )
load( fullfile( projectRoot, 'work', 'HinfGain.mat' ) )

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
Ctrl.BYPASS_ROTATION = 1;
Ctrl.K = K; clearvars K;

%% Find operating point
toLoad = { 'attRatePID' };
FindOpPx4v1_8Cont

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