%COMPAREFREQUENCYRESPONSE Compare freq. responses of simple and full models
%   Written by:    J.X.J. Bannwarth, 2019/07/17
%	Last modified: J.X.J. Bannwarth, 2019/07/17

%% First initialization
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;

%% Load wind data
ULin = [0 0 0];

%% Set up simulation
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 20; % windInputs{1}.Time(end);

project = simulinkproject; projectRoot = project.RootFolder;

%% Simulation parameters
% Model
model = 'MultirotorSimPx4v1_8ContAttOnly';
tEnd = 20;

%% Load parameters
load_system( model )

% Get wind data and aero parameters
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );
load( fullfile( projectRoot, 'data_wind', 'TurbSimOC', 'TurbSim_40_01' ) );

% Coment out what might cause issues
set_param( [model '/Sinusoidal input']  , 'Commented', 'on' )
set_param( [model '/Varying wind input'], 'Commented', 'on' )
set_param( [model '/Step wind input']   , 'Commented', 'on' )
set_param( [model '/Manual Switch etaDes']      , 'sw', '0' )
set_param( [model '/Manual Switch thrustDes']   , 'sw', '0' )
set_param( [model '/Manual Switch horThrustDes'], 'sw', '0' )

% Load UAV parameters
[Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( true );
Simulation = InitializeModel( model, Initial, tEnd );
Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;

%% Find operating point
toLoad = { 'attRatePID', 'fullInput' };
FindOpPx4

%% Finish setting up simulation
set_param( [model '/Manual Switch etaDes']      , 'sw', '1' )
set_param( [model '/Manual Switch thrustDes']   , 'sw', '1' )
set_param( [model '/Manual Switch horThrustDes'], 'sw', '1' )
set_param( [model '/Fixed etaDes']      , 'Value', sprintf( '[%.15e;%.15e;%.15e]', op.Input(2).u ) )
set_param( [model '/Fixed thrustDes']   , 'Value', sprintf( '%.15e', op.Input(3).u ) )
set_param( [model '/Fixed horThrustDes'], 'Value', sprintf( '[%.15e;%.15e]', op.Input(4).u ) )

%% Run frequency analysis
% set_param( [model '/Input choice'], 'Value', '2' )
% % set_param( [model '/Varying wind input'], 'Commented', 'on' )
% set_param( [model '/Fixed wind input'], 'Value', sprintf( '[%.3f; %.3f; %.3f]', ULin ) )
% 
% io = getlinio( model )
%
% input = frest.Sinestream('Frequency', logspace(-3,2,30));
% sysest = frestimate(model,op,io,input);
% size(sysest)

%io(1) = linio( [model '/Input switch'], 1, 'input' );
io(1) = linio( [model '/Manual Switch horThrustDes'], 1, 'input' );
io(2) = linio( [model '/Manual Switch thrustDes'], 1, 'input' );
io(3) = linio( [model '/Quadrotor Euler Model'], 6, 'output' );
sysFull = linearize( model, io );

% io2(1) = linio( [model '/Manual Switch horThrustDes'], 1, 'input' );
% io2(2) = linio( [model '/Manual Switch thrustDes'], 1, 'input' );
% io2(3) = linio( [model '/Quadrotor Euler Model'], 6, 'output' );
input = frest.Sinestream('Frequency',logspace(-1,2,5)); %frest.Sinestream(sys);
% 
srcblks = frest.findSources(model,io);
opts = frestimateOptions;
opts.BlocksToHoldConstant = srcblks;
set_param(model,'SignalLogging','off')
[sysest,simout] = frestimate(model,io,input,opts);
set_param(model,'SignalLogging','on')
% 
frest.simView(simout,input,sysest)
frest.simView(simout,input,sysest,sysFull)