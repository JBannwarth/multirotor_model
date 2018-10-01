%CHECKATTITUDECONTROL Send attitude step commands to system and record perf
%   Written by: J.X.J. Bannwarth, 2017/08/21
clearvars; close all; clc;

%% Load model
model = 'MultirotorSimPx4';
dragModel = 'DragModelAIAAv3';
load_system(model);

%% Initialization
project = simulinkproject;
projectRoot = project.RootFolder;
Simulation.TS_MAX = 0.01;
load( 'AeroBothAIAA_3.mat' )
InitializeModel
Px4Bus;
selfBus;
mpc_self;

%% Set up model
% Set switches and comment out unneeded blocks
UseWindProfile( model, false );
UseEstimators( model, false );
UsePositionController( model, false );

% Block parameters
set_param( [model '/Fixed wind input'], 'value', '[0 0 0]' );
set_param( [model '/Drag model'], 'ModelName', dragModel );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelAIAAv3' );

%% Prepare I/O
% Get list of files to plot
prefix = 'step_att_small';
inputFiles = dir('data_validation');
inputFiles = {inputFiles.name};
toRemove = zeros( size(inputFiles) );
for i = 1:length(inputFiles)
    if ~strncmp( inputFiles{i}, prefix, length(prefix) )
        toRemove(i) = 1;
    end
end
inputFiles( logical(toRemove) ) = [];

% Create folder for results with date (user can rename folder later anyway
tTmp  = clock;
outputFolder = fullfile( projectRoot, 'data_results', ...
    sprintf( 'AttSim_%d-%02d-%02d_%02d-%02d-%02d', tTmp(1:5), floor(tTmp(6)) ));
mkdir( outputFolder )

%% Perform simulation(s)
for n = 1:length( inputFiles )
    clc; fprintf('Sim %d of %d...\n', n, length( inputFiles) )
    
    % Get and process input
    load( inputFiles{n} )
    PrepareAttitudeStepDataSingleAxis;
    
    % Set all simulation parameters
    Simulation.T_END = AttInput.qDes(end,1);
    load( 'AeroBothAIAA_3.mat' )
    InitializeModel
    Uav.PITCH_ONLY = 1;
    Simulation.T_START_STEP = AttInput.tDesOffset;
    LoadPx4Parameters( model, params )
    set_param( [model '/Sensor Model/attitude_estimator_q'], ...
        'ATT_EXT_HDG_M', '2')
    set_param( [model '/Sensor Model/attitude_estimator_q'], ...
        'INIT_Q', [ '[' num2str( Initial.Q' ) ']' ] )
    
    % Run simulation
    output = sim( model, 'SimulationMode', 'normal');
    
    % Add custom data to log
    userData.Simulation = Simulation;
    userData.Uav = Uav;
    userData.Motor = Motor;
    userData.Aero = Aero;
    userData.Initial = Initial;
    userData.Blocks.MotorModel = get_param( [model '/Motor model'], 'ModelName' );
    userData.Blocks.DragModel  = get_param( [model '/Drag model'], 'ModelName' );
    userData.InputCtrlFilename = inputFiles{n};
    userData.Params = params;
    userData.FlogOriginal = flogOriginal;
    userData.AttInput = AttInput;
    output = output.setUserData( userData );
    
    % Save to file
    save( fullfile( outputFolder, [ inputFiles{n}(1:end-4) 'Sim.mat' ] ), 'output' );
end

% CheckAttitudeControlPlot;

% tableOut = '';
% for i = 1:length( tableStrs )
%     tableOut = sprintf( '%s%s\n', tableOut, tableStrs{i});
% end
% 
% disp(tableOut)