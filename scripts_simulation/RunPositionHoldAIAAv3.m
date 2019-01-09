%RUNPOSITIONHOLDSIM Run simulation for position hold case
%   Written by:    J.X.J. Bannwarth, 2017
%   Last Modified: J.X.J. Bannwarth, 2018/12/10
close all; clc;
clear all; %#ok<CLALL>

%% 1) Prepare I/O
useClosedContraptionData = false;

project = simulinkproject; projectRoot = project.RootFolder;
if ( useClosedContraptionData )
    inFolder = fullfile( projectRoot, 'data_wind', 'CobraCC' );
else
    inFolder = fullfile( projectRoot, 'data_wind', 'TurbSimOC' );
    Uav.M = 1.5; % UAV was a bit heavier at the time this data was taken
end

windFiles = dir(inFolder);
windFiles = {windFiles.name};
% Only keep actual files
toRemove = [];
for i = 1:length(windFiles)
    if ~contains( windFiles{i}, '.mat' )
        toRemove(end+1) = i;
    end
end
windFiles(toRemove) = [];

% Create folder for results with date (user can rename folder later anyway
if ( useClosedContraptionData )
    fileName = 'CC';
else
    fileName = 'OC';
end

tTmp  = clock;
outputFolder = fullfile( projectRoot, 'data_results', ...
    sprintf( 'HoverSim_%d-%02d-%02d_%02d-%02d-%02d_%s', tTmp(1:5), floor(tTmp(6)), fileName ));
mkdir( outputFolder )

%% 2) Load model
model = 'MultirotorSimPx4';
load_system(model);
SwitchRotorNumber( model, 4 );

%% 3) Set simulation parameters
load( 'DefaultPx4Params.mat' )
Params = flog.params; clearvars( 'flog' );
LoadPx4Parameters( model, Params )
UseWindProfile( model, true );
UseEstimators( model, true );
UsePositionController( model, true );

%% 4) Select submodels
set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelAIAAv3' );

%% 5) Loop over the number of iterations
load( fullfile( inFolder , windFiles{1} ) )
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = windInput.Time(end);
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );
InitializeModel;
clearvars( 'windInput' )

% No wind
if ( useClosedContraptionData )
    UseWindProfile( model, false );
    set_param( [model '/Fixed wind input'], 'Value', '[0;0;0]' );
    output(1) = sim( model, 'SimulationMode', 'normal' );
    userData.ClosedContraption = useClosedContraptionData;
    userData.Simulation = Simulation;
    userData.Uav = Uav;
    userData.Initial = Initial;
    userData.Blocks.MotorModel = get_param( [model '/Motor model'], 'ModelName' );
    userData.Blocks.DragModel = get_param( [model '/Drag model'], 'ModelName' );
    userData.Params = Params;    
    output = output.setUserData( userData );
    save( fullfile( outputFolder, ['0windspeed_output.mat'] ), 'output' )
    index = 1;
else
    index = 0;
end

for i = 1:length( windFiles )
    fprintf( 'Simulating %d/%d\n', i, length(windFiles) );
    
    % Set-up wind profile
    load( fullfile( inFolder , windFiles{i} ) )
    
    % Set simulation parameters
    UseWindProfile( model, true );
    set_param( [model '/Varying wind input'], 'VariableName', 'windInput' );
    output = sim( model, 'SimulationMode', 'normal' );
    
    % Add custom data
    userData.ClosedContraption = useClosedContraptionData;
    userData.Simulation = Simulation;
    userData.Uav = Uav;
    userData.Initial = Initial;
    userData.Blocks.MotorModel = get_param( [model '/Motor model'], 'ModelName' );
    userData.Blocks.DragModel = get_param( [model '/Drag model'], 'ModelName' );
    userData.Params = Params;    
    output = output.setUserData( userData );
    
    save( fullfile( outputFolder, windFiles{i} ), 'output' )
end