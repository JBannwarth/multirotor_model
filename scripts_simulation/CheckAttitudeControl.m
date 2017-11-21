%CHECKATTITUDECONTROL Send attitude step commands to system and record perf
%   Written by: J.X.J. Bannwarth, 2017/08/21
clearvars;
close all;

%% Load model
model = 'MultirotorSimPx4SeparateRotors';
dragModel = 'DragModelMomentDragNew';
load_system(model);

%% Initialization
project = simulinkproject;
projectRoot = project.RootFolder;
Simulation.TS_MAX = 0.01;
InitializeModel
%Motor.B([1 3]) = 1.4* Motor.B([1 3]);
%Motor.B([2 4]) = 1.5* Motor.B([2 4]);
Px4Bus;
selfBus;
mpc_self;
% Uav.I(3,3) = 3 * Uav.I(3,3);

%% Set up model
UseWindProfile( model, false );
UseEstimators( model, false );
UsePositionController( model, false );
set_param( [model '/att_thrustDesSwitch'], 'sw', '1' )
set_param( [model '/Fixed wind input'], 'value', '[0 0 0]' );
set_param( [model '/Drag model'], 'ModelName', dragModel );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelNew' );

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
inputFiles = { 'step_att_small_pitch-5_7.mat' };

%% Perform simulation(s)
for n = 1:length( inputFiles )
    load( inputFiles{n} )
    PrepareAttitudeStepDataSingleAxis;
    Simulation.T_END = qDesInput(end,1);
    %Simulation.T_END = 35;
    % Initial.Q = [1 1 -1 -1]' .* qDes(1,:)';
    %Uav.M = Uav.M*1.5;
    InitializeModel
    %Motor.B([1 3]) = 1.3* Motor.B([1 3]);
    %Motor.B([2 4]) = 1.5* Motor.B([2 4]);
    LoadPx4Parameters( model, params )
    set_param( [model '/Sensor Model/attitude_estimator_q'], ...
        'ATT_EXT_HDG_M', '2')
    set_param( [model '/Sensor Model/attitude_estimator_q'], ...
        'INIT_Q', [ '[' num2str( Initial.Q' ) ']' ] )
    set_param( [model '/Drag model'], 'ModelName', dragModel );
    output = sim( model, 'SimulationMode', 'normal');
    userData.Simulation = Simulation;
    userData.Uav = Uav;
    userData.Initial = Initial;
    userData.Blocks.MotorModel = get_param( [model '/Motor model'], 'ModelName' );
    userData.Blocks.DragModel = get_param( [model '/Drag model'], 'ModelName' );
    userData.Params = params;    
    output = output.setUserData( userData );
    save( fullfile(projectRoot, 'data_results', [ inputFiles{n}(1:end-4) 'Sim.mat' ] ), 'output' );
%     set_param( [model '/Drag model'], 'ModelName', 'DragModelNew' );
%     out = sim( model, 'SimulationMode', 'normal');
    CheckAttitudeControlPlot;
end

%% Format data
% logsout = out.get('logsout');
% clear output

% Assign units manually
% unitsLookupTable = readtable('SignalUnits.csv');
% logsout = SetDatasetUnits(logsout, unitsLookupTable,   true);