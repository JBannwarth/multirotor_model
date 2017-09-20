%RUNPOSITIONHOLDSIM Run simulation for position hold case
%   Written by:    J.X.J. Bannwarth, 2017
%   Last Modified: J.X.J. Bannwarth, 2017/09/19
close all; clc;
clear all; %#ok<CLALL>

%% 1) Load input data
useClosedContraptionData = false;

if ( useClosedContraptionData )
    windFiles = { 'p5_z150_tfwt15', 'p5_z150_tfwt20', 'p5_z150_tfwt25', ...
                  'p5_z150_tfwt30', 'p5_z150_tfwt35', 'p5_z150_tfwt40' };
else
    windFiles = { '170223_grid_30', '170223_grid_35', ...
                  '170223_grid_40', '170223_grid_45', '170223_grid_50' };
    Uav.M = 1.5; % UAV was a bit heavier at the time this data was taken
end

suffix = 'NoDrops';
inFolder  = 'data_wind';


%% 2) Load model
model = 'MultirotorSimPx4SeparateRotors';
load_system(model);

%% 3) Set simulation parameters
load( 'DefaultPx4Params.mat' )
Params = flog.params; clearvars( 'flog' );
LoadPx4Parameters( model, Params )
UseWindProfile( model, true );
UseEstimators( model, true );
UsePositionController( model, true );

%% 4) Select submodels
set_param( [model '/Drag model'], 'ModelName', 'DragModelNew' );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelVariable' );

%% 5) Loop over the number of iterations
load( [ inFolder '/' windFiles{1} suffix '.mat' ] )
Simulation.T_END = ( 2 * windInput.Time(end) ) - windInput.Time(1);
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
    output(1) = output(1).setUserData( userData );
    index = 1;
else
    index = 0;
end

for i = 1:length( windFiles )
    fprintf( 'Simulating %d/%d\n', i, length(windFiles) );
    
    % Set-up wind profile
    load( [ inFolder '/' windFiles{i} suffix '.mat' ] )
    timeTmp = [windInput.time; windInput.time + windInput.time(end)];
    timeTmp = timeTmp - timeTmp(1);
    dataTmp = [windInput.Data; windInput.Data];
    windInput = timeseries( dataTmp, timeTmp );
    
    % Set simulation parameters
    UseWindProfile( model, true );
    set_param( [model '/Varying wind input'], 'VariableName', 'windInput' );
    output(index + i) = sim( model, 'SimulationMode', 'normal' );
    
    % Add custom data
    userData.ClosedContraption = useClosedContraptionData;
    userData.Simulation = Simulation;
    userData.Uav = Uav;
    userData.Initial = Initial;
    userData.Blocks.MotorModel = get_param( [model '/Motor model'], 'ModelName' );
    userData.Blocks.DragModel = get_param( [model '/Drag model'], 'ModelName' );
    userData.Params = Params;    
    output(index + i) = output(index + i).setUserData( userData );
end

%% 6) Save data
if ( useClosedContraptionData )
    fileName = 'PosHoldClosedContraption';
else
    fileName = 'PosHoldOpenContraption';
end
save( ['data_results/' fileName num2str(Uav.M) 'NoRotDrag' '.mat'], 'output' ) 

%% 7) Plot data
% PlotPositionHoldError;

%% 8) Clean up
% close_system(model, 0);