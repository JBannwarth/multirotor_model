close all; clc;
clear all; %#ok<CLALL>

%% 1) Load model and initialize
TestControllerInitOnly;


model = 'MultirotorSimPx4SeparateRotors';
load_system(model);

useClosedContraptionData = true;

% if ( useClosedContraptionData )
%     Simulation.T_END = 60;
% else
%     Simulation.T_END = 120;
% end

%% 2) Set up the iterations that we want to compute
if ( useClosedContraptionData )
    windFiles = { 'p5_z150_tfwt15', 'p5_z150_tfwt20', 'p5_z150_tfwt25', ...
                  'p5_z150_tfwt30', 'p5_z150_tfwt35', 'p5_z150_tfwt40' };
else
    windFiles = { '170223_grid_30', '170223_grid_35', ...
                  '170223_grid_40', '170223_grid_45', '170223_grid_50' };
end

suffix = 'NoDrops'; % 'NoDrops'

inFolder  = 'inputdata';

%% 3) Loop over the number of iterations

load( [ inFolder '/' windFiles{1} suffix '.mat' ] )
Simulation.T_END = ( 2 * windInput.Time(end) ) - windInput.Time(1);
clear windInput

% No wind
if ( useClosedContraptionData )
    set_param( [model '/Varying wind input'], 'commented', 'on' );
    set_param( [model '/Wind switch'], 'sw', '0' );
    set_param( [model '/Fixed wind input'], 'Value', '[0;0;0]' );
    output(1) = sim( model, 'SimulationMode', 'normal' );
    index = 1;
else
    index = 0;
end

for i = 1:length( windFiles )
    fprintf( 'Simulating %d/%d\n', i, length(windFiles) );
    load( [ inFolder '/' windFiles{i} suffix '.mat' ] )
    timeTmp = [windInput.time; windInput.time + windInput.time(end)];
    timeTmp = timeTmp - timeTmp(1);
    
    dataTmp = [windInput.Data; windInput.Data];
%    dataTmp(:,1) = 4 * (dataTmp(:,1) - mean( dataTmp(:,1) ) ) + mean( dataTmp(:,1) );
%     dataTmp(:,3) = 6 * (dataTmp(:,3) - mean( dataTmp(:,3) ) ) + mean( dataTmp(:,3) );
    windInput = timeseries( dataTmp, timeTmp );
    set_param( [model '/Varying wind input'], 'commented', 'off' );
    set_param( [model '/Wind switch'], 'sw', '1' );
    set_param( [model '/Varying wind input'], 'VariableName', 'windInput' );
    output(index + i) = sim( model, 'SimulationMode', 'normal' );
end

%% 4) Plot data
PlotPositionHoldError;

%% 5) Clean up
% close_system(model, 0);