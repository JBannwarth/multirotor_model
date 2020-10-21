%HORTHRUST_POSHOLD Run position hold experiment
%   Written by:    J.X.J. Bannwarth, 2019/05/16
%	Last modified: J.X.J. Bannwarth, 2019/06/20

%% First initialization
close all; clc; clearvars;
ctrlName = 'baseline';
project = simulinkproject; projectRoot = project.RootFolder;

%% Generate output folder
tTmp = clock;
outputFile = sprintf( 'PosHold_%s_%d-%02d-%02d_%02d-%02d-%02.f', ctrlName, tTmp(1:6));
outputFolder = fullfile( projectRoot, 'data_results', 'pos_hold' );

%% Load wind data
inFolder = fullfile( projectRoot, 'data_wind', 'TurbSimOC' );
windFiles = dir(inFolder);
windFiles = {windFiles.name};
windFiles(~endsWith(windFiles, '.mat' )) = [];

for i = 1:length(windFiles)
    load( fullfile( inFolder, windFiles{i} ) )
    windInputs{i} = windInput;
    ULin(i,:) = mean( windInputs{i} );
    ULin( ULin < 1e-3 ) = 0; % Makes trimming easier
end
clearvars windInput

%% Set up simulation
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 20; % windInputs{1}.Time(end);

%% Find OP for different wind speeds
switch ctrlName
    case 'baseline'
        InitializePx4v1_8Cont
    case 'IHT'
        InitializePx4v1_8IHT
    case 'FPHT'
        InitializePx4v1_8FPHT
    case 'FPHTSimple'
        InitializePx4v1_8FPHTTranslationOnly
    case 'MIS'
        InitializePx4v1_8MIS
end

%% Finish setting up simulation
set_param( [model '/Input choice'], 'Value', '1' )
set_param( [model '/Varying wind input'], 'Commented', 'off' )
for i = 1:length(windInputs)
    simIn(i) = Simulink.SimulationInput( model );
    simIn(i) = setVariable( simIn(i), 'op', ops(i) );
    simIn(i) = setVariable( simIn(i), 'windInput', windInputs{i} );
end

%% Run simulation
simOut = sim( simIn );

%% Save data
save( fullfile( outputFolder, outputFile ), 'simIn', 'simOut', 'ULin' )