%HORTHRUST_POSHOLD Run position hold for des. controller and wind profile(s)
%   Inputs:
%       - ctrlName : 'baseline', 'IHT', 'FPHT', 'FPHTSimple', or 'MIS'
%       - windFiles: wind profiles to run position hold for (default: all)
%       - tEnd     : how long to run simulation for (default: until end of
%                    wind file)
%       - saveData : whether to save the results to a file
%   Output:
%       - simIn    : simulation inputs
%       - simOut   : simulation outputs
%       - ULin     : mean wind speeds for each input
%
%   Note: cannot be written as a function because operspec always reads
%         from the base workspace
%
%   Written by:    J.X.J. Bannwarth, 2019/05/16
%   Last modified: J.X.J. Bannwarth, 2020/12/09

%% Input processing
if ~exist( 'ctrlName', 'var' )
    ctrlName = 'baseline';
end

if ~exist( 'windFiles', 'var' )
    windFiles = [];
end

if ~exist( 'tEnd', 'var' )
    tEnd = -1;
end

if ~exist( 'saveData', 'var' )
    saveData = true;
end

% Check the controller exists
controllers = {'baseline', 'IHT', 'FPHT', 'FPHTSimple', 'MIS'};
if ~any( strcmp( controllers, ctrlName ) )
    error( 'Unknown controller name, make sure it is in the list' )
end

% Convert char inputs to 1x1 cell arrays to simplify loading code
if ischar( windFiles )
    windFiles = { windFiles };
end

%% Generate output folder
% Reference project root to run the same regardless of the current dir
project = simulinkproject; projectRoot = project.RootFolder;

if saveData
    tTmp = clock;
    outputFile = sprintf( 'PosHold_%s_%d-%02d-%02d_%02d-%02d-%02.f', ...
        ctrlName, tTmp(1:6));
    outputFolder = fullfile( projectRoot, 'data_results', 'pos_hold' );
end

%% Load wind data
inFolder = fullfile( projectRoot, 'data_wind', 'TurbSimOC' );
if isempty( windFiles )
    windFiles = dir(inFolder);
    windFiles = {windFiles.name};
    windFiles(~endsWith(windFiles, '.mat' )) = [];
end

windInputs = cell( length(windFiles), 1 );
ULin = zeros( length(windFiles), 3 );
for ii = 1:length(windFiles)
    load( fullfile( inFolder, windFiles{ii} ), 'windInput' )
    windInputs{ii} = windInput;
    ULin(ii,:) = mean( windInputs{ii} );
    ULin( ULin < 1e-3 ) = 0; % Makes trimming easier
end
clearvars windInput

%% Set up simulation
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
if isempty( tEnd ) || ( tEnd < 0 )
    Simulation.T_END = 20;
else
    if tEnd > windInputs{1}.Time(end)
        warning( [ 'tEnd larger than length of wind profile, setting' ...
            ' it to maximum allowable value' ] )
    end
    Simulation.T_END = windInputs{1}.Time(end);
end

%% Find OP for different wind speeds
switch ctrlName
    case 'baseline'
        InitializePx4v1_8Cont
    case 'IHT'
        InitializePx4v1_8IHT
    case 'FPHT'
        InitializePx4v1_8FPHT
    case 'FPHTFullGain'
        InitializePx4v1_8FPHTFullGain
    case 'FPHTSimple'
        InitializePx4v1_8FPHTTranslationOnly
    case 'MIS'
        InitializePx4v1_8MIS
end

%% Finish setting up simulation
set_param( [model '/Input choice'], 'Value', '1' )
set_param( [model '/Varying wind input'], 'Commented', 'off' )
for ii = 1:length(windInputs)
    simIn(ii) = Simulink.SimulationInput( model );
    simIn(ii) = setVariable( simIn(ii), 'op', ops(ii) );
    simIn(ii) = setVariable( simIn(ii), 'windInput', windInputs{ii} );
    simIn(ii) = setVariable( simIn(ii), 'windFile', windFiles{ii} );
    fprintf( '[Simulation %02d/%02d] Controller: % 10s, wind: % 20s\n', ...
        ii, length(windInputs), ctrlName, windFiles{ii} );
end

%% Run simulation
simOut = sim( simIn );

%% Save data
if saveData
    if ~exist( outputFolder, 'dir' )
        mkdir( outputFolder )
    end
    save( fullfile( outputFolder, outputFile ), ...
        'simIn', 'simOut', 'ULin' )
end