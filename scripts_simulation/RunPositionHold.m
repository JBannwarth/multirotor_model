%RUNPOSITIONHOLD Run simulation for position hold case
%   Written: 2017, J.X.J. Bannwarth

%% Set-up
clearvars -except uavType canted;

if ~exist( 'uavType', 'var' )
    uavType = 'quad_x';
end

if ~exist( 'canted', 'var' )
    canted = false;
end

%% 1) Prepare I/O
% Get project root directory, so that the script can be run regardless of
% what the current directory is
project = simulinkproject; projectRoot = project.RootFolder;

% Create folder for results with date (user can rename folder later anyway)
tStr  = datetime( clock, 'format', 'yyyy-MM-dd_HH-mm-ss' );
outputFolder = fullfile( projectRoot, 'data_results', 'poshold_full' );
outputFile = sprintf( '%s_%s', uavType, tStr );

%% 2) Load wind
% Load wind profiles
inFolder = fullfile( projectRoot, 'data_wind', 'blwt' );
windFiles = dir( fullfile( inFolder, '*.mat' ) );
windFiles = {windFiles.name}';

for ii = 1:length( windFiles )
    load( fullfile( inFolder, windFiles{ii} ), 'windInput' )
    windInputs{ii,1} = windInput;
end
clearvars windInput;

% Find end time
tEnd = floor( windInputs{1}.Time(end) ); % End on a round number

% Insert zero wind speed at the beginning
windInputs(2:end+1) = windInputs;
windInputs{1} = timeseries( zeros(2,3), [0, tEnd] );
windFiles = [ 'zero'; windFiles ];

% Get mean wind speeds
UMean = zeros( length( windFiles ), 3 );
for ii = 1:length(windFiles)
    UMean(ii,:) = mean( windInputs{ii} );
end

% Zero small elements to make trimming easier
UMean( UMean < 1e-3 ) = 0;

%% 3) Load model and set mask parameters
% Load UAV model
model = 'MultirotorSimPx4';
load_system(model);

% Select submodules
set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
if strcmp( uavType, 'quat_x' )
    set_param( [model '/Motor model'], 'ModelName', 'MotorModelZJChen' );
else
    set_param( [model '/Motor model'], 'ModelName', 'MotorModelAIAAv3' );
end

% Set mask parameters
SwitchAirframeConfiguration( model, uavType );

load( 'DefaultPx4Params.mat' )
Params = flog.params; clearvars( 'flog' );
LoadPx4Parameters( model, Params )

UseWindProfile( model, true );
UseEstimators( model, true );
UsePositionController( model, true );

%% 4) Load UAV parameters
if strcmp( uavType, 'quad_x' )
    [Uav, Motor, Aero, Initial] = InitializeParametersQuadcopter( );
elseif strcmp( uavType, 'octa_x' )
    [Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( canted );
end
Simulation = InitializeModel( model, Initial, tEnd, true );

%% 5) Trim the system
% Find equilibrium point to start-off the simulation

% Deactivate initial states as we want to find them
set_param( model, 'LoadInitialState', 'off' );
set_param( model, 'LoadExternalInput', 'off' );

% Select inport
UseWindProfile( model, false );
set_param( [model '/Input choice'], 'Value', '2' );
set_param( [model '/Fixed wind input'], 'Value', '[UIn 0 0]' );

UIn = UMean(2,1);
param.Name = 'UIn';
param.Value = UMean(:,1);
ops = findop( model, 50, param );
op = ops(1);

set_param( model, 'LoadInitialState', 'on' );
set_param( model, 'InitialState', 'getstatestruct(op)' );
set_param( [model '/Input choice'], 'Value', '1' );

%% 7) Run simulation
UseWindProfile( model, true );

% Set-up the simulation inputs
for ii = 1:length( windInputs )
    simIn(ii) = Simulink.SimulationInput( model );
    % Set the wind profile to use
    simIn(ii) = simIn(ii).setVariable( 'windInput', windInputs{ii} );
    
    % Set operating point
    simIn(ii) = simIn(ii).setVariable( 'op', ops(ii) );
    
    % Also add simulation parameters to the input for logging purposes
    simIn(ii) = simIn(ii).setVariable( 'uavType', uavType );
    simIn(ii) = simIn(ii).setVariable( 'windFile', windFiles{ii} );
    simIn(ii) = simIn(ii).setVariable( 'Simulation', Simulation );
    simIn(ii) = simIn(ii).setVariable( 'Uav', Uav );
    simIn(ii) = simIn(ii).setVariable( 'Motor', Motor );
    simIn(ii) = simIn(ii).setVariable( 'Initial', Initial );
end

% Display simulation settings
disp( 'Simulation runs:' )
for ii = 1:length( simIn )
    windMean = mean( simIn(ii).getVariable('windInput') );
    fprintf( '\t[%d/%d] wind file: % 18s, Umean = %5.2f m/s\n', ii, ...
        length( simIn ), simIn(ii).getVariable('windFile'), windMean(1) );
end

simOut = sim( simIn );

if ~isfolder( outputFolder )
    mkdir( outputFolder )
end
save( fullfile( outputFolder, outputFile ), 'simIn', 'simOut' )