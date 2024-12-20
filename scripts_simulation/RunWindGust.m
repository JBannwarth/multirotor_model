%RUNWINDGUST Run simulation for position hold case.
%
%   Inputs (created in workspace):
%       - uavType: 'quad_x' (default) or 'octa_x', type of UAV to simulate.
%       - canted : false (default) or true, whether the rotors are canted.
%
%   Simulate the response of the UAV subjected to a wind gust
%   Start from the linearisation point of the UAV and try a range of wind
%   gusts. Use steady wind
%
%   Written: 2017, J.X.J. Bannwarth

%% Set-up
clearvars -except uavType canted testCase;

if ~exist( 'uavType', 'var' )
    uavType = 'octa_x';
end

if ~exist( 'canted', 'var' )
    canted = 31;
end

if ~exist( 'testCase', 'var' )
    testCase = 'baseline';
end

%% 1) Prepare I/O
% Get project root directory, so that the script can be run regardless of
% what the current directory is
project = simulinkproject; projectRoot = project.RootFolder;

% Create folder for results with date (user can rename folder later anyway)
tStr  = datetime( clock, 'format', 'yyyy-MM-dd_HH-mm-ss' );
outputFolder = fullfile( projectRoot, 'data_results', 'gust_full' );
outputFile = sprintf( '%s_%s_%s', testCase, uavType, tStr );

%% 2) Load model and set mask parameters
% Load UAV model
if strcmp ( testCase, 'FPHT' )
    model = 'MultirotorSimPx4HorThrust';
else
    model = 'MultirotorSimPx4';
end
load_system(model);

% Select submodules
set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
if strcmp( uavType, 'quad_x' )
    set_param( [model '/Motor model'], 'ModelName', 'MotorModelAIAAv3' );
else
    set_param( [model '/Motor model'], 'ModelName', 'MotorModelZJChen' );
end

% Set mask parameters
SwitchAirframeConfiguration( model, uavType );

UseWindProfile( model, true );
UseEstimators( model, true );
UsePositionController( model, true );

%% 3) Load UAV parameters
% Step time
tStart = 100;
tEnd = 200;

if strcmp( uavType, 'quad_x' )
    [Uav, Motor, Aero, Initial] = InitializeParametersQuadcopter( );
elseif strcmp( uavType, 'octa_x' )
    [Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( canted );
end
Simulation = InitializeModel( model, Initial, tEnd );

if strcmp( testCase, 'FPHT' )
    load( fullfile( projectRoot, 'work', 'HinfGain.mat' ), 'K', 'thrustOp' )
    dt = 1/str2double(get_param( [ model '/mc_pos_control/'],  'loop_update_rate_hz' ));
    Kd = c2d( K, dt, 'Tustin' );
    Ctrl.A = Kd.A;
    Ctrl.B = Kd.B;
    Ctrl.C = Kd.C;
    Ctrl.D = Kd.D;
end

%% 4) Prepare wind
% Load linearised model to get linearisation point
load( fullfile( projectRoot, 'work', 'Octocopter_LinMod_Att.mat' ), 'ULin' )
stepSizes = [0.1 0.5 1.0 2.0 5.0];

% Create wind steps
windInputs = cell( length( stepSizes ), 1 );
for ii = 1:length( stepSizes )
    wind = repmat( ULin, 4, 1 );
    wind(3:4,1) = wind(3:4,1) + stepSizes(ii);
    windInputs{ii} = timeseries( wind, [0, tStart-Simulation.TS_OUT, tStart, tEnd] );
end

%% 5) Trim the system - not for this system
% Usually you would trim the simulation at this point, but the discrete
% states of the PX4 controllers make it hard to do so.
% Instead, it is much easier to simply simulate for a longer period and
% cut-off the part where the controller is recovering from unbalanced
% initial conditions

% Deactivate initial states
set_param( model, 'LoadInitialState', 'off' );
set_param( model, 'LoadExternalInput', 'off' );

%% 7) Run simulation
% Set-up the simulation inputs
for ii = 1:length( windInputs )
    simIn(ii) = Simulink.SimulationInput( model );
    % Set the wind profile to use
    simIn(ii) = simIn(ii).setVariable( 'windInput', windInputs{ii} );
    
    % Also add simulation parameters to the input for logging purposes
    simIn(ii) = simIn(ii).setVariable( 'uavType', uavType );
    simIn(ii) = simIn(ii).setVariable( 'stepSize', stepSizes(ii) );
    simIn(ii) = simIn(ii).setVariable( 'Simulation', Simulation );
    simIn(ii) = simIn(ii).setVariable( 'Uav', Uav );
    simIn(ii) = simIn(ii).setVariable( 'Motor', Motor );
    simIn(ii) = simIn(ii).setVariable( 'Initial', Initial );
end

% Display simulation settings
disp( 'Simulation runs:' )
for ii = 1:length( simIn )
    fprintf( '\t[%d/%d] step size: % .2f, Umean = %5.2f m/s\n', ii, ...
        length( simIn ), simIn(ii).getVariable('stepSize'), ULin(1) );
end

simOut = sim( simIn );

if ~isfolder( outputFolder )
    mkdir( outputFolder )
end
save( fullfile( outputFolder, outputFile ), 'simIn', 'simOut' )