%RUNPARAMETERVARIATIONS Run simulation for position hold case with varied parameters.
%
%   Inputs (created in workspace):
%       - uavType: 'quad_x' (default) or 'octa_x', type of UAV to simulate.
%       - canted : false (default) or true, whether the rotors are canted.
%
%   Using workspace variables as inputs is not elegant, but Simulink models
%   require variables to be present in the base workspace or in the model
%   workspace. Both of those solutions require code that is not elegant
%   either.
%
%   Written: 2017, J.X.J. Bannwarth

%% Set-up
clearvars -except canted testCase toVary

uavType = 'octa_x';

if ~exist( 'canted', 'var' )
    canted = 31;
end

if ~exist( 'testCase', 'var' )
    testCase = 'baseline';
end

if ~exist( 'toVary', 'var' )
    toVary = 'mass';
end

%% 1) Prepare I/O
% Get project root directory, so that the script can be run regardless of
% what the current directory is
project = simulinkproject; projectRoot = project.RootFolder;

% Create folder for results with date (user can rename folder later anyway)
tStr  = datetime( clock, 'format', 'yyyy-MM-dd_HH-mm-ss' );
outputFolder = fullfile( projectRoot, 'data_results', 'poshold_variation' );
outputFile = sprintf( '%s_%s_%s_%s', testCase, uavType, toVary, tStr );

%% 2) Load wind
% Load wind profiles
inFolder = fullfile( projectRoot, 'data_wind', 'blwt' );
windFiles = dir( fullfile( inFolder, '*.mat' ) );
windFiles = {windFiles.name}';

for ii = 1:length( windFiles )
    load( fullfile( inFolder, windFiles{ii} ), 'windInput' )
    windInputs{ii,1} = windInput;
end

% Find end time
tEnd = 300;

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
if strcmp ( testCase, 'FPHT' )
    model = 'MultirotorSimPx4HorThrust';
else
    model = 'MultirotorSimPx4';
end
load_system(model);

% Select submodules
set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelZJChen' );

% Set mask parameters
SwitchAirframeConfiguration( model, uavType );

UseWindProfile( model, true );
UseEstimators( model, true );
UsePositionController( model, true );

%% 4) Load UAV parameters
[Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( canted );
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

%% 5) Trim the system - not for this system
% Usually you would trim the simulation at this point, but the discrete
% states of the PX4 controllers make it hard to do so.
% Instead, it is much easier to simply simulate for a longer period and
% cut-off the part where the controller is recovering from unbalanced
% initial conditions

% Deactivate initial states
set_param( model, 'LoadInitialState', 'off' );
set_param( model, 'LoadExternalInput', 'off' );

%% 6) Parameter variations
mass = cellfun( @(x)(x*1.72), { 0.7; 1.0; 1.3 }, 'UniformOutput', false );
cant = cellfun( @(x)(x*31), {0.8; 1.0; 1.2}, 'UniformOutput', false );
kScale = {0.8; 1.0; 1.2};
param = struct( 'mass', mass, 'cant', cant, 'kScale', kScale );

%% 7) Run simulation
% Set-up the simulation inputs
idx = 0;
for ii = 1:3
    for jj = 1:length( windInputs )
        idx = idx+1;
        simIn(idx) = Simulink.SimulationInput( model );
        % Set the wind profile to use
        simIn(idx) = simIn(idx).setVariable( 'windInput', windInputs{jj} );

        paramLog.kScale = 1;
        switch toVary
            case 'canted'
                [Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( param(ii).cant );
            case 'mass'
                [Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( canted, param(ii).mass );
            case 'kScale'
                [Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( canted, -1, param(ii).kScale );
                paramLog.kScale = param(ii).kScale;
            otherwise
                error( 'Not recognised' )
        end
        Simulation = InitializeModel( model, Initial, tEnd );
        
        paramLog.mass = Uav.M;
        paramLog.cant = rad2deg(Uav.ZETA);

        % Also add simulation parameters to the input for logging purposes
        simIn(idx) = simIn(idx).setVariable( 'uavType', uavType );
        simIn(idx) = simIn(idx).setVariable( 'windFile', windFiles{jj} );
        simIn(idx) = simIn(idx).setVariable( 'Simulation', Simulation );
        simIn(idx) = simIn(idx).setVariable( 'Uav', Uav );
        simIn(idx) = simIn(idx).setVariable( 'Motor', Motor );
        simIn(idx) = simIn(idx).setVariable( 'Initial', Initial );
        simIn(idx) = simIn(idx).setVariable( 'Aero', Aero );
        simIn(idx) = simIn(idx).setVariable( 'param', paramLog );
    end
end

% Display simulation settings
disp( 'Simulation runs:' )
for ii = 1:length( simIn )
    windMean = mean( simIn(ii).getVariable('windInput') );
    fprintf( '\t[%02d/%02d] wind file: % 18s, m=%5.2f kg, cant=%3.1f deg, kScale = %3.1f (-), Umean = %5.2f m/s\n', ii, ...
        length( simIn ), simIn(ii).getVariable('windFile'), ...
        simIn(ii).getVariable('param').mass, simIn(ii).getVariable('param').cant, ...
        simIn(ii).getVariable('param').kScale, windMean(1) );
end

simOut = sim( simIn );

if ~isfolder( outputFolder )
    mkdir( outputFolder )
end
save( fullfile( outputFolder, outputFile ), 'simIn', 'simOut' )