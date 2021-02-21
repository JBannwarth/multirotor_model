%COMPAREFREQUENCYRESPONSEFULL Compare freq. responses of simple and full models
%   Written by:    J.X.J. Bannwarth, 2019/07/17
%	Last modified: J.X.J. Bannwarth, 2019/07/17

%% First initialization
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;

%% Load wind data
ULin = [4 0 0];

%% Set up simulation
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 20; % windInputs{1}.Time(end);

project = simulinkproject; projectRoot = project.RootFolder;

%% Simulation parameters - full
% Model
model = 'MultirotorSimPx4v1_8ContAttOnly';
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 20;

%% Load parameters
load_system( model )

% Get wind data and aero parameters
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );
load( fullfile( projectRoot, 'data_wind', 'TurbSimOC', 'TurbSim_40_01' ) );

% Coment out what might cause issues
set_param( [model '/Sinusoidal input']  , 'Commented', 'on' )
set_param( [model '/Varying wind input'], 'Commented', 'on' )
set_param( [model '/Step wind input']   , 'Commented', 'on' )
set_param( [model '/Manual Switch etaDes']      , 'sw', '0' )
set_param( [model '/Manual Switch thrustDes']   , 'sw', '0' )
set_param( [model '/Manual Switch horThrustDes'], 'sw', '0' )

% Load UAV parameters
loadBuses = false;
InitializeParametersOctocopterCanted
InitializeModel
Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;

% Set wind speed
set_param( [model '/Input choice'], 'Value', '2' )
set_param( [model '/Fixed wind input'], 'Value', sprintf( '[%.15e;%.15e;%.15e]', ULin ) )

%% Find operating point
toLoad = { 'fullInput', 'attRatePID' };
FindOpPx4

%% Finish setting up simulation
set_param( [model '/Manual Switch etaDes']      , 'sw', '1' )
set_param( [model '/Manual Switch thrustDes']   , 'sw', '1' )
set_param( [model '/Manual Switch horThrustDes'], 'sw', '1' )
set_param( [model '/Fixed etaDes']      , 'Value', sprintf( '[%.15e;%.15e;%.15e]', op.Input(2).u ) )
set_param( [model '/Fixed thrustDes']   , 'Value', sprintf( '%.15e', op.Input(3).u ) )
set_param( [model '/Fixed horThrustDes'], 'Value', sprintf( '[%.15e;%.15e]', op.Input(4).u ) )

io(1) = linio( [model '/Input switch'], 1, 'input' );
io(2) = linio( [model '/Manual Switch horThrustDes'], 1, 'input' );
io(3) = linio( [model '/Manual Switch thrustDes'], 1, 'input' );
io(4) = linio( [model '/Quadrotor Euler Model'], 6, 'output' );
sysFull = linearize( model, io );

%% Simulation parameters - simple
model = 'MultirotorSimPlantTranslationOnly';

%% Load parameters
load_system( model )

% Coment out what might cause issues
set_param( [model '/Sinusoidal input']  , 'Commented', 'on' )
set_param( [model '/Varying wind input'], 'Commented', 'on' )
set_param( [model '/Step wind input']   , 'Commented', 'on' )
set_param( [model '/Manual Switch etaDes']      , 'sw', '0' )
set_param( [model '/Manual Switch thrustDes']   , 'sw', '0' )
set_param( [model '/Manual Switch horThrustDes'], 'sw', '0' )

% Load UAV parameters
loadBuses = false;
InitializeParametersOctocopterCanted
InitializeModel
Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;

% Set wind speed
set_param( [model '/Input choice'], 'Value', '2' )
set_param( [model '/Fixed wind input'], 'Value', sprintf( '[%.15e;%.15e;%.15e]', ULin ) )

%% Find operating point
toLoad = { 'posOnly', 'fullInput' };
FindOpPx4

%% Finish setting up simulation
set_param( [model '/Manual Switch etaDes']      , 'sw', '1' )
set_param( [model '/Manual Switch thrustDes']   , 'sw', '1' )
set_param( [model '/Manual Switch horThrustDes'], 'sw', '1' )
set_param( [model '/Fixed etaDes']      , 'Value', sprintf( '[%.15e;%.15e;%.15e]', op.Input(2).u ) )
set_param( [model '/Fixed thrustDes']   , 'Value', sprintf( '%.15e', op.Input(3).u ) )
set_param( [model '/Fixed horThrustDes'], 'Value', sprintf( '[%.15e;%.15e]', op.Input(4).u ) )

io(1) = linio( [model '/Input switch'], 1, 'input' );
io(2) = linio( [model '/Manual Switch horThrustDes'], 1, 'input' );
io(3) = linio( [model '/Manual Switch thrustDes'], 1, 'input' );
io(4) = linio( [model '/integrator_xiDot'], 1, 'output' );
sysSimple = linearize( model, io );

%% Plot results
bode( sysFull, sysSimple )
wLims = [1e-5 1e5];
[magFull,phaseFull,wFull] = bode( sysFull, num2cell(wLims) );
[magSimple,phaseSimple,wSimple] = bode( sysSimple, num2cell(wLims) );
magFull = 20*log10(magFull);
magSimple = 20*log10(magSimple);

% Plot diagonal elements
inputs = {'U_x', 'U_y', 'U_z', 'T_x', 'T_y', 'T_z' };
outputs = {'xDot', 'yDot', 'zDot' };
for col = 1:length(inputs)
    row = rem(col,3);
    if row == 0
        row = 3;
    end
    
    figure( 'Name', [ inputs{col} ' - ' outputs{row} ] );
    subplot(2,1,1);
    semilogx( wFull, reshape( magFull(row,col,:), [1 size(magFull,3)] ) );
    hold on; box on; grid on;
    semilogx( wSimple, reshape( magSimple(row,col,:), [1 size(magSimple,3)] ) );
    xlim( wLims )
    ylabel( 'Magnitude (dB)' )
    if col == 1
        legend( 'Full', 'Simple', 'Location', 'best' )
    end
    
    subplot(2,1,2);
    semilogx( wFull, reshape( phaseFull(row,col,:), [1 size(phaseFull,3)] ) );
    hold on; box on; grid on;
    semilogx( wSimple, reshape( phaseSimple(row,col,:), [1 size(phaseSimple,3)] ) );
    xlim( wLims )
    ylabel( 'Phase (deg)' )
    xlabel( 'Frequency (rad/s)' )
    FormatFigure( [5, 8], 11 )
    PrintFigure( fullfile( projectRoot, 'work', [inputs{col} '-' outputs{row}] ) )
end