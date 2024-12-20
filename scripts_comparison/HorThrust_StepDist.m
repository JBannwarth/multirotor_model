%HORTHRUST_STEPDIST Run step disturbance experiment
%   Written by:    J.X.J. Bannwarth, 2019/05/16
%	Last modified: J.X.J. Bannwarth, 2019/05/16

%% First initialization
close all; clc; clearvars;
ctrlName = 'FPHTFullGain';
project = simulinkproject; projectRoot = project.RootFolder;

%% Generate output folder
tTmp = clock;
outputFile = sprintf( 'StepDist_%s_%d-%02d-%02d_%02d-%02d-%02.f', ctrlName, tTmp(1:6));
outputFolder = fullfile( projectRoot, 'data_results', 'step_dist' );
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder)
end

%% Load wind data
ULin = [4 0 0]; % About halfway through the wind tunnel range

%% Find OP
[Aero, Ctrl, Initial, model, Motor, Simulation, Uav, windInput, toLoad] = InitializePx4( ctrlName );
FindOpPx4

%% Finish setting up simulation
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 50;
Simulation.T_OUT = Simulation.TS_OUT:Simulation.TS_OUT:Simulation.T_END-Simulation.TS_OUT;

set_param( [model '/Input choice'], 'Value', '3' )
set_param( [model '/Step wind input'], 'Commented', 'off' )

Wind.StepTime = 25;
Wind.StepInit = ULin;
Wind.StepFinal = ULin*1.5;
steps = [3 5 6 7];

for i = 1:size(steps,2)
    simIn(i) = Simulink.SimulationInput( model );
    simIn(i) = setVariable( simIn(i), 'Wind.StepFinal', [steps(i) 0 0]' );
end

%% Run simulation
simOut = sim( simIn );

%% Save data
save( fullfile( outputFolder, outputFile ), 'simIn', 'simOut', 'ULin' )