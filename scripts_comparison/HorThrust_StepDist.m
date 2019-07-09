%HORTHRUST_STEPDIST Run step disturbance experiment
%   Written by:    J.X.J. Bannwarth, 2019/05/16
%	Last modified: J.X.J. Bannwarth, 2019/05/16

%% First initialization
close all; clc; clearvars;
ctrlName = 'baseline';
project = simulinkproject; projectRoot = project.RootFolder;

%% Generate output folder
tTmp = clock;
outputFile = sprintf( 'StepDist_%s_%d-%02d-%02d_%02d-%02d-%02.f', ctrlName, tTmp(1:6));
outputFolder = fullfile( projectRoot, 'data_results', 'step_dist' );

%% Load wind data
ULin = [4 0 0]; % About halfway through the wind tunnel range

%% Find OP
switch ctrlName
    case 'baseline'
        InitializePx4v1_8Cont
    case 'IHT'
        InitializePx4v1_8IHT
    case 'FPHT'
        InitializePx4v1_8FPHT
    case 'FPHTSimple'
        InitializePx4v1_8FPHTSimple
    case 'MIS'
        InitializePx4v1_8MIS
end

%% Finish setting up simulation
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 30;
Simulation.T_OUT = Simulation.TS_OUT:Simulation.TS_OUT:Simulation.T_END-Simulation.TS_OUT;

set_param( [model '/Input choice'], 'Value', '3' )
set_param( [model '/Step wind input'], 'Commented', 'off' )

Wind.StepTime = 5;
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