%HORTHRUST_SINUSOIDALSWEEP Run sine sweep experiment
%   Written by:    J.X.J. Bannwarth, 2018/05/16
%	Last modified: J.X.J. Bannwarth, 2019/05/16

%% First initialization
close all; clc; clearvars;
ctrlName = 'FPHT';
project = simulinkproject; projectRoot = project.RootFolder;

%% Generate output folder
tTmp = clock;
outputFile = sprintf( 'SinusoidalSweep_%s_%d-%02d-%02d_%02d-%02d-%02.f', ctrlName, tTmp(1:6));
outputFolder = fullfile( projectRoot, 'data_results', 'sine_sweep' );

%% Load wind data
ULin = [4 0 0]; % About halfway through the wind tunnel range

%% Set up simulation
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 5;

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
set_param( [model '/Input choice'], 'Value', '5' )
set_param( [model '/Sinusoidal input'], 'Commented', 'off' )

Wind.Ampl = [0.5;0;0];
Wind.Bias = ULin;
Wind.Freq = [1;0;0];
freqs = logspace(-1,1,10);
periods = 2*pi./freqs;
tEnd = 4*periods;
tEnd( periods < 10 ) = tEnd( periods<10) + 10;

for i = 1:size(freqs,2)
    simIn(i) = Simulink.SimulationInput( model );
    simIn(i) = setVariable( simIn(i), 'Wind.Freq', freqs(i) );
    simIn(i) = setVariable( simIn(i), 'Simulation.T_END', tEnd(i) );
    simIn(i) = setVariable( simIn(i), 'Simulation.T_OUT', ...
        Simulation.TS_OUT:Simulation.TS_OUT:(4*periods(i)-Simulation.TS_OUT) );
end

%% Run simulation
simOut = sim( simIn );

%% Save data
save( fullfile( outputFolder, outputFile ), 'simIn', 'simOut', 'ULin' )