%HORTHRUST_SINUSOIDALSWEEP Run sine sweep experiment
%   Written by:    J.X.J. Bannwarth, 2018/05/16
%	Last modified: J.X.J. Bannwarth, 2020/04/23

%% First initialization
close all; clc; clearvars -except ctrlName;

if ~exist( 'ctrlName', 'var' )
    ctrlName = 'FPHTFullGain';
end
project = simulinkproject; projectRoot = project.RootFolder;

%% Generate output folder
tTmp = clock;
outputFile = sprintf( 'SinusoidalSweep_%s_%d-%02d-%02d_%02d-%02d-%02.f', ctrlName, tTmp(1:6));
outputFolder = fullfile( projectRoot, 'data_results', 'sine_sweep' );
if ~exist(outputFolder, 'dir')
   mkdir(outputFolder)
end

%% Load wind data
ULin = [5.6 0 0]; % About halfway through the wind tunnel range

%% Set up simulation
tEnd = 5;

%% Find OP
[Aero, Ctrl, Initial, model, Motor, Simulation, Uav, windInput, toLoad] = InitializePx4( ctrlName, tEnd );
FindOpPx4

%% Finish setting up simulation
set_param( [model '/Input choice'], 'Value', '5' )
set_param( [model '/Sinusoidal input'], 'Commented', 'off' )

Wind.Ampl = [0.5;0;0];
Wind.Bias = ULin;
Wind.Freq = [1;0;0];
freqs = 2*pi*logspace(-1,2,50);
periods = 2*pi./freqs;
tEnd = 4*periods;
tEnd( periods < 20 ) = tEnd( periods<20) + 20;

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
save( fullfile( outputFolder, outputFile ), 'simIn', 'simOut', 'ULin', '-v7.3' )