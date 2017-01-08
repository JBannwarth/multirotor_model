clear all; %#ok<CLALL>

%% Create input profile
InitializeParameters

Simulation.TS_MAX = 0.01;
Simulation.T_END = 100;
%Input.PWM_IN = SinusoidInputPWM([0; 0; 0; 0], [0, 0, 0, 0], ...
%    Simulation, Uav);
init_eta = deg2rad( [ 0, 20, 30 ] );
clearvars -except Input Simulation init_eta

Px4Bus;
selfBus;

%% Load model
model = 'MultirotorSimulationController.slx';

if bdIsLoaded(model)
    close_system(model, 0);
end

load_system(model);

%% Run model simulation
simout = sim(model, 'SimulationMode', 'normal');

clearvars -except Input simoutOld Simulation init_eta

% Format data
logsout = simout.get('logsout');

clear simout

% Assign units manually (apparently this won't be necessary anymore in
% 2016a+)
unitsLookupTable = readtable('SignalUnits.csv');
logsout = SetDatasetUnits(logsout, unitsLookupTable, true);

PlotResultsSingle