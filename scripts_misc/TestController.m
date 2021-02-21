clear all; %#ok<CLALL>

%% Create input profile
InitializeParametersQuadcopter

Simulation.TS_MAX = 0.01;
Simulation.T_END = 10;
%Input.PWM_IN = SinusoidInputPWM([0; 0; 0; 0], [0, 0, 0, 0], ...
%    Simulation, Uav);
init_eta = deg2rad( [ 10, 10, -30 ] );
Initial.Q = EulerToQuaternionM(init_eta);
clearvars -except Input Simulation init_eta Initial

Px4Bus;
selfBus;
mpc_self;

%% Load model
model = 'MultirotorSimulationController';

if bdIsLoaded(model)
    close_system(model, 0);
end

load_system(model);

%% Run model simulation
simout = sim(model, 'SimulationMode', 'normal');

% Format data
logsout = simout.get('logsout');

clear simout

% Assign units manually (apparently this won't be necessary anymore in
% 2016a+)
unitsLookupTable = readtable('SignalUnits.csv');
logsout = SetDatasetUnits(logsout, unitsLookupTable, true);

PlotResultsSingle