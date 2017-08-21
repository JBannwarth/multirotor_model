%CHECKATTITUDECONTROL Send attitude step commands to system and record perf
%   Written by: J.X.J. Bannwarth, 2017/08/21
clear all;

%% Initialization
InitializeModel

Simulation.TS_MAX = 0.01;
Px4Bus;
selfBus;
mpc_self;
PrepareAttitudeStepData;
Simulation.T_END = qDesInput(end,1);
Initial.Q = [1 1 -1 -1]' .* qDesInput(1,2:end)';

%% Load model
model = 'MultirotorSimPx4SeparateRotors';
load_system(model);

%% Set up model
UseWindProfile( model, false );
UseEstimators( model, true );
UsePositionController( model, false );
set_param( [model '/Fixed wind input'], 'value', '[0 0 0]' );

%% Perform simulation(s)
output = sim( model, 'SimulationMode', 'normal');

%% Format data
logsout = output.get('logsout');
clear output

% Assign units manually
unitsLookupTable = readtable('SignalUnits.csv');
logsout = SetDatasetUnits(logsout, unitsLookupTable, true);