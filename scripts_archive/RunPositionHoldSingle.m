clear all;

%% Initialization
[Uav, Motor, Aero, Initial] = InitializeParametersQuadcopter( );

Simulation.TS_MAX = 0.01;
Simulation.T_END = 120;
init_eta = [0, 0, 0];
Initial.Q = EulerToQuaternionM(init_eta);
clearvars -except Input Simulation init_eta Initial

BusAttitudeParameters;
BusAttitude;
BusPosition;

[Uav, Motor, Aero, Initial] = InitializeParametersQuadcopter( );

load turb_5ms

%% Load model
model = 'MultirotorSimulationController';
load_system(model);

%% Set up iterations
windInput = 'windData';
UseWindProfile( model, true );

%% Perform simulation(s)
output = sim( model, 'SimulationMode', 'normal');

%% Format data
logsout = output.get('logsout');
clear output

% Assign units manually
unitsLookupTable = readtable('SignalUnits.csv');
logsout = SetDatasetUnits(logsout, unitsLookupTable, true);