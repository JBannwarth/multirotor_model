%CHECKATTITUDECONTROL Send attitude step commands to system and record perf
%   Written by: J.X.J. Bannwarth, 2017/08/21
clear all;

%% Initialization
InitializeModel

Simulation.TS_MAX = 0.01;
Px4Bus;
selfBus;
mpc_self;
% Uav.I(3,3) = 3 * Uav.I(3,3);

%% Load model
model = 'MultirotorSimPx4SeparateRotors';
load_system(model);

%% Set up model
UseWindProfile( model, false );
UseEstimators( model, true );
UsePositionController( model, false );
set_param( [model '/Fixed wind input'], 'value', '[0 0 0]' );
inputFiles = { ...
    'step_roll-1.mat', ...
    'step_roll-2.mat', ...
    'step_roll+1.mat', ...
    'step_roll+2.mat', ...
    'step_pitch-1.mat', ...
    'step_pitch-2.mat', ...
    'step_pitch+1.mat', ...
    'step_pitch+2.mat', ...
    'step_yaw-1.mat', ...
    'step_yaw-2.mat', ...
    'step_yaw+1.mat', ...
    'step_yaw+2.mat', ...
    };

% inputFiles = { '' };

%% Perform simulation(s)
for i = 1:length( inputFiles )
    load( inputFiles{i} )
    PrepareAttitudeStepData;
    Simulation.T_END = qDesInput(end,1);
    Initial.Q = [1 1 -1 -1]' .* qExp(1,:)';
    set_param( 'MultirotorSimPx4SeparateRotors/Sensor Model/attitude_estimator_q', ...
        'INIT_Q', [ '[' num2str( Initial.Q' ) ']' ] )
    out = sim( model, 'SimulationMode', 'normal');
    CheckAttitudeControlPlot;
end

%% Format data
% logsout = out.get('logsout');
% clear output

% Assign units manually
% unitsLookupTable = readtable('SignalUnits.csv');
% logsout = SetDatasetUnits(logsout, unitsLookupTable, true);