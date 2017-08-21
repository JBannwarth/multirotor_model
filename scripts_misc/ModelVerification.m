clear all; %#ok<CLALL>

%% Create input profile
InitializeParameters
Simulation.TS_MAX = 0.01;
Simulation.T_END = 100;
Input.PWM_IN = SinusoidInputPWM([0; 0; 0; 0], [0, 0, 0, 0], ...
    Simulation, Uav);
init_eta = deg2rad( [ 0, 20, 30 ] );
clearvars -except Input Simulation init_eta

%% Load old model
modelOld = 'OldSimulationModel';

if bdIsLoaded(modelOld)
    close_system(modelOld, 0);
end

load_system(modelOld);

%% Run old model simulation
simoutOld = sim(modelOld, 'SimulationMode', 'normal');

clearvars -except Input simoutOld Simulation init_eta

%% 2) New model
Initial.Q = EulerToQuaternionM(init_eta);
modelNew = 'MultirotorSimulation';

if bdIsLoaded(modelNew)
    close_system(modelNew, 0);
end

load_system(modelNew);

simoutNew = sim(modelNew, 'SimulationMode', 'normal');

% Format data
logsoutOld = simoutOld.get('logsout');
logsoutNew = simoutNew.get('logsout');

clear simoutOld simoutNew

% Assign units manually (apparently this won't be necessary anymore in
% 2016a+)
unitsLookupTable = readtable('SignalUnits.csv');
logsoutOld = SetDatasetUnits(logsoutOld, unitsLookupTable, true);
logsoutNew = SetDatasetUnits(logsoutNew, unitsLookupTable, true);

CompareResults