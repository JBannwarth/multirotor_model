%TESTCONTROLLERINITONLY
%% Create input profile
InitializeParameters

Simulation.TS_MAX = 0.01;
Simulation.T_END = 60;
Simulation.T_OUT = 1:TS_MAX:T_END-TS_MAX;

init_eta = [0, 0, 0];
Initial.Q = EulerToQuat(init_eta);
clearvars -except Input Simulation init_eta Initial

Px4Bus;
selfBus;
mpc_self;

InitializeParameters;