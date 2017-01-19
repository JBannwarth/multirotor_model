clear all; %#ok<CLALL>

%% Create input profile
InitializeParameters

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

InitializeParameters;