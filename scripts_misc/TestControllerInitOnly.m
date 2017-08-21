clear all; %#ok<CLALL>

%% Create input profile
InitializeParameters

Simulation.TS_MAX = 0.01;
Simulation.T_END = 60;
%Input.PWM_IN = SinusoidInputPWM([0; 0; 0; 0], [0, 0, 0, 0], ...
%    Simulation, Uav);
init_eta = [0, 0, 0]; %deg2rad( [ 0, 0, 20 ] ); % 
Initial.Q = EulerToQuat(init_eta);
clearvars -except Input Simulation init_eta Initial

Px4Bus;
selfBus;
mpc_self;

InitializeParameters;