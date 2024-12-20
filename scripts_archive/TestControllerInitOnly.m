%TESTCONTROLLERINITONLY
%   Written by:    J.X.J. Bannwarth, 2017
%   Last Modified: J.X.J. Bannwarth, 2018/12/10
%% Create input profile
[Uav, Motor, Aero, Initial] = InitializeParametersQuadcopter( );

if ~isfield( Simulation, 'T_END' )
    Simulation.T_END = 60;
end
Simulation.TS_MAX = 0.01;
Simulation.T_OUT = 1:Simulation.TS_MAX:Simulation.T_END-Simulation.TS_MAX;

init_eta = [0, 0, 0];
Initial.Q = EulerToQuat(init_eta);
clearvars -except Input Simulation init_eta Initial Aero

BusAttitudeParameters;
BusAttitude;
BusPosition;

[Uav, Motor, Aero, Initial] = InitializeParametersQuadcopter( );;