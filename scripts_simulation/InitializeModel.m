%INITIALIZEMODEL Load parameters
%   Written by:    J.X.J. Bannwarth, 2017
%   Last Modified: J.X.J. Bannwarth, 2017/09/19
%% Load UAV parameters
InitializeParameters

%% Set simulation parameters
if (~exist('Simulation', 'var')) || (~isfield( Simulation, 'TS_MAX' ))
    Simulation.TS_MAX = 0.01;
end
if ~isfield( Simulation, 'TS_OUT' )
    Simulation.TS_OUT = Simulation.TS_MAX;
end
if ~isfield( Simulation, 'T_END' )
    Simulation.T_END = 60;
end
Simulation.T_OUT = Simulation.TS_MAX:Simulation.TS_MAX:Simulation.T_END-Simulation.TS_MAX;

%% Set initial orientation
init_eta = [0, 0, 0];
Initial.Q = EulerToQuat(init_eta)';

if ~exist( 'model', 'var' )
    % Default
    model = 'MultirotorSimPx4SeparateRotors';
    %error( 'Need to define variable ''model'' before calling this function' );
end

load_system(model);
set_param( [ model '/Sensor Model/attitude_estimator_q' ], ...
    'INIT_Q', [ '[' num2str( Initial.Q' ) ']' ] )

%% Load buses for controllers
% Ideally should rewrite controller to remove the need for this - would
% clear up workspace a lot)
Px4Bus;
selfBus;
mpc_self;

clearvars i ans init_eta