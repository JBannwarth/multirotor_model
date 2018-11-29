%INITIALIZEMODEL Load parameters
%   Written by:    J.X.J. Bannwarth, 2017
%   Last Modified: J.X.J. Bannwarth, 2018/11/29
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
Simulation.T_OUT = Simulation.TS_OUT:Simulation.TS_OUT:Simulation.T_END-Simulation.TS_OUT;
if ~isfield( Simulation, 'T_START_STEP' )
    Simulation.T_START_STEP = 0;
end

%% Set initial orientation
if ~isfield( Initial, 'Q' )      % Initial orientation (quaternion)
    init_eta = [0, 0, 0];
    Initial.Q = EulerToQuat(init_eta)';
end

if ~exist( 'model', 'var' )
    % Default
    model = 'MultirotorSimPx4SeparateRotors';
    %error( 'Need to define variable ''model'' before calling this function' );
end

try
    load_system(model);
    set_param( [ model '/Sensor Model/attitude_estimator_q' ], ...
        'INIT_Q', [ '[' num2str( Initial.Q' ) ']' ] )
catch
    warning( 'No sensor model' )
end

%% Load buses for controllers
% Ideally should rewrite controller to remove the need for this - would
% clear up workspace a lot)
Px4Bus;
selfBus;
mpc_self;

clearvars i ans init_eta