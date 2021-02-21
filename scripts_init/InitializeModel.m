function [Simulation, Initial] = InitializeModel( modelName, Initial, tEnd, loadBuses )
%INITIALIZEMODEL Initialize simulation model.
%
%   See also INITIALIZEPARAMETERSOCTOCOPTER.
%
%   Written:    J.X.J. Bannwarth, 2017

    arguments
        modelName (1,:) char    = 'MultirotorSimPx4'
        tEnd      (1,1) double  = 20    % [s]
        loadBuses (1,1) logical = false
    end

    %% Initial Conditions
    % For quaternion:
    % q = [cos(alpha); sin(alpha)*rotX; sin(alpha)*rotY; sin(alpha)*rotZ]
    % where: - alpha is the angle the UAV is rotated aronud the rotation axis
    %        - [rotX, rotY, rotZ] is a unit vector defining the rotation axis  
    Initial.XI      = [0; 0; 0];            % Initial position in inertial frame
    Initial.XI_DOT  = [0; 0; 0];            % Initial velocity in inertial frame
    Initial.ETA     = rad2deg( [0; 0; 0] ); % Initial orientation (roll, pitch, yaw)
    Initial.Q       = [1; 0; 0; 0];         % Initial orientation (quaternion)
    Initial.NU_BODY = [0; 0; 0];            % Initial angular velocity in body frame
    Initial.OMEGA   = Uav.OMEGA_HOVER .* ones(Uav.N_ROTORS,1); % Initial rotor speed

    %% Simulation parameters
    Simulation.T_END = tEnd;
    Simulation.TS_MAX = 0.01;
    Simulation.TS_OUT = Simulation.TS_MAX;
    Simulation.T_OUT = Simulation.TS_OUT:Simulation.TS_OUT:Simulation.T_END-Simulation.TS_OUT;
    Simulation.T_START_STEP = 0;

    %% Set initial orientation
    load_system(modelName);
    try
        set_param( [ modelName '/Sensor Model/attitude_estimator_q' ], ...
            'INIT_Q', [ '[' num2str( Initial.Q' ) ']' ] )
    catch
        warning( 'No sensor model' )
    end

    %% Load buses for controllers
    % Ideally should rewrite controller to remove the need for this - would
    % clear up workspace a lot)
    if loadBuses
        Px4Bus;
        selfBus;
        mpc_self;
    end
end