function Simulation = InitializeModel( modelName, Initial, tEnd, loadBuses )
%INITIALIZEMODEL Initialize simulation model.
%   [__] = INITIALIZEMODEL( MODELNAME, INITIAL ) sets model's initial cond.
%   [__] = INITIALIZEMODEL( MODELNAME, INITIAL, TEND ) sets sim. end time.
%   [__] = INITIALIZEMODEL( MODELNAME, INITIAL, TEND, LOADBUSES ) loads PX4 Simulink buses.
%
%   See also INITIALIZEPARAMETERSOCTOCOPTER, INITIALIZEPARAMETERSQUADCOPTER.
%
%   Written: 2017, J.X.J. Bannwarth

    arguments
        modelName (1,:) char
        Initial         struct          % Structure of initial conditions
        tEnd      (1,1) double  = 20    % [s]
        loadBuses (1,1) logical = false
    end

    %% Simulation parameters
    Simulation.T_END  = tEnd;
    Simulation.TS_MAX = 0.01;
    Simulation.TS_OUT = Simulation.TS_MAX;
    Simulation.T_OUT  = Simulation.TS_OUT:Simulation.TS_OUT:(Simulation.T_END - Simulation.TS_OUT);
    Simulation.T_START_STEP = 0;

    %% Set initial orientation
    load_system(modelName);
    estimatorPath = [ modelName '/Sensor Model/attitude_estimator_q' ];
    hEstimator = getSimulinkBlockHandle( estimatorPath );
    if hEstimator ~= -1
        set_param( estimatorPath, 'INIT_Q', mat2str( Initial.Q' ) )
    end

    %% Load buses for controllers
    % Ideally should rewrite controller to remove the need for this - would
    % clear up workspace a lot)
    if loadBuses
        BusPosition;
    end
end