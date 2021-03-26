function Simulation = InitializeModel( modelName, Initial, tEnd )
%INITIALIZEMODEL Initialize simulation model.
%   [__] = INITIALIZEMODEL( MODELNAME, INITIAL ) sets model's initial cond.
%   [__] = INITIALIZEMODEL( MODELNAME, INITIAL, TEND ) sets sim. end time.
%
%   See also INITIALIZEPARAMETERSOCTOCOPTER, INITIALIZEPARAMETERSQUADCOPTER.
%
%   Written: 2017, J.X.J. Bannwarth

    arguments
        modelName (1,:) char
        Initial         struct          % Structure of initial conditions
        tEnd      (1,1) double  = 20    % [s]
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
end