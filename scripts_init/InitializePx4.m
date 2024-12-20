function [Aero, Ctrl, Initial, model, Motor, Simulation, Uav, windInput, toLoad] = InitializePx4( ctrlName, tEnd )
%INITIALIZEPX4 Initialize UAV model with PX4 controller
%   [__] = INITIALIZEPX4( ) loads sim. parameters for baseline controller
%   [__] = INITIALIZEPX4( CTRLNAME ) specifies controller to use
%   [__] = INITIALIZEPX4( CTRLNAME, TEND ) specifies end time of simulation
%
%   Inputs:
%       - ctrlName: controller to use. Recognised controllers:
%                   'baseline', 'IHT', 'MIS', 'FPHT', 'FPHTFullGain',
%                   'FPHTTranslationOnly', 'FPHTTranslationOnlyThrust'
%       - tEnd:     end time of the simulation (default 20s)
%   Outputs:
%       - Aero:       aerodynamic model parameters
%       - Ctrl:       controller parameters
%       - Initial:    initial conditions of the system (non-trimmed)
%       - model:      Simulink model corresponding to the chosen controller
%       - Motor:      motor parameters
%       - Simulation: simulation parameters
%       - Uav:        general uav parameters
%       - windInput:  input wind profile - loaded to prevent errors
%
%   Note that this function does not trim the system.
%
%   See also FINDOPPX4.
%
%   Written: 2021/02/18, J.X.J. Bannwarth
    arguments
        ctrlName (1,:) char   = 'baseline'
        tEnd     (1,1) double = 20
    end
    
    %% Set-up
    project = simulinkproject;
    projectRoot = project.RootFolder;

    %% Simulation parameters
    % Get model name
    ctrlFiles = ...
        { 'baseline'                 , 'MultirotorSimPx4v1_8Cont' ;
          'IHT'                      , 'MultirotorSimPx4v1_8IHT'  ;
          'MIS'                      , 'MultirotorSimPx4v1_8MIS'  ;
          'FPHT'                     , 'MultirotorSimPx4v1_8FPHT' ;
          'FPHTFullGain'             , 'MultirotorSimPx4v1_8FPHTFullGainMatrix'  ;
          'FPHTTranslationOnly'      , 'MultirotorSimPx4v1_8FPHTTranslationOnly' ;
          'FPHTTranslationOnlyThrust', 'MultirotorSimPx4v1_8FPHTTranslationOnlyForceControl' };
    model = ctrlFiles{ strcmp( ctrlFiles(:,1), ctrlName ), 2};

    %% Load parameters
    load_system( model )

    % Get wind data and aero parameters
    load( fullfile( projectRoot, 'data_wind', 'blwt', 'turbsim_35_01' ), ...
        'windInput' );

    % Select submodels if present
    if ~contains( ctrlName, 'TranslationOnly' )
        set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
        set_param( [model '/Motor model'], 'ModelName', 'MotorModelZJChen' );
    end

    % Coment out what might cause issues
    set_param( [model '/Sinusoidal input'], 'Commented', 'on' )
    set_param( [model '/Varying wind input'], 'Commented', 'on' )
    set_param( [model '/Step wind input'], 'Commented', 'on' )

    % Load UAV parameters
    [Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( true );
    Simulation = InitializeModel( model, Initial, tEnd );

    % Controller parameters
    Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;
    Ctrl.BYPASS_ROTATION = 1;

    %% Controller-dependent parameters
    switch ctrlName
        case 'FPHTFullGain'
            Ctrl.BYPASS_ROTATION = 1;
            ctrlFile = fullfile( projectRoot, 'work', 'HinfGain.mat' );
            if isfile( ctrlFile )
                load( ctrlFile, 'K', 'thrustOp', 'ULin' )
            else
                error( 'Controller file %s not found', ctrlFile )
            end
            Ctrl.K = K;
            Ctrl.U_OPERATING = thrustOp;
            Ctrl.WIND_OPERATING = ULin;
        case 'IHT'
            Ctrl.CONTROLLER_TYPE = 3;
    end
    
    % States for finding operating point
    opSpecs = ...
        { 'baseline'                 , { 'attRatePID', 'velPID' };
          'IHT'                      , { 'attRatePID', 'velPID', 'leadComp' };
          'MIS'                      , { 'attRatePID', 'velPID', 'lpThrust' };
          'FPHT'                     , { 'attRatePID', 'velD' };
          'FPHTFullGain'             , { 'attRatePID' };
          'FPHTTranslationOnly'      , { 'posOnly', 'velD' } ;
          'FPHTTranslationOnlyThrust', { 'posOnly', 'velD' } };
    toLoad = opSpecs{ strcmp( opSpecs(:,1), ctrlName ), 2};
end