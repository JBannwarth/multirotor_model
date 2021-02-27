%LINEARIZEUAVATT Linearize UAV model with attitude controller
%
%   See also MULTIROTORSIMPX4V1_8CONTATTONLYFORCECONTROL,
%   CREATEDOFHINFCONTROLLER.
%
%   Written: 2019/05/02, J.X.J. Bannwarth

clearvars;
project = simulinkproject; projectRoot = project.RootFolder;

%% Configuration
% Wind
ULin = [4.09 0 0];

% Model
model = 'MultirotorSimPx4v1_8ContAttOnlyForceControl';
tEnd = 50;

%% Load parameters
load_system( model )

% Get wind data and aero parameters
load( fullfile( projectRoot, 'data_wind', 'TurbSimOC', 'turbsim_35_01' ) );

% Select submodels
set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelZJChen' );

% Deactivate initial states/input since we aim to find them
set_param( model, 'LoadInitialState', 'off' );
set_param( model, 'LoadExternalInput', 'off' );

% Use input blocks
set_param( [model '/Input choice'], 'Value', '4' ) % Wind input
switches = find_system( model, 'SearchDepth', '1', 'regexp', 'on', ...
        'IncludeCommented', 'on', 'Name', 'Manual Switch \w');
constants = find_system( model, 'SearchDepth', '1', 'regexp', 'on', ...
        'IncludeCommented', 'on', 'Name', 'Fixed \w');
for ii = 1:length( switches )
    set_param( switches{ii}, 'sw', '0' )
end
for ii = 1:length(constants)
    set_param( constants{ii}, 'Commented', 'off' )
end
    
% Load UAV parameters
[Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( true );
Simulation = InitializeModel( model, Initial );

%% Trim model
% Deactivate initial states/input since we aim to find them
set_param( model, 'LoadInitialState', 'off' );
set_param( model, 'LoadExternalInput', 'off' );

% Set trimming parameters
opspec = operspec( model );

initAttGuess = [0 -deg2rad(10) 0]';

% States
stateSpecs = { ...
               'omega'             , 'Min'  , zeros(Uav.N_ROTORS,1) ;
               'nuBody'            , 'Known', [1 1 1]'              ;
               'xiDot'             , 'Known', [1 1 1]'              ;
               'xi'                , 'Known', [1 1 1]'              ;
               'AttRatePID_D_roll' , 'Known', [1 1]                 ;
               'AttRatePID_D_pitch', 'Known', [1 1]                 ;
               'AttRatePID_D_yaw'  , 'Known', [1 1]                 ;
               'AttRatePID_I_roll' , 'Known', 1                     ;
               'AttRatePID_I_pitch', 'Known', 0                     ;
               'AttRatePID_I_yaw'  , 'Known', 1                     ;
               };

if getSimulinkBlockHandle( [ model '/Quadrotor Quaternion Model' ]) == -1
    % Using Euler model
    stateSpecs(end+1,:) = { 'eta', 'x'    , initAttGuess } ;
    stateSpecs(end+1,:) = { 'eta', 'Known', [ 1 0 1 ] } ;
else
    stateSpecs(end+1,:) = { 'q', 'x', EulerToQuat(initAttGuess) } ;
end

for i = 1:size(stateSpecs,1)
    stateInd = opspec.getStateIndex( stateSpecs{i,1} );
    if stateInd(end,end) ~= length( stateSpecs{i,end} )
        error( 'Length of state spec vector does not match' )
    end
    opspec.States( stateInd(1,1) ).(stateSpecs{i,2}) = stateSpecs{i,end};
end

% Inputs
inputSpecs = { ...
               'U'               , 'u'    , ULin         ;
               'U'               , 'Known', [ 1 1 1 ]'   ;
               'virtualThrustDes', 'Known', [ 0 1 0 ]'   ;
               'virtualThrustDes', 'u'    , [ 0 0 -0.5 ]';
               'yawDes'          , 'Known', 1            ;
               'yawRateDes'      , 'Known', 1            ;
               'horThrustDes'    , 'Known', [ 1 1 ]'     ;
               };

for i = 1:size(inputSpecs,1)
    inputInd = opspec.getInputIndex( [ model '/' inputSpecs{i,1} ] );
    if inputInd(end,end) ~= length( inputSpecs{i,end} )
        error( 'Length of state spec vector does not match' )
    end
    opspec.Inputs( inputInd(1,1) ).(inputSpecs{i,2}) = inputSpecs{i,end};
end

options = findopOptions( ...
    'OptimizerType', 'graddescent-elim', ...
    'OptimizationOptions', optimset( 'MaxIter', 20000 ) ...
    );

op = findop( model, opspec, options );

% Initialise model
set_param( model, 'LoadInitialState', 'on' );
set_param( model, 'InitialState', 'getstatestruct(op)' );

% Write equilibrium points to constant blocks for manual validation
for ii = 1:length(constants)
    var = strsplit( constants{ii}, ' ' );
    var = char( join( var(2:end) ) );
    
    % getInputIndex returns multiple indexes but only the first one is
    % necessary
    idxs = getInputIndex( op, [model '/' var] );
    vals = op.Inputs( idxs(1,1) ).u;
    set_param( constants{ii}, 'Value', mat2str( vals ) )
end

%% Linearize model
% Get linear model
inputs = { ...
    'U'               , 1;
    'virtualThrustDes', 1;
    'horThrustDes'    , 1;
    'yawDes'          , 1;
    'yawRateDes'      , 1;
    };

outputs = { ...
%     'xiInt'                , 1; % xiInt
    'Quadrotor Euler Model', 5; % xi
    'Quadrotor Euler Model', 6; % xidot
    };

for i = 1:size(inputs,1)
    io(i) = linio( [ model '/' inputs{i,1} ], inputs{i,2}, 'input' );
end
for i = 1:size(outputs,1)
    io(end+1) = linio( [model '/' outputs{i,1} ], outputs{i,2}, 'openoutput' );
end

linsys = linearize( model, io, op );

save( fullfile( projectRoot, 'work', 'Octocopter_LinMod_Att'), ...
    'linsys', 'op', 'ULin' )