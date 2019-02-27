%INITIALIZEHINFOCTO Initialize Hinf controller for octocopter
%   For testing purposes
%   Written by:    J.X.J. Bannwarth, 2018/02/06
%	Last modified: J.X.J. Bannwarth, 2019/02/06

clearvars
project = simulinkproject; projectRoot = project.RootFolder;

%% Configuration
uX = 0;
uY = 0;
useWind = false;
layout = 'octo_neg';

%% Trim using Euler model
model = 'TestMultirotorSimPx4v1_8Lin';
load_system( model )
% UseEstimators( model, false );

% Get wind data and aero parameters
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );
load( fullfile( projectRoot, 'data_wind', 'TurbSimOC', 'TurbSim_40_01' ) );

loadBuses = false;
switch layout
    case 'octo_neg'
        InitializeParametersOctocopterCanted
        InitializeModel
        Uav.ROTOR_DIRECTION = Uav.ROTOR_DIRECTION .* -1;
    case 'octo'
        InitializeParametersOctocopterCanted
        InitializeModel
    case 'quad'
        InitializeModel
    otherwise
        InitializeModel
end

if useWind
    UMean = mean( windInput );
    uX = UMean(1);
    uY = 0;
end

% Select submodels
set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelAIAAv3' );


% Set trimming parameters
opspec = operspec( model );

initAttGuess = [0 -deg2rad(10) 0]';

% States
stateSpecs = { ...
               'omega'     , 'Min'  , zeros(Uav.N_ROTORS,1) ;
               'nuBody'    , 'Known', [1 1 1]'              ;
%                'accFilterX', 'Known', 1                     ;
%                'accFilterY', 'Known', 1                     ;
%                'accFilterZ', 'Known', 1                     ;
               'xiDot'     , 'Known', [1 1 1]'              ;
               'xi'        , 'Known', [1 1 1]'              };

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
% inputSpecs = { 'w'         , 'u'    , [ uX uY 0 ]' ;
%                'w'         , 'Known', [ 1 1 1 ]'   ;
%                'etaDes'    , 'u'    , initAttGuess ;
%                'thrustDes' , 'Min'  , 0            ;
%                'yawRateDes', 'Known', 1            ;
%                'thrustHor' , 'Known', 0            };

inputSpecs = { 'w'         , 'u'    , [ uX uY 0 ]' ;
               'w'         , 'Known', [ 1 1 1 ]'   ;
%                'etaDes'    , 'Known', [ 1 0 1 ]'   ;
               'thrustDes' , 'Min'  , 0            ;
               'yawRateDes', 'Known', 1            ;
               'HorThrust' , 'Known', [ 1 1 ]'     ;
               };

for i = 1:size(inputSpecs,1)
    inputInd = opspec.getInputIndex( [ model '/' inputSpecs{i,1} ] );
    if inputInd(end,end) ~= length( inputSpecs{i,end} )
        error( 'Length of state spec vector does not match' )
    end
    opspec.Inputs( inputInd(1,1) ).(inputSpecs{i,2}) = inputSpecs{i,end};
end

op = findop( model, opspec );

% Set initial states
cor = { ...
    'OMEGA', 'omega';
    'NU_BODY', 'nuBody';
    'XI_DOT', 'xiDot';
    'XI', 'xi';
    };

for i = 1:size(cor,1)
    stateInd = opspec.getStateIndex( cor{i,2} );
    Initial.( cor{i,1} ) = op.States( stateInd(1,1) ).x;
    Initial.( cor{i,1} )( Initial.( cor{i,1} ) < 1e-10 ) = 0;
end

if getSimulinkBlockHandle( [ model '/Quadrotor Quaternion Model' ]) == -1
    % Using Euler model
    stateInd = opspec.getStateIndex( 'eta' );
    Initial.ETA = op.States( stateInd(1,1) ).x;
    Initial.ETA( abs(Initial.ETA) < 1e-10 ) = 0;
    Initial.Q = EulerToQuat(Initial.ETA')';
else
    stateInd = opspec.getStateIndex( 'q' );
    Initial.Q = op.States( stateInd(1,1) ).x;
    Initial.Q( abs(Initial.Q) < 1e-10 ) = 0;
    Initial.Q = Initial.Q ./ norm(Initial.Q);
    Initial.ETA = QuatToEuler(Initial.Q')';
end

Initial.ETA_DES = op.Inputs(2).u;
Initial.THRUST_DES = op.Inputs(3).u;

% Initial.OMEGA   = op.states(1).x;
% Initial.ETA     = op.states(2).x;
% Initial.Q       = EulerToQuat(Initial.ETA')';
% Initial.NU_BODY = op.states(3).x;
% Initial.XI_DOT  = op.states(4).x;
% Initial.XI      = op.states(5).x;

% Initial.U_M     = op.Inputs(2).u;

% Initial.Y1      = [ Initial.XI_DOT([2 1]); Initial.XI([2 1]) ];
% Initial.Y2      = [ Initial.XI_DOT(3); Initial.NU_BODY; Initial.XI(3); Initial.ETA ];
% Initial.Y       = [ Initial.Y1; Initial.Y2 ];

% Get linear model (not used by controller)
inputs = { ...
    'etaDes'     , 1 ;
    'thrustDes'  , 1 ;
    'yawRateDes' , 1 ;
    'wDist'      , 1 ;
    };

outputs = { ...
    'Quadrotor Euler Model' , 5; % xi
    };

for i = 1:size(inputs,1)
    io(i) = linio( [ model '/' inputs{i,1} ], inputs{i,2}, 'input' );
end
for i = 1:size(outputs,1)
    io(end+1) = linio( [model '/' outputs{i,1} ], outputs{i,2}, 'openoutput' );
end

linsys = linearize( model, io, op );

%% Trim the real model
model = 'TestMultirotorSimPx4v1_8Hinf';
load_system( model )

if useWind
    set_param( [model '/Varying wind input'], 'VariableName', 'windInput' );
    UseWindProfile( model, true );
else
    windVelString = sprintf('[%f; %f; 0]', uX, uY);
    set_param( [model '/Fixed wind input'], 'value', windVelString );
    UseWindProfile( model, false );
end

% Simulation parameters
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 30;
InitializeModel

opspecFull = operspec( model );

%% Run on real model
% model = 'Masse_MultirotorSimHinf';
% 
% 
% if useWind
%     set_param( [model '/Varying wind input'], 'VariableName', 'windInput' );
%     UseWindProfile( model, true );
% else
%     windVelString = sprintf('[%f; %f; 0]', uX, uY);
%     set_param( [model '/Fixed wind input'], 'value', windVelString );
%     UseWindProfile( model, false );
% end
% 
% 
% Simulation.TS_MAX = 0.001;
% Simulation.TS_OUT = 0.01;
% Simulation.T_END = 30;
% InitializeModel
% 
% % Cols: thrust, roll, pitch, yaw
% Upsilon = [ -0.25, -0.25,  0.25, -0.25 ;
%             -0.25, -0.25, -0.25,  0.25 ;
%             -0.25,  0.25, -0.25, -0.25 ;
%             -0.25,  0.25,  0.25,  0.25 ];
% 
% % Use PX4-style weigthing - rotors further apart from the axis of rotation
% % are given a higher control command (likely because of the higher moment
% % sensitivity)
% c = cosd(22.5) ./ (4*cosd(22.5)+4*sind(22.5));
% s = sind(22.5) ./ (4*cosd(22.5)+4*sind(22.5));
% Upsilon = [ -1/8, -c,  s, -1/8 ;
%             -1/8, -s,  c,  1/8 ;
%             -1/8, -s, -c, -1/8 ;
%             -1/8, -c, -s,  1/8 ;
%             -1/8,  c, -s, -1/8 ;
%             -1/8,  s, -c,  1/8 ;
%             -1/8,  s,  c, -1/8 ;
%             -1/8,  c,  s,  1/8 ];
% 
% % Initialize all matrices (not used)
% SOF2     = [eye(4), eye(4)];
% SOF2Free = [eye(4), eye(4)];
% 
% SOF1 = [ 0 0 0 0 ;
%          1 0 1 0 ;
%          0 1 0 1 ;
%          0 0 0 0 ];
% SOF1Free = SOF1;
% 
% K_p = [ 0 0 1 0 ;
%         0 1 0 0 ;
%         1 0 0 0 ;
%         0 0 0 1 ];
% K_pFree = K_p;
% 
% K_i = [ 0 0 1 0 ;
%         0 1 0 0 ;
%         1 0 0 0 ;
%         0 0 0 1 ];
% K_iFree = K_i;
% 
% % Reset
% if false
%     sof2Blocks = find_system( model, 'SearchDepth', '2', 'regexp', 'on', 'Name', 'SOF2_');
%     sof1Blocks = find_system( model, 'SearchDepth', '2', 'regexp', 'on', 'Name', 'SOF1_');
%     kpBlocks = find_system( model, 'SearchDepth', '2', 'regexp', 'on', 'Name', 'K_p_');
%     kiBlocks = find_system( model, 'SearchDepth', '2', 'regexp', 'on', 'Name', 'K_i_');
%     blocks = [ sof2Blocks; sof1Blocks; kpBlocks; kiBlocks ];
%     for i = 1:length(blocks)
%         set_param( blocks{i},  'Gain', '1' );
%     end
% end