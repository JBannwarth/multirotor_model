%FINDOPPX4V1_8CONT Find the operating point of the continuous px4 impl.
%   Parameters need to be loaded up before running this script
%   Use the 'toLoad' variable to control which states are set to 0
%   Written by:    J.X.J. Bannwarth, 2019/05/15
%	Last modified: J.X.J. Bannwarth, 2019/07/09

% Deactivate initial states/input since we aim to find them
set_param( model, 'LoadInitialState', 'off' );
set_param( model, 'LoadExternalInput', 'off' );

% Linearisation point if not defined
if ~exist( 'ULin', 'var' )
    ULin = [4 0 0];
end

% Select correct input
initInput = get_param( [model '/Input choice'], 'Value' );
set_param( [model '/Input choice'], 'Value', '4' );

% Need to load data first
opspec = operspec( model, [size(ULin,1) 1] );

initAttGuess = [0 -deg2rad(10) 0]';

% States
if any( strcmp( toLoad, 'posOnly' ) )
    stateSpecs = { ...
                   'xiDot' , 'Known', [1 1 1]'              ;
                   'xi'    , 'Known', [1 1 1]'              ; };
else
    stateSpecs = { ...
                   'omega' , 'Min'  , zeros(Uav.N_ROTORS,1) ;
                   'nuBody', 'Known', [1 1 1]'              ;
                   'xiDot' , 'Known', [1 1 1]'              ;
                   'xi'    , 'Known', [1 1 1]'              ; };
end

if any( strcmp( toLoad, 'attRatePID' ) )
    stateSpecs = [ stateSpecs;
                     { ...
                       'AttRatePID_D_roll' , 'Known', [1 1]';
                       'AttRatePID_D_pitch', 'Known', [1 1]';
                       'AttRatePID_D_yaw'  , 'Known', [1 1]';
                       'AttRatePID_I_roll' , 'Known', 1;
                       'AttRatePID_I_yaw'  , 'Known', 1;
                     };
                 ];
end
if any( strcmp( toLoad, 'velPID' ) )
    stateSpecs = [ stateSpecs;
                     { ...
                       'velPID_D_x', 'Known', 1;
                       'velPID_D_y', 'Known', 1;
                       'velPID_D_z', 'Known', 1;
                       'velPID_I_y', 'Known', 1;
                     };
                 ];
end
if any( strcmp( toLoad, 'velD' ) )
    stateSpecs = [ stateSpecs;
                     { ...
                       'velPID_D_x', 'Known', 1;
                       'velPID_D_y', 'Known', 1;
                       'velPID_D_z', 'Known', 1;
                     };
                 ];
end
if any( strcmp( toLoad, 'lpThrust' ) )
    stateSpecs = [ stateSpecs;
                     { ...
                       'lpThrust_x', 'Known', 0;
                       'lpThrust_y', 'Known', 1;
                     };
                 ];
end
if any( strcmp( toLoad, 'leadComp' ) )
    stateSpecs = [ stateSpecs;
                     { ...
                       'lcThrust_D_x', 'Known', 1;
                       'lcThrust_D_y', 'Known', 1;
                     };
                 ];
end

if ~any( strcmp( toLoad, 'posOnly' ) )
    if (getSimulinkBlockHandle( [ model '/Quadrotor Quaternion Model' ]) == -1)
        % Using Euler model
        stateSpecs(end+1,:) = { 'eta', 'x'    , initAttGuess } ;
        stateSpecs(end+1,:) = { 'eta', 'Known', [ 1 0 1 ] } ;
    else
        stateSpecs(end+1,:) = { 'q', 'x', EulerToQuat(initAttGuess) } ;
    end
end

for i = 1:size(stateSpecs,1)
    stateInd = opspec(1,1).getStateIndex( stateSpecs{i,1} );
    if stateInd(end,end) ~= length( stateSpecs{i,end} )
        error( 'Length of state spec vector does not match' )
    end
    for j = 1:length(opspec)
            opspec(j,1).States( stateInd(1,1) ).(stateSpecs{i,2}) = stateSpecs{i,end};
    end
end

% Inputs
for i = 1:length(opspec)
    inputSpecs = { ...
                   'U'         , 'u'    , ULin(i,:)'   ;
                   'U'         , 'Known', [ 1 1 1 ]'   ;
                   };
   if any( strcmp( toLoad, 'fullInput' ) )
       inputSpecs = [ inputSpecs; 
                        { ...
                           'etaDes'       , 'Known', [ 1 0 1 ]'   ;
                           'thrustDes'    , 'Min'  , 0            ;
                           'horThrustDes' , 'Known', [ 1 1 ]'     ;
                        };
                    ];
   end
   if any( strcmp( toLoad, 'fullInputThrustControl' ) )
       inputSpecs = [ inputSpecs; 
                        { ...
                           'attThrustDes' , 'Known', [ 0 1 0 ]'   ;
                           'attThrustDes' , 'u', [ 0 0 -0.5 ]'   ;
                           'horThrustDes' , 'Known', [ 1 1 ]'     ;
                        };
                    ];
   end

    for j = 1:size(inputSpecs,1)
        inputInd = opspec(i,1).getInputIndex( [ model '/' inputSpecs{j,1} ] );
        if inputInd(end,end) ~= length( inputSpecs{j,end} )
            error( 'Length of state spec vector does not match' )
        end
        opspec(i,1).Inputs( inputInd(1,1) ).(inputSpecs{j,2}) = inputSpecs{j,end};
    end

    options = findopOptions( ...
        'OptimizerType', 'graddescent-elim', ...
        'OptimizationOptions', optimset( 'MaxIter', 20000 ) ...
        );
end

ops = findop( model, opspec, options );
op = ops(1);

% Initialise model
set_param( model, 'LoadInitialState', 'on' );
set_param( model, 'InitialState', 'getstatestruct(op)' );

% Go back to initial input
set_param( [model '/Input choice'], 'Value', initInput );