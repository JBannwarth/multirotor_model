%HORTHRUST_COMPARETHRUSTCOMPENSATIONMETHODS Compare ways to compensate thrust
%   Written by:    J.X.J. Bannwarth, 2018/03/12
%	Last modified: J.X.J. Bannwarth, 2019/04/04

clearvars;
project = simulinkproject; projectRoot = project.RootFolder;

%% Configuration
% Wind
ULin = [3 0 0]';
ULin = [-0.26345 0 0]';
UStepInit = [3 0 0]';
UStepEnd  = [5 0 0]';
stepTime  = 5;

% Model
model = 'TestMultirotorSimPx4v1_8Hinf';
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 50;

%% Load parameters
load_system( model )

% Get wind data and aero parameters
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );
load( fullfile( projectRoot, 'data_wind', 'TurbSimOC', 'TurbSim_40_01' ) );

% Choose fixed wind input
set_param( [model '/Input choice'], 'Value', '2' )
set_param( [model '/Fixed wind input'], 'Value', mat2str(ULin) )

% Select submodels
set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelAIAAv3' );

% Deactivate initial states/input since we aim to find them
set_param( model, 'LoadInitialState', 'off' );
set_param( model, 'LoadExternalInput', 'off' );

% Deactivate sinusoidal input
set_param( [model '/Sinusoidal input'], 'commented', 'on' )

% Load UAV parameters
loadBuses = false;
InitializeParametersOctocopterCanted
InitializeModel
Motor.K = 0.5*Uav.RHO_AIR*Uav.D_PROP^2*Uav.A_PROP*Aero.Cz2.coefs(2);

% Set controller parameters
Ctrl.HOR_GAIN = 0;
Ctrl.VERTICAL_COMP_TYPE = 3;
Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;
Ctrl.CONTROLLER_TYPE = 1;

%% Trim model
% Deactivate initial states/input since we aim to find them
set_param( model, 'LoadInitialState', 'off' );
set_param( model, 'LoadExternalInput', 'off' );

opFile = 'horthrustOp';
if exist( fullfile( projectRoot, 'work', [opFile '.mat'] ), 'file' ) == 2
    % File exists, load it
    load( fullfile( projectRoot, 'work', [opFile '.mat'] ), 'op' )
else
    
    % Set trimming parameters
    opspec = operspec( model );

    initAttGuess = [0 -deg2rad(10) 0]';

    % States
    stateSpecs = { ...
                   'omega'     , 'Min'  , zeros(Uav.N_ROTORS,1) ;
                   'nuBody'    , 'Known', [1 1 1]'              ;
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
    inputSpecs = { ...
    %                'w'         , 'u'    , ULin         ;
    %                'w'         , 'Known', [ 1 1 1 ]'   ;
    %                'etaDes'    , 'Known', [ 1 0 1 ]'   ;
    %                'thrustDes' , 'Min'  , 0            ;
    %                'yawRateDes', 'Known', 1            ;
    %                'HorThrust' , 'Known', [ 1 1 ]'     ;
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
    save( fullfile( projectRoot, 'work', [opFile '.mat'] ), 'op' )
end

% Initialise model
set_param( model, 'LoadInitialState', 'on' );
set_param( model, 'InitialState', 'getstatestruct(op)' );

%% Test step disturbance rejection for different horizontal thrust gains
set_param( [model '/Input choice'], 'Value', '3' );
set_param( [model '/Step wind input'], 'Time', '5' );
set_param( [model '/Step wind input'], 'Before', '[3 0 0]''' );
set_param( [model '/Step wind input'], 'After' , '[5 0 0]''' );

% Prepare horizontal gain sweep
Ctrl.HOR_GAIN = 0;
Ctrl.VERTICAL_COMP_TYPE = 1;
set_param( [model '/Pos Control/Horizontal thrust control/Proportional gain/hor_xy'], 'Gain', 'Ctrl.HOR_GAIN' )
gainVals = linspace(0,0.04,5);
vertCompVals = 1:3;
ind = 1;
for j = 1:length(vertCompVals)
    for i = 1:length( gainVals )
        simIn(ind) = Simulink.SimulationInput( model );
        simIn(ind) = setVariable( simIn(ind), 'Ctrl.HOR_GAIN', gainVals(i));
        simIn(ind) = setVariable( simIn(ind), 'Ctrl.VERTICAL_COMP_TYPE', vertCompVals(j) );
        ind = ind + 1;
    end
end

% Simulate the system
% set_param( model, 'UnderSpecifiedDimensionMsg', 'none' );
simOutputs = sim(simIn);
% set_param( model, 'UnderSpecifiedDimensionMsg', 'warning' );

%% Plot results
close all
f1 = figure('Name', 'Position');
axX = subplot(4,1,1); hold on; grid on; ylabel('North $x$ (m)')
axY = subplot(4,1,2); hold on; grid on; ylabel('East $y$ (m)')
axZ = subplot(4,1,3); hold on; grid on; ylabel('Down $z$ (m)'); 
axN = subplot(4,1,4); hold on; grid on; ylabel('$|\mathbf{x}|$ (m)');  xlabel('t (s)')
f2 = figure('Name', 'Attitude');
axRoll  = subplot(3,1,1); hold on; grid on; ylabel('$\phi$ ($^\circ$)')
axPitch = subplot(3,1,2); hold on; grid on; ylabel('$\theta$ ($^\circ$)')
axYaw   = subplot(3,1,3); hold on; grid on; ylabel('$\psi$ ($^\circ$)'); xlabel('t (s)')
f3 = figure('Name', 'Thrust');
axTx = subplot(3,1,1); hold on; grid on; ylabel('$T_{B,x}$ (N)')
axTy = subplot(3,1,2); hold on; grid on; ylabel('$T_{B,y}$ (N)')
axTz = subplot(3,1,3); hold on; grid on; ylabel('$T_{B,z}$ (N)'); xlabel('t (s)')

% Colors
c = get(gca,'colororder');
cols = repmat( c(1:length(gainVals),:), length(vertCompVals), 1 );
lineStyles = {'-', '--', '-.', ':'};
lineStyle = {};
for i=1:length( vertCompVals )
    lineStyle(end+1:end+length(gainVals)) = repmat(lineStyles(i),1,length(gainVals));
end

for i = 1:length( simOutputs )
    logsout = simOutputs(i).logsout;
    % Position
    set(0, 'currentfigure', f1);
    set(f1, 'currentaxes', axX);
    plot( logsout.get('xi').Values.Time(1:100:end), logsout.get('xi').Values.Data(1:100:end,1), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f1, 'currentaxes', axY);
    plot( logsout.get('xi').Values.Time(1:100:end), logsout.get('xi').Values.Data(1:100:end,2), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f1, 'currentaxes', axZ);
    plot( logsout.get('xi').Values.Time(1:100:end), logsout.get('xi').Values.Data(1:100:end,3), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f1, 'currentaxes', axN);
    plot( logsout.get('posNorm').Values.Time(1:100:end), logsout.get('posNorm').Values.Data(1:100:end), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    
    % Attitude
    set(0, 'currentfigure', f2);
    set(f2, 'currentaxes', axRoll);
    plot( logsout.get('eta').Values.Time(1:100:end), rad2deg( logsout.get('eta').Values.Data(1:100:end,1) ), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f2, 'currentaxes', axPitch);
    plot( logsout.get('eta').Values.Time(1:100:end), rad2deg( logsout.get('eta').Values.Data(1:100:end,2) ), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f2, 'currentaxes', axYaw);
    plot( logsout.get('eta').Values.Time(1:100:end), rad2deg( logsout.get('eta').Values.Data(1:100:end,3) ), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    
    % Body thrust
    set(0, 'currentfigure', f3);
    set(f3, 'currentaxes', axTx);
    plot( logsout.get('TBody').Values.Time(1:100:end), logsout.get('TBody').Values.Data(1:100:end,1), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f3, 'currentaxes', axTy);
    plot( logsout.get('TBody').Values.Time(1:100:end), logsout.get('TBody').Values.Data(1:100:end,2), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f3, 'currentaxes', axTz);
    plot( logsout.get('TBody').Values.Time(1:100:end), logsout.get('TBody').Values.Data(1:100:end,3), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
end

% Legend
% legendStr = compose('Gain = %.2f, mode = %d', ...
%     [ repmat(gainVals', length(vertCompVals), 1), ...
%     [ones(size(gainVals')); ones(size(gainVals')).*2; ones(size(gainVals')).*3]] );

legendStr = compose( 'Gain = %.2f', gainVals );

set(0, 'currentfigure', f1); set(f1, 'currentaxes', axX);
legend( legendStr )
SetFigProp( [12,20] )
MatlabToLatexEps( fullfile( projectRoot, 'work', 'StepComp_x' ) )

set(0, 'currentfigure', f2); set(f2, 'currentaxes', axRoll);
legend( legendStr )
SetFigProp( [12,20] )
MatlabToLatexEps( fullfile( projectRoot, 'work', 'StepComp_eta' ) )

set(0, 'currentfigure', f3); set(f3, 'currentaxes', axTx);
legend( legendStr )
SetFigProp( [12,20] )
MatlabToLatexEps( fullfile( projectRoot, 'work', 'StepComp_TBody' ) )