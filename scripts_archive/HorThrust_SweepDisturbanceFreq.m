%HORTHRUST_SWEEPDISTURBANCEFREQ Compare disturbance rejection performance
%   Written by:    J.X.J. Bannwarth, 2018/04/05
%	Last modified: J.X.J. Bannwarth, 2019/04/05

clearvars;
project = simulinkproject; projectRoot = project.RootFolder;

%% Configuration
ULin = [3 0 0]';

% Model
model = 'TestMultirotorSimPx4v1_8Hinf';
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 150;

%% Load parameters
load_system( model )

% Get wind data and aero parameters
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );

% Choose fixed wind input
set_param( [model '/Input choice'], 'Value', '4' )
set_param( [model '/Varying wind input'], 'commented', 'on' )
set_param( [model '/Sinusoidal input'], 'commented', 'on' )

% Select submodels
set_param( [model '/Drag model'],  'ModelName', 'DragModelAIAAv3' );
set_param( [model '/Motor model'], 'ModelName', 'MotorModelAIAAv3' );

% Deactivate initial states/input since we aim to find them
set_param( model, 'LoadInitialState', 'off' );
set_param( model, 'LoadExternalInput', 'off' );

% Load UAV parameters
loadBuses = false;
InitializeParametersOctocopterCanted
InitializeModel

% Set controller parameters
Ctrl.HOR_GAIN = 0;
Ctrl.VERTICAL_COMP_TYPE = 3;
Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;
Ctrl.CONTROLLER_TYPE = 1;

%% Trim model
% Deactivate initial states/input since we aim to find them
set_param( model, 'LoadInitialState', 'off' );
set_param( model, 'LoadExternalInput', 'off' );

opFile = 'horthrustSineOps';
if exist( fullfile( projectRoot, 'work', [opFile '.mat'] ), 'file' ) == 2
    % File exists, load it
    load( fullfile( projectRoot, 'work', [opFile '.mat'] ), 'opts' )
else
    % set_param( [model '/Fixed wind input'], 'Value', mat2str(ULin(i,:)') )
    % Set trimming parameters
    opspec = operspec( model, [size(ULin, 2) 1] );

    initAttGuess = [0 -deg2rad(10) 0]';

    % States - same for every
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
                       'U'         , 'u'    , [ULin(i,1);0;0];
                       'U'         , 'Known', [1;1;1]        ;
                       };

        for j = 1:size(inputSpecs,1)
            inputInd = opspec(i,1).getInputIndex( [ model '/' inputSpecs{j,1} ] );
            if inputInd(end,end) ~= length( inputSpecs{j,end} )
                error( 'Length of state spec vector does not match' )
            end
            opspec(i,1).Inputs( inputInd(1,1) ).(inputSpecs{j,2}) = inputSpecs{j,end};
        end
    end

    options = findopOptions( ...
        'OptimizerType', 'graddescent-elim', ...
        'OptimizationOptions', optimset( 'MaxIter', 20000 ) ...
        );

    opts = findop( model, opspec, options );
        
    save( fullfile( projectRoot, 'work', [opFile '.mat'] ), 'opts' )
end

% Initialise model
set_param( model, 'LoadInitialState', 'on' );
set_param( model, 'InitialState', 'getstatestruct(op)' );

%% Test step disturbance rejection for different horizontal thrust gains
set_param( [model '/Sinusoidal input'], 'commented', 'off' )
set_param( [model '/Input choice'], 'Value', '5' );

% Prepare horizontal gain sweep
Wind.Ampl = [0.5;0;0];
Wind.Bias = ULin;
Wind.Freq = [1;0;0];
Ctrl.HOR_GAIN = 0;
Ctrl.VERTICAL_COMP_TYPE = 3;
op = opts(1,1);
freqs = logspace(-2,1,10);
set_param( [model '/Pos Control/Horizontal thrust control/Proportional gain/hor_xy'], 'Gain', 'Ctrl.HOR_GAIN' )
gainVals = [0,0.05]; %linspace(0,0.05,6);
for i = 1:size(freqs, 2)
    for j = 1:length( gainVals )
        simIn(i,j) = Simulink.SimulationInput( model );
        simIn(i,j) = setVariable( simIn(i,j), 'Ctrl.HOR_GAIN', gainVals(j) );
        simIn(i,j) = setVariable( simIn(i,j), 'Wind.Freq', freqs(i) );
        % simIn(i,j) = setVariable( simIn(i,j), 'op', op );
    end
end

% Simulate the system
simOutputs = sim( simIn(:) );

simOutputs = reshape( simOutputs, [length(freqs),length(gainVals)] );

%% Plot mean results
set(groot,'defaulttextinterpreter','latex'); % necessary to avoid warnings
% Compute metrics
for i = 1:length( freqs )
    for j = 1:length( gainVals )
        logsout = simOutputs(i,j).logsout;
        meanPos.N(i,j) = mean( logsout.get('posNorm').Values.Data );
        meanPos.x(i,j) = mean( logsout.get('xi').Values.Data(:,1) );
        meanPos.y(i,j) = mean( logsout.get('xi').Values.Data(:,2) );
        meanPos.z(i,j) = mean( logsout.get('xi').Values.Data(:,3) );
        stdPos.N(i,j) = std(  logsout.get('posNorm').Values.Data );
        stdPos.x(i,j) = std( logsout.get('xi').Values.Data(:,1) );
        stdPos.y(i,j) = std( logsout.get('xi').Values.Data(:,2) );
        stdPos.z(i,j) = std( logsout.get('xi').Values.Data(:,3) );
        [xData, yData] = prepareCurveData( logsout.get('xi').Values.Time , logsout.get('xi').Values.Data(:,1) );
        ft = fittype( 'sin1');
        opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
        opts.Display = 'Off';
        opts.Lower = [-Inf 0 -Inf];
        [fitresult, gof] = fit( xData, yData, ft, opts);
        gain.x(i,j) = mag2db( fitresult.a1 / Wind.Ampl(1) );
    end
end

% Axes
close all
axMean.f = figure('Name', 'Mean position');
axMean.X = subplot(4,1,1); hold on; grid on; ylabel('$x_\mathrm{mean}$ (m)'); set(gca,'XScale', 'log'); %ylim([-0.0015 0.0015]); xlim([-inf 6])
axMean.Y = subplot(4,1,2); hold on; grid on; ylabel('$y_\mathrm{mean}$ (m)'); set(gca,'XScale', 'log'); %ylim([-0.0015 0.0015]); xlim([-inf 6])
axMean.Z = subplot(4,1,3); hold on; grid on; ylabel('$z_\mathrm{mean}$ (m)'); set(gca,'XScale', 'log'); %ylim([-0.0015 0.0015]); xlim([-inf 6])
axMean.N = subplot(4,1,4); hold on; grid on; ylabel('$|\mathbf{x}|_\mathrm{mean}$ (m)'); set(gca,'XScale', 'log'); xlabel('$\omega_{U}$ (rad/s)'); %ylim([0 0.017]); xlim([-inf 6]); 
axStd.f = figure('Name', 'Position standard dev.');
axStd.X = subplot(4,1,1); hold on; grid on; ylabel('$x_\mathrm{std}$ (m)'); set(gca,'XScale', 'log');  %ylim([0 0.017]); xlim([-inf 6])
axStd.Y = subplot(4,1,2); hold on; grid on; ylabel('$y_\mathrm{std}$ (m)'); set(gca,'XScale', 'log');  %ylim([0 0.017]); xlim([-inf 6])
axStd.Z = subplot(4,1,3); hold on; grid on; ylabel('$z_\mathrm{std}$ (m)'); set(gca,'XScale', 'log');  %ylim([0 0.017]); xlim([-inf 6])
axStd.N = subplot(4,1,4); hold on; grid on; ylabel('$|\mathbf{x}|_\mathrm{std}$ (m)'); set(gca,'XScale', 'log'); xlabel('$\omega_U$ (rad/s)'); % ylim([0 inf]); xlim([-inf 6]);

% Colours
c = get(gca,'colororder');
cols = c(1:length(gainVals),:);
lineStyles = {'-', '--', '-.', ':'};
lineStyle = {};
for i=1:length( gainVals )
    lineStyle{end+1} = lineStyles{1};
end

% Plot
for i = 1:length( gainVals )
    % Mean
    set(0, 'currentfigure', axMean.f);
    set(axMean.f, 'currentaxes', axMean.X);
    plot( freqs, meanPos.x(:,i), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(axMean.f, 'currentaxes', axMean.Y);
    plot( freqs, meanPos.y(:,i), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(axMean.f, 'currentaxes', axMean.Z);
    plot( freqs, meanPos.z(:,i), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(axMean.f, 'currentaxes', axMean.N);
    plot( freqs, meanPos.N(:,i), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    
    % Std
    set(0, 'currentfigure', axStd.f);
    set(axStd.f, 'currentaxes', axStd.X);
    plot( freqs, stdPos.x(:,i), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(axStd.f, 'currentaxes', axStd.Y);
    plot( freqs, stdPos.y(:,i), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(axStd.f, 'currentaxes', axStd.Z);
    plot( freqs, stdPos.z(:,i), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(axStd.f, 'currentaxes', axStd.N);
    plot( freqs, stdPos.N(:,i), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
end

% Set legends
legendStr = compose('$%.2f$', gainVals);

set(0, 'currentfigure', axMean.f); set(axMean.f, 'currentaxes', axMean.X);
legend( legendStr, 'Orientation', 'vertical', 'Location', 'best' )
SetFigProp( [12,20], 10 )
MatlabToLatexEps( fullfile( projectRoot, 'work', 'PropGain_MeanFreq' ) )

set(0, 'currentfigure', axStd.f); set(axStd.f, 'currentaxes', axStd.X);
legend( legendStr, 'Orientation', 'vertical', 'Location', 'best' )
SetFigProp( [12,20], 10 )
MatlabToLatexEps( fullfile( projectRoot, 'work', 'PropGain_StdFreq' ) )

%% Actuator usage



%% Plot results
close all
f1 = figure('Name', 'Position');
axX = subplot(4,1,1); hold on; grid on; ylabel('$x$ (m)');
axY = subplot(4,1,2); hold on; grid on; ylabel('$y$ (m)'); xlim([-inf 6])
axZ = subplot(4,1,3); hold on; grid on; ylabel('$z$ (m)'); xlim([-inf 6])
axN = subplot(4,1,4); hold on; grid on; ylabel('$|\mathbf{x}|$ (m)'); xlim([-inf 6]); xlabel('t (s)')
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
    plot( logsout.get('xi').Values.Time, logsout.get('xi').Values.Data(:,1), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f1, 'currentaxes', axY);
    plot( logsout.get('xi').Values.Time, logsout.get('xi').Values.Data(:,2), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f1, 'currentaxes', axZ);
    plot( logsout.get('xi').Values.Time, logsout.get('xi').Values.Data(:,3), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f1, 'currentaxes', axN);
    plot( logsout.get('posNorm').Values.Time, logsout.get('posNorm').Values.Data, 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    
    % Attitude
    set(0, 'currentfigure', f2);
    set(f2, 'currentaxes', axRoll);
    plot( logsout.get('eta').Values.Time, rad2deg( logsout.get('eta').Values.Data(:,1) ), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f2, 'currentaxes', axPitch);
    plot( logsout.get('eta').Values.Time, rad2deg( logsout.get('eta').Values.Data(:,2) ), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f2, 'currentaxes', axYaw);
    plot( logsout.get('eta').Values.Time, rad2deg( logsout.get('eta').Values.Data(:,3) ), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    
    % Body thrust
    set(0, 'currentfigure', f3);
    set(f3, 'currentaxes', axTx);
    plot( logsout.get('TBody').Values.Time, logsout.get('TBody').Values.Data(:,1), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f3, 'currentaxes', axTy);
    plot( logsout.get('TBody').Values.Time, logsout.get('TBody').Values.Data(:,2), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
    set(f3, 'currentaxes', axTz);
    plot( logsout.get('TBody').Values.Time, logsout.get('TBody').Values.Data(:,3), 'LineStyle', lineStyle{i}, 'Color', cols(i,:) )
end

% Legend
legendStr = compose('Gain = %.2f, mode = %d', ...
    [ repmat(gainVals', length(vertCompVals), 1), ...
    [ones(size(gainVals')); ones(size(gainVals')).*2; ones(size(gainVals')).*3]] );

set(0, 'currentfigure', f1); set(f1, 'currentaxes', axX);
legend( legendStr )
SetFigProp( [12,20] )

set(0, 'currentfigure', f2); set(f2, 'currentaxes', axRoll);
legend( legendStr )
SetFigProp( [12,20] )

set(0, 'currentfigure', f3); set(f3, 'currentaxes', axTx);
legend( legendStr )
SetFigProp( [12,20] )