%PLOTPOSITIONHOLDERRORCOMPARISON Plot position hold error
%   Written by:    J.X.J. Bannwarth, 2017/09/19
%   Last Modified: J.X.J. Bannwarth, 2017/09/19
clearvars all;
close all;

%% Load data
toLoad = { 'PosHoldOpenContraption1.5TranslRotDrag.mat', ...
    'PosHoldOpenContraption1.5TranslRotDragNoLin.mat', ...
    'PosHoldOpenContraption1.5LinRotDrag.mat'};
description = { 'Transl+LinRotDrag', 'TranslRotDrag', 'LinRotDrag' };

for i = 1:length( toLoad )
    load( toLoad{i} );
    dataSim{i} = output;
    clearvars output;
end

%% Output setup
outFolder = '../journal_paper_1/fig';
fontSize  = 9;
outSize   = [8.85684 5];
printResults = false;

windXLimits = [0 7];

useClosedContraptionData = dataSim{1}(1).getSimulationMetadata.UserData.ClosedContraption;

if ( useClosedContraptionData )
    suffix = '-narrow';
else
    suffix = '-wide';
end

%% Compute statistics
if ( useClosedContraptionData )
    load( 'ExpFlightData' )
else
    load( 'ExpFlightDataWide' )
end

for i = 1:length( dataSim )
    for j = 1:length( dataSim{i} )
        % Get error
        Simulation = dataSim{i}(j).getSimulationMetadata.UserData.Simulation;
        tCuttOff = Simulation.T_END / 2;
        logsout = dataSim{i}(j).get('logsout');
        error(i,j).x = getsampleusingtime( logsout.get('posTrackingX').Values, tCuttOff, Simulation.T_END);
        error(i,j).y  = getsampleusingtime( logsout.get('posTrackingY').Values, tCuttOff, Simulation.T_END);
        error(i,j).z  = getsampleusingtime( logsout.get('posTrackingZ').Values, tCuttOff, Simulation.T_END);

        meanE.x(i,j) = mean( error(i,j).x );
        meanE.y(i,j) = mean( error(i,j).y );
        meanE.z(i,j) = mean( error(i,j).z );
        stdE.x(i,j)  =  std( error(i,j).x );
        stdE.y(i,j)  =  std( error(i,j).y );
        stdE.z(i,j)  =  std( error(i,j).z );

        % Pitch
        angle(i,j).pitch = getsampleusingtime( logsout.get('pitch').Values, tCuttOff, Simulation.T_END);
        meanTmp = mean( angle(i,j).pitch );
        stdTmp =  std( angle(i,j).pitch );
        meanA.pitch(i,j) = meanTmp(1);
        stdA.pitch(i,j)  = stdTmp(1);
    end
end

%% Plot data
legendStr = description;
legendStr{end+1} = 'exp';

ax = ['x', 'y', 'z'];
for i = 1:length(ax)
    figure( 'Name', [ 'Simulation error statistics - ' ax(i) ] )
    hold on; grid on; box on
    plot( avgWindSpeed, stdE.(ax(i)), 'o', avgWindSpeed, avgStd.(ax(i)), '+' )
    axErr(i) = gca;
    ylabel(['Stdev $' ax(i) '$-axis error (m)'], 'Interpreter', 'LaTeX')
    xlabel('$U_\mathrm{mean}$ (m/s)', 'Interpreter', 'LaTeX')
    set( gca, 'TickLabelInterpreter', 'latex' )
    legend( legendStr, 'Interpreter', 'LaTeX', 'Orientation', ...
        'Vertical', 'Location', 'NorthWest' )
    
    if i == 1
        xlim( windXLimits )
        ylim( [0 axErr(1).YLim(2)] )
    end
    
    if ( printResults )
        fileName = [ outFolder '/' 'UavErrorMean-' ax(i) '-simvexp' suffix];
        SetFigProp( outSize , fontSize );
        MatlabToLatexEps( fileName, [], false );
    end
end

linkaxes( axErr )

% Angles mean
figure; grid on; box on; hold on;
plot( avgWindSpeed, meanA.pitch, 'o', avgWindSpeed, -rad2deg(avgPitch), 'x',...
    'linewidth', 1);
axA = gca;
xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');
ylabel('Mean hover pitch angle (deg)', 'Interpreter', 'latex');
legend( legendStr, 'Interpreter', 'LaTeX', 'Orientation', ...
    'Vertical', 'Location', 'NorthWest' )
set( axA, 'TickLabelInterpreter', 'latex' )

xlim( windXLimits )
ylim([0 axA.YLim(2)])

if ( printResults )
    fileName = [ outFolder '/' 'PitchMean-' 'simvexp' suffix];
    SetFigProp( outSize , fontSize );
    MatlabToLatexEps( fileName, [], false );
end

% Angles std
figure; grid on; box on; hold on;
plot( avgWindSpeed, stdA.pitch, 'o', avgWindSpeed, rad2deg(avgStd.pitch), 'x', ...
    'linewidth', 1)
axAStd = gca;
xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');
ylabel('Stdev pitch angle (deg)', 'Interpreter', 'latex');
legend( legendStr, 'Interpreter', 'LaTeX', 'Orientation', ...
    'Vertical', 'Location', 'NorthWest' )
set( axAStd, 'TickLabelInterpreter', 'latex' )

xlim( windXLimits )
ylim([0 axAStd.YLim(2)])

if ( printResults )
    fileName = [ outFolder '/' 'PitchStd-' 'simvexp' suffix];
    SetFigProp( outSize , fontSize );
    MatlabToLatexEps( fileName, [], false );
end
