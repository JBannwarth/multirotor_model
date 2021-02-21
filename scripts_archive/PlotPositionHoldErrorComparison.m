%PLOTPOSITIONHOLDERRORCOMPARISON Plot position hold error
%   Written by:    J.X.J. Bannwarth, 2017/09/19
%   Last Modified: J.X.J. Bannwarth, 2017/09/19
%clearvars;
close all;

%% Load data
% toLoad = { ...
%     'PosHoldOpenContraption1.5TranslRotDrag.mat', ...
%     'PosHoldOpenContraption1.5NoRotDrag.mat', ...
%     'PosHoldOpenContraption1.5RotDragNewWindDelay.mat'
%     'PosHoldOpenContraption1.5TranslRotDragNoLin.mat', ...
%     'PosHoldOpenContraption1.5LinRotDrag.mat', ...
%     'PosHoldOpenContraption1.5RotDragFixedController.mat', ...
%     'PosHoldOpenContraption1.5RotDragNewModel.mat', ...
%     'PosHoldOpenContraption1.5RotDragNewModelPwmOffset.mat', ...
%     'PosHoldOpenContraption1.5RotDragLinPwmOffset.mat', ...
%     'PosHoldOpenContraption1.5RotDragNewDamping.mat', ...
%       'PosHoldOpenContraption1.5LinRotDragOldMotor.mat' ...
%     };
% toLoad = { ...
% %     'PosHoldClosedContraption1.204TranslRotDrag.mat', ...
% %     'PosHoldClosedContraption1.204TranslRotDragNoLin.mat', ...
% %     'PosHoldClosedContraption1.204LinRotDrag.mat', ...
%     'PosHoldClosedContraption1.204NoRotDrag.mat', ...
%     'PosHoldClosedContraption1.204RotDragNewWindDelay.mat'
% %     'PosHoldOpenContraption1.5LinRotDragOldMotor.mat' ...
%     };
toLoad = { 'PosHoldOpenContraption1.5TurbSimTest30.mat', ...
           'PosHoldOpenContraption1.5TurbSimTest35.mat', ...
           'PosHoldOpenContraption1.5TurbSimTest40.mat', ...
           'PosHoldOpenContraption1.5TurbSimTest45.mat', ...
           'PosHoldOpenContraption1.5TurbSimTest50.mat' ...
           };
description = { ...
    '30', ...
    '35', ...
    '40', ...
    '45', ...
    '50' ...
%     '$\propto {^\mathcal{B}\mathbf{V}}_\mathrm{app}, {^\mathcal{B}}${\boldmath$\nu$}$^2$', ...
%     'No drag', ...
%     'newest'
%     '$\propto {^\mathcal{B}\mathbf{V}}_\mathrm{app}$', ...
%     '$\propto {^\mathcal{B}}${\boldmath$\nu$}$^2$', ...
%     'fixed ctrl', ...
%     '$f(\omega,\theta)$', ...
%     'PWM off $f(\omega,\theta)$', ...
%     'PWM off lin', ...
%     'fixed $\propto {^\mathcal{B}\mathbf{V}}_\mathrm{app}, {^\mathcal{B}}${\boldmath$\nu$}$^2$', ...
%     '$\propto {^\mathcal{B}}${\boldmath$\nu$}$^2$, lin motor' ...
    };

for i = 1:length( toLoad )
    load( toLoad{i} );
    dataSim{i} = output;
    clearvars output;
end

%% Output setup
outFolder = '../multirotor_model_verification_report/fig';
fontSize  = 9;
outSize   = [8.85684 8.85684];
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

binEdges = linspace(-.5,.5,100)';
binWidth = mean(diff(binEdges));
binMid = binEdges(1:end-1)+binWidth;
for i = 1:length( dataSim )
    for j = 1:length( dataSim{i} )
        % Get error
        Simulation = dataSim{i}(j).getSimulationMetadata.UserData.Simulation;
        tCutOff = 100; % Simulation.T_END / 2;
        tCutOffEnd = tCutOff + 120; % Simulation.T_END;
        logsout = dataSim{i}(j).get('logsout');
        error(i,j).x = getsampleusingtime( logsout.get('posTrackingX').Values, tCutOff, tCutOffEnd);
        error(i,j).y  = getsampleusingtime( logsout.get('posTrackingY').Values, tCutOff, tCutOffEnd);
        error(i,j).z  = getsampleusingtime( logsout.get('posTrackingZ').Values, tCutOff, tCutOffEnd);

        meanE.x(i,j) = mean( error(i,j).x );
        meanE.y(i,j) = mean( error(i,j).y );
        meanE.z(i,j) = mean( error(i,j).z );
        stdE.x(i,j)  =  std( error(i,j).x );
        stdE.y(i,j)  =  std( error(i,j).y );
        stdE.z(i,j)  =  std( error(i,j).z );

        temp(i,j).Nx = histcounts(squeeze(error(i,j).x.Data),binEdges+meanE.x(i,j),'Normalization','pdf');
        temp(i,j).Ny = histcounts(squeeze(error(i,j).y.Data),binEdges+meanE.y(i,j),'Normalization','pdf');
        temp(i,j).Nz = histcounts(squeeze(error(i,j).z.Data),binEdges+meanE.z(i,j),'Normalization','pdf');
        temp(i,j).NTheoreticalx = (1/sqrt(2*pi*stdE.x(i,j)^2))*exp(-(binMid).^2/(2*stdE.x(i,j)^2));
        temp(i,j).NTheoreticaly = (1/sqrt(2*pi*stdE.y(i,j)^2))*exp(-(binMid).^2/(2*stdE.y(i,j)^2));
        temp(i,j).NTheoreticalz = (1/sqrt(2*pi*stdE.z(i,j)^2))*exp(-(binMid).^2/(2*stdE.z(i,j)^2));
        
        % Pitch
        angle(i,j).roll = getsampleusingtime( logsout.get('roll').Values, tCutOff, tCutOffEnd);
        angle(i,j).pitch = getsampleusingtime( logsout.get('pitch').Values, tCutOff, tCutOffEnd);
        angle(i,j).yaw = getsampleusingtime( logsout.get('yaw').Values, tCutOff, tCutOffEnd);
        meanTmp = mean( angle(i,j).roll );
        stdTmp =  std( angle(i,j).roll );
        meanA.roll(i,j) = meanTmp(1);
        stdA.roll(i,j)  = stdTmp(1);
        meanTmp = mean( angle(i,j).pitch );
        stdTmp =  std( angle(i,j).pitch );
        meanA.pitch(i,j) = meanTmp(1);
        stdA.pitch(i,j)  = stdTmp(1);
        meanTmp = mean( angle(i,j).yaw );
        stdTmp =  std( angle(i,j).yaw );
        meanA.yaw(i,j) = meanTmp(1);
        stdA.yaw(i,j)  = stdTmp(1);
    end
end

%% JEREMY
figure('color',[1,1,1])
hold on; grid on; box on;
j = 1;
for i=1:length(logs)
    plot(windSpeed(i)+(temp(i,j).Nx)/max(temp(i,j).Nx)*0.4,binMid+meanE.x(i,j),'k.')
    plot(windSpeed(i)+(temp(i,j).NTheoreticalx)/max(temp(i,j).NTheoreticalx)*0.4,binMid+meanE.x(i,j),'k')
end

figure('color',[1,1,1])
hold on; grid on; box on;
for i=1:length(logs)
    plot(windSpeed(i)+(temp(i,j).Ny)/max(temp(i,j).Ny)*0.4,binMid+meanE.y(i,j),'k.')
    plot(windSpeed(i)+(temp(i,j).NTheoreticaly)/max(temp(i,j).NTheoreticaly)*0.4,binMid+meanE.y(i,j),'k')
end

figure('color',[1,1,1])
hold on; grid on; box on;
for i=1:length(logs)
    plot(windSpeed(i)+(temp(i,j).Nz)/max(temp(i,j).Nz)*0.4,binMid+meanE.z(i,j),'k.')
    plot(windSpeed(i)+(temp(i,j).NTheoreticalz)/max(temp(i,j).NTheoreticalz)*0.4,binMid+meanE.z(i,j),'k')
end

%return

%% Plot data
legendStr = description;
legendStr{end+1} = 'Exp';

ax = ['x', 'y', 'z'];
markers = 'o+xds^*v><ph.';
for i = 1:length(ax)
    figure( 'Name', [ 'Simulation error statistics - ' ax(i) ] )
    hold on; grid on; box on
    for j = 1:size(meanE.(ax(i)),1)
        plot( avgWindSpeed(j), stdE.(ax(i))(j,:), markers(j) )
    end
    plot( avgWindSpeed, avgStd.(ax(i)), markers(j+1) ) % Experimental
    axErr(i) = gca;
    ylabel(['Stdev $' ax(i) '$-axis error (m)'], 'Interpreter', 'LaTeX')
    xlabel('$U_\mathrm{mean}$ (m/s)', 'Interpreter', 'LaTeX')
    set( gca, 'TickLabelInterpreter', 'latex' )
    legend( legendStr, 'Interpreter', 'LaTeX', 'Orientation', ...
        'Vertical', 'Location', 'NorthWest' )
    
    xlim( windXLimits )
    ylim( [0 axErr(1).YLim(2)] )
    FormatFigure( outSize , fontSize );
    
    if ( printResults )
        fileName = [ outFolder '/' 'UavErrorMean-' ax(i) '-simvexp' suffix];
        PrintFigure( fileName );
    end
end

linkaxes( axErr )

% Angles mean
ax = {'roll', 'pitch', 'yaw'};
figure; grid on; box on; hold on;
for i = 1:length( ax )
    for j = 1:size(meanA.(ax{i}),1)
        plot( avgWindSpeed(j), meanA.(ax{i})(j,:), markers(j) )
    end
    plot(avgWindSpeed, -rad2deg(avgAngle.(ax{i})), markers(j+1) ) % Experimental
    axA = gca;
    xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');
    ylabel(['Mean hover ' ax{i} ' angle (deg)'], 'Interpreter', 'latex');
    legend( legendStr, 'Interpreter', 'LaTeX', 'Orientation', ...
        'Vertical', 'Location', 'NorthWest' )
    set( axA, 'TickLabelInterpreter', 'latex' )

    xlim( windXLimits )
    ylim([0 axA.YLim(2)])
    FormatFigure( outSize , fontSize );
end

if ( printResults )
    fileName = [ outFolder '/' 'PitchMean-' 'simvexp' suffix];
    PrintFigure( fileName );
end

% Angles std
ax = {'roll', 'pitch', 'yaw'};
for i = 1:length( ax )
    figure; grid on; box on; hold on;
    for j = 1:size(stdA.(ax{i}),1)
        plot( avgWindSpeed(j), stdA.(ax{i})(j,:), markers(j) )
    end
    plot(avgWindSpeed, rad2deg(avgStd.(ax{i})), markers(j+1) ) % Experimental
    axAStd = gca;
    xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');
    ylabel(['Stdev ' ax{i} ' angle (deg)'], 'Interpreter', 'latex');
    legend( legendStr, 'Interpreter', 'LaTeX', 'Orientation', ...
        'Vertical', 'Location', 'NorthWest' )
    set( axAStd, 'TickLabelInterpreter', 'latex' )

    xlim( windXLimits )
    ylim([0 axAStd.YLim(2)])
    FormatFigure( outSize , fontSize );
end

if ( printResults )
    fileName = [ outFolder '/' 'PitchStd-' 'simvexp' suffix];
    PrintFigure( fileName );
end
