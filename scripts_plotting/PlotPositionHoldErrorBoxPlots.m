%PLOTPOSITIONHOLDERRORCOMPARISON Plot position hold error
%   Written by:    J.X.J. Bannwarth, 2017/09/19
%   Last Modified: J.X.J. Bannwarth, 2017/09/19
clearvars;
close all;

%% Output setup
project = simulinkproject; projectRoot = project.RootFolder;
outFolder = fullfile( projectRoot, '..', 'journal_paper_1', 'fig' );
inFolderExp = fullfile( projectRoot, 'data_validation', 'logsWideCropped' );
inFolderSim = fullfile( projectRoot, 'data_results', 'HoverSim_2017-12-08_04-52-08_OC' );
fontSize  = 9;
outSize   = [8 8];
printResults = true;

%% Load simulation data
files = dir(inFolderSim); files = {files.name};
% Only keep actual files
toRemove = [];
for i = 1:length(files)
    if ~contains( files{i}, '.mat' )
        toRemove(end+1) = i;
    end
end
files(toRemove) = [];

for i = 1:length(files)
    load( fullfile( inFolderSim, files{i} ) )
    dataSim{i} = output;
    clearvars output;
end

%% Load experimental data
files = dir(inFolderExp); files = {files.name};
% Only keep actual files
toRemove = [];
for i = 1:length(files)
    if ~contains( files{i}, '.mat' )
        toRemove(end+1) = i;
    end
end
files(toRemove) = [];

for i = 1:length(files)
    load( fullfile( inFolderExp, files{i} ) )
    flogs(i) = flog;
    clearvars flog
end

%% Compute statistics

for i = 1:length( dataSim )
    logsout = dataSim{i}.get('logsout');
    
    % Get average wind speed - Simulation
    meanWindSpeed(i) = norm( mean(logsout.get('windInput').Values.Data) );
    
    % Get pos error - Simulation
    Simulation = dataSim{i}.getSimulationMetadata.UserData.Simulation;
    tCutOff = 100;
    tCutOffEnd = tCutOff + 600;
    errXTmp = getsampleusingtime( logsout.get('posTrackingX').Values, tCutOff, tCutOffEnd);
    errYTmp  = getsampleusingtime( logsout.get('posTrackingY').Values, tCutOff, tCutOffEnd);
    errZTmp  = getsampleusingtime( logsout.get('posTrackingZ').Values, tCutOff, tCutOffEnd);
    errSim(i).x = permute(errXTmp.Data, [3 1 2]);
    errSim(i).y = permute(errYTmp.Data, [3 1 2]);
    errSim(i).z = permute(errZTmp.Data, [3 1 2]);
    errSim(i).time = errXTmp.Time - errXTmp.Time(1);
    
    % Get pos error - Experiment
    errExp(i).x = flogs(i).vehicle_local_position_setpoint.x - flogs(i).vehicle_local_position.x;
    errExp(i).y = -(flogs(i).vehicle_local_position_setpoint.y - flogs(i).vehicle_local_position.y);
    errExp(i).z = -(flogs(i).vehicle_local_position_setpoint.z - flogs(i).vehicle_local_position.z);
    errExp(i).time = flogs(i).vehicle_local_position.time;
    
    % Get angles - Simulation
    rollTmp = getsampleusingtime( logsout.get('roll').Values, tCutOff, tCutOffEnd);
    pitchTmp  = getsampleusingtime( logsout.get('pitch').Values, tCutOff, tCutOffEnd);
    yawTmp  = getsampleusingtime( logsout.get('yaw').Values, tCutOff, tCutOffEnd);
    
    angSim(i).roll  = rollTmp.Data(:,1);
    angSim(i).pitch = pitchTmp.Data(:,1);
    angSim(i).yaw   = yawTmp.Data(:,1);
    angSim(i).time  = rollTmp.Time - rollTmp.Time(1);
    angDesSim(i).roll  = rollTmp.Data(:,2);
    angDesSim(i).pitch = pitchTmp.Data(:,2);
    angDesSim(i).yaw   = yawTmp.Data(:,2);
    angDesSim(i).time  = rollTmp.Time - rollTmp.Time(1);
    
    % Get angles - Experiment
    quatTmp = [ flogs(i).vehicle_attitude.q_0_, ...
                flogs(i).vehicle_attitude.q_1_, ...
                flogs(i).vehicle_attitude.q_2_, ...
                flogs(i).vehicle_attitude.q_3_];
    quatDesTmp = [ flogs(i).vehicle_attitude_setpoint.q_d_0_, ...
                   flogs(i).vehicle_attitude_setpoint.q_d_0_, ...
                   flogs(i).vehicle_attitude_setpoint.q_d_0_, ...
                   flogs(i).vehicle_attitude_setpoint.q_d_0_];
    
    [angExp(i).roll, angExp(i).pitch, angExp(i).yaw ] = QuatToEuler( quatTmp );
    angExp(i).pitch = -angExp(i).pitch; angExp(i).yaw = -angExp(i).yaw;
    angExp(i).time = flogs(i).vehicle_attitude.time;
    [angDesExp(i).roll, angDesExp(i).pitch, angDesExp(i).yaw ] = QuatToEuler( quatDesTmp );
    angDesExp(i).pitch = -angDesExp(i).pitch; angDesExp(i).yaw = -angDesExp(i).yaw;
    angDesExp(i).time = flogs(i).vehicle_attitude_setpoint.time;
end

%% Plot data - Example response
figure; grid on; box on; hold on;
h = plot( errSim(end).time, errSim(end).x, errSim(end).time, errSim(end).y, errSim(end).time, errSim(end).z );
ylabel( 'Error (m)' )
xlabel( 'Time (s)' )
legend( {'x', 'y', 'z'}, 'orientation', 'horizontal', 'Location', 'southwest' )
xlim([0 150])
ylim([-0.2 0.2])
SetFigProp( outSize, fontSize )
if (printResults); MatlabToLatexEps( fullfile( outFolder, 'Hover_ExRespSim' ), [], false ); end

figure; grid on; box on; hold on;
plot( errExp(end).time, errExp(end).x, errExp(end).time, errExp(end).y, errExp(end).time, errExp(end).z )
ylabel( 'Error (m)' )
xlabel( 'Time (s)' )
legend( {'x', 'y', 'z'}, 'orientation', 'horizontal', 'Location', 'southwest' )
xlim([0 150])
ylim([-0.2 0.2])
SetFigProp( outSize, fontSize )
if (printResults); MatlabToLatexEps( fullfile( outFolder, 'Hover_ExRespExp' ), [], false ); end

%% Plot data - Pos Error
c = get(h,'Color');
ax = { 'x', 'y', 'z' };
for a = 1:length(ax)
    figure; grid on; box on; hold on;
    %dirty hack for line colors
    h1 = plot( [-1 -1], [-1 -1]);
    h2 = plot( [-1 -1], [-1 -1]);
    for i = 1:length( meanWindSpeed )
        bplot( errExp(i).(ax{a}), meanWindSpeed(i)-0.057, 'width', 0.1, 'color', c{1}, 'nomean' )
        bplot( errSim(i).(ax{a}), meanWindSpeed(i)+0.057, 'width', 0.1, 'color', c{2}, 'nomean' )
    end
    
    meanWindSpeedDisp = 0.01*floor( 100*meanWindSpeed );
    
    xlabel( 'Wind speed (m/s)' )
    ylabel( ['$' ax{a} '$-axis error (m)'] )
    legend( [h1 h2], { 'Exp', 'Sim' }, 'Orientation', 'horizontal', 'Location', 'northwest' )
    xlim( [ meanWindSpeed(1)-0.5 meanWindSpeed(end)+0.5 ] )
    ylim( [-0.08, 0.08] )
    xticks( meanWindSpeedDisp )
    SetFigProp( outSize , fontSize );
    if (printResults); MatlabToLatexEps( fullfile( outFolder, ['Hover_' ax{a} 'Error'] ), [], false ); end
end

%% Plot data - Angles
ax = { 'roll', 'pitch', 'yaw' };
lims = [ -1.5 1.5; 0 10; -1.5 1.5 ];
for a = 1:length(ax)
    figure; grid on; box on; hold on;
    %dirty hack for line colors
    h1 = plot( [-1 -1], [-1 -1]);
    h2 = plot( [-1 -1], [-1 -1]);
    for i = 1:length( meanWindSpeed )
        if (a == 3)
            bplot( rad2deg( angExp(i).(ax{a}) - mean( angExp(i).(ax{a}) ) ), meanWindSpeed(i)-0.057, 'width', 0.1, 'color', c{1}, 'nomean' )
            bplot( angSim(i).(ax{a}), meanWindSpeed(i)+0.057, 'width', 0.1, 'color', c{2}, 'nomean' )
        else
            bplot( rad2deg(angExp(i).(ax{a})), meanWindSpeed(i)-0.057, 'width', 0.1, 'color', c{1}, 'nomean' )
            bplot( angSim(i).(ax{a}), meanWindSpeed(i)+0.057, 'width', 0.1, 'color', c{2}, 'nomean' )
        end
        
    end
    
    meanWindSpeedDisp = 0.01*floor( 100*meanWindSpeed );
    
    xlabel( 'Wind speed (m/s)' )
    axLabel = [ upper(ax{a}(1)) ax{a}(2:end) ];
    ylabel( [axLabel ' angle ($^\circ$)'] )
    legend( [h1 h2], { 'Exp', 'Sim' }, 'Orientation', 'horizontal', 'Location', 'northwest' )
    xlim( [ meanWindSpeed(1)-0.5 meanWindSpeed(end)+0.5 ] )
    ylim( lims(a,:) )
    xticks( meanWindSpeedDisp )
    SetFigProp( outSize , fontSize );
    if (printResults); MatlabToLatexEps( fullfile( outFolder, ['Hover_' ax{a} ] ), [], false ); end
end

% binEdges = linspace(-.5,.5,100)';
% binWidth = mean(diff(binEdges));
% binMid = binEdges(1:end-1)+binWidth;
% for i = 1:length( dataSim )
%     for j = 1:length( dataSim{i} )
%         % Get error
%         Simulation = dataSim{i}(j).getSimulationMetadata.UserData.Simulation;
%         tCutOff = 100; % Simulation.T_END / 2;
%         tCutOffEnd = tCutOff + 120; % Simulation.T_END;
%         logsout = dataSim{i}(j).get('logsout');
%         error(i,j).x = getsampleusingtime( logsout.get('posTrackingX').Values, tCutOff, tCutOffEnd);
%         error(i,j).y  = getsampleusingtime( logsout.get('posTrackingY').Values, tCutOff, tCutOffEnd);
%         error(i,j).z  = getsampleusingtime( logsout.get('posTrackingZ').Values, tCutOff, tCutOffEnd);
% 
%         meanE.x(i,j) = mean( error(i,j).x );
%         meanE.y(i,j) = mean( error(i,j).y );
%         meanE.z(i,j) = mean( error(i,j).z );
%         stdE.x(i,j)  =  std( error(i,j).x );
%         stdE.y(i,j)  =  std( error(i,j).y );
%         stdE.z(i,j)  =  std( error(i,j).z );
% 
%         temp(i,j).Nx = histcounts(squeeze(error(i,j).x.Data),binEdges+meanE.x(i,j),'Normalization','pdf');
%         temp(i,j).Ny = histcounts(squeeze(error(i,j).y.Data),binEdges+meanE.y(i,j),'Normalization','pdf');
%         temp(i,j).Nz = histcounts(squeeze(error(i,j).z.Data),binEdges+meanE.z(i,j),'Normalization','pdf');
%         temp(i,j).NTheoreticalx = (1/sqrt(2*pi*stdE.x(i,j)^2))*exp(-(binMid).^2/(2*stdE.x(i,j)^2));
%         temp(i,j).NTheoreticaly = (1/sqrt(2*pi*stdE.y(i,j)^2))*exp(-(binMid).^2/(2*stdE.y(i,j)^2));
%         temp(i,j).NTheoreticalz = (1/sqrt(2*pi*stdE.z(i,j)^2))*exp(-(binMid).^2/(2*stdE.z(i,j)^2));
        
        % Pitch
%         angle(i,j).roll = getsampleusingtime( logsout.get('roll').Values, tCutOff, tCutOffEnd);
%         angle(i,j).pitch = getsampleusingtime( logsout.get('pitch').Values, tCutOff, tCutOffEnd);
%         angle(i,j).yaw = getsampleusingtime( logsout.get('yaw').Values, tCutOff, tCutOffEnd);
%         meanTmp = mean( angle(i,j).roll );
%         stdTmp =  std( angle(i,j).roll );
%         meanA.roll(i,j) = meanTmp(1);
%         stdA.roll(i,j)  = stdTmp(1);
%         meanTmp = mean( angle(i,j).pitch );
%         stdTmp =  std( angle(i,j).pitch );
%         meanA.pitch(i,j) = meanTmp(1);
%         stdA.pitch(i,j)  = stdTmp(1);
%         meanTmp = mean( angle(i,j).yaw );
%         stdTmp =  std( angle(i,j).yaw );
%         meanA.yaw(i,j) = meanTmp(1);
%         stdA.yaw(i,j)  = stdTmp(1);
%     end
% end

%% JEREMY
% figure('color',[1,1,1])
% hold on; grid on; box on;
% j = 1;
% for i=1:length(logs)
%     plot(windSpeed(i)+(temp(i,j).Nx)/max(temp(i,j).Nx)*0.4,binMid+meanE.x(i,j),'k.')
%     plot(windSpeed(i)+(temp(i,j).NTheoreticalx)/max(temp(i,j).NTheoreticalx)*0.4,binMid+meanE.x(i,j),'k')
% end
% 
% figure('color',[1,1,1])
% hold on; grid on; box on;
% for i=1:length(logs)
%     plot(windSpeed(i)+(temp(i,j).Ny)/max(temp(i,j).Ny)*0.4,binMid+meanE.y(i,j),'k.')
%     plot(windSpeed(i)+(temp(i,j).NTheoreticaly)/max(temp(i,j).NTheoreticaly)*0.4,binMid+meanE.y(i,j),'k')
% end
% 
% figure('color',[1,1,1])
% hold on; grid on; box on;
% for i=1:length(logs)
%     plot(windSpeed(i)+(temp(i,j).Nz)/max(temp(i,j).Nz)*0.4,binMid+meanE.z(i,j),'k.')
%     plot(windSpeed(i)+(temp(i,j).NTheoreticalz)/max(temp(i,j).NTheoreticalz)*0.4,binMid+meanE.z(i,j),'k')
% end

%return

%% Plot data
% legendStr = description;
% legendStr{end+1} = 'Exp';
% 
% ax = ['x', 'y', 'z'];
% markers = 'o+xds^*v><ph.';
% for i = 1:length(ax)
%     figure( 'Name', [ 'Simulation error statistics - ' ax(i) ] )
%     hold on; grid on; box on
%     for j = 1:size(meanE.(ax(i)),1)
%         plot( avgWindSpeed(j), stdE.(ax(i))(j,:), markers(j) )
%     end
%     plot( avgWindSpeed, avgStd.(ax(i)), markers(j+1) ) % Experimental
%     axErr(i) = gca;
%     ylabel(['Stdev $' ax(i) '$-axis error (m)'], 'Interpreter', 'LaTeX')
%     xlabel('$U_\mathrm{mean}$ (m/s)', 'Interpreter', 'LaTeX')
%     set( gca, 'TickLabelInterpreter', 'latex' )
%     legend( legendStr, 'Interpreter', 'LaTeX', 'Orientation', ...
%         'Vertical', 'Location', 'NorthWest' )
%     
%     xlim( windXLimits )
%     ylim( [0 axErr(1).YLim(2)] )
%     SetFigProp( outSize , fontSize );
%     
%     if ( printResults )
%         fileName = [ outFolder '/' 'UavErrorMean-' ax(i) '-simvexp' suffix];
%         MatlabToLatexEps( fileName, [], false );
%     end
% end
% 
% linkaxes( axErr )
% 
% % Angles mean
% ax = {'roll', 'pitch', 'yaw'};
% figure; grid on; box on; hold on;
% for i = 1:length( ax )
%     for j = 1:size(meanA.(ax{i}),1)
%         plot( avgWindSpeed(j), meanA.(ax{i})(j,:), markers(j) )
%     end
%     plot(avgWindSpeed, -rad2deg(avgAngle.(ax{i})), markers(j+1) ) % Experimental
%     axA = gca;
%     xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');
%     ylabel(['Mean hover ' ax{i} ' angle (deg)'], 'Interpreter', 'latex');
%     legend( legendStr, 'Interpreter', 'LaTeX', 'Orientation', ...
%         'Vertical', 'Location', 'NorthWest' )
%     set( axA, 'TickLabelInterpreter', 'latex' )
% 
%     xlim( windXLimits )
%     ylim([0 axA.YLim(2)])
%     SetFigProp( outSize , fontSize );
% end
% 
% if ( printResults )
%     fileName = [ outFolder '/' 'PitchMean-' 'simvexp' suffix];
%     MatlabToLatexEps( fileName, [], false );
% end
% 
% % Angles std
% ax = {'roll', 'pitch', 'yaw'};
% for i = 1:length( ax )
%     figure; grid on; box on; hold on;
%     for j = 1:size(stdA.(ax{i}),1)
%         plot( avgWindSpeed(j), stdA.(ax{i})(j,:), markers(j) )
%     end
%     plot(avgWindSpeed, rad2deg(avgStd.(ax{i})), markers(j+1) ) % Experimental
%     axAStd = gca;
%     xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');
%     ylabel(['Stdev ' ax{i} ' angle (deg)'], 'Interpreter', 'latex');
%     legend( legendStr, 'Interpreter', 'LaTeX', 'Orientation', ...
%         'Vertical', 'Location', 'NorthWest' )
%     set( axAStd, 'TickLabelInterpreter', 'latex' )
% 
%     xlim( windXLimits )
%     ylim([0 axAStd.YLim(2)])
%     SetFigProp( outSize , fontSize );
% end
% 
% if ( printResults )
%     fileName = [ outFolder '/' 'PitchStd-' 'simvexp' suffix];
%     MatlabToLatexEps( fileName, [], false );
% end
