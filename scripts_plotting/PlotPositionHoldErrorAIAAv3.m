%PLOTPOSITIONHOLDERRORAIAAV3 Plot position hold error
%   Written by:    J.X.J. Bannwarth, 2017/09/19
%   Last Modified: J.X.J. Bannwarth, 2020/04/15
clearvars;
close all;

%% Output setup
project = simulinkproject; projectRoot = project.RootFolder;
outFolder = fullfile( projectRoot, '..', 'journal_paper_1', 'fig' );
outFolderRaw = fullfile( projectRoot, '..', 'journal_paper_1', 'fig', 'tikz', 'data_poshold' );
inFolderExp = fullfile( projectRoot, 'data_validation', 'HoverExpLogsOCAIAAv3' );
inFolderSim = fullfile( projectRoot, 'data_results', 'HoverSim_2020-04-15_16-24-40_OC' );
fontSize  = 9;
outSize   = [8 8];
printResults = false;

%% Load simulation data
files = dir( fullfile( inFolderSim, '*.mat' ) );
files = {files.name};

for i = 1:length(files)
    load( fullfile( inFolderSim, files{i} ) )
    dataSim{i} = output;
    clearvars output;
end

%% Load experimental data
files = dir( fullfile( inFolderExp, '*.mat' ) );
files = {files.name};

for i = 1:length(files)
    load( fullfile( inFolderExp, files{i} ) )
    flogs(i) = flog;
    clearvars flog
end

%% Compute statistics

for i = 1:length( dataSim )
    logsout = dataSim{i}.get('logsout');
    
    % Get average wind speed - Simulation
    meanWindSpeed(i,1) = norm( mean(logsout.get('windInput').Values.Data) );
    
    % Get pos error - Simulation
    Simulation = dataSim{i}.getSimulationMetadata.UserData.Simulation;
    tCutOff = 100;
    tCutOffEnd = tCutOff + 600;
    errXTmp  = getsampleusingtime( logsout.get('posTrackingX').Values, tCutOff, tCutOffEnd);
    errYTmp  = getsampleusingtime( logsout.get('posTrackingY').Values, tCutOff, tCutOffEnd);
    errZTmp  = getsampleusingtime( logsout.get('posTrackingZ').Values, tCutOff, tCutOffEnd);
    errSim(i).x = permute(errXTmp.Data, [3 1 2]);
    errSim(i).y = permute(errYTmp.Data, [3 1 2]);
    errSim(i).z = permute(errZTmp.Data, [3 1 2]);
    errSim(i).time = errXTmp.Time - errXTmp.Time(1);
    
    % Get pos error - Experiment
    errExp(i).x = flogs(i).vehicle_local_position_setpoint.x - flogs(i).vehicle_local_position.x;
    errExp(i).y = (flogs(i).vehicle_local_position_setpoint.y - flogs(i).vehicle_local_position.y);
    errExp(i).z = (flogs(i).vehicle_local_position_setpoint.z - flogs(i).vehicle_local_position.z);
    errExp(i).time = flogs(i).vehicle_local_position.time;
    
    % Get angles - Simulation
    rollTmp   = getsampleusingtime( logsout.get('roll').Values,  tCutOff, tCutOffEnd);
    pitchTmp  = getsampleusingtime( logsout.get('pitch').Values, tCutOff, tCutOffEnd);
    yawTmp    = getsampleusingtime( logsout.get('yaw').Values,   tCutOff, tCutOffEnd);
    
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
    angExp(i).time = flogs(i).vehicle_attitude.time;
    [angDesExp(i).roll, angDesExp(i).pitch, angDesExp(i).yaw ] = QuatToEuler( quatDesTmp );
    angDesExp(i).time = flogs(i).vehicle_attitude_setpoint.time;
end

for i=1:length( meanWindSpeed )
    ax = { 'x', 'y', 'z' };
    for a = 1:length(ax)
        stdErrExp(i,a) = std( errExp(i).(ax{a}) );
        stdErrSim(i,a) = std( errSim(i).(ax{a}) );
    end
    
    ax = { 'roll', 'pitch', 'yaw' };
    for a = 1:length(ax)
        stdAngleExp(i,a)  = std( rad2deg( angExp(i).(ax{a}) ) );
        stdAngleSim(i,a)  = std( angSim(i).(ax{a}) );
        meanAngleExp(i,a) = mean( rad2deg( angExp(i).(ax{a}) ) );
        meanAngleSim(i,a) = mean( angSim(i).(ax{a}) );
    end
end

%% Plot data - Example response
figure('name', 'Bannwarth et al. (2018) Fig. 8')
subplot(3,1,1); grid on; box on; hold on; xlim([0 150]); ylim([-0.15, 0.15]);
plot( errExp(end).time, errExp(end).x);
plot( errSim(end).time, errSim(end).x)
ylabel( '$x$-error (m)' )
subplot(3,1,2); grid on; box on; hold on; xlim([0 150]); ylim([-0.15, 0.15]);
plot( errExp(end).time, errExp(end).y)
plot( errSim(end).time, errSim(end).y)
ylabel( '$y$-error (m)' )
subplot(3,1,3); grid on; box on; hold on; xlim([0 150]); ylim([-0.15, 0.15]);
plot( errExp(end).time, errExp(end).z)
plot( errSim(end).time, errSim(end).z)
ylabel( '$z$-error (m)' )
xlabel( 'Time (s)' )
legend( {'Exp', 'Sim'}, 'orientation', 'horizontal', 'Location', 'southwest' )

FormatFigure( outSize, fontSize )

% Save simulation results
if (printResults)
    fileID = fopen(fullfile(outFolderRaw, 'example_sim.csv'),'w');
    fprintf(fileID, 't x y z\n');
    fclose( fileID );
    data = [ errSim(end).time, errSim(end).x, errSim(end).y, errSim(end).z ];
    data = data(1:30:end,:);
    dlmwrite( fullfile(outFolderRaw, 'example_sim.csv'),  ...
        data, ...
        'precision', '%e', 'delimiter', ' ', '-append' )
end

% Save experimental results
if (printResults)
    fileID = fopen(fullfile(outFolderRaw, 'example_exp.csv'),'w');
    fprintf(fileID, 't x y z\n');
    fclose( fileID );
    data = [ errExp(end).time, errExp(end).x, errExp(end).y, errExp(end).z ];
    data = data(1:20:end,:);
    dlmwrite( fullfile(outFolderRaw, 'example_exp.csv'),  ...
        data, ...
        'precision', '%e', 'delimiter', ' ', '-append' )
    
%     outFolder2 = fullfile( '..', 'ConferencePaperAIM', 'fig', 'tikz', 'data_stationkeeping' );
%     fileID = fopen(fullfile(outFolder2, 'example_exp.csv'),'w');
%     fprintf(fileID, 't x y z\n');
%     fclose( fileID );
%     data = [ errExp(3).time, errExp(3).x, errExp(3).y, errExp(3).z ];
%     fprintf( 'Ex resp., x:%.4f y:%.4f z:%.4f\n', std(errExp(3).x), std(errExp(3).y), std(errExp(3).z) )
%     data = data(1:20:end,:);
%     dlmwrite( fullfile(outFolder2, 'example_exp.csv'),  ...
%         data, ...
%         'precision', '%e', 'delimiter', ' ', '-append' )
end

%% Plot data - Pos Error
c = get(gca,'colororder');
mark = 'so^';
figure('name', 'Bannwarth et al. (2018) Fig. 9'); grid on; box on; hold on
xlim( [2.8, 5.7] ); ylim( [0, 0.04] )
for a = 1:length(ax)
    plot( meanWindSpeed, stdErrExp(:,a), [ mark(a) '-' ], 'Color', c(a,:) )
    plot( meanWindSpeed, stdErrSim(:,a), [ mark(a) '--'], 'Color', c(a,:) )
end
legend( { '$x_\mathrm{exp}$', '$x_\mathrm{sim}$'  , ...
          '$y_\mathrm{exp}$', '$y_\mathrm{sim}$', ...
          '$z_\mathrm{exp}$', '$z_\mathrm{sim}$' }, ...
    'Location', 'northeastoutside', 'interpreter', 'latex' )
xlabel( '$U_\mathrm{mean}$ (m/s)' )
ylabel( 'Error standard dev. (m)' )
FormatFigure

%% Plot data - Angles
c = get(gca,'colororder');
mark = 'so^';
figure('name', 'Bannwarth et al. (2018) Fig. 10.a'); grid on; box on; hold on
xlim( [2.8, 5.7] ); ylim( [0, 0.7] )
for a = 1:length(ax)
    plot( meanWindSpeed, stdAngleExp(:,a), [ mark(a) '-' ], 'Color', c(a,:) )
    plot( meanWindSpeed, stdAngleSim(:,a), [ mark(a) '--'], 'Color', c(a,:) )
end
legend( { '$\phi_\mathrm{exp}$'  , '$\phi_\mathrm{sim}$'  , ...
          '$\theta_\mathrm{exp}$', '$\theta_\mathrm{sim}$', ...
          '$\psi_\mathrm{exp}$'  , '$\psi_\mathrm{sim}$' }, ...
    'Location', 'northeastoutside', 'interpreter', 'latex' )
xlabel( '$U_\mathrm{mean}$ (m/s)' )
ylabel( 'Angle standard dev. ($^\circ$)' )
FormatFigure

figure('name', 'Bannwarth et al. (2018) Fig. 10.b'); grid on; box on; hold on
ax = { 'roll', 'pitch', 'yaw' };
xlim( [2.8, 5.7] ); ylim( [-9, 0] )
plot( meanWindSpeed, meanAngleExp(:,2), 's-' )
plot( meanWindSpeed, meanAngleSim(:,2), 's-' )
xlabel( '$U_\mathrm{mean}$ (m/s)' )
ylabel( 'Mean pitch angle ($^\circ$)' )
legend( 'Exp', 'Sim' )
FormatFigure

%% Export data
if printResults
    data = [ meanWindSpeed stdErrExp stdErrSim stdAngleExp stdAngleSim meanAngleExp meanAngleSim ];
    fileID = fopen(fullfile(outFolderRaw, 'std_error.csv'),'w');
    fprintf(fileID, 'w xE yE zE xS yS zS rollE pitchE yawE rollS pitchS yawS rollEM pitchEM yawEM rollSM pitchSM yawSM\n');
    fclose( fileID );
    dlmwrite( fullfile(outFolderRaw, 'std_error.csv'),  ...
        data, ...
        'precision', '%e', 'delimiter', ' ', '-append' )
end

% Display some results
for i = 1:length(errExp)
    meanErrExp.x(i) = mean(errExp(i).x);
    meanErrExp.y(i) = mean(errExp(i).y);
    meanErrExp.z(i) = mean(errExp(i).z);
    meanStdExp.x(i) = std(errExp(i).x);
    meanStdExp.y(i) = std(errExp(i).y);
    meanStdExp.z(i) = std(errExp(i).z);
    meanStdSim.x(i) = std(errSim(i).x);
    meanStdSim.y(i) = std(errSim(i).y);
    meanStdSim.z(i) = std(errSim(i).z);
end
meanStdRatio.x = 100 .* ( meanStdSim.x - meanStdExp.x ) ./ meanStdExp.x;
meanStdRatio.y = 100 .* ( meanStdSim.y - meanStdExp.y ) ./ meanStdExp.y;
meanStdRatio.z = 100 .* ( meanStdSim.z - meanStdExp.z ) ./ meanStdExp.z;
    
fprintf('Mean error\n')
fprintf('x || 3.1|% 2.4f, 3.6|% 2.4f, 4.1|% 2.4f, 4.7|% 2.4f, 5.2|% 2.4f\n', meanErrExp.x)
fprintf('y || 3.1|% 2.4f, 3.6|% 2.4f, 4.1|% 2.4f, 4.7|% 2.4f, 5.2|% 2.4f\n', meanErrExp.y)
fprintf('z || 3.1|% 2.4f, 3.6|% 2.4f, 4.1|% 2.4f, 4.7|% 2.4f, 5.2|% 2.4f\n', meanErrExp.z)
fprintf('Mean std error\n')
fprintf('x || 3.1|% 2.4f, 3.6|% 2.4f, 4.1|% 2.4f, 4.7|% 2.4f, 5.2|% 2.4f\n', meanStdExp.x)
fprintf('y || 3.1|% 2.4f, 3.6|% 2.4f, 4.1|% 2.4f, 4.7|% 2.4f, 5.2|% 2.4f\n', meanStdExp.y)
fprintf('z || 3.1|% 2.4f, 3.6|% 2.4f, 4.1|% 2.4f, 4.7|% 2.4f, 5.2|% 2.4f\n', meanStdExp.z)
fprintf('Mean std error\n')
fprintf('x-exp || 3.1|% 2.4f, 3.6|% 2.4f, 4.1|% 2.4f, 4.7|% 2.4f, 5.2|% 2.4f\n', meanStdExp.x)
fprintf('x-sim || 3.1|% 2.4f, 3.6|% 2.4f, 4.1|% 2.4f, 4.7|% 2.4f, 5.2|% 2.4f\n', meanStdSim.x)
fprintf('Diff std error\n')
fprintf('x || 3.1|% 2.1f%%, 3.6|% 2.1f%%, 4.1|% 2.1f%%, 4.7|% 2.1f%%, 5.2|% 2.1f%% -- % 2.1f%%\n', [meanStdRatio.x mean(meanStdRatio.x)])
fprintf('y || 3.1|% 2.1f%%, 3.6|% 2.1f%%, 4.1|% 2.1f%%, 4.7|% 2.1f%%, 5.2|% 2.1f%% -- % 2.1f%%\n', [meanStdRatio.y mean(meanStdRatio.y)])
fprintf('z || 3.1|% 2.1f%%, 3.6|% 2.1f%%, 4.1|% 2.1f%%, 4.7|% 2.1f%%, 5.2|% 2.1f%% -- % 2.1f%%\n', [meanStdRatio.z mean(meanStdRatio.z)])