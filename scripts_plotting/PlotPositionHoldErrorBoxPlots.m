%PLOTPOSITIONHOLDERRORCOMPARISON Plot position hold error
%   Written by:    J.X.J. Bannwarth, 2017/09/19
%   Last Modified: J.X.J. Bannwarth, 2017/09/19
clearvars;
close all;

%% Output setup
project = simulinkproject; projectRoot = project.RootFolder;
outFolder = fullfile( projectRoot, '..', 'journal_paper_1', 'fig' );
outFolderRaw = fullfile( projectRoot, '..', 'journal_paper_1', 'fig', 'tikz', 'data_poshold' );
inFolderExp = fullfile( projectRoot, 'data_validation', 'logsWideCropped' );
inFolderSim = fullfile( projectRoot, 'data_results', 'HoverSim_2018-02-02_01-19-36_OC' );
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
if (printResults)
    MatlabToLatexEps( fullfile( outFolder, 'Hover_ExRespSim' ), [], false );
    fileID = fopen(fullfile(outFolderRaw, 'example_sim.csv'),'w');
    fprintf(fileID, 't x y z\n');
    fclose( fileID );
    data = [ errSim(end).time, errSim(end).x, errSim(end).y, errSim(end).z ];
    data = data(1:30:end,:);
    dlmwrite( fullfile(outFolderRaw, 'example_sim.csv'),  ...
        data, ...
        'precision', '%e', 'delimiter', ' ', '-append' )
end

figure; grid on; box on; hold on;
plot( errExp(end).time, errExp(end).x, errExp(end).time, errExp(end).y, errExp(end).time, errExp(end).z )
ylabel( 'Error (m)' )
xlabel( 'Time (s)' )
legend( {'x', 'y', 'z'}, 'orientation', 'horizontal', 'Location', 'southwest' )
xlim([0 150])
ylim([-0.2 0.2])
SetFigProp( outSize, fontSize )
if (printResults)
    MatlabToLatexEps( fullfile( outFolder, 'Hover_ExRespExp' ), [], false );
    fileID = fopen(fullfile(outFolderRaw, 'example_exp.csv'),'w');
    fprintf(fileID, 't x y z\n');
    fclose( fileID );
    data = [ errExp(end).time, errExp(end).x, errExp(end).y, errExp(end).z ];
    data = data(1:20:end,:);
    dlmwrite( fullfile(outFolderRaw, 'example_exp.csv'),  ...
        data, ...
        'precision', '%e', 'delimiter', ' ', '-append' )
    
    outFolder2 = fullfile( '..', 'ConferencePaperAIM', 'fig', 'tikz', 'data_stationkeeping' );
    fileID = fopen(fullfile(outFolder2, 'example_exp.csv'),'w');
    fprintf(fileID, 't x y z\n');
    fclose( fileID );
    data = [ errExp(3).time, errExp(3).x, errExp(3).y, errExp(3).z ];
    fprintf( 'x:%.4f y:%.4f z:%.4f', std(errExp(3).x), std(errExp(3).y), std(errExp(3).z) )
    data = data(1:20:end,:);
    dlmwrite( fullfile(outFolder2, 'example_exp.csv'),  ...
        data, ...
        'precision', '%e', 'delimiter', ' ', '-append' )
end

%% Plot data - Pos Error
header = [ ...
        '\\def\\boxw{0.1}\n' ...
        '\\begin{tikzpicture}\n' ...
        '\t\\begin{axis}[\n' ...
        '\t\txlabel = %s,\n' ...
        '\t\tylabel = %s,\n' ...
        '\t\t%slegend entries = {Exp, Sim},\n\t\tlegend pos = south west,\n\t\tlegend columns = 2,\n' ...
        '\t\tboxplot/draw direction = y,\n' ...
        '\t\tgrid,\n' ...
        '\t\tymin = -%f,\n' ...
        '\t\tymax = %f,\n' ...
        '\t\txmin = 2.86,\n' ...
        '\t\txmax = 5.51,\n' ...
        '\t\txtick = { 3.06, 3.70, 4.24, 4.78, 5.31 },\n' ...
        '\t\tyticklabel style={\n' ...
			'\t\t\t/pgf/number format/fixed,\n' ...
			'\t\t\t/pgf/number format/precision=3\n' ...
		'\t\t},\n' ...
		'\t\tscaled y ticks=false,\n' ...
        '%s' ...
        '\t\twidth = \\boxplotw,\n' ...
        '\t\theight = \\boxploth,\n' ...
        '\t\tname = main,\n' ...
        '\t]\n' ...
        '\t\\addplot+[color=color1, solid, mark=none] coordinates { (1,1) };\n' ...
        '\t\\addplot+[color=color2, solid, mark=none] coordinates { (1,1) };\n' ...
    ];
plotStr = [ ...
    '\t\\addplot+[%%\n' ...
        '\t\tboxplot prepared = {\n' ...
            '\t\t\tbox extend = \\boxw,\n' ...
            '\t\t\tdraw position = %f,\n' ...
            '\t\t\tlower whisker = %f,\n' ...
            '\t\t\tlower quartile = %f,\n' ...
            '\t\t\tmedian = %f,\n' ...
            '\t\t\tupper quartile = %f,\n' ...
            '\t\t\tupper whisker = %f\n' ...
        '\t\t},\n' ...
        '\t\t%s,\n' ...
        '\t\tsolid,\n' ...
        '\t\t]\n' ...
        '\t\tcoordinates {};\n' ...
    ];
footer = '\t\\end{axis}\n%s\\end{tikzpicture}';

c = get(h,'Color');
ax = { 'x', 'y', 'z' };
for a = 1:length(ax)
    figure; grid on; box on; hold on;
    %dirty hack for line colors
    h1 = plot( [-1 -1], [-1 -1]);
    h2 = plot( [-1 -1], [-1 -1]);
    for i = 1:length( meanWindSpeed )
        bplot( errExp(i).(ax{a}), meanWindSpeed(i)-0.057, 'width', 0.1, 'color', c{1}, 'nomean', 'std' );
        bplot( errSim(i).(ax{a}), meanWindSpeed(i)+0.057, 'width', 0.1, 'color', c{2}, 'nomean', 'std' );
    end
    
    meanWindSpeedDisp = 0.01*floor( 100*meanWindSpeed );
    
    xlabel( 'Wind speed (m/s)' )
    ylabel( ['$' ax{a} '$-axis error (m)'] )
    legend( [h1 h2], { 'Exp', 'Sim' }, 'Orientation', 'horizontal', 'Location', 'northwest' )
    xlim( [ meanWindSpeed(1)-0.5 meanWindSpeed(end)+0.5 ] )
    ylim( [-0.08, 0.08] )
    xticks( meanWindSpeedDisp )
    SetFigProp( outSize , fontSize );
    if (printResults)
        MatlabToLatexEps( fullfile( outFolder, ['Hover_' ax{a} 'Error'] ), [], false );
        fileID = fopen(fullfile(outFolderRaw, '..', ['HoverTest_' ax{a} 'Error.tex']),'w');
        if (a ~= length(ax))
            fprintf( fileID, header, '', ...
            ['$' ax{a} '$-axis error (\si{\metre})'], '%', 0.05, 0.05, sprintf('\t\txticklabels={,,},\n') );
        else
            fprintf( fileID, header, 'Wind speed (\si{\metre\per\second})', ...
            ['$' ax{a} '$-axis error (\si{\metre})'], '', 0.05, 0.05, '' );
        end
        
        for i=1:length( meanWindSpeed )
            % Exp
            quartiles = quantile(errExp(i).(ax{a}), [0.25, 0.5, 0.75]);
            lowerQuartile = quartiles(1);
            upperQuartile = quartiles(3);
            IQR = upperQuartile - lowerQuartile;
            medianPoint = quartiles(2);
            lowerWhisker = mean(errExp(i).(ax{a})) - std( errExp(i).(ax{a}) ); %min( errExp(i).(ax{a})(errExp(i).(ax{a}) > ( lowerQuartile - 1.5*IQR )) );
            upperWhisker = mean(errExp(i).(ax{a})) + std( errExp(i).(ax{a}) ); %max( errExp(i).(ax{a})(errExp(i).(ax{a}) < ( upperQuartile + 1.5*IQR )) );
            fprintf( fileID, plotStr, meanWindSpeed(i)-0.06, ...
                lowerWhisker, lowerQuartile, medianPoint, upperQuartile, upperWhisker, 'color=color1' );
            
            % Sim
            quartiles = quantile(errSim(i).(ax{a}), [0.25, 0.5, 0.75]);
            lowerQuartile = quartiles(1);
            upperQuartile = quartiles(3);
            IQR = upperQuartile - lowerQuartile;
            medianPoint = quartiles(2);
            lowerWhisker = mean(errSim(i).(ax{a})) - std( errSim(i).(ax{a}) ); %min( errSim(i).(ax{a})(errSim(i).(ax{a}) > ( lowerQuartile - 1.5*IQR )) );
            upperWhisker = mean(errSim(i).(ax{a})) + std( errSim(i).(ax{a}) ); %max( errSim(i).(ax{a})(errSim(i).(ax{a}) < ( upperQuartile + 1.5*IQR )) );
            fprintf( fileID, plotStr, meanWindSpeed(i)+0.06, ...
                lowerWhisker, lowerQuartile, medianPoint, upperQuartile, upperWhisker, 'color=color2' );
            
            stdErrExp(i,a) = std( errExp(i).(ax{a}) );
            stdErrSim(i,a) = std( errSim(i).(ax{a}) );
        end
        letters = 'abcd';
        extra = sprintf(['\t\\path (main.north east) -- +(0.25,-0.15) node {\\textbf{(' letters(a) ')}};\n']);
        fprintf( fileID, footer, extra);
        fclose( fileID );
    end
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
            bplot( rad2deg( angExp(i).(ax{a}) - mean( angExp(i).(ax{a}) ) ), meanWindSpeed(i)-0.057, 'width', 0.1, 'color', c{1}, 'nomean', 'std' );
            bplot( angSim(i).(ax{a}), meanWindSpeed(i)+0.057, 'width', 0.1, 'color', c{2}, 'nomean', 'std' );
        else
            bplot( rad2deg(angExp(i).(ax{a})), meanWindSpeed(i)-0.057, 'width', 0.1, 'color', c{1}, 'nomean', 'std' );
            bplot( angSim(i).(ax{a}), meanWindSpeed(i)+0.057, 'width', 0.1, 'color', c{2}, 'nomean', 'std' );
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
    if (printResults)
        MatlabToLatexEps( fullfile( outFolder, ['Hover_' ax{a} ] ), [], false );
        fileID = fopen(fullfile(outFolderRaw, '..', ['HoverTest_' ax{a} '.tex']),'w');
        if (a == 1)
            fprintf( fileID, header, '', ...
            ['Roll angle (\si{\degree})'], '%', 1.5, 1.5, sprintf('\t\txticklabels={,,},\n') );
        elseif (a == 2)
            fprintf( fileID, header, '', ...
            [ 'Pitch angle (\si{\degree})'], '%', 0, 10, sprintf('\t\txticklabels={,,},\n') );
        else
            fprintf( fileID, header, 'Wind speed (\si{\metre\per\second})', ...
            [ 'Yaw angle (\si{\degree})'], '', 1.5, 1.5, '' );
        end
        
        for i=1:length( meanWindSpeed )
            % Exp
            if (a == 3)
                quartiles = quantile(rad2deg(angExp(i).(ax{a}) - mean( angExp(i).(ax{a}))), [0.25, 0.5, 0.75]);
            else
                quartiles = quantile(rad2deg(angExp(i).(ax{a})), [0.25, 0.5, 0.75]);
            end
            lowerQuartile = quartiles(1);
            upperQuartile = quartiles(3);
            IQR = upperQuartile - lowerQuartile;
            medianPoint = quartiles(2);
            if (a == 3)
                lowerWhisker = rad2deg( - std( angExp(i).(ax{a}) ) );
                upperWhisker = rad2deg( std( angExp(i).(ax{a}) ) );
            else
                lowerWhisker = rad2deg(mean(angExp(i).(ax{a})) - std( angExp(i).(ax{a}) ));
                upperWhisker = rad2deg(mean(angExp(i).(ax{a})) + std( angExp(i).(ax{a}) ));
            end
            fprintf( fileID, plotStr, meanWindSpeed(i)-0.06, ...
                lowerWhisker, lowerQuartile, medianPoint, upperQuartile, upperWhisker, 'color=color1' );
            
            % Sim
            quartiles = quantile(angSim(i).(ax{a}), [0.25, 0.5, 0.75]);
            lowerQuartile = quartiles(1);
            upperQuartile = quartiles(3);
            IQR = upperQuartile - lowerQuartile;
            medianPoint = quartiles(2);
            lowerWhisker = mean(angSim(i).(ax{a})) - std( angSim(i).(ax{a}) );
            upperWhisker = mean(angSim(i).(ax{a})) + std( angSim(i).(ax{a}) );
            fprintf( fileID, plotStr, meanWindSpeed(i)+0.06, ...
                lowerWhisker, lowerQuartile, medianPoint, upperQuartile, upperWhisker, 'color=color2' );
            stdErrAngleExp(i,a) = std( rad2deg( angExp(i).(ax{a}) ) );
            stdErrAngleSim(i,a) = std( angSim(i).(ax{a}) );
            meanErrAngleExp(i,a) = mean( rad2deg( angExp(i).(ax{a}) ) );
            meanErrAngleSim(i,a) = mean( angSim(i).(ax{a}) );
        end
        letters = 'abcd';
        extra = sprintf(['\t\\path (main.north east) -- +(0.25,-0.15) node {\\textbf{(' letters(a) ')}};\n']);
        fprintf( fileID, footer, extra);
        fclose( fileID );
    end
end


data = [ meanWindSpeed' stdErrExp stdErrSim stdErrAngleExp stdErrAngleSim meanErrAngleExp meanErrAngleSim ];

fileID = fopen(fullfile(outFolderRaw, 'std_error.csv'),'w');
fprintf(fileID, 'w xE yE zE xS yS zS rollE pitchE yawE rollS pitchS yawS rollEM pitchEM yawEM rollSM pitchSM yawSM\n');
fclose( fileID );
dlmwrite( fullfile(outFolderRaw, 'std_error.csv'),  ...
    data, ...
    'precision', '%e', 'delimiter', ' ', '-append' )