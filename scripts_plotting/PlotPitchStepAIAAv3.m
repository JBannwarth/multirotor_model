%CHECKATTITUDECONTROLPLOT Plot step responses
%   Written by: J.X.J. Bannwarth, 2017/08/21

close all; clearvars;

% Setup
project = simulinkproject; projectRoot = project.RootFolder;
inFolder  = fullfile( projectRoot, 'data_results', 'AttSim_2019-01-10_13-33-33' );
outFolder = fullfile( projectRoot, '..', 'journal_paper_1', 'fig' );
outFolderRaw = fullfile( projectRoot, '..', 'journal_paper_1', 'fig', 'tikz', 'data_step' );
indexToPrint = 8;
fontSize  = 9;
outSize   = [8 8];
printResults = false;

% Get data - might require a lot of memory
inputFiles = dir( inFolder );
inputFiles = {inputFiles.name};
inputFiles( ~contains( inputFiles, '.mat' ) ) = [];

for i = 1:length(inputFiles)
    load( fullfile( inFolder, inputFiles{i} ) )
    simData{i} = output;
end
clearvars output;

%% Process data
% Find shortest data
minStepLength = simData{1}.getSimulationMetadata.UserData.AttInput.stepLength;
stepStart = zeros(1, length(simData));
stepLength = zeros(1, length(simData));
offset = zeros(1, length(simData));
for i = 1:length( simData )
    stepStart(i) = simData{i}.getSimulationMetadata.UserData.AttInput.stepTime;
    stepLength(i) = simData{i}.getSimulationMetadata.UserData.AttInput.stepLength;
    offset(i) = simData{i}.getSimulationMetadata.UserData.AttInput.tDesOffset;
end
stepEnd = stepStart+min(stepLength) .* ones(size(stepLength));

%% Plot data
toSave = 8;
tableStr = '';
for i = 1:length( inputFiles )
    if ( i == toSave )
        disp('Saving')
    end
    logsout = simData{i}.get('logsout');
    pwm     = logsout.get('pwm').Values;
    pitch   = logsout.get('pitch').Values;
    
    % Isolate relevant data
    pwm   = getsampleusingtime( pwm, stepStart(i)+offset(i), stepEnd(i)+offset(i) );
    pitch = getsampleusingtime( pitch, stepStart(i)+offset(i), stepEnd(i)+offset(i) );
    pwm.Time = pwm.Time - (stepStart(i)+offset(i));
    pitch.Time = pitch.Time - (stepStart(i)+offset(i));
    
    % Experimental
    flog = simData{i}.getSimulationMetadata.UserData.FlogOriginal;
    
    % PWM
    pwmExp = [ flog.actuator_outputs.output_0_, ...
            flog.actuator_outputs.output_1_,    ...
            flog.actuator_outputs.output_2_,    ...
            flog.actuator_outputs.output_3_     ];
    pwmExpT = flog.actuator_outputs.time - stepStart(i);
        
    % Measured pitch
    qExp = [ flog.vehicle_attitude.q_0_, ...
             flog.vehicle_attitude.q_1_, ...
             flog.vehicle_attitude.q_2_, ...
             flog.vehicle_attitude.q_3_  ];
    [~, pitchExp, ~] = QuatToEuler( qExp );
    pitchExpT = flog.vehicle_attitude.time - stepStart(i);
    
    % Desired pitch
    qDesExp = [ flog.vehicle_attitude_setpoint.q_d_0_, ...
             flog.vehicle_attitude_setpoint.q_d_1_, ...
             flog.vehicle_attitude_setpoint.q_d_2_, ...
             flog.vehicle_attitude_setpoint.q_d_3_  ];
    [~, pitchDesExp, ~] = QuatToEuler( qDesExp );
    pitchDesExpT = flog.vehicle_attitude_setpoint.time - stepStart(i);
    
    % Rearrange PWM and change axes to match sim
    pwmExp      = [ pwmExp(:,1), pwmExp(:,4), pwmExp(:,2), pwmExp(:,3) ];
%     pitchExp    = - pitchExp;
%     pitchDesExp = - pitchDesExp;
    
    % Resample
    TResample   = 1/250;
    pitchExp    = resample( timeseries(pitchExp, pitchExpT), 0:TResample:min(stepLength) );
    pitchDesExp = resample( timeseries(pitchDesExp, pitchDesExpT), 0:TResample:min(stepLength) );
    pwmExp      = resample( timeseries(pwmExp, pwmExpT), 0:TResample:min(stepLength) );
    
    % Get data stats
    errPitchExp = pitchDesExp - pitchExp;
    errPitchSim = pitch.Data(:,2) - pitch.Data(:,1);
    avgErrSim(i) = abs(mean(errPitchSim));
    maxErrSim(i) = max(abs(errPitchSim));
    rmsErrSim(i) = rms(errPitchSim);
    
    avgErrExp(i) = rad2deg( abs(mean(errPitchExp)) );
    maxErrExp(i) = rad2deg( max(abs(errPitchExp.Data)) );
    rmsErrExp(i) = rad2deg( rms(errPitchExp.Data) );

    % Plot results
    fileNameCurrent = inputFiles{i};
    
    % Pitch response
    figure('name', [ inputFiles{i} ' - pitch' ] )
    hold on; grid on; box on;
    stairs( pitchDesExp.Time, rad2deg(pitchDesExp.Data) )
    stairs( pitchExp.Time, rad2deg(pitchExp.Data) )
    stairs( pitch.Time, pitch.Data(:,1) )
    xlim( [0 inf] )
    %ylim( [-inf inf] )
    xlabel( 'Time (s)' )
    ylabel( [ 'Pitch ($^\circ$)' ] )
    legend( { 'Des', 'Exp', 'Sim' }, 'location', 'best')
    SetFigProp( outSize , fontSize );
    
    if ( printResults && ( i == indexToPrint ) )
        fileName = fullfile( outFolder, 'PitchStep_PitchResponse' );
        MatlabToLatexEps( fileName );
        if ( i == toSave )
            % Exp
            data = [ pitchExp.Time, rad2deg(pitchExp.Data), rad2deg(pitchDesExp.Data) ];
            data = data(1:2:end,:);
            fileID = fopen(fullfile(outFolderRaw, 'pitch_exp.csv'),'w');
            fprintf(fileID, 't exp des\n');
            fclose( fileID );
            dlmwrite( fullfile(outFolderRaw, 'pitch_exp.csv'),  ...
                data, ...
                'precision', '%e', 'delimiter', ' ', '-append' )
            
            % Sim
            data = [ pitch.Time, pitch.Data ];
            fileID = fopen(fullfile(outFolderRaw, 'pitch_sim.csv'),'w');
            fprintf(fileID, 't sim\n');
            fclose( fileID );
            dlmwrite( fullfile(outFolderRaw, 'pitch_sim.csv'),  ...
                data, ...
                'precision', '%e', 'delimiter', ' ', '-append' )
        end
    end

    % PWM
    figure('name', [ inputFiles{i} ' - pwm' ] )
    hold on; grid on; box on;
    col = get(gca,'colororder');
    for j = 1:4
        stairs( pwmExp.Time, pwmExp.Data(:,j), 'color', col(j,:) )
    end
    for j = 1:4
        stairs( pwm.Time, 1000+1000.*pwm.Data(:,j), '--', 'color', col(j,:) )
    end
    xlim( [0 inf] )
    ylim( [1400 1900] )
    xlabel( 'Time (s)' )
    ylabel( 'PWM magnitude (-)' )
    hleg = legend( {'FL', 'BL', 'BR', 'FR'}, ...
        'Orientation', 'horizontal', 'location', 'northoutside');
    title( hleg, 'solid: experiment, dot: simulation' )
    SetFigProp( outSize , fontSize );
    legend('boxoff')
    
    if ( printResults && ( i == indexToPrint ) )
        fileName = fullfile( outFolder, 'PitchStep_PWMResponse' );
        MatlabToLatexEps( fileName );
        
        if ( i == toSave )
%           % Exp
            offsetsTmp = mean( getsampleusingtime( pwmExp, 0, 0.8 ) );
            offsets = offsetsTmp;
%             offsets(1) = mean( offsetsTmp([1,3]));
%             offsets(2) = mean( offsetsTmp([2,4]));
%             offsets(3) = mean( offsetsTmp([1,3]));
%             offsets(4) = mean( offsetsTmp([2,4]));
            data = [ pwmExp.Time, pwmExp.Data - offsets ];
            fileID = fopen(fullfile(outFolderRaw, 'pwm_exp.csv'),'w');
            fprintf(fileID, 't fl bl br fr\n');
            fclose( fileID );
            dlmwrite( fullfile(outFolderRaw, 'pwm_exp.csv'),  ...
                data, ...
                'precision', '%e', 'delimiter', ' ', '-append' )
            
            % Sim
            offsetsSim = 1626;
            data = [ pwm.Time, 1000+1000.*pwm.Data - offsetsSim ];
            fileID = fopen(fullfile(outFolderRaw, 'pwm_sim.csv'),'w');
            fprintf(fileID, 't fl bl br fr\n');
            fclose( fileID );
            dlmwrite( fullfile(outFolderRaw, 'pwm_sim.csv'),  ...
                data, ...
                'precision', '%e', 'delimiter', ' ', '-append' )
        end
    end
end

% Print table of results
line1 = '$e_\mathrm{RMS,exp}$ (\si{\degree}) & ';
line2 = '$e_\mathrm{RMS,sim}$ (\si{\degree}) & ';
line3 = '$e_\mathrm{max,exp}$ (\si{\degree}) & ';
line4 = '$e_\mathrm{max,sim}$ (\si{\degree}) & ';
steps = '& ';

for i = 1:(length( avgErrSim )-1)
    line1 = sprintf( '%s%.2f & ', line1, rmsErrExp(i) );
    line2 = sprintf( '%s%.2f & ', line2, rmsErrSim(i) );
    line3 = sprintf( '%s%.2f & ', line3, maxErrExp(i) );
    line4 = sprintf( '%s%.2f & ', line4, maxErrSim(i) );
    steps = sprintf( '%sStep %d & ', steps, i );
end

line1 = sprintf( '%s%.2f \\\\', line1, rmsErrExp(end) );
line2 = sprintf( '%s%.2f \\\\', line2, rmsErrSim(end) );
line3 = sprintf( '%s%.2f \\\\', line3, maxErrExp(end) );
line4 = sprintf( '%s%.2f \\\\', line4, maxErrSim(end) );
steps = sprintf( '%sStep %d \\\\', steps, i );

heading = sprintf( '\\begin{tabular}{%s}\n\\toprule\n%s\n\\midrule', ...
    repmat( 'l', 1, length(maxErrExp)+1 ), steps );

tableStr = sprintf( '%s\n%s\n%s\n%s\n%s\n\\bottomrule\n\\end{tabular}', ...
    heading, line1, line2, line3, line4 );
    
disp( tableStr )

fprintf( 'Average RMS: %.2f (sim) %.2f (exp): %.2f%% diff\n', mean(rmsErrSim), mean(rmsErrExp), 100*(mean(rmsErrSim)-mean(rmsErrExp))/mean(rmsErrExp) )
fprintf( 'Average peak: %.2f (sim) %.2f (exp): %.2f%% diff\n', mean(maxErrSim), mean(maxErrExp), 100*(mean(maxErrSim)-mean(maxErrExp))/mean(maxErrExp) )