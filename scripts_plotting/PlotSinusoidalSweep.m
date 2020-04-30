%PLOTSINUSOIDALSWEEP Plot sine sweep results
%   Written by:    J.X.J. Bannwarth, 2020/04/23
%	Last modified: J.X.J. Bannwarth, 2020/04/26

%% First initialization
close all; clc; clearvars;
ctrlName = 'FPHTFullGain';
project = simulinkproject; projectRoot = project.RootFolder;
fontSize  = 9;
outSize   = [15 8];

%% Configuration
inputFolder = fullfile( projectRoot, 'data_results', 'sine_sweep' );
filesToLoad = { 'SinusoidalSweep_FPHTFullGain_2020-04-26_19-10-59.mat', ...
                'SinusoidalSweep_baseline_2020-04-26_19-38-45.mat', ...
                'SinusoidalSweep_MIS_2020-04-26_19-55-42' };
labels = { 'FPHT', 'Baseline', 'MIS' };

%% Load data
for ii = 1:length( filesToLoad )
    load( fullfile( inputFolder, filesToLoad{ii} ) )
    for jj = 1:length( simIn )
        simInputs{ii,jj} = simIn(jj);
        simOutputs{ii,jj} = simOut(jj).logsout;
    end
    ULins{ii,1} = ULin;
    clearvars simIn simOut ULin
end

%% Process data
windF = zeros( size( simInputs ) );
windAmpl  = zeros( size( simInputs ) ); 
pitchTorqueAmpl = zeros( size( simInputs ) );
pitchTorqueMean = zeros( size( simInputs ) );
pitchDesAmpl  = zeros( size( simInputs ) );
pitchDesMean  = zeros( size( simInputs ) );
horThrustAmpl = zeros( size( simInputs ) );
horThrustMean = zeros( size( simInputs ) );

% Fit data to sin waves
ft = fittype('sin((x - shift)*xscale)*yscale+yoff','coefficients',{'shift','xscale','yscale','yoff'});

% figure( 'Name', 'Actuator Controls' ); hold on; grid on; box on
for ii = 1:length( filesToLoad )
    for jj = 1:length( simInputs(ii,:) )
        windF(ii,jj) = simInputs{ii,jj}.getVariable('Wind').Freq;
        windAmpl(ii,jj) = simInputs{ii,jj}.getVariable('Wind').Ampl(1);
        actControls = simOutputs{ii,jj}.get('actControls').Values;
        horThrust = simOutputs{ii,jj}.get('horThrustDes').Values;
        qDes = simOutputs{ii,jj}.get('qDes').Values;
        eulDes = QuatToEuler(qDes.Data);
        pitchDes = eulDes(:,2);
        
%         figure; hold on; grid on; box on
%         plot( qDes.Time, pitchDes );

        % Find mean and magnitude for desired pitch
        % Guesses
        yOffset = mean(pitchDes);
        yMin = min( pitchDes( floor(end/2):end) );
        yMax = max( pitchDes( floor(end/2):end) );
        
        % Estimate frequency by finding number of peaks
%         nPeaks = sum( diff( (movmean(pitchDes,10)-mean(pitchDes))>0 ) > 0 );
%         fEst = 2*pi*nPeaks / ( qDes.Time(end) - qDes.Time(1) );
        
        mdl = fit( qDes.Time, pitchDes, ft, ...
            'startpoint', [0, windF(ii,jj), yMax-yMin, yOffset] );
%         plot( qDes.Time, mdl.yoff+sin((qDes.Time-mdl.shift)*mdl.xscale)*mdl.yscale);
        pitchDesAmpl(ii,jj) = abs(mdl.yscale);
        pitchDesMean(ii,jj) = mdl.yoff;
        
        % Find mean and magnitude for pitch control
        % Guesses
        yOffset = mean(actControls.Data(:,2));
        yMin = min( actControls.Data( floor(end/2):end,2) );
        yMax = max( actControls.Data( floor(end/2):end,2) );
        mdl = fit( actControls.Time, actControls.Data(:,2), ft, ...
            'startpoint', [0, windF(ii,jj), yMax-yMin, yOffset] );
%         plot( actControls.Time, mdl.yoff+sin((actControls.Time-mdl.shift)*mdl.xscale)*mdl.yscale);
        pitchTorqueAmpl(ii,jj) = abs(mdl.yscale);
        pitchTorqueMean(ii,jj) = mdl.yoff;
        
        % Find mean and magnitude for horizontal thrust
        % Guesses
        yOffset = mean(horThrust.Data(:,1));
        yMin = min( horThrust.Data( floor(end/2):end,1) );
        yMax = max( horThrust.Data( floor(end/2):end,1) );
        
        mdl = fit( horThrust.Time, horThrust.Data(:,1), ft, ...
            'startpoint', [0, windF(ii,jj), yMax-yMin, yOffset] );
%         plot( horThrust.Time, mdl.yoff+sin((horThrust.Time-mdl.shift)*mdl.xscale)*mdl.yscale);
        horThrustAmpl(ii,jj) = abs(mdl.yscale);
        horThrustMean(ii,jj) = mdl.yoff;
    end
end

%% Plot response
figure( 'Name', 'Frequency response' ); hold on; grid on; box on;
pitchTorquedB = 20*log10( pitchTorqueAmpl./windAmpl );
pitchDesdB = 20*log10( pitchDesAmpl./windAmpl );
horThrustdB = 20*log10( horThrustAmpl./windAmpl );

colours = get(gca,'colororder');
for ii = 1:length( filesToLoad )
    plot( windF(ii,:)./(2*pi), pitchTorquedB(ii,:), 'Color', colours(ii,:) )
end
for ii = 1:length( filesToLoad )
    plot( windF(ii,:)./(2*pi), pitchDesdB(ii,:), '-.', 'Color', colours(ii,:) )
end
for ii = 1:length( filesToLoad )
    plot( windF(ii,:)./(2*pi), horThrustdB(ii,:), '--', 'Color', colours(ii,:) )
end
legend( labels, 'Location', 'Best' )
set( gca,'xscale','log' )

xlabel( 'Wind frequency (Hz)' )
ylabel( 'Gain (dB)' )
title( 'Solid: pitch torque, dash-dotted: des pitch, dashed: horizontal thrust' )

SetFigProp( outSize , fontSize );