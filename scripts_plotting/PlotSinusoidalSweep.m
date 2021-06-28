%PLOTSINUSOIDALSWEEP Plot sine sweep results
%   Written by:    J.X.J. Bannwarth, 2020/04/23
%	Last modified: J.X.J. Bannwarth, 2020/04/26

%% First initialization
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;
fontSize  = 9;
outSize   = [15 8];

%% Configuration
inputFolder = fullfile( projectRoot, 'data_results', 'sine_sweep' );
filesToLoad = { 'SinusoidalSweep_FPHTFullGain_2021-05-25_22-07-18';
    'SinusoidalSweep_baseline_2021-05-25_15-14-46' };
labels = {'FPHT', 'Baseline'};
debug = false;
filenameOut = 'bode_actuator_allocation';

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
aAxAmpl  = zeros( size( simInputs ) );
aAxMean  = zeros( size( simInputs ) );
aHxAmpl  = zeros( size( simInputs ) );
aHxMean  = zeros( size( simInputs ) );
horThrustAmpl = zeros( size( simInputs ) );
horThrustMean = zeros( size( simInputs ) );

% Fit data to sin waves
ft = fittype('sin((x - shift)*xscale)*yscale+yoff','coefficients',{'shift','xscale','yscale','yoff'});

% figure( 'Name', 'Actuator Controls' ); hold on; grid on; box on
for ii = 1:length( filesToLoad )
    for jj = 1:length( simInputs(ii,:) )
        windF(ii,jj) = simInputs{ii,jj}.getVariable('Wind').Freq;
        windAmpl(ii,jj) = simInputs{ii,jj}.getVariable('Wind').Ampl(1);
        
        aH = simOutputs{ii,jj}.get('TH').Values;
        aA = simOutputs{ii,jj}.get('Td').Values;
        
        if debug
            figure('Name', sprintf( 'F = %.2f Hz', windF(ii,jj) ))
            hold on; grid on; box on
            title( sprintf( 'F = %.2f Hz', windF(ii,jj) ) )
            plot( aA.Time, aA.Data(:,1) );
        end

        % Find mean and magnitude for desired attitude acceleration in x
        range = 3/4;
        % Guesses
        yOffset = mean( aA.Data( floor(end*range):end, 1 ) );
        yMin = min( aA.Data( floor(end*range):end, 1 ) );
        yMax = max( aA.Data( floor(end*range):end, 1 ) );
        
        mdl = fit( aA.Time( floor(end*range):end ), aA.Data( floor(end*range):end,1), ft, ...
            'startpoint', [0, windF(ii,jj), (yMax-yMin)/2, yOffset] );
        
        if (mdl.yscale < (yMax-yMin)/2 )
            % We know yscale is at least as large as the observed
            % difference
            mdl.yscale = (yMax-yMin)/2;
        end
        
        if debug
            plot( aA.Time, mdl.yoff+sin((aA.Time-mdl.shift)*mdl.xscale)*mdl.yscale );
        end
        aAxAmpl(ii,jj) = abs(mdl.yscale);
        aAxMean(ii,jj) = mdl.yoff;

        % Find mean and magnitude for desired attitude acceleration in z
        range = 3/4;
        % Guesses
        yOffset = mean( aA.Data( floor(end*range):end, 3 ) );
        yMin = min( aA.Data( floor(end*range):end, 3 ) );
        yMax = max( aA.Data( floor(end*range):end, 3 ) );
        
        mdl = fit( aA.Time( floor(end*range):end ), aA.Data( floor(end*range):end,3), ft, ...
            'startpoint', [0, windF(ii,jj), (yMax-yMin)/2, yOffset] );
        
        if (mdl.yscale < (yMax-yMin)/2 )
            % We know yscale is at least as large as the observed
            % difference
            mdl.yscale = (yMax-yMin)/2;
        end
        
        aAzAmpl(ii,jj) = abs(mdl.yscale);
        aAzMean(ii,jj) = mdl.yoff;
        
        % Find mean and magnitude for desired horizontal thrust
        % acceleration
        if length(aH.Time) > 1
            % Guesses
            yOffset = mean( aH.Data(floor(end*range):end, 1 ) );
            yMin = min( aH.Data( floor(end*range):end, 1 ) );
            yMax = max( aH.Data( floor(end*range):end, 1 ) );
            mdl = fit( aH.Time(floor(end*range):end), aH.Data(floor(end*range):end,2), ft, ...
                'startpoint', [0, windF(ii,jj), yMax-yMin, yOffset] );
            aHxAmpl(ii,jj) = abs(mdl.yscale);
            aHxMean(ii,jj) = mdl.yoff;
        else
            aHxAmpl(ii,jj) = nan;
            aHxMean(ii,jj) = nan;
        end
    end
    disp('Done')
end

%% Plot response
figure( 'Name', 'Frequency response' ); hold on; grid on; box on;
aAxdB = 20*log10( aAxAmpl./windAmpl );
aHxdB = 20*log10( aHxAmpl./windAmpl );
aAzdB = 20*log10( aAzAmpl./windAmpl );

colours = get(gca,'colororder');
for ii = 1:length( filesToLoad )
    plot( windF(ii,:), aAxdB(ii,:), 'Color', colours(ii,:) )
end
for ii = 1:length( filesToLoad )
    plot( windF(ii,:), aHxdB(ii,:), '-.', 'Color', colours(ii,:) )
end
for ii = 1:length( filesToLoad )
    plot( windF(ii,:), aAzdB(ii,:), ':', 'Color', colours(ii,:) )
end
legend( labels, 'Location', 'Best' )
set( gca,'xscale','log' )

xlabel( 'Wind frequency (rad/s)' )
ylabel( 'Gain (dB)' )
title( 'Solid: $a_{A,x}$, dash-dotted: $a_{H,x}$, dotted: $a_{A,z}$' )

FormatFigure( outSize , fontSize );

% Export data
header = {'w', 'aAxVT', 'aAzVT', 'aHxVT', 'aAxBL', 'aAzBL',};
data = [ windF(1,:)' aAxdB(1,:)' aAzdB(1,:)' aHxdB(1,:)' aAxdB(2,:)' aAzdB(2,:)' ];
fileOut = fullfile( projectRoot, 'work', 'output', [filenameOut '.csv' ] );
fid = fopen( fileOut, 'w' );
fprintf( fid, char(join( header, ' ' )) );
fclose( fid )
writematrix( data, fileOut, 'Delimiter', ' ', 'WriteMode', 'append' );