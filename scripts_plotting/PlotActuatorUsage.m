%PLOTACTUATORUSAGE Compute and plot actuator usage metrics
%   Input:
%       - resultFile: string or cell array of result files to analyse. If
%                     not defined, use most recent file.
%       - windFile:   if the result files contain result for several wind
%                     files, selects which to use.
%
%   See also HORTHRUST_POSHOLD.
%
%   Written: 2020/12/09, J.X.J. Bannwarth

%% Clean-up and set-up
clearvars -except resultFile windFile
close all
project = simulinkproject; projectRoot = project.RootFolder;

%% Input processing
resultFolder = fullfile( projectRoot, 'data_results', 'pos_hold' );
windFolder = fullfile( projectRoot, 'data_wind', 'TurbSimOC' );

if ~exist( 'resultFile', 'var' )
    % Select latest baseline file and latest other file
    resultFile{1} = GetLatestFile( resultFolder, '*baseline*.mat' );
    resultFile{2} = GetLatestFile( resultFolder, '*.mat', 'baseline' );
    if strcmp( resultFile{1}, resultFile{2} )
        % Latest file is latest baseline
        resultFile = resultFile{1};
    end
end

% Get legends
legendStr = split( resultFile, '_' );
if size( legendStr, 2) == 1
    legendStr = legendStr(2); % 1 file
else
    legendStr = legendStr(:,:,2); % > 1 file
end

% Set colors for comparison plots
colors = lines( length(legendStr) );

if ~exist( 'windFile', 'var' )
    % Select highest wind speed
    windFile = 'TurbSim_40_01';
end

% Strip extensions
[~, windFile, ~] = fileparts( windFile );

if ~iscell( resultFile )
    % To prevent errors when calling resultFile{ii}
    resultFile = {resultFile};
end

% Maximum filename length
maxLen = max( cellfun( @(s)(length(s)), resultFile ) );

%% Process results
% Figure handles
hXi   = figure( 'Name', 'xi' );
hAct  = figure( 'Name', 'Actuator usage' );
hSpec = figure( 'Name', 'Spectrum' );

% Process
for ii = 1:length( resultFile )
    %% Load data and select desired response
    load( fullfile( resultFolder, resultFile{ii} ), 'ULin', 'simIn', 'simOut'  )
    fprintf( sprintf('[%% %ds] ', maxLen), resultFile{ii} );

    analysedWindFiles = simIn.getVariable( 'windFile' );
    % Strip extensions
    [~, analysedWindFiles, ~] = fileparts( analysedWindFiles );
    if ~iscell(analysedWindFiles)
        analysedWindFiles = {analysedWindFiles};
    end
    
    % Check the desired wind file exists
    idx = find( ismember( analysedWindFiles, windFile ) );
    if isempty( idx )
        error( [ 'The result file does not contain results for the desired ' ...
            'speed' ] )
    end

    simOut = simOut(idx);
    simIn = simIn(idx);
    ULin = ULin(idx, :);

    %% Process data
    logsout = simOut.get('logsout');
    
    % Calculate actuator metrics
    pwmData = logsout.get('realPwm').Values;
    t = pwmData.Time;
    pwm = pwmData.Data;

    pwmStd = std( pwm, 1 );

    % Print standard deviations of PWM signals
    fprintf( 'PWM std (us):' )
    for jj = 1:length(pwmStd)
        fprintf( ' [%d]% 6.2f,', jj, pwmStd(jj) )
    end
    fprintf( ' [mean]% 6.2f, [min]% 6.2f, [max]% 6.2f\n', mean( pwmStd ), ...
        min(pwmStd), max(pwmStd) )
    
    % Calculate position metrics
    xiRms = rms( logsout.get( 'xi' ).Values.Data );
    xiMax = max( logsout.get( 'xi' ).Values.Data );
    
    % Print position metrics
    fprintf( sprintf('[%% %ds] ', maxLen), resultFile{ii} );
    fprintf( 'Pos error (m): [rms] (%.4f, %.4f %.4f) [max] (%.4f, %.4f %.4f)\n', ...
        xiRms(1), xiRms(2), xiRms(3), xiMax(1), xiMax(2), xiMax(3) );
    
    % Calculate actuator frequency usage
    % Attitude thrust
    attThrust = logsout.get( 'Td' ).Values.Data;
    for jj = 1:3
        [ fVert{jj}, P1Vert{jj} ] = SingleSidedSpectrum( t, attThrust(:,jj) );
    end
    
    % Horizontal thrust
    if ~strcmp( legendStr{ii}, 'baseline' )
        horThrust = logsout.get( 'horThrustDes' ).Values.Data;
        [ fx, P1x ] = SingleSidedSpectrum( t, horThrust(:,1) );
        [ fy, P1y ] = SingleSidedSpectrum( t, horThrust(:,2) );
    end
    
    %% Plot data
    figSize = [15 15];
    fontSize = 12;

    figure( 'name', sprintf( 'PWM signals - %s', resultFile{ii} ) )
    plot( t, pwm )
    
    % Format
    xlabel( 'Time (s)' )
    ylabel( 'PWM ($\mu$s)' )
    legend( compose( 'Rotor %d', 1:size(pwm, 2) ), 'location', 'best' )
    title( sprintf( '$\\bar{\\sigma}_\\mathrm{PWM} = %.2f \\,\\mu$s', ...
        mean( pwmStd ) ) )
    FormatFigure( figSize, fontSize )
    
    % Plot position error
    set( 0, 'CurrentFigure', hXi )
    xi = logsout.get('xi').Values;
    axsLong = {'North, x', 'East, y', 'Down, z'};
    for jj = 1:3
        subplot( 3, 1, jj )
        hold on; grid on; box on
        plot( xi.Time, xi.Data(:,jj) )
        ylabel( sprintf( '%s pos (m)', axsLong{jj} ) )
    end
    xlabel( 'Time (s)' )
    
    % Plot actuators
    set( 0, 'CurrentFigure', hAct )
    axs = {'x', 'y', 'z'};
    for jj = 1:3
        hActSub{jj} = subplot( 5, 1, jj );
        hold on; grid on; box on
        plot(t, attThrust(:,jj) )
        ylabel( sprintf( 'T_%s (-)', axs{jj} ) )
    end
    if exist( 'fx', 'var' )
        subplot( 5, 1, 4 )
        hold on; grid on; box on;
        plot( t, horThrust(:,1), 'Color', colors(ii,:) )
        ylabel( 'T_{x,H} (-)' )
        
        subplot( 5, 1, 5 )
        hold on; grid on; box on;
        plot( t, horThrust(:,2), 'Color', colors(ii,:) )
        ylabel( 'T_{y,H} (-)' )
    end
    xlabel( 'Time (s)' )
    
    % Plot frequency usage
    set( 0, 'CurrentFigure', hSpec )
    for jj = 1:3
        hSpecSub{jj} = subplot( 5, 1, jj );
        hold on; grid on; box on
        plot( fVert{jj}, P1Vert{jj}, 'Color', colors(ii,:) )
        ylabel( sprintf( '|T_%s| (-)', axs{jj} ) )
    end
    if exist( 'fx', 'var' )
        subplot( 5, 1, 4 )
        hold on; grid on; box on;
        plot( fx, P1x, 'Color', colors(ii,:) )
        ylabel( '|T_{x,H}| (-)' )
        
        subplot( 5, 1, 5 )
        hold on; grid on; box on;
        plot( fy, P1y, 'Color', colors(ii,:) )
        ylabel( '|T_{y,H}| (-)' )
        clearvars fx P1x fy P1y
    end
    xlabel( 'Frequency (Hz)' )
end

% Add legends
set( 0, 'CurrentFigure', hXi )
legend( legendStr )
set( 0, 'CurrentFigure', hSpec )
legend( hSpecSub{1}, legendStr )
set( 0, 'CurrentFigure', hAct )
legend( hActSub{1}, legendStr )