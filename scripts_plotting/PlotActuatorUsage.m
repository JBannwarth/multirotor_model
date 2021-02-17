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
    % Select latest file
    resultFile = GetLatestFile( resultFolder, '*.mat' );
end

if ~exist( 'windFile', 'var' )
    % Select highest wind speed
    windFile = 'TurbSim_40_01.mat';
end

if ~iscell( resultFile )
    % To prevent errors when calling resultFile{ii}
    resultFile = {resultFile};
end

% Maximum filename length
maxLen = max( cellfun( @(s)(length(s)), resultFile ) );

%% Process results
for ii = 1:length( resultFile )
    %% Load data and select desired response
    load( fullfile( resultFolder, resultFile{ii} ), 'ULin', 'simIn', 'simOut'  )
    fprintf( sprintf('[%% %ds] ', maxLen), resultFile{ii} )

    windFiles = simIn.getVariable( 'windFile' );
    idx = find( ismember( windFiles, windFile ) );
    if isempty( idx )
        error( [ 'The result file does not contain results for the desired ' ...
            'speed' ] )
    end

    simOut = simOut(idx);
    simIn = simIn(idx);
    ULin = ULin(idx, :);

    %% Process data
    logsout = simOut.get('logsout');
    pwmData = logsout.get('realPwm').Values;
    t = pwmData.Time;
    pwm = pwmData.Data;

    pwmStd = std( pwm, 1 );

    % Print standard deviations
    fprintf( 'PWM std (us):' )
    for jj = 1:length(pwmStd)
        fprintf( ' [%d]% 6.2f,', jj, pwmStd(jj) )
    end
    fprintf( ' [mean]% 6.2f, [min]% 6.2f, [max]% 6.2f\n', mean( pwmStd ), ...
        min(pwmStd), max(pwmStd) )

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
    SetFigProp( figSize, fontSize )
end