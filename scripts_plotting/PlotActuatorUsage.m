%PLOTACTUATORUSAGE Compute and plot actuator usage metrics
%   Written by: J.X.J. Bannwarth, 2020/12/09

%% Clean-up and set-up
clearvars -except resultFile windFile
clc; close all;
project = simulinkproject; projectRoot = project.RootFolder;

%% Input processing
resultFolder = fullfile( projectRoot, 'data_results', 'pos_hold' );
windFolder = fullfile( projectRoot, 'data_wind', 'TurbSimOC' );

if ~exist( 'resultFile', 'var' )
    % Select latest file
    allResultFiles = dir(resultFolder);
    allResultFiles = {allResultFiles.name};
    % Naming convention is chronological when sorted in ASCII order
    resultFile = allResultFiles{end};
end

if ~exist( 'windFile', 'var' )
    windFile = 'TurbSim_40_01.mat';
end

%% Load data and select desired response
load( fullfile( resultFolder, resultFile ), 'ULin', 'simIn', 'simOut'  )

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

fprintf( 'Standard dev. PWM values (us):' )
for ii = 1:length(pwmStd)
    fprintf( ' [%d] %.2f,', ii, pwmStd(ii) )
end
fprintf( ' [mean] %.2f, [min] %.2f, [max] %.2f\n', mean( pwmStd ), ...
    min(pwmStd), max(pwmStd) )

%% Plot data
figSize = [15 15];
fontSize = 12;

figure( 'name', 'PWM signals' )
plot( t, pwm )
xlabel( 'Time (s)' )
ylabel( 'PWM ($\mu$s)' )
legend( compose( 'Rotor %d', 1:size(pwm, 2) ), 'location', 'best' )
title( sprintf( '$\\bar{\\sigma}_\\mathrm{PWM} = %.2f \\,\\mu$s', mean( pwmStd ) ) )
SetFigProp( figSize, fontSize )