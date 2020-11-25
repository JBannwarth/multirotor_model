%APPROXIMATELPE Approximate LPE using transfer functions
%   Written by:    J.X.J. Bannwarth, 2020/11/24
%% Set-up
clearvars; close all;
project = simulinkproject; projectRoot = project.RootFolder;

%% Load data
load( fullfile( projectRoot, 'data_results', ...
    'HoverSim_2020-11-25_11-24-29_OC', 'TurbSim_30_01.mat' ) )

%% Extract relevant signals
tS = output.SimulationMetadata.UserData.Simulation.T_S;
logs = output.get('logsout');
signals = { 'eta'; 'nuBody'; 'xi'; 'xiDot' };
signalsMeas = strcat( signals, 'Measured' );
units = { 'rad'; 'rad/s'; 'm'; 'm/s' };
axes = { '_x'; '_y'; '_z' };

%% Find transfer functions
for ii = 1:length(signals)
    for ax = 1:3
        vals = logs.get(signals{ii}).Values.Data(:,ax);
        valsMeas = logs.get(signalsMeas{ii}).Values.Data(:,ax);
        
        if length( valsMeas ) < length( vals )
            [~, idx, ~] = intersect( ...
                tS*round( (1/tS)*logs.get(signals{ii}).Values.Time ), ...
                tS*round( (1/tS)*logs.get(signalsMeas{ii}).Values.Time ) );
            vals = vals( idx );
        end
        tSCur = median( diff( logs.get(signalsMeas{ii}).Values.Time ) );
        ioData = iddata( vals, valsMeas, tSCur, ...
            'InputUnit', units{ii}, ...
            'OutputUnit', units{ii}, ...
            'InputName', [signals{ii} axes{ax}], ...
            'OutputName', [signalsMeas{ii} axes{ax}] );
        est{ii,ax} = tfest( ioData, 1, 0 );
        if est{ii,ax}.Report.Fit.FitPercent < 80
            est{ii,ax} = tfest( ioData, 1, 1 );
        end
        fprintf( '%s | Fit: %.1f%%\n', [signals{ii} axes{ax}], ...
            est{ii,ax}.Report.Fit.FitPercent )
    end
end