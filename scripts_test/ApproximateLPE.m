%APPROXIMATELPE Approximate LPE using transfer functions
%   Written by:    J.X.J. Bannwarth, 2020/11/24
%% Set-up
clearvars; close all;
project = simulinkproject; projectRoot = project.RootFolder;

%% Load data
load( fullfile( projectRoot, 'data_results', ...
    'HoverSim_2020-11-24_16-37-57_OC', 'TurbSim_25_01.mat' ) )

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
            [~, idx, ~] = intersect( logs.get(signals{ii}).Values.Time, ...
                logs.get(signalsMeas{ii}).Values.Time );
            vals = vals( idx );
        end

        ioData = iddata( vals, valsMeas, tS, ...
            'InputUnit', units{ii}, ...
            'OutputUnit', units{ii}, ...
            'InputName', [signals{ii} axes{ax}], ...
            'OutputName', [signalsMeas{ii} axes{ax}] );
        est{ii,ax} = tfest( ioData, 1, 0 );
    end
end