%COMPAREALLSIGNALS Compare all signals between exp results and sim results
%   Written by:    J.X.J. Bannwarth, 2017/10/31
%   Last modified: J.X.J. Bannwarth, 2017/10/31

%% List signals to plot
fileId = fopen ( 'SimPX4ParamCorrespondence.tsv' );
fgets( fileId ); % skip headers
tmp = textscan( fileId, '%s %s', 'Delimiter', '\t' );
fclose( fileId );
strSim = tmp{1}; strPx4 = tmp{2};

for i = 1:length(strSim)
    curStr = strSim{i};
    % Separate names of signals that are to be bundled together as an array
    %  - they are denoted by commas. E.g. signal1,signal2,signal3,signal4
    % Support up to 10 signals
    signalToBundle = textscan( curStr, '%s %s %s %s %s %s %s %s %s %s', 'Delimiter', ',' );
    for j = 1:size(signalToBundle, 2)
        if ( isempty(signalToBundle{j}) )
            break;
        else
            % All signals should be in form messageName.signalName, but
            % support up to a.b.c.d to be safe
            nameComponent = textscan( char(signalToBundle{j}), '%s %s %s %s', 'Delimiter', '.' );
            for k = 1:size(nameComponent, 2)
                if ( isempty(nameComponent{k}) )
                    break;
                else
                    signalsSim(i,j,k) = string(nameComponent{k});
                end
            end
        end
    end
end
clearvars signalToBundle nameComponent

for i = 1:length(strPx4)
    curStr = strPx4{i};
    % Separate names of signals that are to be bundled together as an array
    %  - they are denoted by commas. E.g. signal1,signal2,signal3,signal4
    % Support up to 10 signals
    signalToBundle = textscan( curStr, '%s %s %s %s %s %s %s %s %s %s', 'Delimiter', ',' );
    for j = 1:size(signalToBundle, 2)
        if ( isempty(signalToBundle{j}) )
            break;
        else
            % All signals should be in form messageName.signalName, but
            % support up to a.b.c.d to be safe
            nameComponent = textscan( char(signalToBundle{j}), '%s %s %s %s', 'Delimiter', '.' );
            for k = 1:size(nameComponent, 2)
                if ( isempty(nameComponent{k}) )
                    break;
                else
                    signalsExp(i,j,k) = string(nameComponent{k});
                end
            end
        end
    end
end
clearvars -except signalsSim signalsExp

%% Load experimental and simulation data
filename = 'step_att_small_pitch-5_1';
load( [filename '.mat'], 'flogOriginal')
load( [filename 'Sim.mat'])
simLogs = output.get('logsout');
clearvars output

% Put exp data in same format as sim
for i = 1:size(signalsExp, 1)
    data = [];
    for j = 1:size(signalsExp,2)
        data
    end
end