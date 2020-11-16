%PLOTSTEPDIST Plot wind gust response results
%   Assume:
%       - All loaded files contain results for the same wind step sizes.
%       - Wind steps are in x-direction only.
%   Written by: J.X.J. Bannwarth, 2020/11/16

%% First initialization
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;
fontSize  = 9;
outSize   = [15 8];

%% Configuration
inputFolder = fullfile( projectRoot, 'data_results', 'step_dist' );
filesToLoad = { ...
    'StepDist_baseline_2020-11-16_16-12-31';
    'StepDist_FPHTFullGain_2020-11-16_15-17-54';
    };
labels = { ...
    'Baseline';
    'FPHT';
    };

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
for ii = 1:size(simInputs, 2)
    UInit(ii,:) = simInputs{1,ii}.getVariable('Wind').StepInit;
    UFinal(ii,:) = simInputs{1,ii}.getVariable('Wind').StepFinal;
end

UInitX = UInit(:,1);
UFinalX = UFinal(:,1);

%% Plot response
figH = zeros( length(UFinalX), 2 );
axH = zeros( length(UFinalX), 3, 2 );
axs = {'x', 'y', 'z'};

% Create actuator figures and add subplot handles
for ii = 1:length( UFinalX )
    figH(ii,1) = figure( 'Name', sprintf('Actuator usage (%.1f to %.1f)', ...
        UInitX(ii), UFinalX(ii)) );
    axH(ii,1,1) = gca;
    hold on; grid on; box on;
    xlabel( 'Time (s)' )
    ylabel( 'Normalised PWM (-)' )
end
% Create output figures and add subplot handles
for ii = 1:length( UFinalX )
    figH(ii,2) = figure( 'Name', sprintf('Position response (%.1f to %.1f)', ...
        UInitX(ii), UFinalX(ii)) );
    for jj = 1:3
        axH(ii,jj,2) = subplot(3, 1, jj);
        hold on; grid on; box on;
        xlabel( 'Time (s)' )
        ylabel( sprintf('$%s$-axis position (m)', axs{jj}) )
    end
end

%% Plot all data
for ii = 1:length( UFinalX )
    colours = get(gca,'colororder');
    for jj = 1:length( filesToLoad )
        % Input
        pwm = simOutputs{jj,ii}.get('pwm').Values.Data;
        pwm = reshape(pwm, size(pwm, 3), size(pwm, 1) );
        plot( axH(ii,1,1), simOutputs{jj,ii}.get('xi').Values.Time, ...
            max(pwm,[],2), 'color', colours(jj,:) );
        plot( axH(ii,1,1), simOutputs{jj,ii}.get('xi').Values.Time, ...
            min(pwm,[],2), 'color', colours(jj,:) );
        plot( axH(ii,1,1), simOutputs{jj,ii}.get('xi').Values.Time, ...
            mean(pwm,2), '--', 'color', colours(jj,:) );
        
        % Output
        for kk = 1:3
            plot( axH(ii,kk,2), simOutputs{jj,ii}.get('xi').Values.Time, ...
                simOutputs{jj,ii}.get('xi').Values.Data(:,kk) )
        end
    end
end

%% Add legends and finish formatting
for ii = 1:length(figH)
    figure( figH(ii) )
    legend( axH(ii,end), labels, 'Location', 'Best' )
    SetFigProp( outSize , fontSize );
end