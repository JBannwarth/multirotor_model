%PLOTSTEPDIST Plot wind gust response results
%   Assume:
%       - All loaded files contain results for the same wind step sizes.
%       - Wind steps are in x-direction only.
%       - Load latest files only
%   Written by: J.X.J. Bannwarth, 2020/11/16

%% First initialization
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;
fontSize  = 9;
outSize   = [15 8];

%% Configuration
inputFolder = fullfile( projectRoot, 'data_results', 'step_dist' );
prefixToLoad = { ...
    'StepDist_baseline_*';
    'StepDist_FPHTFullGain_*';
    };
labels = { ...
    'Baseline';
    'FPHT';
    };

%% Load data
for ii = 1:length( prefixToLoad )
    matchingFiles = dir( fullfile( inputFolder, prefixToLoad{ii} ) );
    % Load latest file
    load( fullfile( inputFolder, matchingFiles(end).name ) )
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
figH = zeros( length(UFinalX), 3 );
% axH = zeros( length(UFinalX), 3, 3 );
axs = { 'x', 'y', 'z' };
lineStyles = { '-', '--', ':' };
eulerAxs = { 'Roll', 'Pitch' };

% Create actuator figures and add subplot handles
for ii = 1:length( UFinalX )
    figH(ii,1) = figure( 'Name', sprintf('Actuator usage (%.1f to %.1f)', ...
        UInitX(ii), UFinalX(ii)) );
    axH{ii,1,1} = gca;
    hold on; grid on; box on;
    xlabel( 'Time (s)' )
    ylabel( 'Normalised PWM (-)' )
end

% Create position figures and add subplot handles
for ii = 1:length( UFinalX )
    figH(ii,2) = figure( 'Name', sprintf('Position response (%.1f to %.1f)', ...
        UInitX(ii), UFinalX(ii)) );
    for jj = 1:3
        axH{ii,jj,2} = subplot(3, 1, jj);
        hold on; grid on; box on;
        xlabel( 'Time (s)' )
        ylabel( sprintf('$%s$-axis position (m)', axs{jj}) )
    end
end

% Create controller output figures and add subplot handles
for ii = 1:length( UFinalX )
    figH(ii,3) = figure( 'Name', sprintf('Commands (%.1f to %.1f)', ...
        UInitX(ii), UFinalX(ii)) );
    for jj = 1:2
        axH{ii,jj,3} = subplot(3, 1, jj);
        hold on; grid on; box on;
        xlabel( 'Time (s)' )
        yyaxis( axH{ii,jj,3}, 'left' )
        ylabel( sprintf('%s (deg)', eulerAxs{3-jj}) )
        yyaxis( axH{ii,jj,3}, 'right' )
        ylabel( sprintf('Norm $%s$-axis hor. thrust (--)', axs{jj}) )
    end
    axH{ii,3,3} = subplot(3, 1, 3);
    hold on; grid on; box on;
    xlabel( 'Time (s)' )
    ylabel( 'Norm. vertical thrust (--)' )
    
end

%% Plot all data
for ii = 1:length( UFinalX )
    colours = get(gca,'colororder');
    for jj = 1:length( prefixToLoad )
        % Input
        pwm = simOutputs{jj,ii}.get('realPwm').Values.Data;
%         pwm = reshape(pwm, size(pwm, 3), size(pwm, 1) );
        plot( axH{ii,1,1}, simOutputs{jj,ii}.get('xi').Values.Time, ...
            max(pwm,[],2), 'color', colours(jj,:) );
        plot( axH{ii,1,1}, simOutputs{jj,ii}.get('xi').Values.Time, ...
            min(pwm,[],2), 'color', colours(jj,:) );
        plot( axH{ii,1,1}, simOutputs{jj,ii}.get('xi').Values.Time, ...
            mean(pwm,2), '--', 'color', colours(jj,:) );
        
        % Output
        for kk = 1:3
            plot( axH{ii,kk,2}, simOutputs{jj,ii}.get('xi').Values.Time, ...
                simOutputs{jj,ii}.get('xi').Values.Data(:,kk) )
        end
        
        % Inputs
        for kk = 1:2
            qDes = simOutputs{jj,ii}.get('qDes').Values.Data;
            eulerDes = rad2deg( QuatToEuler( qDes ) );
            yyaxis( axH{ii,kk,3}, 'left' )
            plot( axH{ii,kk,3}, ...
                simOutputs{jj,ii}.get('qDes').Values.Time, ...
                eulerDes(:,3-kk), lineStyles{jj} )

            yyaxis( axH{ii,kk,3}, 'right' )
            plot( axH{ii,kk,3}, ...
                simOutputs{jj,ii}.get('horThrustDes').Values.Time, ...
                simOutputs{jj,ii}.get('horThrustDes').Values.Data(:,kk), ...
                lineStyles{jj} )
        end
        plot( axH{ii,3,3}, ...
                simOutputs{jj,ii}.get('thrustDes').Values.Time, ...
                simOutputs{jj,ii}.get('thrustDes').Values.Data, ...
                lineStyles{jj}, 'color', colours(1,:) )
    end
end

%% Add legends and finish formatting
for ii = 1:size(figH,2)
    % Actuator
    figure( figH(ii,1) )
    legend( axH{ii,1,1}, labels, 'Location', 'Best' )
    SetFigProp( outSize , fontSize );
    
    % Position
    figure( figH(ii,2) )
    legend( axH{ii,2,1}, labels, 'Location', 'Best' )
    SetFigProp( outSize , fontSize );
    
    % Controller outputs
    figure( figH(ii,3) )
    legend( axH{ii,3,3}, labels, 'Location', 'Best' )
    SetFigProp( outSize , fontSize );
    
    % Fix zero
    for jj = 1:2
        set( figH(ii,3), 'CurrentAxes', axH{ii,jj,3} )
        yyaxis( axH{ii,jj,3}, 'left' )
        yLim1 = get(gca, 'Ylim');
        [~, maxIdx1] = max( abs(yLim1) );
        yLim1(3-maxIdx1) = -yLim1(maxIdx1);
        set( gca, 'Ylim', yLim1 )
        
        yyaxis( axH{ii,jj,3}, 'right' )
        yLim2 = get( gca, 'Ylim' );
        [~, maxIdx2] = max( abs(yLim2) );
        yLim2(3-maxIdx2) = -yLim2(maxIdx2);
        set( gca, 'Ylim', yLim2 )
    end
end