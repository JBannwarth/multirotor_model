%PLOTPOSITIONHOLD Plot position hold metrics
%   Written: 2021/03/29, J.X.J. Bannwarth

%% Clean-up
clearvars -except filenameOut;
close all

%% Set-up
project     = simulinkproject;
projectRoot = project.RootFolder;
resultDir   = fullfile( projectRoot, 'data_results', 'poshold_full' );
flightLen  = 150; % [s] length of flight to analyse

if ~exist( 'filenameOut', 'var' )
    filenameOut = 'turbulent_wind';
end

%% Load simulation results
testCases = { 'baseline', 'FPHT' };
simsIn = cell( length(testCases), 1 );
simsOut = cell( length(testCases), 1 );
nFlights = 0;
for ii = 1:length( testCases )
    files =  dir( fullfile( resultDir, [ testCases{ii} '*.mat' ] ) );
    
    % Load latest
    load( fullfile( files(end).folder, files(end).name ), 'simIn', 'simOut' )
    simsIn{ii} = simIn;
    simsOut{ii} = simOut;
    nFlights = nFlights + length( simOut );
end
clearvars simIn simOut

%% Process data
% Pre-assign results table
varDefs = { ...
    'identifier'   , 'string'     , ''   ;
    'group'        , 'categorical', ''   ;
    'xLabel'       , 'categorical', ''   ;
    'xVal'         , 'double'     , 'm/s';
    'avgPosErr'    , 'double'     , 'm'  ;
    'avgPosErrNorm', 'double'     , 'm'  ;
    'rmsPosErr'    , 'double'     , 'm'  ;
    'rmsPosErrNorm', 'double'     , 'm'  ;
    'maxPosErr'    , 'double'     , 'm'  ;
    'maxPosErrNorm', 'double'     , 'm'  ;
    'avgAtt'       , 'double'     , 'deg';
    'rmsAttErr'    , 'double'     , 'deg';
    'rmsqDist'     , 'double'     , 'deg';
    'maxqDist'     , 'double'     , 'deg';
    'maxAttErr'    , 'double'     , 'deg';
    'avgPwm'       , 'double'     , 'us' ;
    'rmsPwm'       , 'double'     , 'us' ;
    'maxPwm'       , 'double'     , 'us' ;
    'minPwm'       , 'double'     , 'us' ;
    'avgRmsPwm'    , 'double'     , 'us' ;
    'fileName'     , 'string'     , ''   ;
    };

% Create table
M = table( 'Size', [nFlights size(varDefs, 1)], ...
    'VariableNames', varDefs(:,1), 'VariableTypes', varDefs(:,2) );
M.Properties.VariableUnits = varDefs(:,3);

% Cannot set dimensions right away, so go through and change column
% dimensions
isTriplet = ~contains( varDefs(:,1), {'Norm', 'qDist', 'Pwm', 'xVal'} ) & ...
    strcmp( varDefs(:,2), 'double' ) ;
isOctuplet = contains( varDefs(:,1), 'Pwm' ) & ...
    ~strcmp( varDefs(:,1), 'avgRmsPwm' );
for ii = 1:length( varDefs(:,1) )
    if isTriplet(ii)
        M.(varDefs{ii,1}) = zeros( height(M), 3 );
    elseif isOctuplet(ii)
        M.(varDefs{ii,1}) = zeros( height(M), 8 );
    end
end

% Fill table
deltaT = 0.2;
dataEx = zeros(150/deltaT + 1, 1+(6+8)*2);
idx = 0;
for ii = 1:length( simsOut )
    for jj = 1:length( simsOut{ii} )
        idx = idx+1;
        data = simsOut{ii}(jj).logsout;

        % Isolate x-axis wind speed
        M.xVal(idx) = mean(simsIn{ii}(jj).getVariable('windInput')) * [1; 0; 0];
        M.xLabel(idx) = num2str( M.xVal(idx) );
        M.identifier(idx) = sprintf( 'baseline_%04.1f', M.xVal(idx) );
        M.group(idx) = testCases{ii};
        M.fileName(idx) = files(end).name;

        % End time
        tEnd = data.getElement( 'pwm' ).Values.Time(end);
        tStart = tEnd - flightLen;
        tResample = tStart:0.1:tEnd;

        % Get and crop signals
        thr   = getsampleusingtime( data.getElement( 'thrustDes' ).Values, tStart, tEnd );
        pwm   = getsampleusingtime( data.getElement( 'pwm' ).Values, tStart, tEnd );
        omega = getsampleusingtime( data.getElement( 'omega' ).Values, tStart, tEnd );
        x = getsampleusingtime( data.getElement( 'posTrackingX' ).Values, tStart, tEnd );
        y = getsampleusingtime( data.getElement( 'posTrackingY' ).Values, tStart, tEnd );
        z = getsampleusingtime( data.getElement( 'posTrackingZ' ).Values, tStart, tEnd );
        q = getsampleusingtime( data.getElement( 'qMeasured' ).Values, tStart, tEnd );
        qDes = getsampleusingtime( data.getElement( 'qDes' ).Values, tStart, tEnd );
        qDes.Data = squeeze( qDes.Data )';
        q = resample( q, tResample, 'linear' );
        qDes = resample( qDes, tResample, 'linear' );
        
        % Calculate position error
        posErr = [ x.Data y.Data z.Data ];
        posErrNorm = sqrt( sum( posErr.^2, 2 ) );

        % Calculate attitude error
        quat     = quaternion(q.Data);
        qDes  = quaternion(qDes.Data);
        qErr  = conj(quat) .* qDes;
        qDist = dist( quat, qDes );
        attErr = eulerd( qErr, 'ZYX', 'frame' );
        attErr = attErr(:,[3 2 1]); % Switch to [roll pitch yaw]
        avgAtt = eulerd( meanrot(quat), 'ZYX', 'frame' );
        avgAtt = avgAtt(:,[3 2 1]); % Switch to [roll pitch yaw]
        
        % Get PWM
        pwmTrue = 1075 + squeeze( pwm.Data )' .* (1950-1075);

        % Position columns
        M.avgPosErr(idx,:)   = mean( posErr, 1 );
        M.avgPosErrNorm(idx) = mean( posErrNorm );
        M.rmsPosErr(idx,:)   = rms( posErr, 1 );
        M.rmsPosErrNorm(idx) = rms( posErrNorm );
        M.maxPosErr(idx,:)   = max( abs(posErr), [], 1 );
        M.maxPosErrNorm(idx) = max( posErrNorm );

        % Attitude columns
        M.avgAtt(idx,:)      = avgAtt;
        M.rmsAttErr(idx,:)   = rms( attErr, 1 );
        M.maxqDist(idx)      = rad2deg( max( qDist ) );
        M.maxAttErr(idx,:)   = max( abs(attErr), [], 1 );
        M.rmsqDist(idx)      = rad2deg( rms( qDist ) );

        % Actuator usage columns
        M.avgPwm(idx,:)      = mean( pwmTrue, 1 );
        M.rmsPwm(idx,:)      = rms( pwmTrue - mean( pwmTrue, 1 ), 1 );
        M.minPwm(idx,:)      = min( pwmTrue, [], 1 );
        M.maxPwm(idx,:)      = max( pwmTrue, [], 1 );
        M.avgRmsPwm(idx)     = mean( M.rmsPwm(idx,:) );

        % Thrust column
        M.avgThrust(idx,:)   = mean( thr );
        
        % Save data
        tResampleOut = tStart:deltaT:tEnd;
        if jj == 2
            q = resample( q, tResampleOut, 'linear' );
            eul = eulerd( quaternion(q.Data), 'ZYX', 'frame' );
            eul = eul(:,[3 2 1]);
            x = resample( x, tResampleOut, 'linear' );
            y = resample( y, tResampleOut, 'linear' );
            z = resample( z, tResampleOut, 'linear' );
            pwm = resample( pwm, tResampleOut, 'linear' );
            pwmTrue = 1075 + squeeze( pwm.Data )' .* (1950-1075);
            dataEx(:,1) = tResampleOut';
            dataEx(:,(2:15)+(ii-1)*14) = [ x.Data y.Data z.Data eul pwmTrue ];
        end
    end
end

% Export
writetable( M, fullfile( projectRoot, 'work', 'output', [filenameOut '_metrics.csv' ] ), ...
    'Delimiter', ' ');

filenameOutEx = fullfile( projectRoot, 'work', 'output', [filenameOut '_example.csv' ] );
fid = fopen( filenameOutEx, 'w' );
fprintf( fid, 't xBL yBL zBL rollBL pitchBL yawBL pwm1BL pwm2BL pwm3BL pwm4BL pwm5BL pwm6BL pwm7BL pwm8BL xVT yVT zVT rollVT pitchVT yawVT pwm1VT pwm2VT pwm3VT pwm4VT pwm5VT pwm6VT pwm7VT pwm8VT\n' );
fclose( fid )
writematrix( dataEx, filenameOutEx, 'Delimiter', ' ', 'WriteMode', 'append' );

%% Plot results
xAxisLabel  = 'Wind speed (m/s)';
groupLegend = testCases;
useIdentifier = false;

% Position metrics
PlotMetrics3( M, 'rmsPosErr', xAxisLabel, groupLegend, useIdentifier )
PlotMetrics3( M, 'maxPosErr', xAxisLabel, groupLegend, useIdentifier )

% Attitude metrics
PlotMetrics3( M, 'rmsAttErr', xAxisLabel, groupLegend, useIdentifier )
PlotMetrics3( M, 'maxAttErr', xAxisLabel, groupLegend, useIdentifier )
PlotMetrics3( M, 'avgAtt'   , xAxisLabel, groupLegend, useIdentifier )

% PWM metrics
PlotPWM( M, xAxisLabel, groupLegend, useIdentifier )

% Hover thrust
figure( 'name', 'Hover thrust' )
tiledlayout( 1, 1, 'TileSpacing', 'compact', 'Padding', 'compact' );
nexttile(1); hold on; grid on; box on
markers = 'op^dvh';
groups = string(unique( M.group ));
for ii = 1:length( groups )
    scatter( M(M.group==groups{ii},:).xVal, ...
        M(M.group==groups{ii},:).avgThrust.*100, ...
        markers(ii) )
end
xlabel( xAxisLabel )
ylabel( 'Mean hover throttle ($\%$)' )
ylim( [0 100] )
legend( groupLegend, 'location', 'best' )
FormatFigure

%% Helper
function PlotPWM( M, xAxisLabel, groupLegend, useIdentifier )
%PLOTPWM Plot PWM metrics
    % Get group names and identifiers
    groups = unique( M.group );
    xLabels = string( unique( M.xLabel ) );
    
    figure( 'name', 'PWM metrics' )
    markers = 'op^dvh';
    tiledlayout( 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact' );
    
    fieldNames = { 'avgPwm', 'rmsPwm', 'minPwm', 'maxPwm' };
    labelNames = { 'Average', 'RMS' , 'Min', 'Max' };
    
    for ii = 1:length( fieldNames )
        nexttile(ii); hold on; grid on;  box on
        for jj = 1:length( groups )
            vals = M(M.group==groups(jj),:).(fieldNames{ii});
            switch fieldNames{ii}
                case 'minPwm'
                    vals = min( vals, [], 2 );
                case 'maxPwm'
                    vals = max( vals, [], 2 );
                otherwise
                    vals = mean( vals, 2 );
            end
            scatter( M(M.group==groups(jj),:).xVal, ...
                vals, ...
                markers(jj) )
        end

        % Start at zero to give a better scale of the results
        if ~contains( fieldNames{ii}, 'rms' )
            ylim( [1000 2000] )
        end
        
        % Axis ticks and labels
        % Get unit
        idx = strcmp( M.Properties.VariableNames, fieldNames{ii} ) ;
        ylabel( sprintf( '%s PWM (%s)', labelNames{ii}, ... 
            M.Properties.VariableUnits{idx} ) )
        if useIdentifier
            xticks( 1:length(xLabels) )
            xticklabels( xLabels )
        end
    end
    
    % Formatting
    xlabel( xAxisLabel )
    legend( groupLegend, 'location', 'best' )
    linkaxes( [nexttile(1) nexttile(2) nexttile(3)], 'x' )
    FormatFigure
end

function PlotMetrics3( M, fieldName, xAxisLabel, groupLegend, useIdentifier )
%PLOTMETRICS3 Plot a 3-axis group of metrics.
    % Get group names and identifiers
    groups = unique( M.group );
    xLabels = string( unique( M.xLabel ) );
    
    % Select axes labels and create figure title
    varType = fieldName(1:3);
    varType(1) = upper(varType(1));
    if contains( fieldName, 'Att' )
        axs = {'roll', 'pitch', 'yaw'};
        varCat = 'angle';
    else
        axs = {'x', 'y', 'z'};
        varCat = 'position';
    end
    
    if contains( fieldName, 'Err' )
        suffix = ' error';
    else
        suffix = '';
    end
    
    % Initialise table
    figure( 'name', [ varType ' ' varCat suffix ] )
    markers = 'op^dvh';
    tiledlayout( 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact' );
    
    % Plot each axis
    for ii = 1:length(axs)
        nexttile( ii ); hold on; grid on; box on
        
        % Plot each group
        for jj = 1:length( groups )
            scatter( M(M.group==groups(jj),:).xVal, ...
                M(M.group==groups(jj),:).(fieldName)(:,ii), ...
                markers(jj) )
        end
        
        % Start at zero to give a better scale of the results
        if min( M.(fieldName)(:,ii) ) > 0
            ylim( [0, max(M.(fieldName)(:,ii))*1.2 ] )
        end
        
        % Axis ticks and labels
        % Get unit
        idx = strcmp( M.Properties.VariableNames, fieldName ) ;
        ylabel( sprintf( '%s %s %s%s (%s)', varType, axs{ii}, varCat, ...
            suffix, M.Properties.VariableUnits{idx} ) )
        if useIdentifier
            xticks( 1:length(xLabels) )
            xticklabels( xLabels )
        end
    end
    
    % Formatting
    xlabel( xAxisLabel )
    legend( groupLegend, 'location', 'best' )
    linkaxes( [nexttile(1) nexttile(2) nexttile(3)], 'x' )
    FormatFigure
end