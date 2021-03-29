%PLOTPOSITIONHOLD Plot position hold metrics
%   Written: 2021/03/29, J.X.J. Bannwarth

%% Clean-up
clearvars;
close all

%% Set-up
project     = simulinkproject;
projectRoot = project.RootFolder;
resultDir   = fullfile( projectRoot, 'data_results', 'poshold_full' );
flightLen  = 50; % [s] length of flight to analyse

%% Load simulation results
files = dir( resultDir );
files = files( ~[files.isdir] );

% Load latest results
load( fullfile( files(end).folder, files(end).name ), 'simIn', 'simOut' )

%% Process data
% Pre-assign results table
varDefs = { ...
    'identifier'   , 'string'     , ''   ;
    'group'        , 'categorical', ''   ;
    'xLabel'       , 'double'     , 'm/s';
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
    'fileName'     , 'string'     , ''   ;
    };

% Create table
M = table( 'Size', [length(simOut) size(varDefs, 1)], ...
    'VariableNames', varDefs(:,1), 'VariableTypes', varDefs(:,2) );
M.Properties.VariableUnits = varDefs(:,3);

% Cannot set dimensions right away, so go through and change column
% dimensions
isTriplet = ~contains( varDefs(:,1), {'Norm', 'qDist', 'Pwm', 'xLabel'} ) & ...
    strcmp( varDefs(:,2), 'double' ) ;
isOctuplet = contains( varDefs(:,1), 'Pwm' );
for ii = 1:length( varDefs(:,1) )
    if isTriplet(ii)
        M.(varDefs{ii,1}) = zeros( height(M), 3 );
    elseif isOctuplet(ii)
        M.(varDefs{ii,1}) = zeros( height(M), 8 );
    end
end

for ii = 1:length( simOut )
    data = simOut(ii).logsout;
    
    % Isolate x-axis wind speed
    M.xLabel(ii) = mean(simIn(ii).getVariable('windInput')) * [1; 0; 0];
    M.identifier(ii) = sprintf( 'baseline_%04.1f', M.xLabel(ii) );
    M.group(ii) = 'baseline';
    
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
    q     = quaternion(q.Data);
    qDes  = quaternion(qDes.Data);
    qErr  = conj(q) .* qDes;
    qDist = dist( q, qDes );
    attErr = eulerd( qErr, 'ZYX', 'frame' );
    attErr = attErr(:,[3 2 1]); % Switch to [roll pitch yaw]
    avgAtt = eulerd( meanrot(q), 'ZYX', 'frame' );
    avgAtt = avgAtt(:,[3 2 1]); % Switch to [roll pitch yaw]
    
    % Get PWM
    pwm = squeeze( pwm.Data )';
    
    % Position columns
    M.avgPosErr(ii,:)   = mean( posErr, 1 );
    M.avgPosErrNorm(ii) = mean( posErrNorm );
    M.rmsPosErr(ii,:)   = rms( posErr, 1 );
    M.rmsPosErrNorm(ii) = rms( posErrNorm );
    M.maxPosErr(ii,:)   = max( abs(posErr), [], 1 );
    M.maxPosErrNorm(ii) = max( posErrNorm );

    % Attitude columns
    M.avgAtt(ii,:)      = avgAtt;
    M.rmsAttErr(ii,:)   = rms( attErr, 1 );
    M.maxqDist(ii)      = rad2deg( max( qDist ) );
    M.maxAttErr(ii,:)   = max( abs(attErr), [], 1 );
    M.rmsqDist(ii)      = rad2deg( rms( qDist ) );
    
    % Actuator usage columns
    M.avgPwm(ii,:)      = mean( pwm, 1 );
    M.rmsPwm(ii,:)      = rms( pwm - mean( pwm, 1 ), 1 );
    M.minPwm(ii,:)      = min( pwm, [], 1 );
    M.maxPwm(ii,:)      = max( pwm, [], 1 );
    
    % Thrust column
    M.avgThrust(ii,:)   = mean( thr );
end