%PLOTEXPERIMENTALDATA Plot experimental data response
%   Written by: J.X.J. Bannwarth, 2017/08/24

%% Set-up
close all;
logFile = '2017-07-18-14-56-00_fullrate_pos_control.mat';
%logFile = '2017-07-18-13-36-00_fullrate_left_right_yaw.mat';

% Output settings - Not currently implemented
outFolder = '../multirotor_model_verification_report/fig';
fontSize  = 9;
outSize   = [8.85684 8.85684];
printResults = false;

%% Pre-process data
load( logFile )

% Check if file has position reference
if isfield( flogOriginal, 'vehicle_local_position_setpoint' )
    hasPosRef = 1;
else
    hasPosRef = 0;
end

% Get time vectors
tDesAtt = flogOriginal.vehicle_attitude_setpoint.time;
tExpAtt = flogOriginal.vehicle_attitude.time;
tExpPos = flogOriginal.vehicle_local_position.time;
tDesAtt = tDesAtt-tExpAtt(1);
tExpPos = tExpPos-tExpAtt(1);
if hasPosRef
    tDesPos = flogOriginal.vehicle_local_position_setpoint.time;
    tDesPos = tDesPos-tExpAtt(1);
end
tExpAtt = tExpAtt-tExpAtt(1);

% Get attitude vectors
qDesAtt = table2array( flogOriginal.vehicle_attitude_setpoint(:,6:9) );
qExpAtt = table2array( flogOriginal.vehicle_attitude(:,5:8) );
[ eulExp.Roll, eulExp.Pitch, eulExp.Yaw ] = QuatToEuler( qExpAtt );
[ eulDes.Roll, eulDes.Pitch, eulDes.Yaw ] = QuatToEuler( qDesAtt );
eulExp.Yaw = Unwrap( eulExp.Yaw );
eulDes.Yaw = Unwrap( eulDes.Yaw );

% Get position vectors
if hasPosRef
    posDes.x = flogOriginal.vehicle_local_position_setpoint.x;
    posDes.y = flogOriginal.vehicle_local_position_setpoint.y;
    posDes.z = flogOriginal.vehicle_local_position_setpoint.z;
end
posExp.x = flogOriginal.vehicle_local_position.x;
posExp.y = flogOriginal.vehicle_local_position.y;
posExp.z = flogOriginal.vehicle_local_position.z;

%% Plot data
ax = { 'Roll', 'Pitch', 'Yaw' };
figure( 'Name', 'Angular Response - Euler angles' )
for i = 1:length(ax)
    subplot(length(ax),1,i)
    hold on; grid on; box on;
    stairs( tDesAtt, rad2deg( eulDes.(ax{i}) ) )
    stairs( tExpAtt, rad2deg( eulExp.(ax{i}) ) )
    xlim( [0 inf] )
    %ylim( [-inf inf] )
    xlabel( 'Time (s)', 'interpreter', 'latex' )
    ylabel( [ ax{i} ' (deg) ' ], 'interpreter', 'latex' )
    legend( { 'Des', 'Exp' }, 'location', 'southeast', 'interpreter', 'latex')
end

ax = { 'x', 'y', 'z' };
figure( 'Name', 'Angular Response - Euler angles' )
for i = 1:length(ax)
    subplot(length(ax),1,i)
    hold on; grid on; box on;
    if hasPosRef
        stairs( tDesPos, posDes.(ax{i}) )
    end
    stairs( tExpPos, posExp.(ax{i}) )
    xlim( [0 inf] )
    %ylim( [-inf inf] )
    xlabel( 'Time (s)', 'interpreter', 'latex' )
    ylabel( [ '$' ax{i} '$-axis position (m) ' ], 'interpreter', 'latex' )
    if hasPosRef
        legend( { 'Des', 'Exp' }, 'location', 'southeast', 'interpreter', 'latex')
    end
end