%PREPAREATTITUDESTEPDATA Prepare step data for attitude validation
%   Written by: J.X.J. Bannwarth, 2017/08/21

% Setup
load( '2017-07-18-13-36-00_fullrate_back_forth.mat' );
clear( 'dataIn' );

tDes    = dataInOriginal.vehicle_attitude_setpoint.time;
qDes = table2array( dataInOriginal.vehicle_attitude_setpoint(:,6:9) );
eulerTmp = QuatToEuler( qDes );
eulDes.Roll = eulerTmp(:,1);
eulDes.Pitch = eulerTmp(:,2);
eulDes.Yaw = eulerTmp(:,3);
thrustDes = dataInOriginal.vehicle_attitude_setpoint.thrust;
yawRateDes = dataInOriginal.vehicle_attitude_setpoint.yaw_sp_move_rate;

t    = dataInOriginal.vehicle_attitude.time;
q = table2array( dataInOriginal.vehicle_attitude(:,5:8) );
eulerTmp = QuatToEuler( q );
eul.Roll = eulerTmp(:,1);
eul.Pitch = eulerTmp(:,2);
eul.Yaw = eulerTmp(:,3);

tDes = tDes - t(1);
t = t - t(1);

ind = 1;
for ax = { 'Roll', 'Pitch', 'Yaw' }
    subplot( 3, 1, ind )
    hold on; grid on; box on;
    stairs( tDes, rad2deg( eulDes.(char(ax)) ) )
    stairs( t, rad2deg( eul.(char(ax)) ) )
    xlabel( 'Time (s)' )
    ylabel( [ ax ' (deg) ' ] )
    legend( { 'Desired', 'Measured' } )
    ind = ind + 1;
end

qDesInput = [ tDes qDes ];
thrustDesInput = [ tDes thrustDes ];
yawRateDesInput = [ tDes yawRateDes ];