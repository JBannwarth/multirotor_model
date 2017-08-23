%PREPAREATTITUDESTEPDATA Prepare step data for attitude validation
%   Written by: J.X.J. Bannwarth, 2017/08/21

% Setup
% load( '2017-07-18-13-36-00_fullrate_back_forth.mat' );
% % load( '2017-07-18-13-36-00_fullrate_left_right_yaw' );
% clear( 'dataIn' );
toPlot = false;
% 
% % Extract useful data
% tDes    = dataInOriginal.vehicle_attitude_setpoint.time;
% qDes = table2array( dataInOriginal.vehicle_attitude_setpoint(:,6:9) );
% [ eulDes.Roll, eulDes.Pitch, eulDes.Yaw ] = QuatToEuler( qDes );
% thrustDes = dataInOriginal.vehicle_attitude_setpoint.thrust;
% yawRateDes = dataInOriginal.vehicle_attitude_setpoint.yaw_sp_move_rate;
% 
% tExp = dataInOriginal.vehicle_attitude.time;
% qExp = table2array( dataInOriginal.vehicle_attitude(:,5:8) );
% [ eul.Roll, eul.Pitch, eul.Yaw ] = QuatToEuler( qExp );

tDes = tDes - tExp(1);
tExp = tExp - tExp(1);

% Plot data if required
if toPlot
    ind = 1;
    for ax = { 'Roll', 'Pitch', 'Yaw' }
        subplot( 3, 1, ind )
        hold on; grid on; box on;
        stairs( tDes, rad2deg( eulDes.(char(ax)) ) )
        stairs( tExp, rad2deg( eulExp.(char(ax)) ) )
        xlim( [0 inf] )
        ylim( [-inf inf] )
        xlabel( 'Time (s)' )
        ylabel( [ ax ' (deg) ' ] )
        legend( { 'Desired', 'Measured' } )
        ind = ind + 1;
    end
end

% Add offset to give estimator the time to converge and format data for sim
tDesOffset = 0;%60;
qDesInput = [ tDes+tDesOffset, qDes ];
thrustDesInput = [ tDes+tDesOffset, thrustDes ];
yawRateDesInput = [ tDes+tDesOffset, yawRateDes ];