%PLOTALLSTEPDATA Plot all step data corresponding to prefix
%   Written by: J.X.J. Bannwarth, 11/09/2017

%% Setup
prefix = 'att_step_small';
close all;

%% Get list of files to plot
files = dir('data_validation');
files = {files.name};
toRemove = zeros( size(files) );
for i = 1:length(files)
    if isempty( strfind( files{i}, prefix ) )
        toRemove(i) = 1;
    end
end
files( logical(toRemove) ) = [];

%% Plot files
for i = 1:length(files)
    load( files{i} )
    quat = [ flogOriginal.vehicle_attitude.q_0_, flogOriginal.vehicle_attitude.q_1_, ...
        flogOriginal.vehicle_attitude.q_2_, flogOriginal.vehicle_attitude.q_3_ ];
    quatRef = [ flogOriginal.vehicle_attitude_setpoint.q_d_0_, flogOriginal.vehicle_attitude_setpoint.q_d_1_, ...
        flogOriginal.vehicle_attitude_setpoint.q_d_2_, flogOriginal.vehicle_attitude_setpoint.q_d_3_ ];
    
    time = flogOriginal.vehicle_attitude.time;
    timeRef = flogOriginal.vehicle_attitude_setpoint.time;
    
    [eul.Roll, eul.Pitch, eul.Yaw ] = QuatToEuler( quat );
    eul.Yaw = Unwrap( eul.Yaw );
    [eulRef.Roll, eulRef.Pitch, eulRef.Yaw ] = QuatToEuler( quatRef );
    eulRef.Yaw = Unwrap( eulRef.Yaw );
    
    figure( 'name', files{i} )
    ax = {'Roll', 'Pitch', 'Yaw'};
    for j = 1:length(ax)
        subplot(3,1,j)
        hold on; grid on; box on;
        plot( time, rad2deg(eul.(ax{j})) )
        plot( timeRef, rad2deg(eulRef.(ax{j})) )
        xlabel( 'Time (s)' )
        ylabel( [ ax{j} ' (deg)' ] )
    end
end