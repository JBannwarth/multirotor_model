%ISOLATESTEPDATA Split data into separate steps
%   Written by: J.X.J. Bannwarth, 2017/08/22

fileId = fopen ( 'StepDetail.csv' );
fgets( fileId ); % skip headers
tmp = textscan( fileId, '%d %d %s %s', 'Delimiter', '\t' );
fclose( fileId );
start = tmp{1}; stop = tmp{2}; name = tmp{3}; source = tmp{4};

project = simulinkproject;
projectRoot = project.RootFolder;

for i = 1:length(start)
    load( source{i} );
    clear( 'flog' );

    % Extract useful data
    tDes    = flogOriginal.vehicle_attitude_setpoint.time;
    qDes = table2array( flogOriginal.vehicle_attitude_setpoint(:,6:9) );
    thrustDes = flogOriginal.vehicle_attitude_setpoint.thrust;
    yawRateDes = flogOriginal.vehicle_attitude_setpoint.yaw_sp_move_rate;
    tExp = flogOriginal.vehicle_attitude.time;
    qExp = table2array( flogOriginal.vehicle_attitude(:,5:8) );
    
    toKeep = ( tDes > start(i) ) & ( tDes <= stop(i) );
    qDes = qDes(toKeep,:);
    thrustDes = thrustDes(toKeep,:);
    yawRateDes = yawRateDes(toKeep,:);
    tDes = tDes(toKeep);
    
    toKeepExp = ( tExp > start(i) ) & ( tExp <= stop(i) );
    qExp = qExp(toKeepExp,:);
    tExp = tExp(toKeepExp,:);
    
    tDes = tDes - tExp(1);
    tExp = tExp - tExp(1);
    
    [ eulExp.Roll, eulExp.Pitch, eulExp.Yaw ] = QuatToEuler( qExp );
    [ eulDes.Roll, eulDes.Pitch, eulDes.Yaw ] = QuatToEuler( qDes );
    
    save( fullfile(projectRoot, 'inputdata', ['step_' name{i}] ), 'tDes', 'qDes', 'thrustDes', 'yawRateDes', ...
        'tExp', 'qExp', 'eulExp', 'eulDes' )
    
end
