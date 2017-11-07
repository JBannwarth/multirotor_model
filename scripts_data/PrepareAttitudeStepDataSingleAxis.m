%PREPAREATTITUDESTEPDATA Prepare step data for attitude validation
%   Written by: J.X.J. Bannwarth, 2017/08/21

toPlot = false;
% [ eulDes.Roll, eulDes.Pitch, eulDes.Yaw ] = QuatToEuler( qDes );
% [ eulExp.Roll, eulExp.Pitch, eulExp.Yaw ] = QuatToEuler( qDes );

fileNameCurrent = inputFiles{n};
if ~isempty( strfind( fileNameCurrent, 'roll' ) )
    ax = 'Roll';
    eulDes.Pitch = eulDes.Pitch .* 0;
    eulDes.Yaw = ones(size(eulDes.Yaw)) .* 0; %;eulExp.Yaw(1);
    initEuler = [ eulExp.Roll(1), 0, 0 ];
elseif ~isempty( strfind( fileNameCurrent, 'pitch' ) )
    ax = 'Pitch';
    eulDes.Roll = eulDes.Roll .* 0;
    eulDes.Yaw = ones(size(eulDes.Yaw)) .* 0; %eulExp.Yaw(1);
    initEuler = [ 0, eulExp.Pitch(1), 0 ];
elseif ~isempty( strfind( fileNameCurrent, 'yaw' ) )
    ax = 'Yaw';
    eulDes.Pitch = eulDes.Pitch .* 0;
    eulDes.Roll = eulDes.Roll .* 0;
    initEuler = [ 0, 0, eulExp.Yaw(1) ];
else
    error('No axis')
end

qDes = EulerToQuat( [eulDes.Roll, eulDes.Pitch, eulDes.Yaw] );
Initial.Q = [1 1 -1 -1]' .* EulerToQuat( initEuler )';

tDes = tDes - tExp(1);
tExp = tExp - tExp(1);

% Plot data if required
if toPlot
    figure
    hold on; grid on; box on;
    stairs( tDes, rad2deg( eulDes.(ax) ) )
    stairs( tExp, rad2deg( eulExp.(ax) ) )
    xlim( [0 inf] )
    ylim( [-inf inf] )
    xlabel( 'Time (s)' )
    ylabel( [ ax ' (deg) ' ] )
    legend( { 'Desired', 'Measured' } )
end

% Add offset to give estimator the time to converge and format data for sim
tDesOffset = 250;
qDesInput = [ 0, qDes(1,:); tDes+tDesOffset, qDes ];
thrustDesInput = [ 0, thrustDes(1); tDes+tDesOffset, thrustDes ];
yawRateDesInput = [ tDes+tDesOffset, zeros(size(yawRateDes)) ]; % yawRateDes