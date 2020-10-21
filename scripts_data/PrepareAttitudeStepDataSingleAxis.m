%PREPAREATTITUDESTEPDATA Prepare step data for attitude validation
%   Written by: J.X.J. Bannwarth, 2017/08/21

toPlot = false;

fileNameCurrent = inputFiles{n};
if contains( fileNameCurrent, 'roll' )
    ax = 'Roll';
    eulDes.Pitch = eulDes.Pitch .* 0;
    eulDes.Yaw = ones(size(eulDes.Yaw)) .* 0; %;eulExp.Yaw(1);
    initEuler = [ eulExp.Roll(1), 0, 0 ];
elseif contains( fileNameCurrent, 'pitch' )
    ax = 'Pitch';
    eulDes.Roll = eulDes.Roll .* 0;
    eulDes.Yaw = ones(size(eulDes.Yaw)) .* 0; %eulExp.Yaw(1);
    initEuler = [ 0, eulExp.Pitch(1), 0 ];
elseif contains( fileNameCurrent, 'yaw' )
    ax = 'Yaw';
    eulDes.Pitch = eulDes.Pitch .* 0;
    eulDes.Roll = eulDes.Roll .* 0;
    initEuler = [ 0, 0, eulExp.Yaw(1) ];
else
    error('No axis')
end

[stepMax, maxInd] = max( abs(eulDes.(ax)) );
margin = 0.02* abs(eulDes.(ax)(maxInd) - eulDes.(ax)(1) );
stepIndex = find( abs(eulDes.(ax) - eulDes.(ax)(1)) < margin, 1, 'last' );

qDes = EulerToQuat( [eulDes.Roll, eulDes.Pitch, eulDes.Yaw] );
Initial.Q = [1 1 1 1]' .* EulerToQuat( initEuler )';

AttInput.tDes = tDes - tExp(1);
tExp = tExp - tExp(1);

% Plot data if required
if toPlot
    figure
    hold on; grid on; box on;
    stairs( AttInput.tDes, rad2deg( eulDes.(ax) ) )
    stairs( tExp, rad2deg( eulExp.(ax) ) )
    xlim( [0 inf] )
    ylim( [-inf inf] )
    xlabel( 'Time (s)' )
    ylabel( [ ax ' (deg) ' ] )
    legend( { 'Desired', 'Measured' } )
end

% Add offset to give estimator the time to converge and format data for sim
AttInput.stepTime   = tDes(stepIndex);
AttInput.stepLength = tDes(end) - tDes(stepIndex);
AttInput.tDesOffset = 20;
AttInput.qDes = [ 0, 1, 0, 0, 0; AttInput.tDes+AttInput.tDesOffset, qDes ];
% attInput.thrustDes = [ 0, thrustDes(1); tDes+tDesOffset, thrustDes ];
AttInput.yawRateDes = [ AttInput.tDes+AttInput.tDesOffset, zeros(size(yawRateDes)) ];