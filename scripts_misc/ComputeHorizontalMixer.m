%COMPUTEHORIZONTALMIXER Find mixing matrix for horizontal thrust
%   For octocopter with x/y thrust inputs
%   Written:       J.X.J. Bannwarth, 2019/02/26
%   Last modified: J.X.J. Bannwarth, 2019/05/01

% Load parameters
loadBuses = false;
InitializeParametersOctocopterCanted
InitializeModel

a = 2 / ( sin(Uav.CANT_ANGLE) );
b = tan( Uav.BETA(1) );
mixerX = [ a*b; -a; a; -a*b; -a*b; a; -a; a*b ] .* cos(Uav.CANT_ANGLE); % need cos to get same scaling as vertical thrust since that one is not scaled
mixerY = [ -a; a*b; a*b; -a; a; -a*b; -a*b; a ] .* cos(Uav.CANT_ANGLE);

mixer = [ RotorMapSimToPx4(mixerX), RotorMapSimToPx4(mixerY) ];

mixerStr = sprintf( ' %+.6f, %+.6f;\\n', mixer' );
mixerStr = sprintf( [ '[' mixerStr(2:end-3) ']' ] );

set_param( 'TestMultirotorSimPx4v1_8Hinf/Horizontal mixing matrix', 'value', mixerStr )