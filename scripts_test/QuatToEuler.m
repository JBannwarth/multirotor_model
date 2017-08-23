function [ varargout ] = QuatToEuler( quat )
%QUATTOEULER Convert quaternion to euler angles (roll, pitch, yaw)
%   Based on PX4 Firmware code
%   Written by: J.X.J. Bannwarth, 07/08/2017
    
    nout = max(nargout,1);
    if ~( ( nout == 1 ) || ( nout == 3 ) )
        error( 'Invalid number of output: only 1 and 3 output are supported' )
    end
        
    [m, n] = size( quat );
    
    if ( (m == 4) && (n == 1) )
        quat = quat';
    end

    eul(:,1) = atan2(2.0 .* (quat(:,1) .* quat(:,2) + quat(:,3) .* quat(:,4)), ...
            1.0 - 2.0 .* (quat(:,2) .* quat(:,2) + quat(:,3) .* quat(:,3)));
	eul(:,2) = asin(2.0 .* (quat(:,1) .* quat(:,3) - quat(:,4) .* quat(:,2)));
	eul(:,3) = atan2(2.0 .* (quat(:,1) .* quat(:,4) + quat(:,2) .* quat(:,3)), ...
            1.0 - 2.0 .* (quat(:,3) .* quat(:,3) + quat(:,4) .* quat(:,4)));

    if ( (m == 4) && (n == 1) )
        eul = eul';
    end
    
    if ( nout == 3 )
        varargout{1} = eul(:,1);
        varargout{2} = eul(:,2);
        varargout{3} = eul(:,3);
    else
        varargout{1} = eul;
    end
    
end
