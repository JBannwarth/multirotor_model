function y = QuatToEuler( quat )
%QUATTOEULER Convert quaternion to euler angles (roll, pitch, yaw)
%   Based on PX4 Firmware code
%   Written by: J.X.J. Bannwarth, 07/08/2017

    y(1) = atan2(2.0 * (quat(1) * quat(2) + quat(3) * quat(4)), ...
            1.0 - 2.0 * (quat(2) * quat(2) + quat(3) * quat(3)));
	y(2) = asin(2.0 * (quat(1) * quat(3) - quat(4) * quat(2)));
	y(3) = atan2(2.0 * (quat(1) * quat(4) + quat(2) * quat(3)), ...
            1.0 - 2.0 * (quat(3) * quat(3) + quat(4) * quat(4)));

end
