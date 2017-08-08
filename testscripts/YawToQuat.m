function quat = YawToQuat( yaw )
%YAWTOQUAT Convert yaw angle to quaternion
%   Based on PX4 Firmware code
%   Written by: J.X.J. Bannwarth, 08/08/2017

    quat = zeros(4,1);
    quat(1) = cos(yaw / 2.0);
    quat(2) = 0.0;
    quat(3) = 0.0;
    quat(4) = sin(yaw / 2.0);

end
