function quat = AxisAngleToQuat( axisAngle )
%AXISANGLETOQUAT Convert axis angle representation to quaternion
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Quaternion.hpp
%   axisAngle = [x; y; z]*angle where [x; y; z] is a unit vector
%   Written:       J.X.J. Bannwarth, 2019/01/15
%   Last modified: J.X.J. Bannwarth, 2019/01/15

    angle = norm(axisAngle);
    axis = axisAngle / angle;
    quat = zeros(4,1);
    if (angle < 1e-10)
        quat(1) = 1.0;
        % quat(1) = quat(2) = quat(3) = 0;
    else
        magnitude = sin(angle / 2.0);
        quat(1) = cos(angle / 2.0);
        quat(2) = axis(1) * magnitude;
        quat(3) = axis(2) * magnitude;
        quat(4) = axis(3) * magnitude;
    end
end