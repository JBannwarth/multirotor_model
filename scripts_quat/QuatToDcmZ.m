function dcmZ = QuatToDcmZ( quat )
    %QUATTODCMZ Convert quaternion to corresponding body z-axis
    %	Last orthogonal unit basis vector
    %   == last column of the equivalent rotation matrix
    %   but calculated more efficiently than a full conversion
    %   Based on code from PX4 Firmware:
    %       https://github.com/PX4/Matrix/blob/master/matrix/Dcm.hpp
    %   Written:       J.X.J. Bannwarth, 2019/01/15
    %   Last modified: J.X.J. Bannwarth, 2019/01/15

    a = quat(1);
    b = quat(2);
    c = quat(3);
    d = quat(4);
    dcmZ = [ 2 * (a * c + b * d);
        2 * (c * d - a * b);
        a * a - b * b - c * c + d * d ];
end