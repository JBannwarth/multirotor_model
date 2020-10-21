function dcm = QuatToDcm( quat )
%QUATTODCM Convert quaternion to DCM
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Dcm.hpp
%   Should give the same results as QuatToRot(), but it is based on a more
%   recent version of the source code.
%   Written:       J.X.J. Bannwarth, 2019/01/15
%   Last modified: J.X.J. Bannwarth, 2019/01/15
    a = quat(1);
    b = quat(2);
    c = quat(3);
    d = quat(4);
    aa = a * a;
    ab = a * b;
    ac = a * c;
    ad = a * d;
    bb = b * b;
    bc = b * c;
    bd = b * d;
    cc = c * c;
    cd = c * d;
    dd = d * d;

    dcm = zeros(3, 3);
    dcm(1, 1) = aa + bb - cc - dd;
    dcm(1, 2) = 2 * (bc - ad);
    dcm(1, 3) = 2 * (ac + bd);
    dcm(2, 1) = 2 * (bc + ad);
    dcm(2, 2) = aa - bb + cc - dd;
    dcm(2, 3) = 2 * (cd - ab);
    dcm(3, 1) = 2 * (bd - ac);
    dcm(3, 2) = 2 * (ab + cd);
    dcm(3, 3) = aa - bb - cc + dd;
end