function R = QuatToRot( q )
%QUATTOROT Convert quaternion to rotation matrix
%   Written by:    J.X.J. Bannwarth, 2017/03/27
%   Last modified: J.X.J. Bannwarth, 2018/12/11
    W = q(1);
    X = q(2);
    Y = q(3);
    Z = q(4);
    
    xx = X * X;
    xy = X * Y;
    xz = X * Z;
    xw = X * W;

    yy = Y * Y;
    yz = Y * Z;
    yw = Y * W;
    
    zz = Z * Z;
    zw = Z * W;

    m00 = 1 - 2 * ( yy + zz );
    m01 =     2 * ( xy - zw );
    m02 =     2 * ( xz + yw );

    m10 =     2 * ( xy + zw );
    m11 = 1 - 2 * ( xx + zz );
    m12 =     2 * ( yz - xw );

    m20 =     2 * ( xz - yw );
    m21 =     2 * ( yz + xw );
    m22 = 1 - 2 * ( xx + yy );
    
    R = [m00 m01 m02;
         m10 m11 m12;
         m20 m21 m22];
end