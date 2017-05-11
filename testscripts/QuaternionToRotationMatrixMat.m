function R = QuaternionToRotationMatrixMat( q )
% QUATERNIONTOROTATIONMATRIXMAT Convert quaternion to rotation matrix
% Written bR: J.X.J. Bannwarth, 27/03/17

%     aSq = q(1) * q(1);
%     bSq = q(2) * q(2);
%     cSq = q(3) * q(3);
%     dSq = q(4) * q(4);
%     
%     R(1,1) = aSq + bSq - cSq - dSq;
%     R(1,2) = 2.0 * (q(2) * q(3) - q(1) * q(4));
%     R(1,3) = 2.0 * (q(1) * q(3) + q(2) * q(4));
%     R(2,1) = 2.0 * (q(2) * q(3) + q(1) * q(4));
%     R(2,2) = aSq - bSq + cSq - dSq;
%     R(2,3) = 2.0 * (q(3) * q(4) - q(1) * q(2));
%     R(3,1) = 2.0 * (q(2) * q(4) - q(1) * q(3));
%     R(3,2) = 2.0 * (q(1) * q(2) + q(3) * q(4));
%     R(3,3) = aSq - bSq - cSq + dSq;

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