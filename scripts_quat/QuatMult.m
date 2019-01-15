function [ qOut ] = QuatMult( q1, q2 )
%QUATMULT Multiply quaternion q1 by quaternion q2
%   Written by:    J.X.J. Bannwarth, 2017/03/28
%   Last modified: J.X.J. Bannwarth, 2017/01/15
    size1 = size( q1 );
    size2 = size( q2 );
    
    if ( sum(size1 == size2) ~= 2 )
        error('Input sizes do not match')
    end
    
    if ( ( size1(1) == 4 ) && ( size2(2) == 1 ) )
        % Normal quaternion
        qOut(1,1) = q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3) - q1(4) * q2(4);
        qOut(2,1) = q1(1) * q2(2) + q1(2) * q2(1) + q1(3) * q2(4) - q1(4) * q2(3);
        qOut(3,1) = q1(1) * q2(3) - q1(2) * q2(4) + q1(3) * q2(1) + q1(4) * q2(2);
        qOut(4,1) = q1(1) * q2(4) + q1(2) * q2(3) - q1(3) * q2(2) + q1(4) * q2(1);
    else
        % Array of quaternions
        qOut = zeros( size(q1) );
        
        for i = 1:size1(1)
            qOut(i,1) = q1(i,1) * q2(i,1) - q1(i,2) * q2(i,2) - q1(i,3) * q2(i,3) - q1(i,4) * q2(i,4);
            qOut(i,2) = q1(i,1) * q2(i,2) + q1(i,2) * q2(i,1) + q1(i,3) * q2(i,4) - q1(i,4) * q2(i,3);
            qOut(i,3) = q1(i,1) * q2(i,3) - q1(i,2) * q2(i,4) + q1(i,3) * q2(i,1) + q1(i,4) * q2(i,2);
            qOut(i,4) = q1(i,1) * q2(i,4) + q1(i,2) * q2(i,3) - q1(i,3) * q2(i,2) + q1(i,4) * q2(i,1);
        end
    end
end

