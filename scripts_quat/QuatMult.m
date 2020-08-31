function [ qOut ] = QuatMult( p, q )
%QUATMULT Multiply quaternion p by quaternion q using Hamilton convention
%   Written by:    J.X.J. Bannwarth, 2017/03/28
%   Last modified: J.X.J. Bannwarth, 2017/01/15
    size1 = size( p );
    size2 = size( q );
    
    if ( sum(size1 == size2) ~= 2 )
        error('Input sizes do not match')
    end
    
    if ( ( size1(1) == 4 ) && ( size2(2) == 1 ) )
        % Normal quaternion
        qOut(1,1) = p(1) * q(1) - p(2) * q(2) - p(3) * q(3) - p(4) * q(4);
        qOut(2,1) = p(1) * q(2) + p(2) * q(1) + p(3) * q(4) - p(4) * q(3);
        qOut(3,1) = p(1) * q(3) - p(2) * q(4) + p(3) * q(1) + p(4) * q(2);
        qOut(4,1) = p(1) * q(4) + p(2) * q(3) - p(3) * q(2) + p(4) * q(1);
    else
        % Array of quaternions
        qOut = zeros( size(p) );
        
        for i = 1:size1(1)
            qOut(i,1) = p(i,1) * q(i,1) - p(i,2) * q(i,2) - p(i,3) * q(i,3) - p(i,4) * q(i,4);
            qOut(i,2) = p(i,1) * q(i,2) + p(i,2) * q(i,1) + p(i,3) * q(i,4) - p(i,4) * q(i,3);
            qOut(i,3) = p(i,1) * q(i,3) - p(i,2) * q(i,4) + p(i,3) * q(i,1) + p(i,4) * q(i,2);
            qOut(i,4) = p(i,1) * q(i,4) + p(i,2) * q(i,3) - p(i,3) * q(i,2) + p(i,4) * q(i,1);
        end
    end
end

