function [ qInv ] = QuatInv( q )
%QUATINV Calculate the inverse of a quaternion
%   Written by: J.X.J. Bannwarth, 28/03/17
    qSize = size( q );
    
    if ( ( qSize(1) == 4 ) && ( qSize(2) == 1 ) )
        % Normal quaternion
        qInv = q .* [1; -1; -1; -1] ./ (q(1)^2 + q(2)^2 + q(3)^2 + q(4)^2);
    else
        % Array of quaternions
        qInv = zeros( size(q) );
        
        for i = 1:qSize(1)
            qInv(i,:) = ( q(i,:)' .* [1; -1; -1; -1] ./ ...
                (q(i,1)^2 + q(i,2)^2 + q(i,3)^2 + q(i,4)^2) )';
        end
    end
end

