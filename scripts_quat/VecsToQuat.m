function quat = VecsToQuat( src, dst )
    %VECSTOQUAT Quaternion from two vectors
    %   Generates shortest rotation from source to destination vector
    %   Based on code from PX4 Firmware:
    %       https://github.com/PX4/Matrix/blob/master/matrix/Quaternion.hpp
    %   Written:       J.X.J. Bannwarth, 2019/01/15
    %   Last modified: J.X.J. Bannwarth, 2019/01/15
    cr = cross(src, dst);
    dt = dot(src, dst);
    quat = zeros(4,1);
    locEps = 1e-5; % Don't use eps as it's a reserved variable in MATLAB
    if (norm(cr) < locEps && dt < 0)
        % Handle corner cases with 180 degree rotations
        % If the two vectors are parallel, cross product is zero
        % If they point opposite, the dot product is negative
        cr = abs(src);
        if (cr(1) < cr(2))
            if (cr(1) < cr(3))
                cr = [1; 0; 0];
            else
                cr = [0; 0; 1];
            end
        else
            if (cr(2) < cr(3))
                cr = [0; 1; 0];
            else
                cr = [0; 0; 1];
            end
        end
        quat(1) = 0;
        cr = cross(src, cr);
    else
        % Normal case, do half-way quaternion solution
        quat(1) = dt + sqrt( norm(src)^2 * norm(dst)^2 );
    end
    quat(2) = cr(1);
    quat(3) = cr(2);
    quat(4) = cr(3);
    quat = MulticopterAttitudeControl.normalize( quat );
end