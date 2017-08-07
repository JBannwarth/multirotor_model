function quat = RotToQuat( dcm )
%ROTTOQUAT Convert rotation matrix to quaternion
%   Based on PX4 Firmware code
%   Written by: J.X.J. Bannwarth, 07/08/2017
    quat = zeros(4,1);
    tr = dcm(1,1) + dcm(2,2) + dcm(3,3);
    if (tr > 0.0)
        s = sqrt(tr + 1.0);
        quat(1) = s * 0.5;
        s = 0.5 / s;
        quat(2) = (dcm(3,2) - dcm(2,3) ) * s;
        quat(3) = (dcm(1,3) - dcm(3,1) ) * s;
        quat(4) = (dcm(2,1) - dcm(1,2) ) * s;
    else
        % Find maximum diagonal element in dcm
        % store index in dcm_i
        dcm_i = 1;
        for i = 2:3 % (int i = 1; i < 3; i++)
            if ( dcm(i,i) > dcm(dcm_i,dcm_i) )
                dcm_i = i;
            end
        end
        dcm_j = rem( ( (dcm_i-1) + 1), 3) + 1;
        dcm_k = rem( ( (dcm_i-1) + 2), 3) + 1;
        s = sqrt( (dcm(dcm_i, dcm_i) - dcm(dcm_j, dcm_j) - ...
            dcm(dcm_k,dcm_k)) + 1.0 );
        quat(dcm_i + 1) = s * 0.5;
        s = 0.5 / s;
        quat(dcm_j + 1) = ( dcm(dcm_i,dcm_j) + dcm(dcm_j,dcm_i) ) * s;
        quat(dcm_k + 1) = ( dcm(dcm_k,dcm_i) + dcm(dcm_i,dcm_k) ) * s;
        quat(1) = ( dcm(dcm_k,dcm_j) - dcm(dcm_j,dcm_k) ) * s;
    end
end