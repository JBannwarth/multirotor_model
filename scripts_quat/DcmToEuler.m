function euler = DcmToEuler( dcm )
%DCMTOEULER Convert DCM to Euler angles
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Euler.hpp
%   Written:       J.X.J. Bannwarth, 2019/01/15
%   Last modified: J.X.J. Bannwarth, 2019/01/15
    phi_val   = atan2( dcm(3,2), dcm(3,3) );
    theta_val = asin( -dcm(3,1) );
    psi_val   = atan2( dcm(2,1), dcm(1,1) );
    if abs( theta_val - pi/2 ) < 1e-3
        phi_val = 0;
        psi_val = atan2( dcm(2,3), dcm(1,3) );
    elseif abs( theta_val + pi /2 ) < 1e-3
        phi_val = 0;
        psi_val = atan2( -dcm(2,3), -dcm(1,3) );
    end
    
    euler = [ phi_val; theta_val; psi_val ];
end