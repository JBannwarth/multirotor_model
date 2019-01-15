function [ quat ] = EulerToQuat( eulerAngles )
%EULERTOQUAT Convert Euler angles to a quaternion
%   Works on vector inputs
%   Based on PX4 Firmware code: https://github.com/PX4/Firmware
%   Written by: J.X.J. Bannwarth
%   Last modified: 15/01/2019

    rotateInput = false;
    if (size(eulerAngles,1) == 3) && (size(eulerAngles,2) ~= 3)
        eulerAngles = eulerAngles';
        rotateInput = true;
    elseif (size(eulerAngles,1) ~= 3) && (size(eulerAngles,2) ~= 3)
        error( [ 'Incorrect input size, Input must have at least three' ...
            'elements for roll, pitch, and yaw' ] )
    end
    
    roll  = eulerAngles(:,1);
    pitch = eulerAngles(:,2);
    yaw   = eulerAngles(:,3);
    
    cosPhi_2   = cos(roll / 2.0);
    sinPhi_2   = sin(roll / 2.0);
    cosTheta_2 = cos(pitch / 2.0);
    sinTheta_2 = sin(pitch / 2.0);
    cosPsi_2   = cos(yaw / 2.0);
    sinPsi_2   = sin(yaw / 2.0);
        
    quat(:,1) = cosPhi_2 .* cosTheta_2 .* cosPsi_2 + sinPhi_2 .* sinTheta_2 .* sinPsi_2;
    quat(:,2) = sinPhi_2 .* cosTheta_2 .* cosPsi_2 - cosPhi_2 .* sinTheta_2 .* sinPsi_2;
    quat(:,3) = cosPhi_2 .* sinTheta_2 .* cosPsi_2 + sinPhi_2 .* cosTheta_2 .* sinPsi_2;
    quat(:,4) = cosPhi_2 .* cosTheta_2 .* sinPsi_2 - sinPhi_2 .* sinTheta_2 .* cosPsi_2;

    if rotateInput
        quat = quat';
    end
end