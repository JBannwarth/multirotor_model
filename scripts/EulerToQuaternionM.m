function [ quat ] = EulerToQuaternionM( eulerAngles )
%EULERTOQUATERNIONM Convert Euler angles to a quaternion
%   Works on vector inputs
%   Based on PX4 Firmware code: https://github.com/PX4/Firmware
%   Written by: J.X.J. Bannwarth
%   Last modified: 29/08/2016

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

end