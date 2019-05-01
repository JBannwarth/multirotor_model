%MOTORTMOTORMN1806 Load motor parameters
%   Parameters computed by Z.J. Chen for static drag test
%   Needs to be used with 'MotorModelJeremy.slx' 
%   Written by:    J.X.J. Bannwarth, 2019/05/01
%   Last modified: J.X.J. Bannwarth, 2019/05/01

%% MOTOR ELECTRICAL PROPERTIES
% RCTimer MN1806-14, 2300KV
Motor.R   = 0.117;      % Motor resistance [Ohm]
% Motor.K_E = ;         % Motor back emf Constant [Vs/rad] (no value)
Motor.K_T = 1.928e-08;  % Effective motor torque constant [Nm/A]
Motor.K   = 1.5280e-06; % Rotor thrust coefficient

Motor.V_0 = 12.4;
Motor.I_0 = 0.6;
MOTOR.K_V = 240.8554;

%% MOTOR MECHANICAL PROPERTIES
% Rotor drag coefficient (no value, but necessary)
Motor.B = 4.4335282346e-7 .* ones(1,Uav.N_ROTORS);

% Masses (no values) (kg)
% Motor.M_M = ; % Motor
% Motor.M_P = ; % Propeller

% Motor inertia (no values) (kgm^2)
% Motor.R_M = ;
% Motor.H_M = ;
% Motor.I_M = Motor.M_M * diag( [(1/12) * (3*Motor.R_M^2 + Motor.H_M^2), ...
%                                (1/12) * (3*Motor.R_M^2 + Motor.H_M^2), ...
%                                (1/2)  * Motor.R_M^2] );

% Propeller inertia (only model inertia in z-axis) (no values) (kgm^s)
% Motor.R_P = ;
% Motor.I_P = Motor.M_P*diag([0, 0, (1/12) * (2*Motor.R_P)^2]);
       
% Rotor inertia (spinning part of motor + propeller) (kgm^s)
% Motor.M_R = ;
% Motor.R_R = ;
% Motor.H_R = ;
for i = 1:Uav.N_ROTORS
    Motor.I_R(:,:,i) = diag(0, 0, 3.2903e-06);
end
Motor.I_R_ZZ = reshape( Motor.I_R(3,3,:), [1, size(Motor.I_R,3)] );

% clear thrust voltage current