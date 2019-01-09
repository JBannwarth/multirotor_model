%UAVOCTOCOPTER500MM Load octocopter parameters
%   Last configuration change:
%       2019/01/09 - Start parameters
%   Written by:    J.X.J. Bannwarth, 2019/01/09
%   Last modified: J.X.J. Bannwarth, 2019/01/09

%% GEOMETRICAL PARAMETERS
Uav.D_UAV = 0.5;
Uav.D_PROP = 6 * 0.0254;
Uav.A_UAV = (pi*Uav.D_UAV^2)/4;
Uav.A_PROP = (pi*Uav.D_PROP^2)/4;
Uav.L    = (0.5 / 2) * ones(4,1); % Distances from centre to motor axis [m]
Uav.BETA = deg2rad(22.5:45:337.5)'; % Arm angles [rad]
Uav.N_ROTORS = length(Uav.BETA); % Number of rotors [-]
% Note: rotor directions are inverted compared to default PX4 arrangement
Uav.ROTOR_DIRECTION = (-1).^((1:Uav.N_ROTORS)-1); % 1 = CCW, -1 = CW [-]
% Rotation matrices from motor to body frame for each rotor
% E.g. Non-rotated rotors (rotors parallel to body z-axis)
%      [ 1 0 0 | 1 0 0 | 1 0 0 | 1 0 0
%        0 1 0 | 0 1 0 | 0 1 0 | 0 1 0
%        0 0 1 | 0 0 1 | 0 0 1 | 0 0 1 ]
%       (:,:,1) (:,:,2) (:,:,3) (:,:,4)
%       rotor1  rotor2  rotor3  rotor4
Uav.R_MOTOR_TO_BODY(:,:,1) = eye(3);
Uav.R_MOTOR_TO_BODY(:,:,2) = eye(3);
Uav.R_MOTOR_TO_BODY(:,:,3) = eye(3);
Uav.R_MOTOR_TO_BODY(:,:,4) = eye(3);

for i = 1:length( Uav.L )
    Uav.L_VEC(:,i) = [ Uav.L(i) * cos( Uav.BETA(i) );
                       Uav.L(i) * sin( Uav.BETA(i) ); 
                       0 ];
end

%% MASS/INERTIA
if ~isfield( Uav, 'M' )
    Uav.M   = 1.99; % Frame [kg]
end

% Gravity vector
Uav.G   = Uav.M * [0; 0; GRAVITY]; % [N]

% Frame inertia (from trifilar pendulum tests) [kgm^2]
Uav.I = diag( [0.024, 0.024, 0.0250] ); % Izz not measured yet

% Nominal battery voltage (3S LiPo) [V]
Uav.NOMINAL_BATTERY_VOLTAGE = 11.1;   

%% AERODYNAMICS - Outdated
% Frame drag coefficients (includes reference area)
Uav.C_D  = 0.25*0.507;    % drag coefficient for translation in x, y, z
Uav.C_DM = 0.25*0.681;   % drag coefficient for rotations about x, y, z

% Air density [kg/m^3]
Uav.RHO_AIR = 1.225;

if ~isfield( Uav, 'PITCH_ONLY' )
    Uav.PITCH_ONLY = 0;
end