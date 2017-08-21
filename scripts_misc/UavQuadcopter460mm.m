% Last updated:
% 16/05/2017 - Uav w/o landing gear, w/ 3.85Ah battery, Raspberry Pi on top
% level

%% GEOMETRICAL PARAMETERS
Uav.L    = (0.456 / 2) * ones(4,1); % Distances from centre to motor axis [m]
Uav.BETA = deg2rad(45:90:315)'; % Arm angles [rad]
Uav.N_ROTORS = length(Uav.BETA); % Number of rotors [-]
Uav.ROTOR_DIRECTION = (-1).^(1:Uav.N_ROTORS); % 1 = CCW, -1 = CW [-]
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
Uav.M   = 1.204; % Frame [kg]

% Gravity vector
Uav.G   = Uav.M * [0; 0; -GRAVITY]; % [N]

% Frame inertia (from trifilar pendulum tests) [kgm^2]
Uav.I = diag( [0.0175, 0.0175, 0.0250] );

% Nominal battery voltage (3S LiPo) [V]
Uav.NOMINAL_BATTERY_VOLTAGE = 11.1;   

%% AERODYNAMICS - Outdated
% Frame drag coefficients (includes reference area)
Uav.C_D  = 0.25*0.507;    % drag coefficient for translation in x, y, z
Uav.C_DM = 0.25*0.681;   % drag coefficient for rotations about x, y, z

% Air density [kg/m^3]
Uav.RHO_AIR = 1.225;