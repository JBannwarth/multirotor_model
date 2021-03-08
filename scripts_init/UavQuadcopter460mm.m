function Uav = UavQuadcopter460mm( mass )
%UAVQUADCOPTER460MM Load quadcopter parameters
%   UAV = UAVQUADCOPTER460MM( ) loads planar quadcopter parameters.
%   UAV = UAVQUADCOPTER460MM( MASS ) overrides the default mass.
%
%   Last configuration change:
%       2017/05/16 - Uav w/o landing gear, w/ 3.85Ah battery, Raspberry Pi
%       on top level
%
%   See also UAVOCTOCOPTER500MM.
%
%   Written: 2017, J.X.J. Bannwarth

    arguments
        mass (1,1) double = -1 % Mass [kg]
    end

    %% GEOMETRICAL PARAMETERS
    % Constants
    gravity = 9.80665; % [m/s^2]
    
    % UAV dimensions
    Uav.D_UAV = 0.456;
    Uav.D_PROP = 10 * 0.0254;
    
    Uav.A_UAV = (pi*Uav.D_UAV^2)/4;
    Uav.A_PROP = (pi*Uav.D_PROP^2)/4;
    
    % Angles
    Uav.GAMMA = deg2rad(45:90:315)'; % Arm angles [rad]
    Uav.ZETA = 0;                    % Cant angle [rad] - not supported
    
    % Rotor parameters
    Uav.N_ROTORS = length(Uav.GAMMA); % Number of rotors [-]
    Uav.L        = (0.456 / 2) * ones(Uav.N_ROTORS,1); % Centre to motor axis [m]
    for i = 1:length( Uav.L )
        Uav.L_VEC(:,i) = [ Uav.L(i) * cos( Uav.GAMMA(i) );
                           Uav.L(i) * sin( Uav.GAMMA(i) ); 
                           0 ];
    end
    
    % Notes:
    %   - The terms 'CCW' and 'CW' are defined with respect to the z-axis
    %   of the motors' frames of reference, which by default are NED and
    %   aligned with the UAV's frame of reference (which is also NED)
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

    %% MASS/INERTIA
    % Mass
    Uav.M   = 1.5;% Frame [kg]
    if mass ~= -1
        Uav.M = mass;
    end

    % Gravity vector
    Uav.G   = Uav.M * [0; 0; gravity]; % [N]

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
end