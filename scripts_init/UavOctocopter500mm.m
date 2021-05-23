function Uav = UavOctocopter500mm( cantAngle, mass )
%UAVOCTOCOPTER500MM Load octocopter parameters
%   UAV = UAVOCTOCOPTER500MM( ) loads planar octocopter parameters.
%   UAV = UAVOCTOCOPTER500MM( CANTANGLE ) loads canted rotor parameters.
%   UAV = UAVOCTOCOPTER500MM( CANTANGLE, MASS ) overrides the default mass.
%
%   Input:
%       - cantAngle: angle of cant of the rotors, zeta [rad] (default: 0)
%
%   See also UAVQUADCOPTER460MM.
%
%   Written: 2019/01/09, J.X.J. Bannwarth

    arguments
        cantAngle (1,1) double = 0  % Cant angle [deg]
        mass      (1,1) double = -1 % Mass [kg]
    end

    %% GEOMETRICAL PARAMETERS
    % Constants
    gravity = 9.80665; % [m/s^2]
    
    % UAV dimensions
    Uav.D_UAV  = 0.5;                   % Rotor-to-rotor diameter [m]
    Uav.D_PROP = 6 * 0.0254;            % Propeller diameter [m]
    Uav.A_UAV  = (pi*Uav.D_UAV^2) / 4;  % Representative UAV area [m^2]
    Uav.A_PROP = (pi*Uav.D_PROP^2) / 4; % Representative prop area [m^2]
    
    % Angles
    Uav.GAMMA = deg2rad(22.5:45:337.5)'; % Arm angles [rad]
    Uav.ZETA  = deg2rad(cantAngle);      % Cant angle [rad]
    
    % Rotor parameters
    Uav.N_ROTORS = length(Uav.GAMMA); % Number of rotors [-]
    Uav.L        = (0.5 / 2) * ones(Uav.N_ROTORS,1); % Centre to motor axis [m]
    for i = 1:length( Uav.L )
        Uav.L_VEC(:,i) = [ Uav.L(i) * cos( Uav.GAMMA(i) );
                           Uav.L(i) * sin( Uav.GAMMA(i) ); 
                           0 ];
    end
    
    % Notes:
    %   - The terms 'CCW' and 'CW' are defined with respect to the z-axis
    %   of the motors' frames of reference, which by default are NED and
    %   aligned with the UAV's frame of reference (which is also NED)
    Uav.ROTOR_DIRECTION = -(-1).^(1:Uav.N_ROTORS); % 1 = CCW, -1 = CW [-]
    
    % Rotation matrices from motor to body frame for each rotor
    % E.g. Non-rotated rotors (rotors parallel to body z-axis)
    %      [ 1 0 0 | 1 0 0 | 1 0 0 | 1 0 0
    %        0 1 0 | 0 1 0 | 0 1 0 | 0 1 0
    %        0 0 1 | 0 0 1 | 0 0 1 | 0 0 1 ]
    %       (:,:,1) (:,:,2) (:,:,3) (:,:,4)
    %       rotor1  rotor2  rotor3  rotor4
    Uav.R_MOTOR_TO_BODY = zeros( 3, 3, Uav.N_ROTORS );
    for i = 1:8
        switch i
            case 1
                ax = [ 0, sin(Uav.ZETA), cos(Uav.ZETA) ];
            case 2
                ax = [ sin(Uav.ZETA), 0, cos(Uav.ZETA) ];
            case 3
                ax = [ -sin(Uav.ZETA), 0, cos(Uav.ZETA) ];
            case 4
                ax = [ 0, sin(Uav.ZETA), cos(Uav.ZETA) ];
            case 5
                ax = [ 0, -sin(Uav.ZETA), cos(Uav.ZETA) ];
            case 6
                ax = [ -sin(Uav.ZETA), 0, cos(Uav.ZETA) ];
            case 7
                ax = [ sin(Uav.ZETA), 0, cos(Uav.ZETA) ];
            case 8
                ax = [ 0, -sin(Uav.ZETA), cos(Uav.ZETA) ];
        end
        Uav.R_MOTOR_TO_BODY(:,:,i) = QuatToDcm( VecsToQuat( [0,0,1], ax ) );
    end

    %% MASS/INERTIA
    % Mass
    Uav.M   = 1.72; % Frame [kg]
    if mass ~= -1
        Uav.M = mass;
    end

    % Gravity vector
    Uav.G   = Uav.M * [0; 0; gravity]; % [N]

    % Frame inertia (from trifilar pendulum tests) [kgm^2]
    Uav.I = diag( [0.024, 0.024, 0.1051] );

    % Nominal battery voltage (3S LiPo) [V]
    Uav.NOMINAL_BATTERY_VOLTAGE = 11.1;

    %% AERODYNAMICS - Outdated
    % Frame drag coefficients (includes reference area)
    Uav.C_D  = 0.25*0.507;    % drag coefficient for translation in x, y, z
    Uav.C_DM = 0.25*0.681;   % drag coefficient for rotations about x, y, z

    % Air density
    Uav.RHO_AIR = 1.225; % [kg/m^3]
end