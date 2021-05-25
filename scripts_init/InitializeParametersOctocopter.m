function [Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( cantAngle, mass, kScale )
%INITIALIZEPARAMETERSOCTOCOPTER Initialize parameters for octocopter model
%   [__] = INITIALIZEPARAMETERSOCTOCOPTER( ) loads planar octo parameters.
%   [__] = INITIALIZEPARAMETERSOCTOCOPTER( CANTANGLE ) specifies the cant angle.
%   [__] = INITIALIZEPARAMETERSOCTOCOPTER( CANTANGLE, MASS ) specifies the mass.
%   [__] = INITIALIZEPARAMETERSOCTOCOPTER( CANTANGLE, MASS, KSCALE ) scales the thrust constant.
%
%   Input:
%       - canted: whether the UAV is canted or not.
%
%   See also INITIALIZEPARAMETERSQUADCOPTER.
%
%	Written: 2019/01/09, J.X.J. Bannwarth

    arguments
        cantAngle (1,1) double = 0  % No cant
        mass      (1,1) double = -1 % Default mass
        kScale    (1,1) double = 1  % No scaling
    end

    %% GLOBAL
    % Call the appropriate components
    Uav = UavOctocopter500mm( cantAngle, mass );
    Motor = MotorTMotorMn1806( Uav.N_ROTORS );

    %% AERO PARAMETERS
    load( 'AeroBothAIAAv3.mat', 'Aero' );

    %% ROTOR/MOTOR DYNAMICS
    % Use measured motor thrust constant - more accurate than using the
    % value extrapolated from the static drag testing
    Aero.Cz2.coefs(2) = kScale .* Motor.K ./ (0.5 .* Uav.RHO_AIR .* Uav.D_PROP^2 .* Uav.A_PROP);
    
    % Single motor thrust required for hover [N]
    Uav.THRUST_HOVER = ( abs(Uav.G(3)) / 8 ) / cos(Uav.ZETA);

    % Motor speed at hover thrust [rad/s]
    if strcmp( Aero.Type, 'Body oriented' )
        Uav.OMEGA_HOVER = sqrt( Uav.THRUST_HOVER ...
            / (0.5*Uav.RHO_AIR*Aero.Cz2.coefs(2)*Uav.D_PROP^2*Uav.A_PROP) );
    else
        Uav.OMEGA_HOVER = sqrt( Uav.THRUST_HOVER ...
            / (0.5*Uav.RHO_AIR*Aero.CT1.coefs(1)) );
    end

    % Estimated throttle required to maintain hover
    Vi = (Motor.C_TAU*Uav.OMEGA_HOVER^2) * (Motor.R/Motor.K_T) ...
        + Motor.I_0*Motor.R + Uav.OMEGA_HOVER*Motor.K_E;

    Uav.THROTTLE_HOVER = (Vi - 2.3508) / 6.9817;

    % Sampling time of noise
    Uav.NOISE_TS = 0.002;
        
    % Pitch control
    Uav.PITCH_ONLY = false;

    %% INITIAL CONDITIONS
    % For quaternion:
    % q = [cos(alpha); sin(alpha)*rotX; sin(alpha)*rotY; sin(alpha)*rotZ]
    % where: - alpha is the angle the UAV is rotated aronud the rotation axis
    %        - [rotX, rotY, rotZ] is a unit vector defining the rotation axis  
    Initial.XI      = [0; 0; 0];            % Initial position in inertial frame
    Initial.XI_DOT  = [0; 0; 0];            % Initial velocity in inertial frame
    Initial.ETA     = rad2deg( [0; 0; 0] ); % Initial orientation (roll, pitch, yaw)
    Initial.Q       = [1; 0; 0; 0];         % Initial orientation (quaternion)
    Initial.NU_BODY = [0; 0; 0];            % Initial angular velocity in body frame
    Initial.OMEGA   = Uav.OMEGA_HOVER .* ones(Uav.N_ROTORS,1); % Initial rotor speed
end