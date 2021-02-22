function [Uav, Motor, Aero, Initial] = InitializeParametersQuadcopter( )
%INITIALIZEPARAMETERSQUADCOPTER Initialises parameters for quadcopter model
%   [__] = INITIALIZEPARAMETERSQUADCOPTER( ) loads quadcopter parameters.
%
%	Model properties based off of the quadcopter used for the ICUAS'16 paper:
%	Bannwarth, J.X.J., Chen, Z.J., Stol, K.A. and MacDonald, B.A. (2016)
%	Disturbance Accomodation Control for Wind Rejection of a Quacopter.
%	Proceedings of ICUAS'16 (Arlington, VA, USA), June 7-10.
%	The paper also contains a derivation of the equations of motion used in
%	also attached simulink model.
%
%   See also INITIALIZEPARAMETERSOCTOCOPTER.
%
%	Written by: 2016, J.X.J. Bannwarth, J.Chen, and K.Stol

    %% GLOBAL
    % Call the appropriate components
    Uav = UavQuadcopter460mm( );
    Motor = MotorRcTimerMt2610( Uav.N_ROTORS );

    %% AERO PARAMETERS
    load( 'AeroBothAIAAv3.mat', 'Aero' );

    %% ROTOR/MOTOR DYNAMICS
    % Single motor thrust required for hover [N]
    Uav.THRUST_HOVER = abs(Uav.G(3)) / 4;

    % Motor speed at hover thrust [rad/s]
    if strcmp( Aero.Type, 'Body oriented' )
        Uav.OMEGA_HOVER = sqrt( Uav.THRUST_HOVER / (0.5*Uav.RHO_AIR*Aero.Cz2.coefs(2)*Uav.D_PROP^2*Uav.A_PROP) );
    else
        Uav.OMEGA_HOVER = sqrt( Uav.THRUST_HOVER / (0.5*Uav.RHO_AIR*Aero.CT1.coefs(1)) );
    end

    % Estimated throttle required to maintain hover
    Uav.THROTTLE_HOVER = ( Motor.K_E*Uav.OMEGA_HOVER + ...
        (mean(Motor.B)*Motor.R/Motor.K_T) * Uav.OMEGA_HOVER^2 ) / ...
        Uav.NOMINAL_BATTERY_VOLTAGE;

    % Sampling time of noise
    Uav.NOISE_TS = 0.002;

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