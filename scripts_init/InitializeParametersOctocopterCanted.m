%INITIALIZEPARAMETERSOCTOCOPTERCANTED Init canted octo parameters
%	Written by:    J.X.J. Bannwarth, 2019/01/09
%	Last modified: J.X.J. Bannwarth, 2019/05/01

% Clean up
% clear variables; clc;

%% GLOBAL
% Gravitational vector
GRAVITY = 9.80665; % [m/s^2]

% Call the appropriate components
Uav.ZETA = deg2rad(31); % Cant angle - as per Z.J. Chen's design
UavOctocopter500mmCanted
MotorTMotorMn1806

%% AERO PARAMETERS
if ~exist( 'Aero', 'var' )
    load( 'AeroBothAIAAv3.mat' );
end

%% ROTOR/MOTOR DYNAMICS
% Use measured motor thrust constant - more accurate than using the
% value extrapolated from the static drag testing
Aero.Cz2.coefs(2) = Motor.K / (0.5 .* Uav.RHO_AIR .* Uav.D_PROP^2 .* Uav.A_PROP);

% Single motor thrust required for hover [N]
Uav.THRUST_HOVER = abs(Uav.G(3)) / 8;

% Motor speed at hover thrust [rad/s]
if strcmp( Aero.Type, 'Body oriented' )
    Uav.OMEGA_HOVER = sqrt( Uav.THRUST_HOVER / (0.5*Uav.RHO_AIR*Aero.Cz2.coefs(2)*Uav.D_PROP^2*Uav.A_PROP) );
else
    Uav.OMEGA_HOVER = sqrt( Uav.THRUST_HOVER / (0.5*Uav.RHO_AIR*Aero.CT1.coefs(1)) );
end

% Estimated throttle required to maintain hover
Vi = ( Motor.V_0 - ...
    Motor.K_T*Uav.OMEGA_HOVER^2*(Motor.R * Motor.K_V * Motor.V_0) ) / ...
    Motor.I_0*Motor.R + Motor.I_0*Motor.R + ...
    Uav.OMEGA_HOVER*( Motor.V_0 - Motor.I_0*Motor.R ) / ( Motor.K_V * Motor.V_0 );

Uav.THROTTLE_HOVER = (Vi - 2.3508)/6.9817;

Uav.THROTTLE_HOVER = Uav.THROTTLE_HOVER / cos(Uav.ZETA);

Uav.NOISE_TS = 0.002;

%% INITIAL CONDITIONS
% For quaternion:
% q = [cos(alpha); sin(alpha)*rotX; sin(alpha)*rotY; sin(alpha)*rotZ]
% where: - alpha is the angle the UAV is rotated around the rotation axis
%        - [rotX, rotY, rotZ] is a unit vector defining the rotation axis
if (~exist('Initial', 'var')) || (~isfield( Initial, 'XI' ))     % Initial position in inertial frame
    Initial.XI      = [0; 0; 0];
end
if ~isfield( Initial, 'XI_DOT' ) % Initial velocity in inertial frame
    Initial.XI_DOT  = [0; 0; 0];
end
if ~isfield( Initial, 'ETA' )    % Initial orientation (roll, pitch, yaw)
    Initial.ETA     = rad2deg( [0; 0; 0] ); 
end
if ~isfield( Initial, 'Q' )      % Initial orientation (quaternion)
    Initial.Q       = [1; 0; 0; 0];
end
if ~isfield( Initial, 'NU_BODY' ) % Initial angular velocity in body frame
    Initial.NU_BODY = [0; 0; 0];
end
if ~isfield( Initial, 'OMEGA' )   % Initial rotor speed
    Initial.OMEGA   = Uav.OMEGA_HOVER .* ones(Uav.N_ROTORS,1);
end

%% SIMULATION PARAMETERS
if (~exist('Simulation', 'var')) || ~isfield( Simulation, 'T_S' )
    Simulation.T_S   = 0.01; % Timestep for logging
end
if ~isfield( Simulation, 'T_END' )
    Simulation.T_END = 100; 
end
if ~isfield( Simulation, 'T_SIM' )
    Simulation.T_SIM = (0:Simulation.T_S:Simulation.T_END)';
end