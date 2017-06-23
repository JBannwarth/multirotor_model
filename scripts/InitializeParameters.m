% InitialiseParameters.m
% Initialises parameters for quadcopter model
%
% Model properties based off of the quadcopter used for the ICUAS'16 paper:
% Bannwarth, J.X.J., Chen, Z.J., Stol, K.A. and MacDonald, B.A. (2016)
% Disturbance Accomodation Control for Wind Rejection of a Quacopter.
% Proceedings of ICUAS'16 (Arlington, VA, USA), June 7-10.
% The paper also contains a derivation of the equations of motion used in
% also attached simulink model.
%
% Written by: J.Bannwarth, J.Chen, and K.Stol
% Last modified 10/08/2016 by JB

% Clean up
% clear variables; clc;

%% GLOBAL
% Gravitational vector
GRAVITY = 9.80665; % [m/s^2]

% Call the appropriate components
MotorRcTimerMt2610
UavQuadcopter460mm

%% ROTOR/MOTOR DYNAMICS
% Single motor thrust required for hover [N]
Uav.THRUST_HOVER = abs(Uav.G(3)) / 4;

% Motor speed at hover thrust [rad/s]
Uav.OMEGA_HOVER = 5.16e2;
% sqrt(Uav.THRUST_HOVER/Motor.K);

% Estimated throttle required to maintain hover
Uav.THROTTLE_HOVER = ( Motor.K_E*Uav.OMEGA_HOVER + ...
    (Motor.B*Motor.R/Motor.K_T) * Uav.OMEGA_HOVER^2 ) / ...
    Uav.NOMINAL_BATTERY_VOLTAGE;

% Load controller constants
ArduCopterConstants

%% INITIAL CONDITIONS
% For quaternion:
% q = [cos(alpha); sin(alpha)*rotX; sin(alpha)*rotY; sin(alpha)*rotZ]
% where: - alpha is the angle the UAV is rotated aronud the rotation axis
%        - [rotX, rotY, rotZ] is a unit vector defining the rotation axis 
Initial.XI      = [0; 0; 0];            % Initial position in inertial frame
Initial.XI_DOT  = [0; 0; 0];            % Initial velocity in inertial frame
Initial.ETA     = rad2deg( [0; 0; 0] ); % Initial orientation (roll, pitch, yaw)
% Initial.Q       = [1; 0; 0; 0];         % Initial orientation (quaternion)
Initial.NU_BODY = [0; 0; 0];            % Initial angular velocity in body frame
Initial.OMEGA   = Uav.OMEGA_HOVER;  % Initial rotor speed

%% SIMULATION PARAMETERS
Simulation.T_S   = 0.01; % Timestep for logging
%Simulation.T_END = 100; 
%Simulation.T_SIM = (0:Simulation.T_S:Simulation.T_END)';