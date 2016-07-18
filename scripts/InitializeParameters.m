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
% Last modified 12/07/2016 by KS

% Clean up
clear variables; clc;

%% GLOBAL
% Gravitational vector
GRAVITY = 9.80665; % [m/s^2]

% Call right components
MotorRcTimerMt2610
UavQuadcopter460mm


%% ROTOR/MOTOR DYNAMICS
% Single motor thrust required for hover [N]
Uav.THRUST_HOVER = abs(Uav.G(3)) / 4;
% Motor speed at hover thrust [rad/s]
Uav.W_HOVER = sqrt(Uav.THRUST_HOVER/Motor.K);

% Estimated throttle required to maintain hover
Uav.THROTTLE_HOVER = 1000 * ( Motor.K_E*Uav.W_HOVER + ...
    (Motor.B*Motor.R/Motor.K_T) * Uav.W_HOVER^2 ) / ...
    Uav.NOMINAL_BATTERY_VOLTAGE;

% Load controller constants
ArduCopterConstants

%% INITIAL CONDITIONS
Initial.XI   = [0; 0; 0];            % Initial position in inertial frame
Initial.XI_D = [0; 0; 0];            % Initial velocity in inertial frame
Initial.ETA  = rad2deg( [0; 0; 0] ); % Initial orientation (roll, pitch, yaw)
Initial.NU_B = [0; 0; 0];            % Initial angular velocity in body frame
Initial.W = Uav.THROTTLE_HOVER;      % Initial rotor speed

%% SIMULATION PARAMETERS
Simulation.T_S = 0.01; % Timestep for logging
Simulation.T_END = 10; 
Simulation.T_SIM = (0:Simulation.T_S:Simulation.T_END)';