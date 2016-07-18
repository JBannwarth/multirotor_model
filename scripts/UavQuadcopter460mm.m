%% GEOMETRICAL PARAMETERS
% Geometry
Uav.L    = 0.456/2;             % Distance from centre to motor axis [m]
Uav.BETA = deg2rad(45:90:315)'; % Arm angles [rad]

%% MASS/INERTIA
% Component masses [kg]
Uav.M   = 1.480; % Frame

% Gravity vector
Uav.G   = Uav.M * [0;0;-GRAVITY]; % [N]

% Frame inertia (from trifilar pendulum tests) [kgm^2]
Uav.I = diag( [0.0176, 0.0176, 0.0362] );

% Nominal battery voltage (3S LiPo) [Volts]
Uav.NOMINAL_BATTERY_VOLTAGE = 11.1;   

%% AERODYNAMICS
% Frame drag coefficients (includes reference area)
Uav.C_D  = 0.25*0.507;    % drag coefficient for translation in x,y,z
Uav.C_DM = 0.25*0.681;   % drag coefficient for rotations about x,y,z

% Air density
Uav.RHO_AIR = 1.225;