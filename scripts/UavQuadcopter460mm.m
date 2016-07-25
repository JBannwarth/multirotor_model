%% GEOMETRICAL PARAMETERS
Uav.L    = (0.456 / 2) * ones(4,1); % Distances from centre to motor axis [m]
Uav.BETA = deg2rad(45:90:315)'; % Arm angles [rad]
Uav.N_ROTORS = length(Uav.BETA); % Number of rotors [-]
Uav.ROTOR_DIRECTION = (-1).^(1:Uav.N_ROTORS); % 1 = CCW, -1 = CW [-]

%% MASS/INERTIA
Uav.M   = 1.480; % Frame [kg]

% Gravity vector
Uav.G   = Uav.M * [0;0;-GRAVITY]; % [N]

% Frame inertia (from trifilar pendulum tests) [kgm^2]
Uav.I = diag( [0.0176, 0.0176, 0.0362] );

% Nominal battery voltage (3S LiPo) [V]
Uav.NOMINAL_BATTERY_VOLTAGE = 11.1;   

%% AERODYNAMICS
% Frame drag coefficients (includes reference area)
Uav.C_D  = 0.25*0.507;    % drag coefficient for translation in x,y,z
Uav.C_DM = 0.25*0.681;   % drag coefficient for rotations about x,y,z

% Air density [kg/m^3]
Uav.RHO_AIR = 1.225;