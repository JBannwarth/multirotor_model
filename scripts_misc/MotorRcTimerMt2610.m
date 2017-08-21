%% MOTOR ELECTRICAL PROPERTIES
% RCTimer MT2610, 920KV - modelled as brushed DC motor
Motor.R   = 0.11;              % motor resistance [Ohm]
Motor.K_E = 1 / (920*pi/30);   % motor back emf Constant [Vs/rad]
Motor.K_T = Motor.K_E * (3/2); % effective motor torque constant [Nm/A] (3/2 factor for BLDC c.f. brushed DC)


% Rotor thrust coefficient
% 960g thrust @ 11.1V, 15.2A
thrust = 0.96;  % kg
voltage = 11.1; % V
current = 15.2; % A
Motor.K = 7.59588e-6;%1.125770894557940e-05;%9.556410042213521e-06;%(thrust*GRAVITY) / ( (voltage - current*Motor.R) / Motor.K_E )^2;

% Rotor drag coefficient
Motor.B = 4.4335282346e-7;

% Masses
Motor.M_M = 0.054; % Motor
Motor.M_P = 0.011; % Propeller

% Motor inertia [kgm^2]
Motor.R_M = 0.016;
Motor.H_M = 0.0193;
Motor.I_M = Motor.M_M * diag( [(1/12) * (3*Motor.R_M^2 + Motor.H_M^2), ...
                               (1/12) * (3*Motor.R_M^2 + Motor.H_M^2), ...
                               (1/2)  * Motor.R_M^2] );

% Propeller inertia (only model inertia in z-axis) [kgm^s]
Motor.R_P = 10 * 0.0254 / 2; % 13*0.0254/2;
Motor.I_P = Motor.M_P*diag([0, 0, (1/12) * (2*Motor.R_P)^2]);
       
% Rotor inertia (spinning part of motor + propeller) [kgm^s]
Motor.M_R = 0;
Motor.R_R = 0;
Motor.H_R = 0;
Motor.I_R = Motor.I_M + Motor.I_P;

clear thrust voltage current