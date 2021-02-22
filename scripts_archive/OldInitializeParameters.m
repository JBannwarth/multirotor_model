%OLDINITIALIZEPARAMETERS Initialises parameters for quadcopter model
%
% Model properties based off of the quadcopter used for the ICUAS'16 paper:
% Bannwarth, J.X.J., Chen, Z.J., Stol, K.A. and MacDonald, B.A. (2016)
% Disturbance Accomodation Control for Wind Rejection of a Quacopter.
% Proceedings of ICUAS'16 (Arlington, VA, USA), June 7-10.
% The paper also contains a derivation of the equations of motion used in
% also attached simulink model.
%
% Written by: J.Bannwarth, J.Chen, and K.Stol
% Last modified 28/04/2016 by KS

% Clean up
%clc

%% GEOMETRICAL PARAMETERS
% Geometry
l = 0.456/2;                % distance from centre to motor axis [m]
beta = (45:90:315)'*pi/180; % arm angles [rad]

% Roll/pitch/yaw factors
roll_factor0 = -cos(beta+pi/2);
pitch_factor0 = -cos(beta);

AP_MOTORS_MATRIX_YAW_FACTOR_CCW = 1;
AP_MOTORS_MATRIX_YAW_FACTOR_CW = -1;

roll_factor = roll_factor0;
pitch_factor = pitch_factor0;

yaw_factor([1,3],1) = AP_MOTORS_MATRIX_YAW_FACTOR_CW;
yaw_factor([2,4],1) = AP_MOTORS_MATRIX_YAW_FACTOR_CCW;

%% MASS/INERTIA
% Component masses [kg]
m_M = 0.054;    % Motor
m_P = 0.011;    % Propeller
m = 1.480;      % Frame

% Motor inertia [kgm^s]
r_M = 0.016;
h_M = 0.0193;
I_M = m_M*diag([1/12*(3*r_M^2+h_M^2), 1/12*(3*r_M^2+h_M^2), 1/2*r_M^2]);  

% Propeller inertia (only model inertia in z-axis) [kgm^s]
r_P = 10*0.0254/2; % 13*0.0254/2;
I_P = m_P*diag([0, 0, 1/12*(2*r_P)^2]);
       
% Rotor inertia (spinning part of motor + propeller) [kgm^s]
m_R = 0;
r_R = 0;
h_R = 0;
I_R = I_M;

% Frame inertia (from trifilar pendulum tests) [kgm^2]
I = diag([0.0176, 0.0176, 0.0362]);

%% MOTOR ELECTRICAL PROPERTIES
% RCTimer MT2610, 920KV - modelled as brushed DC motor
R = 0.11;               % motor resistance [Ohm]
K_e = 1/(920*pi/30);    % motor back emf Constant [Vs/rad]
K_T = 3/2*K_e;          % effective motor torque constant [Nm/A] (3/2 factor for BLDC c.f. brushed DC)
batt_volt_nom = 11.1;   % nominal battery voltage (3S LiPo) [Volts]

%% AERODYNAMICS
% Rotor thrust coefficient
k = (0.960*9.81)/((11.1-15.2*R)/K_e)^2; % 960g thrust @ 11.1V, 15.2A

% Rotor drag coefficient
b = 4.4335282346e-7;

% Frame drag coefficients (includes reference area)
Cd = 0.25*0.507;    % drag coefficient for translation in x,y,z
Cdm = 0.25*0.681;   % drag coefficient for rotations about x,y,z

% Air density
rhoAir = 1.225;

%% GLOBAL
% Gravitational vector
GRAVITY_MSS = 9.80665;
G = m*[0;0;-GRAVITY_MSS];   % [N]

%% ROTOR/MOTOR DYNAMICS
thrust_hover = abs(G(3))/4;         % Single motor thrust required for hover [N]
w_hover = sqrt(thrust_hover/k);     % Motor speed at hover thrust [rad/s]
throttle_hover = 1000*(K_e*w_hover + (b*R/K_T)*w_hover^2)/batt_volt_nom;   % Estimated throttle required to maintain hover

%% INITIAL CONDITIONS
init_xi = [0;0;0];          % Initial position in inertial frame
init_xi_d1 = [0;0;0];       % Initial velocity in inertial frame
if ~exist('init_eta')
    init_eta = [0;0;0]*pi/180;  % Initial orientation (roll, pitch, yaw)
end
init_nu_B = [0;0;0];        % Initial angular velocity in body frame about x,y,z axes

init_M = w_hover;           % Initial rotor speed

%% SIMULATION PARAMETERS
ts = 0.01; % Timestep for logging
tEnd = 100; 
tSim = (0:ts:tEnd)';

%% WIND DISTURBANCE PARAMETERS
windFile = false;
if windFile
    load('turb_5ms.mat')
else
    windData = zeros(1,4);
end

%% LOITER PARAMETERS
% AC_Copter::get_pilot_desired_lean_angles
AC_APARM_ANGLE_MAX = 4500;

% AC_Copter::get_pilot_desired_yaw_rate
AC_COPTER_G_ACRO_YAW_P = 4.5; % Controller gain for yaw control in stabilize mode

% AC_AttitudeControl::angle_ef_roll_pitch_rate_ef_yaw
AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX = 1000;
AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN = 36000;
AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX = 72000;
AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN = 9000;
AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX = 36000;
AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT = 110000; % centi-degrees/sec/sec
AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT = 27000; % centi-degrees/sec/sec
AC_ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX = 4500; % centi-degrees
% Stabilize controller gains [config.h]
STABILIZE_ROLL_P = 4.5; % 4.5; % POSITIVE
STABILIZE_PITCH_P = 4.5; % 4.5; % POSITIVE
STABILIZE_YAW_P = 4.5; % 4.5; % POSITIVE

% AC_AttitudeControl::rate_controller_run
% Stabilize (rate) controller gains [config.h]
RATE_ROLL_IMAX = 500; %2000;
RATE_PITCH_IMAX = 500;%2000;
RATE_YAW_IMAX = 8;%1000;
RATE_ROLL_FILT_HZ = 20;
RATE_PITCH_FILT_HZ = 20;
RATE_YAW_FILT_HZ = 5;
RATE_ROLL_P = 0.15; %0.500; % 0.150;
RATE_ROLL_I = 0.100; % 0.100;
RATE_ROLL_D = 0.004; %0.000; % 0.004;
RATE_PITCH_P = 0.15; %0.500; % 0.150;
RATE_PITCH_I = 0.100; % 0.100;
RATE_PITCH_D = 0.004;% 0.000; % 0.004;
RATE_YAW_P = 0.2; %1.000; % 0.200;
RATE_YAW_I = 0.02; %0.200; % 0.020;
RATE_YAW_D = 0.000; % 0.000

% [AC_AttitudeControl.h]
AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX = 5000;
AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX = 4500;

% Stabilize Rate Control [config.h]
ROLL_PITCH_INPUT_MAX = 4500;
DEFAULT_ANGLE_MAX = 4500;
ANGLE_RATE_MAX = 18000;

% AC_PosControl::pos_to_rate_xy
POS_XY_P = 1;
POSCONTROL_ACCEL_XY_MAX = 980;
POSCONTROL_VEL_XY_MAX_FROM_POS_ERR = 200;
POSCONTROL_SPEED = 500;
POSCONTROL_MODE = 1; % 1 = XY_MODE_POS_LIMITED_AND_VEL_FF; 2 = XY_MODE_POS_AND_VEL_FF

% AC_PosControl::rate_to_accel_xy
VEL_XY_P = 1.0; %%% TUNING...
VEL_XY_I = 0.5; %%% TUNING...
VEL_XY_IMAX = 1000;
VEL_XY_FILT_HZ = 5;

% AC_PosControl::accel_to_lean_angles
POSCONTROL_JERK_LIMIT_CMSSS = 1700;
POSCONTROL_ACCEL_FILTER_HZ = 2;
POSCONTROL_FXY_MAX = 14.0871;
POSCONTROL_FXY_P = 0.0; % Translational gain based on position error
VELCONTROL_FXY_P = 10; % Translational gain based on velocity error
POSCONTROL_ACCEL_TO_FXY = m;

% AC_PosControl::pos_to_rate_z
ALT_HOLD_P = 1;
POS_Z_P = ALT_HOLD_P;

% AC_PosControl::rate_to_accel_z
% Velocity (vertical) gains [config.h]
VEL_Z_P = 5;
POSCONTROL_SPEED_DOWN = -150;
POSCONTROL_SPEED_UP = 250;
POSCONTROL_ACCEL_Z = 250;
POSCONTROL_JERK_RATIO = 1;

% AC_PosControl::accel_to_throttle
% Accel (vertical) control gains [config.h]
ACCEL_Z_P = 0.190; % 0.50;
ACCEL_Z_I = 0.021;
ACCEL_Z_D = 0.000;
ACCEL_Z_IMAX = 800;
ACCEL_Z_FILT_HZ = 20;

% Default maximum vertical velocity and acceleration the pilot may request [config.h]
PILOT_VELZ_MAX = 250;
PILOT_ACCEL_Z_DEFAULT = 250;

% Max distance in cm above or below current location that will be used for
% the alt target when transitioning to alt-hold mode [config.h]
ALT_HOLD_INIT_MAX_OVERSHOOT = 200;

% The acceleration used to define the distance-velocity curve [config.h]
ALT_HOLD_ACCEL_MAX = 250;

% Throttle control gains [config.h]
THR_MID_DEFAULT = 500;
THR_MIN_DEFAULT = 130;
THR_MAX_DEFAULT = 1000;
THR_DZ_DEFAULT = 100;

% AC_AttitudeControl_Multi::get_boosted_throttle
% AC:AttitudeControl::set_throttle_out
POSCONTROL_THROTTLE_CUTOFF_FREQ = 2; % [Hz]

% AP_MotorsMatrix::output_armed_stabilizing
AP_MOTORS_YAW_HEADROOM_DEFAULT = 200;
AP_MOTORS_THR_LOW_CMP_DEFAULT = 0.5;
AP_MOTORS_MAX_NUM_MOTORS = length(beta);
THROTTLE_RADIO_LIM = [1100;1900];

% PWM scaling
throttle_pwm_scalar = (THROTTLE_RADIO_LIM(2)-THROTTLE_RADIO_LIM(1))/THR_MAX_DEFAULT;
rpy_pwm_scalar = (1900-1100-130)/9000;
fxy_pwm_scalar = (1900-1100-130)/9000;
hover_throttle_pwm = throttle_hover*throttle_pwm_scalar+THROTTLE_RADIO_LIM(1);

% AP_MotorsMulticopter::apply_thrust_curve_and_volt_scaling
AP_MOTORS_THST_MAX_DEFAULT = 1.0; % Throttle which produces the maximum thrust. (i.e. 0-1 ) of the full throttle range (Default 0.95)