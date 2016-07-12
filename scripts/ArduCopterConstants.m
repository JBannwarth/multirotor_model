%% LOITER PARAMETERS
% AC_Copter::get_pilot_desired_lean_angles
ArduCopter.AC_APARM_ANGLE_MAX = 4500;

% AC_Copter::get_pilot_desired_yaw_rate
ArduCopter.AC_COPTER_G_ACRO_YAW_P = 4.5; % Controller gain for yaw control in stabilize mode

% AC_AttitudeControl::angle_ef_roll_pitch_rate_ef_yaw
ArduCopter.AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX = 1000;
ArduCopter.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN = 36000;
ArduCopter.AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX = 72000;
ArduCopter.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN = 9000;
ArduCopter.AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX = 36000;
ArduCopter.AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT = 110000; % centi-degrees/sec/sec
ArduCopter.AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT = 27000; % centi-degrees/sec/sec
ArduCopter.AC_ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX = 4500; % centi-degrees

% Stabilize controller gains [config.h]
ArduCopter.STABILIZE_ROLL_P = 4.5; % 4.5; % POSITIVE
ArduCopter.STABILIZE_PITCH_P = 4.5; % 4.5; % POSITIVE
ArduCopter.STABILIZE_YAW_P = 4.5; % 4.5; % POSITIVE

% AC_AttitudeControl::rate_controller_run
% Stabilize (rate) controller gains [config.h]
ArduCopter.RATE_ROLL_IMAX = 500; %2000;
ArduCopter.RATE_PITCH_IMAX = 500;%2000;
ArduCopter.RATE_YAW_IMAX = 8;%1000;
ArduCopter.RATE_ROLL_FILT_HZ = 20;
ArduCopter.RATE_PITCH_FILT_HZ = 20;
ArduCopter.RATE_YAW_FILT_HZ = 5;
ArduCopter.RATE_ROLL_P = 0.15; %0.500; % 0.150;
ArduCopter.RATE_ROLL_I = 0.100; % 0.100;
ArduCopter.RATE_ROLL_D = 0.004; %0.000; % 0.004;
ArduCopter.RATE_PITCH_P = 0.15; %0.500; % 0.150;
ArduCopter.RATE_PITCH_I = 0.100; % 0.100;
ArduCopter.RATE_PITCH_D = 0.004;% 0.000; % 0.004;
ArduCopter.RATE_YAW_P = 0.2; %1.000; % 0.200;
ArduCopter.RATE_YAW_I = 0.02; %0.200; % 0.020;
ArduCopter.RATE_YAW_D = 0.000; % 0.000

% [AC_AttitudeControl.h]
ArduCopter.AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX = 5000;
ArduCopter.AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX = 4500;

% Stabilize Rate Control [config.h]
ArduCopter.ROLL_PITCH_INPUT_MAX = 4500;
ArduCopter.DEFAULT_ANGLE_MAX = 4500;
ArduCopter.ANGLE_RATE_MAX = 18000;

% AC_PosControl::pos_to_rate_xy
ArduCopter.POS_XY_P = 1;
ArduCopter.POSCONTROL_ACCEL_XY_MAX = 980;
ArduCopter.POSCONTROL_VEL_XY_MAX_FROM_POS_ERR = 200;
ArduCopter.POSCONTROL_SPEED = 500;
ArduCopter.POSCONTROL_MODE = 1; % 1 = XY_MODE_POS_LIMITED_AND_VEL_FF; 2 = XY_MODE_POS_AND_VEL_FF

% AC_PosControl::rate_to_accel_xy
ArduCopter.VEL_XY_P = 1.0; %%% TUNING...
ArduCopter.VEL_XY_I = 0.5; %%% TUNING...
ArduCopter.VEL_XY_IMAX = 1000;
ArduCopter.VEL_XY_FILT_HZ = 5;

% AC_PosControl::accel_to_lean_angles
ArduCopter.POSCONTROL_JERK_LIMIT_CMSSS = 1700;
ArduCopter.POSCONTROL_ACCEL_FILTER_HZ = 2;
ArduCopter.POSCONTROL_FXY_MAX = 14.0871;
ArduCopter.POSCONTROL_FXY_P = 0.0; % Translational gain based on position error
ArduCopter.VELCONTROL_FXY_P = 10; % Translational gain based on velocity error
ArduCopter.POSCONTROL_ACCEL_TO_FXY = M;

% AC_PosControl::pos_to_rate_z
ArduCopter.ALT_HOLD_P = 1;
ArduCopter.POS_Z_P = ALT_HOLD_P;

% AC_PosControl::rate_to_accel_z
% Velocity (vertical) gains [config.h]
ArduCopter.VEL_Z_P = 5;
ArduCopter.POSCONTROL_SPEED_DOWN = -150;
ArduCopter.POSCONTROL_SPEED_UP = 250;
ArduCopter.POSCONTROL_ACCEL_Z = 250;
ArduCopter.POSCONTROL_JERK_RATIO = 1;

% AC_PosControl::accel_to_throttle
% Accel (vertical) control gains [config.h]
ArduCopter.ACCEL_Z_P = 0.190; % 0.50;
ArduCopter.ACCEL_Z_I = 0.021;
ArduCopter.ACCEL_Z_D = 0.000;
ArduCopter.ACCEL_Z_IMAX = 800;
ArduCopter.ACCEL_Z_FILT_HZ = 20;

% Default maximum vertical velocity and acceleration the pilot may request [config.h]
ArduCopter.PILOT_VELZ_MAX = 250;
ArduCopter.PILOT_ACCEL_Z_DEFAULT = 250;

% Max distance in cm above or below current location that will be used for
% the alt target when transitioning to alt-hold mode [config.h]
ArduCopter.ALT_HOLD_INIT_MAX_OVERSHOOT = 200;

% The acceleration used to define the distance-velocity curve [config.h]
ArduCopter.ALT_HOLD_ACCEL_MAX = 250;

% Throttle control gains [config.h]
ArduCopter.THR_MID_DEFAULT = 500;
ArduCopter.THR_MIN_DEFAULT = 130;
ArduCopter.THR_MAX_DEFAULT = 1000;
ArduCopter.THR_DZ_DEFAULT = 100;

% AC_AttitudeControl_Multi::get_boosted_throttle
% AC:AttitudeControl::set_throttle_out
ArduCopter.POSCONTROL_THROTTLE_CUTOFF_FREQ = 2; % [Hz]

% AP_MotorsMatrix::output_armed_stabilizing
ArduCopter.AP_MOTORS_YAW_HEADROOM_DEFAULT = 200;
ArduCopter.AP_MOTORS_THR_LOW_CMP_DEFAULT = 0.5;
ArduCopter.AP_MOTORS_MAX_NUM_MOTORS = length(Uav.BETA);
ArduCopter.THROTTLE_RADIO_LIM = [1100;1900];

% PWM scaling
ArduCopter.throttle_pwm_scalar = (THROTTLE_RADIO_LIM(2)-THROTTLE_RADIO_LIM(1))/THR_MAX_DEFAULT;
ArduCopter.rpy_pwm_scalar = (1900-1100-130)/9000;
ArduCopter.fxy_pwm_scalar = (1900-1100-130)/9000;
ArduCopter.hover_throttle_pwm = throttle_hover*throttle_pwm_scalar+THROTTLE_RADIO_LIM(1);

% AP_MotorsMulticopter::apply_thrust_curve_and_volt_scaling
ArduCopter.AP_MOTORS_THST_MAX_DEFAULT = 1.0; % Throttle which produces the maximum thrust. (i.e. 0-1 ) of the full throttle range (Default 0.95)
