%% Custom
% self.
%{
  init
  last_run
%}

%% control attitude
% self.
%{
  thrust_sp
  v_att_sp.qd
  v_att_sp.thrust
  v_att_sp.yaw_sp_move_rate
  v_att_sp.disable_mc_yaw_control
  ctrl_state.q
  rates_sp
  rates_int
x params.att_p
x params.auto_rate_max
x params.mc_rate_max
x params.yaw_ff
x params.vtol_wv_yaw_rate_scale
  v_control_mode.flag_control_velocity_enabled
  v_control_mode.flag_control_auto_enabled
  v_control_mode.flag_control_manual_enabled
%}

%% throttle pid attenuation
% self.
%{
  v_rates_sp.thurst
%}

%{
  AXIS_INDEX_ROLL
  AXIS_INDEX_PITCH
  AXIS_INDEX_YAW
  TPA_RATE_LOWER_LIMIT
%}

%% attitude rates controller
% self.
%{
  armed.armed
  vehicle_status.is_rotary_wing
  rates_int.zero
  ctrl_state.roll_rate
  ctrl_state.pitch_rate
  ctrl_state.yaw_rate
x params.rate_p
x params.rate_i
x params.rate_d
x params.rate_int_lim
x params.tpa_breakpoint_p
x params.tpa_breakpoint_i
x params.tpa_breakpoint_d
x params.rate_ff
  rates_sp
  rates_int
  rates_prev
  saturation_status.flags.roll_pos
  saturation_status.flags.pitch_pos
  saturation_status.flags.yaw_pos
  saturation_status.flags.roll_neg
  saturation_status.flags.pitch_neg
  saturation_status.flags.yaw_neg
%}

%{
  AXIS_INDEX_ROLL
  AXIS_INDEX_PITCH
  AXIS_INDEX_YAW
  MIN_TAKEOFF_THRUST
  AXIS_COUNT
%}

%% main
% self.
%{
  v_control_mode.flag_control_attitude_enabled
  v_rates_sp.roll
  v_rates_sp.pitch
  v_rates_sp.yaw
  v_rates_sp.thrust
  v_rates_sp.timestamp
  v_control_mode.flag_control_manual_enabled
  att_control
  v_att_sp.landing_gear
  actuators.control
  actuators.timestamp
  actuators.timestamp_sample
  ctrl_state.timestamp
  rates_sp
  thrust_sp
  manual_control_sp.x
  manual_control_sp.r
  manual_control_sp.z
  battery_status.scale
x params.acro_rate_max
  params.bat_scale_en
  controller_status.roll_rate_integ
  controller_status.pitch_rate_integ
  controller_status.yaw_rate_integ
  controller_status.timestamp
%}

%{
MANUAL_THROTTLE_MAX_MULTICOPTER
%}