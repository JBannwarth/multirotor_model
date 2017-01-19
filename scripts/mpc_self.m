function mpc_self() 
% MPC_SELF initializes a set of bus objects in the MATLAB base workspace 

% Bus object: mpc_arming 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'armed';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

mpc_arming = Simulink.Bus;
mpc_arming.HeaderFile = '';
mpc_arming.Description = '';
mpc_arming.DataScope = 'Auto';
mpc_arming.Alignment = -1;
mpc_arming.Elements = elems;
clear elems;
assignin('base','mpc_arming', mpc_arming);

% Bus object: mpc_att_sp 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'disable_mc_yaw_control';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'landing_gear';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'pitch_body';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'q_d';
elems(4).Dimensions = [4 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'q_d_valid';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'roll_body';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'thrust';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'timestamp';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'yaw_body';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'yaw_sp_move_rate';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

mpc_att_sp = Simulink.Bus;
mpc_att_sp.HeaderFile = '';
mpc_att_sp.Description = '';
mpc_att_sp.DataScope = 'Auto';
mpc_att_sp.Alignment = -1;
mpc_att_sp.Elements = elems;
clear elems;
assignin('base','mpc_att_sp', mpc_att_sp);

% Bus object: mpc_control_mode 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'flag_armed';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'flag_control_acceleration_enabled';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'flag_control_altitude_enabled';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'flag_control_attitude_enabled';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'flag_control_climb_rate_enabled';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'flag_control_manual_enabled';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'flag_control_offboard_enabled';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'flag_control_position_enabled';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'flag_control_velocity_enabled';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

mpc_control_mode = Simulink.Bus;
mpc_control_mode.HeaderFile = '';
mpc_control_mode.Description = '';
mpc_control_mode.DataScope = 'Auto';
mpc_control_mode.Alignment = -1;
mpc_control_mode.Elements = elems;
clear elems;
assignin('base','mpc_control_mode', mpc_control_mode);

% Bus object: mpc_ctrl_state 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'delta_q_reset';
elems(1).Dimensions = [4 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'q';
elems(2).Dimensions = [4 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'quat_reset_counter';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

mpc_ctrl_state = Simulink.Bus;
mpc_ctrl_state.HeaderFile = '';
mpc_ctrl_state.Description = '';
mpc_ctrl_state.DataScope = 'Auto';
mpc_ctrl_state.Alignment = -1;
mpc_ctrl_state.Elements = elems;
clear elems;
assignin('base','mpc_ctrl_state', mpc_ctrl_state);

% Bus object: mpc_current 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'a_x';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'a_y';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'a_z';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'acceleration_valid';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'acceptance_radius';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'alt';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'alt_valid';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'cruising_speed';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'disable_mc_yaw_control';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'lat';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'lon';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'double';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'position_valid';
elems(12).Dimensions = [1 1];
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'double';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).SamplingMode = 'Sample based';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

elems(13) = Simulink.BusElement;
elems(13).Name = 'type';
elems(13).Dimensions = [1 1];
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'double';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).SamplingMode = 'Sample based';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = '';
elems(13).Description = '';

elems(14) = Simulink.BusElement;
elems(14).Name = 'valid';
elems(14).Dimensions = [1 1];
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'double';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).SamplingMode = 'Sample based';
elems(14).Min = [];
elems(14).Max = [];
elems(14).DocUnits = '';
elems(14).Description = '';

elems(15) = Simulink.BusElement;
elems(15).Name = 'velocity_frame';
elems(15).Dimensions = [1 1];
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'double';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).SamplingMode = 'Sample based';
elems(15).Min = [];
elems(15).Max = [];
elems(15).DocUnits = '';
elems(15).Description = '';

elems(16) = Simulink.BusElement;
elems(16).Name = 'velocity_valid';
elems(16).Dimensions = [1 1];
elems(16).DimensionsMode = 'Fixed';
elems(16).DataType = 'double';
elems(16).SampleTime = -1;
elems(16).Complexity = 'real';
elems(16).SamplingMode = 'Sample based';
elems(16).Min = [];
elems(16).Max = [];
elems(16).DocUnits = '';
elems(16).Description = '';

elems(17) = Simulink.BusElement;
elems(17).Name = 'vx';
elems(17).Dimensions = [1 1];
elems(17).DimensionsMode = 'Fixed';
elems(17).DataType = 'double';
elems(17).SampleTime = -1;
elems(17).Complexity = 'real';
elems(17).SamplingMode = 'Sample based';
elems(17).Min = [];
elems(17).Max = [];
elems(17).DocUnits = '';
elems(17).Description = '';

elems(18) = Simulink.BusElement;
elems(18).Name = 'vy';
elems(18).Dimensions = [1 1];
elems(18).DimensionsMode = 'Fixed';
elems(18).DataType = 'double';
elems(18).SampleTime = -1;
elems(18).Complexity = 'real';
elems(18).SamplingMode = 'Sample based';
elems(18).Min = [];
elems(18).Max = [];
elems(18).DocUnits = '';
elems(18).Description = '';

elems(19) = Simulink.BusElement;
elems(19).Name = 'vz';
elems(19).Dimensions = [1 1];
elems(19).DimensionsMode = 'Fixed';
elems(19).DataType = 'double';
elems(19).SampleTime = -1;
elems(19).Complexity = 'real';
elems(19).SamplingMode = 'Sample based';
elems(19).Min = [];
elems(19).Max = [];
elems(19).DocUnits = '';
elems(19).Description = '';

elems(20) = Simulink.BusElement;
elems(20).Name = 'x';
elems(20).Dimensions = [1 1];
elems(20).DimensionsMode = 'Fixed';
elems(20).DataType = 'double';
elems(20).SampleTime = -1;
elems(20).Complexity = 'real';
elems(20).SamplingMode = 'Sample based';
elems(20).Min = [];
elems(20).Max = [];
elems(20).DocUnits = '';
elems(20).Description = '';

elems(21) = Simulink.BusElement;
elems(21).Name = 'y';
elems(21).Dimensions = [1 1];
elems(21).DimensionsMode = 'Fixed';
elems(21).DataType = 'double';
elems(21).SampleTime = -1;
elems(21).Complexity = 'real';
elems(21).SamplingMode = 'Sample based';
elems(21).Min = [];
elems(21).Max = [];
elems(21).DocUnits = '';
elems(21).Description = '';

elems(22) = Simulink.BusElement;
elems(22).Name = 'yaw';
elems(22).Dimensions = [1 1];
elems(22).DimensionsMode = 'Fixed';
elems(22).DataType = 'double';
elems(22).SampleTime = -1;
elems(22).Complexity = 'real';
elems(22).SamplingMode = 'Sample based';
elems(22).Min = [];
elems(22).Max = [];
elems(22).DocUnits = '';
elems(22).Description = '';

elems(23) = Simulink.BusElement;
elems(23).Name = 'yaw_valid';
elems(23).Dimensions = [1 1];
elems(23).DimensionsMode = 'Fixed';
elems(23).DataType = 'double';
elems(23).SampleTime = -1;
elems(23).Complexity = 'real';
elems(23).SamplingMode = 'Sample based';
elems(23).Min = [];
elems(23).Max = [];
elems(23).DocUnits = '';
elems(23).Description = '';

elems(24) = Simulink.BusElement;
elems(24).Name = 'yawspeed';
elems(24).Dimensions = [1 1];
elems(24).DimensionsMode = 'Fixed';
elems(24).DataType = 'double';
elems(24).SampleTime = -1;
elems(24).Complexity = 'real';
elems(24).SamplingMode = 'Sample based';
elems(24).Min = [];
elems(24).Max = [];
elems(24).DocUnits = '';
elems(24).Description = '';

elems(25) = Simulink.BusElement;
elems(25).Name = 'yawspeed_valid';
elems(25).Dimensions = [1 1];
elems(25).DimensionsMode = 'Fixed';
elems(25).DataType = 'double';
elems(25).SampleTime = -1;
elems(25).Complexity = 'real';
elems(25).SamplingMode = 'Sample based';
elems(25).Min = [];
elems(25).Max = [];
elems(25).DocUnits = '';
elems(25).Description = '';

elems(26) = Simulink.BusElement;
elems(26).Name = 'z';
elems(26).Dimensions = [1 1];
elems(26).DimensionsMode = 'Fixed';
elems(26).DataType = 'double';
elems(26).SampleTime = -1;
elems(26).Complexity = 'real';
elems(26).SamplingMode = 'Sample based';
elems(26).Min = [];
elems(26).Max = [];
elems(26).DocUnits = '';
elems(26).Description = '';

mpc_current = Simulink.Bus;
mpc_current.HeaderFile = '';
mpc_current.Description = '';
mpc_current.DataScope = 'Auto';
mpc_current.Alignment = -1;
mpc_current.Elements = elems;
clear elems;
assignin('base','mpc_current', mpc_current);

% Bus object: mpc_global_vel_sp 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'vx';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'vy';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'vz';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

mpc_global_vel_sp = Simulink.Bus;
mpc_global_vel_sp.HeaderFile = '';
mpc_global_vel_sp.Description = '';
mpc_global_vel_sp.DataScope = 'Auto';
mpc_global_vel_sp.Alignment = -1;
mpc_global_vel_sp.Elements = elems;
clear elems;
assignin('base','mpc_global_vel_sp', mpc_global_vel_sp);

% Bus object: mpc_local_pos 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'delta_vxy';
elems(1).Dimensions = [2 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'delta_vz';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'delta_xy';
elems(3).Dimensions = [2 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'delta_z';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'dist_bottom';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'dist_bottom_rate';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'dist_bottom_valid';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'ref_alt';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'ref_lat';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'ref_lon';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'ref_timestamp';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'double';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'timestamp';
elems(12).Dimensions = [1 1];
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'double';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).SamplingMode = 'Sample based';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

elems(13) = Simulink.BusElement;
elems(13).Name = 'vx';
elems(13).Dimensions = [1 1];
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'double';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).SamplingMode = 'Sample based';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = '';
elems(13).Description = '';

elems(14) = Simulink.BusElement;
elems(14).Name = 'vxy_reset_counter';
elems(14).Dimensions = [1 1];
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'double';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).SamplingMode = 'Sample based';
elems(14).Min = [];
elems(14).Max = [];
elems(14).DocUnits = '';
elems(14).Description = '';

elems(15) = Simulink.BusElement;
elems(15).Name = 'vy';
elems(15).Dimensions = [1 1];
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'double';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).SamplingMode = 'Sample based';
elems(15).Min = [];
elems(15).Max = [];
elems(15).DocUnits = '';
elems(15).Description = '';

elems(16) = Simulink.BusElement;
elems(16).Name = 'vz';
elems(16).Dimensions = [1 1];
elems(16).DimensionsMode = 'Fixed';
elems(16).DataType = 'double';
elems(16).SampleTime = -1;
elems(16).Complexity = 'real';
elems(16).SamplingMode = 'Sample based';
elems(16).Min = [];
elems(16).Max = [];
elems(16).DocUnits = '';
elems(16).Description = '';

elems(17) = Simulink.BusElement;
elems(17).Name = 'vz_reset_counter';
elems(17).Dimensions = [1 1];
elems(17).DimensionsMode = 'Fixed';
elems(17).DataType = 'double';
elems(17).SampleTime = -1;
elems(17).Complexity = 'real';
elems(17).SamplingMode = 'Sample based';
elems(17).Min = [];
elems(17).Max = [];
elems(17).DocUnits = '';
elems(17).Description = '';

elems(18) = Simulink.BusElement;
elems(18).Name = 'x';
elems(18).Dimensions = [1 1];
elems(18).DimensionsMode = 'Fixed';
elems(18).DataType = 'double';
elems(18).SampleTime = -1;
elems(18).Complexity = 'real';
elems(18).SamplingMode = 'Sample based';
elems(18).Min = [];
elems(18).Max = [];
elems(18).DocUnits = '';
elems(18).Description = '';

elems(19) = Simulink.BusElement;
elems(19).Name = 'xy_reset_counter';
elems(19).Dimensions = [1 1];
elems(19).DimensionsMode = 'Fixed';
elems(19).DataType = 'double';
elems(19).SampleTime = -1;
elems(19).Complexity = 'real';
elems(19).SamplingMode = 'Sample based';
elems(19).Min = [];
elems(19).Max = [];
elems(19).DocUnits = '';
elems(19).Description = '';

elems(20) = Simulink.BusElement;
elems(20).Name = 'xy_valid';
elems(20).Dimensions = [1 1];
elems(20).DimensionsMode = 'Fixed';
elems(20).DataType = 'double';
elems(20).SampleTime = -1;
elems(20).Complexity = 'real';
elems(20).SamplingMode = 'Sample based';
elems(20).Min = [];
elems(20).Max = [];
elems(20).DocUnits = '';
elems(20).Description = '';

elems(21) = Simulink.BusElement;
elems(21).Name = 'y';
elems(21).Dimensions = [1 1];
elems(21).DimensionsMode = 'Fixed';
elems(21).DataType = 'double';
elems(21).SampleTime = -1;
elems(21).Complexity = 'real';
elems(21).SamplingMode = 'Sample based';
elems(21).Min = [];
elems(21).Max = [];
elems(21).DocUnits = '';
elems(21).Description = '';

elems(22) = Simulink.BusElement;
elems(22).Name = 'z';
elems(22).Dimensions = [1 1];
elems(22).DimensionsMode = 'Fixed';
elems(22).DataType = 'double';
elems(22).SampleTime = -1;
elems(22).Complexity = 'real';
elems(22).SamplingMode = 'Sample based';
elems(22).Min = [];
elems(22).Max = [];
elems(22).DocUnits = '';
elems(22).Description = '';

elems(23) = Simulink.BusElement;
elems(23).Name = 'z_reset_counter';
elems(23).Dimensions = [1 1];
elems(23).DimensionsMode = 'Fixed';
elems(23).DataType = 'double';
elems(23).SampleTime = -1;
elems(23).Complexity = 'real';
elems(23).SamplingMode = 'Sample based';
elems(23).Min = [];
elems(23).Max = [];
elems(23).DocUnits = '';
elems(23).Description = '';

elems(24) = Simulink.BusElement;
elems(24).Name = 'z_valid';
elems(24).Dimensions = [1 1];
elems(24).DimensionsMode = 'Fixed';
elems(24).DataType = 'double';
elems(24).SampleTime = -1;
elems(24).Complexity = 'real';
elems(24).SamplingMode = 'Sample based';
elems(24).Min = [];
elems(24).Max = [];
elems(24).DocUnits = '';
elems(24).Description = '';

mpc_local_pos = Simulink.Bus;
mpc_local_pos.HeaderFile = '';
mpc_local_pos.Description = '';
mpc_local_pos.DataScope = 'Auto';
mpc_local_pos.Alignment = -1;
mpc_local_pos.Elements = elems;
clear elems;
assignin('base','mpc_local_pos', mpc_local_pos);

% Bus object: mpc_local_pos_sp 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'acc_x';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'acc_y';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'acc_z';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'timestamp';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'vx';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'vy';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'vz';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'x';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'y';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'yaw';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'z';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'double';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

mpc_local_pos_sp = Simulink.Bus;
mpc_local_pos_sp.HeaderFile = '';
mpc_local_pos_sp.Description = '';
mpc_local_pos_sp.DataScope = 'Auto';
mpc_local_pos_sp.Alignment = -1;
mpc_local_pos_sp.Elements = elems;
clear elems;
assignin('base','mpc_local_pos_sp', mpc_local_pos_sp);

% Bus object: mpc_lp_block 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'dt';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'fcut';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'state';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

mpc_lp_block = Simulink.Bus;
mpc_lp_block.HeaderFile = '';
mpc_lp_block.Description = '';
mpc_lp_block.DataScope = 'Auto';
mpc_lp_block.Alignment = -1;
mpc_lp_block.Elements = elems;
clear elems;
assignin('base','mpc_lp_block', mpc_lp_block);

% Bus object: mpc_manual 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'gear_switch';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'r';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'x';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'y';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'z';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

mpc_manual = Simulink.Bus;
mpc_manual.HeaderFile = '';
mpc_manual.Description = '';
mpc_manual.DataScope = 'Auto';
mpc_manual.Alignment = -1;
mpc_manual.Elements = elems;
clear elems;
assignin('base','mpc_manual', mpc_manual);

% Bus object: mpc_manual_control_setpoint_s 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'SWITCH_POS_OFF';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'SWITCH_POS_ON';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

mpc_manual_control_setpoint_s = Simulink.Bus;
mpc_manual_control_setpoint_s.HeaderFile = '';
mpc_manual_control_setpoint_s.Description = '';
mpc_manual_control_setpoint_s.DataScope = 'Auto';
mpc_manual_control_setpoint_s.Alignment = -1;
mpc_manual_control_setpoint_s.Elements = elems;
clear elems;
assignin('base','mpc_manual_control_setpoint_s', mpc_manual_control_setpoint_s);

% Bus object: mpc_next 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'alt';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'lat';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'lon';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'valid';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

mpc_next = Simulink.Bus;
mpc_next.HeaderFile = '';
mpc_next.Description = '';
mpc_next.DataScope = 'Auto';
mpc_next.Alignment = -1;
mpc_next.Elements = elems;
clear elems;
assignin('base','mpc_next', mpc_next);

% Bus object: mpc_param_find 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'MC_YAWRATE_MAX';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'MC_YAW_P';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'MPC_ACC_HOR_MAX';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'MPC_ALTCTL_DY';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'MPC_ALTCTL_DZ';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'MPC_ALT_MODE';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'MPC_HOLD_MAX_XY';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'MPC_HOLD_MAX_Z';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'MPC_HOLD_XY_DZ';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'MPC_LAND_SPEED';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'MPC_MAN_P_MAX';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'double';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'MPC_MAN_R_MAX';
elems(12).Dimensions = [1 1];
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'double';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).SamplingMode = 'Sample based';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

elems(13) = Simulink.BusElement;
elems(13).Name = 'MPC_MAN_Y_MAX';
elems(13).Dimensions = [1 1];
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'double';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).SamplingMode = 'Sample based';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = '';
elems(13).Description = '';

elems(14) = Simulink.BusElement;
elems(14).Name = 'MPC_THR_HOVER';
elems(14).Dimensions = [1 1];
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'double';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).SamplingMode = 'Sample based';
elems(14).Min = [];
elems(14).Max = [];
elems(14).DocUnits = '';
elems(14).Description = '';

elems(15) = Simulink.BusElement;
elems(15).Name = 'MPC_THR_MAX';
elems(15).Dimensions = [1 1];
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'double';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).SamplingMode = 'Sample based';
elems(15).Min = [];
elems(15).Max = [];
elems(15).DocUnits = '';
elems(15).Description = '';

elems(16) = Simulink.BusElement;
elems(16).Name = 'MPC_THR_MIN';
elems(16).Dimensions = [1 1];
elems(16).DimensionsMode = 'Fixed';
elems(16).DataType = 'double';
elems(16).SampleTime = -1;
elems(16).Complexity = 'real';
elems(16).SamplingMode = 'Sample based';
elems(16).Min = [];
elems(16).Max = [];
elems(16).DocUnits = '';
elems(16).Description = '';

elems(17) = Simulink.BusElement;
elems(17).Name = 'MPC_TILTMAX_AIR';
elems(17).Dimensions = [1 1];
elems(17).DimensionsMode = 'Fixed';
elems(17).DataType = 'double';
elems(17).SampleTime = -1;
elems(17).Complexity = 'real';
elems(17).SamplingMode = 'Sample based';
elems(17).Min = [];
elems(17).Max = [];
elems(17).DocUnits = '';
elems(17).Description = '';

elems(18) = Simulink.BusElement;
elems(18).Name = 'MPC_TILTMAX_LND';
elems(18).Dimensions = [1 1];
elems(18).DimensionsMode = 'Fixed';
elems(18).DataType = 'double';
elems(18).SampleTime = -1;
elems(18).Complexity = 'real';
elems(18).SamplingMode = 'Sample based';
elems(18).Min = [];
elems(18).Max = [];
elems(18).DocUnits = '';
elems(18).Description = '';

elems(19) = Simulink.BusElement;
elems(19).Name = 'MPC_TKO_SPEED';
elems(19).Dimensions = [1 1];
elems(19).DimensionsMode = 'Fixed';
elems(19).DataType = 'double';
elems(19).SampleTime = -1;
elems(19).Complexity = 'real';
elems(19).SamplingMode = 'Sample based';
elems(19).Min = [];
elems(19).Max = [];
elems(19).DocUnits = '';
elems(19).Description = '';

elems(20) = Simulink.BusElement;
elems(20).Name = 'MPC_VELD_LP';
elems(20).Dimensions = [1 1];
elems(20).DimensionsMode = 'Fixed';
elems(20).DataType = 'double';
elems(20).SampleTime = -1;
elems(20).Complexity = 'real';
elems(20).SamplingMode = 'Sample based';
elems(20).Min = [];
elems(20).Max = [];
elems(20).DocUnits = '';
elems(20).Description = '';

elems(21) = Simulink.BusElement;
elems(21).Name = 'MPC_XY_CRUISE';
elems(21).Dimensions = [1 1];
elems(21).DimensionsMode = 'Fixed';
elems(21).DataType = 'double';
elems(21).SampleTime = -1;
elems(21).Complexity = 'real';
elems(21).SamplingMode = 'Sample based';
elems(21).Min = [];
elems(21).Max = [];
elems(21).DocUnits = '';
elems(21).Description = '';

elems(22) = Simulink.BusElement;
elems(22).Name = 'MPC_XY_FF';
elems(22).Dimensions = [1 1];
elems(22).DimensionsMode = 'Fixed';
elems(22).DataType = 'double';
elems(22).SampleTime = -1;
elems(22).Complexity = 'real';
elems(22).SamplingMode = 'Sample based';
elems(22).Min = [];
elems(22).Max = [];
elems(22).DocUnits = '';
elems(22).Description = '';

elems(23) = Simulink.BusElement;
elems(23).Name = 'MPC_XY_P';
elems(23).Dimensions = [1 1];
elems(23).DimensionsMode = 'Fixed';
elems(23).DataType = 'double';
elems(23).SampleTime = -1;
elems(23).Complexity = 'real';
elems(23).SamplingMode = 'Sample based';
elems(23).Min = [];
elems(23).Max = [];
elems(23).DocUnits = '';
elems(23).Description = '';

elems(24) = Simulink.BusElement;
elems(24).Name = 'MPC_XY_VEL_D';
elems(24).Dimensions = [1 1];
elems(24).DimensionsMode = 'Fixed';
elems(24).DataType = 'double';
elems(24).SampleTime = -1;
elems(24).Complexity = 'real';
elems(24).SamplingMode = 'Sample based';
elems(24).Min = [];
elems(24).Max = [];
elems(24).DocUnits = '';
elems(24).Description = '';

elems(25) = Simulink.BusElement;
elems(25).Name = 'MPC_XY_VEL_I';
elems(25).Dimensions = [1 1];
elems(25).DimensionsMode = 'Fixed';
elems(25).DataType = 'double';
elems(25).SampleTime = -1;
elems(25).Complexity = 'real';
elems(25).SamplingMode = 'Sample based';
elems(25).Min = [];
elems(25).Max = [];
elems(25).DocUnits = '';
elems(25).Description = '';

elems(26) = Simulink.BusElement;
elems(26).Name = 'MPC_XY_VEL_MAX';
elems(26).Dimensions = [1 1];
elems(26).DimensionsMode = 'Fixed';
elems(26).DataType = 'double';
elems(26).SampleTime = -1;
elems(26).Complexity = 'real';
elems(26).SamplingMode = 'Sample based';
elems(26).Min = [];
elems(26).Max = [];
elems(26).DocUnits = '';
elems(26).Description = '';

elems(27) = Simulink.BusElement;
elems(27).Name = 'MPC_XY_VEL_P';
elems(27).Dimensions = [1 1];
elems(27).DimensionsMode = 'Fixed';
elems(27).DataType = 'double';
elems(27).SampleTime = -1;
elems(27).Complexity = 'real';
elems(27).SamplingMode = 'Sample based';
elems(27).Min = [];
elems(27).Max = [];
elems(27).DocUnits = '';
elems(27).Description = '';

elems(28) = Simulink.BusElement;
elems(28).Name = 'MPC_Z_FF';
elems(28).Dimensions = [1 1];
elems(28).DimensionsMode = 'Fixed';
elems(28).DataType = 'double';
elems(28).SampleTime = -1;
elems(28).Complexity = 'real';
elems(28).SamplingMode = 'Sample based';
elems(28).Min = [];
elems(28).Max = [];
elems(28).DocUnits = '';
elems(28).Description = '';

elems(29) = Simulink.BusElement;
elems(29).Name = 'MPC_Z_P';
elems(29).Dimensions = [1 1];
elems(29).DimensionsMode = 'Fixed';
elems(29).DataType = 'double';
elems(29).SampleTime = -1;
elems(29).Complexity = 'real';
elems(29).SamplingMode = 'Sample based';
elems(29).Min = [];
elems(29).Max = [];
elems(29).DocUnits = '';
elems(29).Description = '';

elems(30) = Simulink.BusElement;
elems(30).Name = 'MPC_Z_VEL_D';
elems(30).Dimensions = [1 1];
elems(30).DimensionsMode = 'Fixed';
elems(30).DataType = 'double';
elems(30).SampleTime = -1;
elems(30).Complexity = 'real';
elems(30).SamplingMode = 'Sample based';
elems(30).Min = [];
elems(30).Max = [];
elems(30).DocUnits = '';
elems(30).Description = '';

elems(31) = Simulink.BusElement;
elems(31).Name = 'MPC_Z_VEL_I';
elems(31).Dimensions = [1 1];
elems(31).DimensionsMode = 'Fixed';
elems(31).DataType = 'double';
elems(31).SampleTime = -1;
elems(31).Complexity = 'real';
elems(31).SamplingMode = 'Sample based';
elems(31).Min = [];
elems(31).Max = [];
elems(31).DocUnits = '';
elems(31).Description = '';

elems(32) = Simulink.BusElement;
elems(32).Name = 'MPC_Z_VEL_MAX';
elems(32).Dimensions = [1 1];
elems(32).DimensionsMode = 'Fixed';
elems(32).DataType = 'double';
elems(32).SampleTime = -1;
elems(32).Complexity = 'real';
elems(32).SamplingMode = 'Sample based';
elems(32).Min = [];
elems(32).Max = [];
elems(32).DocUnits = '';
elems(32).Description = '';

elems(33) = Simulink.BusElement;
elems(33).Name = 'MPC_Z_VEL_MAX_DN';
elems(33).Dimensions = [1 1];
elems(33).DimensionsMode = 'Fixed';
elems(33).DataType = 'double';
elems(33).SampleTime = -1;
elems(33).Complexity = 'real';
elems(33).SamplingMode = 'Sample based';
elems(33).Min = [];
elems(33).Max = [];
elems(33).DocUnits = '';
elems(33).Description = '';

elems(34) = Simulink.BusElement;
elems(34).Name = 'MPC_Z_VEL_MAX_UP';
elems(34).Dimensions = [1 1];
elems(34).DimensionsMode = 'Fixed';
elems(34).DataType = 'double';
elems(34).SampleTime = -1;
elems(34).Complexity = 'real';
elems(34).SamplingMode = 'Sample based';
elems(34).Min = [];
elems(34).Max = [];
elems(34).DocUnits = '';
elems(34).Description = '';

elems(35) = Simulink.BusElement;
elems(35).Name = 'MPC_Z_VEL_P';
elems(35).Dimensions = [1 1];
elems(35).DimensionsMode = 'Fixed';
elems(35).DataType = 'double';
elems(35).SampleTime = -1;
elems(35).Complexity = 'real';
elems(35).SamplingMode = 'Sample based';
elems(35).Min = [];
elems(35).Max = [];
elems(35).DocUnits = '';
elems(35).Description = '';

elems(36) = Simulink.BusElement;
elems(36).Name = 'VT_OPT_RECOV_EN';
elems(36).Dimensions = [1 1];
elems(36).DimensionsMode = 'Fixed';
elems(36).DataType = 'double';
elems(36).SampleTime = -1;
elems(36).Complexity = 'real';
elems(36).SamplingMode = 'Sample based';
elems(36).Min = [];
elems(36).Max = [];
elems(36).DocUnits = '';
elems(36).Description = '';

mpc_param_find = Simulink.Bus;
mpc_param_find.HeaderFile = '';
mpc_param_find.Description = '';
mpc_param_find.DataScope = 'Auto';
mpc_param_find.Alignment = -1;
mpc_param_find.Elements = elems;
clear elems;
assignin('base','mpc_param_find', mpc_param_find);

% Bus object: mpc_params 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'acc_hor_max';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'alt_ctl_dy';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'alt_ctl_dz';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'alt_mode';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'global_yaw_max';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'hold_max_xy';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'hold_max_z';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'hold_xy_dz';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'land_speed';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'man_pitch_max';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'man_roll_max';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'double';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'man_yaw_max';
elems(12).Dimensions = [1 1];
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'double';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).SamplingMode = 'Sample based';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

elems(13) = Simulink.BusElement;
elems(13).Name = 'mc_att_yaw_p';
elems(13).Dimensions = [1 1];
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'double';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).SamplingMode = 'Sample based';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = '';
elems(13).Description = '';

elems(14) = Simulink.BusElement;
elems(14).Name = 'opt_recover';
elems(14).Dimensions = [1 1];
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'double';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).SamplingMode = 'Sample based';
elems(14).Min = [];
elems(14).Max = [];
elems(14).DocUnits = '';
elems(14).Description = '';

elems(15) = Simulink.BusElement;
elems(15).Name = 'pos_p';
elems(15).Dimensions = [3 1];
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'double';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).SamplingMode = 'Sample based';
elems(15).Min = [];
elems(15).Max = [];
elems(15).DocUnits = '';
elems(15).Description = '';

elems(16) = Simulink.BusElement;
elems(16).Name = 'sp_offs_max';
elems(16).Dimensions = [3 1];
elems(16).DimensionsMode = 'Fixed';
elems(16).DataType = 'double';
elems(16).SampleTime = -1;
elems(16).Complexity = 'real';
elems(16).SamplingMode = 'Sample based';
elems(16).Min = [];
elems(16).Max = [];
elems(16).DocUnits = '';
elems(16).Description = '';

elems(17) = Simulink.BusElement;
elems(17).Name = 'thr_hover';
elems(17).Dimensions = [1 1];
elems(17).DimensionsMode = 'Fixed';
elems(17).DataType = 'double';
elems(17).SampleTime = -1;
elems(17).Complexity = 'real';
elems(17).SamplingMode = 'Sample based';
elems(17).Min = [];
elems(17).Max = [];
elems(17).DocUnits = '';
elems(17).Description = '';

elems(18) = Simulink.BusElement;
elems(18).Name = 'thr_max';
elems(18).Dimensions = [1 1];
elems(18).DimensionsMode = 'Fixed';
elems(18).DataType = 'double';
elems(18).SampleTime = -1;
elems(18).Complexity = 'real';
elems(18).SamplingMode = 'Sample based';
elems(18).Min = [];
elems(18).Max = [];
elems(18).DocUnits = '';
elems(18).Description = '';

elems(19) = Simulink.BusElement;
elems(19).Name = 'thr_min';
elems(19).Dimensions = [1 1];
elems(19).DimensionsMode = 'Fixed';
elems(19).DataType = 'double';
elems(19).SampleTime = -1;
elems(19).Complexity = 'real';
elems(19).SamplingMode = 'Sample based';
elems(19).Min = [];
elems(19).Max = [];
elems(19).DocUnits = '';
elems(19).Description = '';

elems(20) = Simulink.BusElement;
elems(20).Name = 'tilt_max_air';
elems(20).Dimensions = [1 1];
elems(20).DimensionsMode = 'Fixed';
elems(20).DataType = 'double';
elems(20).SampleTime = -1;
elems(20).Complexity = 'real';
elems(20).SamplingMode = 'Sample based';
elems(20).Min = [];
elems(20).Max = [];
elems(20).DocUnits = '';
elems(20).Description = '';

elems(21) = Simulink.BusElement;
elems(21).Name = 'tilt_max_land';
elems(21).Dimensions = [1 1];
elems(21).DimensionsMode = 'Fixed';
elems(21).DataType = 'double';
elems(21).SampleTime = -1;
elems(21).Complexity = 'real';
elems(21).SamplingMode = 'Sample based';
elems(21).Min = [];
elems(21).Max = [];
elems(21).DocUnits = '';
elems(21).Description = '';

elems(22) = Simulink.BusElement;
elems(22).Name = 'tko_speed';
elems(22).Dimensions = [1 1];
elems(22).DimensionsMode = 'Fixed';
elems(22).DataType = 'double';
elems(22).SampleTime = -1;
elems(22).Complexity = 'real';
elems(22).SamplingMode = 'Sample based';
elems(22).Min = [];
elems(22).Max = [];
elems(22).DocUnits = '';
elems(22).Description = '';

elems(23) = Simulink.BusElement;
elems(23).Name = 'vel_cruise';
elems(23).Dimensions = [3 1];
elems(23).DimensionsMode = 'Fixed';
elems(23).DataType = 'double';
elems(23).SampleTime = -1;
elems(23).Complexity = 'real';
elems(23).SamplingMode = 'Sample based';
elems(23).Min = [];
elems(23).Max = [];
elems(23).DocUnits = '';
elems(23).Description = '';

elems(24) = Simulink.BusElement;
elems(24).Name = 'vel_d';
elems(24).Dimensions = [3 1];
elems(24).DimensionsMode = 'Fixed';
elems(24).DataType = 'double';
elems(24).SampleTime = -1;
elems(24).Complexity = 'real';
elems(24).SamplingMode = 'Sample based';
elems(24).Min = [];
elems(24).Max = [];
elems(24).DocUnits = '';
elems(24).Description = '';

elems(25) = Simulink.BusElement;
elems(25).Name = 'vel_ff';
elems(25).Dimensions = [3 1];
elems(25).DimensionsMode = 'Fixed';
elems(25).DataType = 'double';
elems(25).SampleTime = -1;
elems(25).Complexity = 'real';
elems(25).SamplingMode = 'Sample based';
elems(25).Min = [];
elems(25).Max = [];
elems(25).DocUnits = '';
elems(25).Description = '';

elems(26) = Simulink.BusElement;
elems(26).Name = 'vel_i';
elems(26).Dimensions = [3 1];
elems(26).DimensionsMode = 'Fixed';
elems(26).DataType = 'double';
elems(26).SampleTime = -1;
elems(26).Complexity = 'real';
elems(26).SamplingMode = 'Sample based';
elems(26).Min = [];
elems(26).Max = [];
elems(26).DocUnits = '';
elems(26).Description = '';

elems(27) = Simulink.BusElement;
elems(27).Name = 'vel_max';
elems(27).Dimensions = [3 1];
elems(27).DimensionsMode = 'Fixed';
elems(27).DataType = 'double';
elems(27).SampleTime = -1;
elems(27).Complexity = 'real';
elems(27).SamplingMode = 'Sample based';
elems(27).Min = [];
elems(27).Max = [];
elems(27).DocUnits = '';
elems(27).Description = '';

elems(28) = Simulink.BusElement;
elems(28).Name = 'vel_max_down';
elems(28).Dimensions = [1 1];
elems(28).DimensionsMode = 'Fixed';
elems(28).DataType = 'double';
elems(28).SampleTime = -1;
elems(28).Complexity = 'real';
elems(28).SamplingMode = 'Sample based';
elems(28).Min = [];
elems(28).Max = [];
elems(28).DocUnits = '';
elems(28).Description = '';

elems(29) = Simulink.BusElement;
elems(29).Name = 'vel_max_up';
elems(29).Dimensions = [1 1];
elems(29).DimensionsMode = 'Fixed';
elems(29).DataType = 'double';
elems(29).SampleTime = -1;
elems(29).Complexity = 'real';
elems(29).SamplingMode = 'Sample based';
elems(29).Min = [];
elems(29).Max = [];
elems(29).DocUnits = '';
elems(29).Description = '';

elems(30) = Simulink.BusElement;
elems(30).Name = 'vel_p';
elems(30).Dimensions = [3 1];
elems(30).DimensionsMode = 'Fixed';
elems(30).DataType = 'double';
elems(30).SampleTime = -1;
elems(30).Complexity = 'real';
elems(30).SamplingMode = 'Sample based';
elems(30).Min = [];
elems(30).Max = [];
elems(30).DocUnits = '';
elems(30).Description = '';

mpc_params = Simulink.Bus;
mpc_params.HeaderFile = '';
mpc_params.Description = '';
mpc_params.DataScope = 'Auto';
mpc_params.Alignment = -1;
mpc_params.Elements = elems;
clear elems;
assignin('base','mpc_params', mpc_params);

% Bus object: mpc_params_handles 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'acc_hor_max';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'alt_ctl_dy';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'alt_ctl_dz';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'alt_mode';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'global_yaw_max';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'hold_max_xy';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'hold_max_z';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'hold_xy_dz';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'land_speed';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'man_pitch_max';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'man_roll_max';
elems(11).Dimensions = [1 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'double';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'man_yaw_max';
elems(12).Dimensions = [1 1];
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'double';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).SamplingMode = 'Sample based';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

elems(13) = Simulink.BusElement;
elems(13).Name = 'mc_att_yaw_p';
elems(13).Dimensions = [1 1];
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'double';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).SamplingMode = 'Sample based';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = '';
elems(13).Description = '';

elems(14) = Simulink.BusElement;
elems(14).Name = 'opt_recover';
elems(14).Dimensions = [1 1];
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'double';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).SamplingMode = 'Sample based';
elems(14).Min = [];
elems(14).Max = [];
elems(14).DocUnits = '';
elems(14).Description = '';

elems(15) = Simulink.BusElement;
elems(15).Name = 'thr_hover';
elems(15).Dimensions = [1 1];
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'double';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).SamplingMode = 'Sample based';
elems(15).Min = [];
elems(15).Max = [];
elems(15).DocUnits = '';
elems(15).Description = '';

elems(16) = Simulink.BusElement;
elems(16).Name = 'thr_max';
elems(16).Dimensions = [1 1];
elems(16).DimensionsMode = 'Fixed';
elems(16).DataType = 'double';
elems(16).SampleTime = -1;
elems(16).Complexity = 'real';
elems(16).SamplingMode = 'Sample based';
elems(16).Min = [];
elems(16).Max = [];
elems(16).DocUnits = '';
elems(16).Description = '';

elems(17) = Simulink.BusElement;
elems(17).Name = 'thr_min';
elems(17).Dimensions = [1 1];
elems(17).DimensionsMode = 'Fixed';
elems(17).DataType = 'double';
elems(17).SampleTime = -1;
elems(17).Complexity = 'real';
elems(17).SamplingMode = 'Sample based';
elems(17).Min = [];
elems(17).Max = [];
elems(17).DocUnits = '';
elems(17).Description = '';

elems(18) = Simulink.BusElement;
elems(18).Name = 'tilt_max_air';
elems(18).Dimensions = [1 1];
elems(18).DimensionsMode = 'Fixed';
elems(18).DataType = 'double';
elems(18).SampleTime = -1;
elems(18).Complexity = 'real';
elems(18).SamplingMode = 'Sample based';
elems(18).Min = [];
elems(18).Max = [];
elems(18).DocUnits = '';
elems(18).Description = '';

elems(19) = Simulink.BusElement;
elems(19).Name = 'tilt_max_land';
elems(19).Dimensions = [1 1];
elems(19).DimensionsMode = 'Fixed';
elems(19).DataType = 'double';
elems(19).SampleTime = -1;
elems(19).Complexity = 'real';
elems(19).SamplingMode = 'Sample based';
elems(19).Min = [];
elems(19).Max = [];
elems(19).DocUnits = '';
elems(19).Description = '';

elems(20) = Simulink.BusElement;
elems(20).Name = 'tko_speed';
elems(20).Dimensions = [1 1];
elems(20).DimensionsMode = 'Fixed';
elems(20).DataType = 'double';
elems(20).SampleTime = -1;
elems(20).Complexity = 'real';
elems(20).SamplingMode = 'Sample based';
elems(20).Min = [];
elems(20).Max = [];
elems(20).DocUnits = '';
elems(20).Description = '';

elems(21) = Simulink.BusElement;
elems(21).Name = 'xy_ff';
elems(21).Dimensions = [1 1];
elems(21).DimensionsMode = 'Fixed';
elems(21).DataType = 'double';
elems(21).SampleTime = -1;
elems(21).Complexity = 'real';
elems(21).SamplingMode = 'Sample based';
elems(21).Min = [];
elems(21).Max = [];
elems(21).DocUnits = '';
elems(21).Description = '';

elems(22) = Simulink.BusElement;
elems(22).Name = 'xy_p';
elems(22).Dimensions = [1 1];
elems(22).DimensionsMode = 'Fixed';
elems(22).DataType = 'double';
elems(22).SampleTime = -1;
elems(22).Complexity = 'real';
elems(22).SamplingMode = 'Sample based';
elems(22).Min = [];
elems(22).Max = [];
elems(22).DocUnits = '';
elems(22).Description = '';

elems(23) = Simulink.BusElement;
elems(23).Name = 'xy_vel_cruise';
elems(23).Dimensions = [1 1];
elems(23).DimensionsMode = 'Fixed';
elems(23).DataType = 'double';
elems(23).SampleTime = -1;
elems(23).Complexity = 'real';
elems(23).SamplingMode = 'Sample based';
elems(23).Min = [];
elems(23).Max = [];
elems(23).DocUnits = '';
elems(23).Description = '';

elems(24) = Simulink.BusElement;
elems(24).Name = 'xy_vel_d';
elems(24).Dimensions = [1 1];
elems(24).DimensionsMode = 'Fixed';
elems(24).DataType = 'double';
elems(24).SampleTime = -1;
elems(24).Complexity = 'real';
elems(24).SamplingMode = 'Sample based';
elems(24).Min = [];
elems(24).Max = [];
elems(24).DocUnits = '';
elems(24).Description = '';

elems(25) = Simulink.BusElement;
elems(25).Name = 'xy_vel_i';
elems(25).Dimensions = [1 1];
elems(25).DimensionsMode = 'Fixed';
elems(25).DataType = 'double';
elems(25).SampleTime = -1;
elems(25).Complexity = 'real';
elems(25).SamplingMode = 'Sample based';
elems(25).Min = [];
elems(25).Max = [];
elems(25).DocUnits = '';
elems(25).Description = '';

elems(26) = Simulink.BusElement;
elems(26).Name = 'xy_vel_max';
elems(26).Dimensions = [1 1];
elems(26).DimensionsMode = 'Fixed';
elems(26).DataType = 'double';
elems(26).SampleTime = -1;
elems(26).Complexity = 'real';
elems(26).SamplingMode = 'Sample based';
elems(26).Min = [];
elems(26).Max = [];
elems(26).DocUnits = '';
elems(26).Description = '';

elems(27) = Simulink.BusElement;
elems(27).Name = 'xy_vel_p';
elems(27).Dimensions = [1 1];
elems(27).DimensionsMode = 'Fixed';
elems(27).DataType = 'double';
elems(27).SampleTime = -1;
elems(27).Complexity = 'real';
elems(27).SamplingMode = 'Sample based';
elems(27).Min = [];
elems(27).Max = [];
elems(27).DocUnits = '';
elems(27).Description = '';

elems(28) = Simulink.BusElement;
elems(28).Name = 'z_ff';
elems(28).Dimensions = [1 1];
elems(28).DimensionsMode = 'Fixed';
elems(28).DataType = 'double';
elems(28).SampleTime = -1;
elems(28).Complexity = 'real';
elems(28).SamplingMode = 'Sample based';
elems(28).Min = [];
elems(28).Max = [];
elems(28).DocUnits = '';
elems(28).Description = '';

elems(29) = Simulink.BusElement;
elems(29).Name = 'z_p';
elems(29).Dimensions = [1 1];
elems(29).DimensionsMode = 'Fixed';
elems(29).DataType = 'double';
elems(29).SampleTime = -1;
elems(29).Complexity = 'real';
elems(29).SamplingMode = 'Sample based';
elems(29).Min = [];
elems(29).Max = [];
elems(29).DocUnits = '';
elems(29).Description = '';

elems(30) = Simulink.BusElement;
elems(30).Name = 'z_vel_d';
elems(30).Dimensions = [1 1];
elems(30).DimensionsMode = 'Fixed';
elems(30).DataType = 'double';
elems(30).SampleTime = -1;
elems(30).Complexity = 'real';
elems(30).SamplingMode = 'Sample based';
elems(30).Min = [];
elems(30).Max = [];
elems(30).DocUnits = '';
elems(30).Description = '';

elems(31) = Simulink.BusElement;
elems(31).Name = 'z_vel_i';
elems(31).Dimensions = [1 1];
elems(31).DimensionsMode = 'Fixed';
elems(31).DataType = 'double';
elems(31).SampleTime = -1;
elems(31).Complexity = 'real';
elems(31).SamplingMode = 'Sample based';
elems(31).Min = [];
elems(31).Max = [];
elems(31).DocUnits = '';
elems(31).Description = '';

elems(32) = Simulink.BusElement;
elems(32).Name = 'z_vel_max_down';
elems(32).Dimensions = [1 1];
elems(32).DimensionsMode = 'Fixed';
elems(32).DataType = 'double';
elems(32).SampleTime = -1;
elems(32).Complexity = 'real';
elems(32).SamplingMode = 'Sample based';
elems(32).Min = [];
elems(32).Max = [];
elems(32).DocUnits = '';
elems(32).Description = '';

elems(33) = Simulink.BusElement;
elems(33).Name = 'z_vel_max_up';
elems(33).Dimensions = [1 1];
elems(33).DimensionsMode = 'Fixed';
elems(33).DataType = 'double';
elems(33).SampleTime = -1;
elems(33).Complexity = 'real';
elems(33).SamplingMode = 'Sample based';
elems(33).Min = [];
elems(33).Max = [];
elems(33).DocUnits = '';
elems(33).Description = '';

elems(34) = Simulink.BusElement;
elems(34).Name = 'z_vel_p';
elems(34).Dimensions = [1 1];
elems(34).DimensionsMode = 'Fixed';
elems(34).DataType = 'double';
elems(34).SampleTime = -1;
elems(34).Complexity = 'real';
elems(34).SamplingMode = 'Sample based';
elems(34).Min = [];
elems(34).Max = [];
elems(34).DocUnits = '';
elems(34).Description = '';

mpc_params_handles = Simulink.Bus;
mpc_params_handles.HeaderFile = '';
mpc_params_handles.Description = '';
mpc_params_handles.DataScope = 'Auto';
mpc_params_handles.Alignment = -1;
mpc_params_handles.Elements = elems;
clear elems;
assignin('base','mpc_params_handles', mpc_params_handles);

% Bus object: mpc_pos_sp_triplet 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'mpc_current';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'mpc_current';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'mpc_next';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'mpc_next';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'mpc_previous';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'mpc_previous';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

mpc_pos_sp_triplet = Simulink.Bus;
mpc_pos_sp_triplet.HeaderFile = '';
mpc_pos_sp_triplet.Description = '';
mpc_pos_sp_triplet.DataScope = 'Auto';
mpc_pos_sp_triplet.Alignment = -1;
mpc_pos_sp_triplet.Elements = elems;
clear elems;
assignin('base','mpc_pos_sp_triplet', mpc_pos_sp_triplet);

% Bus object: mpc_position_setpoint_s 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'SETPOINT_TYPE_FOLLOW_TARGET';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'SETPOINT_TYPE_IDLE';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'SETPOINT_TYPE_LAND';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'SETPOINT_TYPE_LOITER';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'SETPOINT_TYPE_POSITION';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'SETPOINT_TYPE_TAKEOFF';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'VELOCITY_FRAME_BODY_NED';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'VELOCITY_FRAME_LOCAL_NED';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

mpc_position_setpoint_s = Simulink.Bus;
mpc_position_setpoint_s.HeaderFile = '';
mpc_position_setpoint_s.Description = '';
mpc_position_setpoint_s.DataScope = 'Auto';
mpc_position_setpoint_s.Alignment = -1;
mpc_position_setpoint_s.Elements = elems;
clear elems;
assignin('base','mpc_position_setpoint_s', mpc_position_setpoint_s);

% Bus object: mpc_previous 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'alt';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'lat';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'lon';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'valid';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

mpc_previous = Simulink.Bus;
mpc_previous.HeaderFile = '';
mpc_previous.Description = '';
mpc_previous.DataScope = 'Auto';
mpc_previous.Alignment = -1;
mpc_previous.Elements = elems;
clear elems;
assignin('base','mpc_previous', mpc_previous);

% Bus object: mpc_ref_pos 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'cos_lat';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'init_done';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'lat_rad';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'lon_rad';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'sin_lat';
elems(5).Dimensions = [1 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'timestamp';
elems(6).Dimensions = [1 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

mpc_ref_pos = Simulink.Bus;
mpc_ref_pos.HeaderFile = '';
mpc_ref_pos.Description = '';
mpc_ref_pos.DataScope = 'Auto';
mpc_ref_pos.Alignment = -1;
mpc_ref_pos.Elements = elems;
clear elems;
assignin('base','mpc_ref_pos', mpc_ref_pos);

% Bus object: mpc_vehicle_land_detected 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'landed';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

mpc_vehicle_land_detected = Simulink.Bus;
mpc_vehicle_land_detected.HeaderFile = '';
mpc_vehicle_land_detected.Description = '';
mpc_vehicle_land_detected.DataScope = 'Auto';
mpc_vehicle_land_detected.Alignment = -1;
mpc_vehicle_land_detected.Elements = elems;
clear elems;
assignin('base','mpc_vehicle_land_detected', mpc_vehicle_land_detected);

% Bus object: mpc_vehicle_status 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'is_rotary_wing';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'is_vtol';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

mpc_vehicle_status = Simulink.Bus;
mpc_vehicle_status.HeaderFile = '';
mpc_vehicle_status.Description = '';
mpc_vehicle_status.DataScope = 'Auto';
mpc_vehicle_status.Alignment = -1;
mpc_vehicle_status.Elements = elems;
clear elems;
assignin('base','mpc_vehicle_status', mpc_vehicle_status);

% Bus object: mpc_vel_x_deriv 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'dt';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'initialized';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'mpc_lp_block';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'mpc_lp_block';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'u';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

mpc_vel_x_deriv = Simulink.Bus;
mpc_vel_x_deriv.HeaderFile = '';
mpc_vel_x_deriv.Description = '';
mpc_vel_x_deriv.DataScope = 'Auto';
mpc_vel_x_deriv.Alignment = -1;
mpc_vel_x_deriv.Elements = elems;
clear elems;
assignin('base','mpc_vel_x_deriv', mpc_vel_x_deriv);

% Bus object: mpc_vel_y_deriv 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'dt';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'initialized';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'mpc_lp_block';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'slBus1_mpc_lp_block';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'u';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

mpc_vel_y_deriv = Simulink.Bus;
mpc_vel_y_deriv.HeaderFile = '';
mpc_vel_y_deriv.Description = '';
mpc_vel_y_deriv.DataScope = 'Auto';
mpc_vel_y_deriv.Alignment = -1;
mpc_vel_y_deriv.Elements = elems;
clear elems;
assignin('base','mpc_vel_y_deriv', mpc_vel_y_deriv);

% Bus object: mpc_vel_z_deriv 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'dt';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'initialized';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'mpc_lp_block';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'slBus2_mpc_lp_block';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'u';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

mpc_vel_z_deriv = Simulink.Bus;
mpc_vel_z_deriv.HeaderFile = '';
mpc_vel_z_deriv.Description = '';
mpc_vel_z_deriv.DataScope = 'Auto';
mpc_vel_z_deriv.Alignment = -1;
mpc_vel_z_deriv.Elements = elems;
clear elems;
assignin('base','mpc_vel_z_deriv', mpc_vel_z_deriv);

% Bus object: slBus1_mpc_lp_block 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'dt';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'fcut';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'state';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

slBus1_mpc_lp_block = Simulink.Bus;
slBus1_mpc_lp_block.HeaderFile = '';
slBus1_mpc_lp_block.Description = '';
slBus1_mpc_lp_block.DataScope = 'Auto';
slBus1_mpc_lp_block.Alignment = -1;
slBus1_mpc_lp_block.Elements = elems;
clear elems;
assignin('base','slBus1_mpc_lp_block', slBus1_mpc_lp_block);

% Bus object: slBus2_mpc_lp_block 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'dt';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'fcut';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'state';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

slBus2_mpc_lp_block = Simulink.Bus;
slBus2_mpc_lp_block.HeaderFile = '';
slBus2_mpc_lp_block.Description = '';
slBus2_mpc_lp_block.DataScope = 'Auto';
slBus2_mpc_lp_block.Alignment = -1;
slBus2_mpc_lp_block.Elements = elems;
clear elems;
assignin('base','slBus2_mpc_lp_block', slBus2_mpc_lp_block);

% Bus object: slBus3 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'mpc_FLT_EPSILON';
elems(1).Dimensions = [1 1];
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'mpc_MANUAL_THROTTLE_MAX_MULTICOPTER';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'mpc_MIN_DIST';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'mpc_ONE_G';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

elems(5) = Simulink.BusElement;
elems(5).Name = 'mpc_R';
elems(5).Dimensions = [3 3];
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'double';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];
elems(5).DocUnits = '';
elems(5).Description = '';

elems(6) = Simulink.BusElement;
elems(6).Name = 'mpc_R_setpoint';
elems(6).Dimensions = [3 3];
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'double';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];
elems(6).DocUnits = '';
elems(6).Description = '';

elems(7) = Simulink.BusElement;
elems(7).Name = 'mpc_SIGMA';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'double';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];
elems(7).DocUnits = '';
elems(7).Description = '';

elems(8) = Simulink.BusElement;
elems(8).Name = 'mpc_TILT_COS_MAX';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'double';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];
elems(8).DocUnits = '';
elems(8).Description = '';

elems(9) = Simulink.BusElement;
elems(9).Name = 'mpc_acc_z_lp';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'double';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];
elems(9).DocUnits = '';
elems(9).Description = '';

elems(10) = Simulink.BusElement;
elems(10).Name = 'mpc_alt_hold_engaged';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'double';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];
elems(10).DocUnits = '';
elems(10).Description = '';

elems(11) = Simulink.BusElement;
elems(11).Name = 'mpc_arming';
elems(11).Dimensions = 1;
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'mpc_arming';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';
elems(11).Min = [];
elems(11).Max = [];
elems(11).DocUnits = '';
elems(11).Description = '';

elems(12) = Simulink.BusElement;
elems(12).Name = 'mpc_arming_sub';
elems(12).Dimensions = [1 1];
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'double';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).SamplingMode = 'Sample based';
elems(12).Min = [];
elems(12).Max = [];
elems(12).DocUnits = '';
elems(12).Description = '';

elems(13) = Simulink.BusElement;
elems(13).Name = 'mpc_att_sp';
elems(13).Dimensions = 1;
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'mpc_att_sp';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).SamplingMode = 'Sample based';
elems(13).Min = [];
elems(13).Max = [];
elems(13).DocUnits = '';
elems(13).Description = '';

elems(14) = Simulink.BusElement;
elems(14).Name = 'mpc_att_sp_sub';
elems(14).Dimensions = [1 1];
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'double';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).SamplingMode = 'Sample based';
elems(14).Min = [];
elems(14).Max = [];
elems(14).DocUnits = '';
elems(14).Description = '';

elems(15) = Simulink.BusElement;
elems(15).Name = 'mpc_control_mode';
elems(15).Dimensions = 1;
elems(15).DimensionsMode = 'Fixed';
elems(15).DataType = 'mpc_control_mode';
elems(15).SampleTime = -1;
elems(15).Complexity = 'real';
elems(15).SamplingMode = 'Sample based';
elems(15).Min = [];
elems(15).Max = [];
elems(15).DocUnits = '';
elems(15).Description = '';

elems(16) = Simulink.BusElement;
elems(16).Name = 'mpc_control_mode_sub';
elems(16).Dimensions = [1 1];
elems(16).DimensionsMode = 'Fixed';
elems(16).DataType = 'double';
elems(16).SampleTime = -1;
elems(16).Complexity = 'real';
elems(16).SamplingMode = 'Sample based';
elems(16).Min = [];
elems(16).Max = [];
elems(16).DocUnits = '';
elems(16).Description = '';

elems(17) = Simulink.BusElement;
elems(17).Name = 'mpc_control_vel_enabled_prev';
elems(17).Dimensions = [1 1];
elems(17).DimensionsMode = 'Fixed';
elems(17).DataType = 'double';
elems(17).SampleTime = -1;
elems(17).Complexity = 'real';
elems(17).SamplingMode = 'Sample based';
elems(17).Min = [];
elems(17).Max = [];
elems(17).DocUnits = '';
elems(17).Description = '';

elems(18) = Simulink.BusElement;
elems(18).Name = 'mpc_ctrl_state';
elems(18).Dimensions = 1;
elems(18).DimensionsMode = 'Fixed';
elems(18).DataType = 'mpc_ctrl_state';
elems(18).SampleTime = -1;
elems(18).Complexity = 'real';
elems(18).SamplingMode = 'Sample based';
elems(18).Min = [];
elems(18).Max = [];
elems(18).DocUnits = '';
elems(18).Description = '';

elems(19) = Simulink.BusElement;
elems(19).Name = 'mpc_ctrl_state_sub';
elems(19).Dimensions = [1 1];
elems(19).DimensionsMode = 'Fixed';
elems(19).DataType = 'double';
elems(19).SampleTime = -1;
elems(19).Complexity = 'real';
elems(19).SamplingMode = 'Sample based';
elems(19).Min = [];
elems(19).Max = [];
elems(19).DocUnits = '';
elems(19).Description = '';

elems(20) = Simulink.BusElement;
elems(20).Name = 'mpc_do_reset_alt_pos_flag';
elems(20).Dimensions = [1 1];
elems(20).DimensionsMode = 'Fixed';
elems(20).DataType = 'double';
elems(20).SampleTime = -1;
elems(20).Complexity = 'real';
elems(20).SamplingMode = 'Sample based';
elems(20).Min = [];
elems(20).Max = [];
elems(20).DocUnits = '';
elems(20).Description = '';

elems(21) = Simulink.BusElement;
elems(21).Name = 'mpc_global_vel_sp';
elems(21).Dimensions = 1;
elems(21).DimensionsMode = 'Fixed';
elems(21).DataType = 'mpc_global_vel_sp';
elems(21).SampleTime = -1;
elems(21).Complexity = 'real';
elems(21).SamplingMode = 'Sample based';
elems(21).Min = [];
elems(21).Max = [];
elems(21).DocUnits = '';
elems(21).Description = '';

elems(22) = Simulink.BusElement;
elems(22).Name = 'mpc_heading_reset_counter';
elems(22).Dimensions = [1 1];
elems(22).DimensionsMode = 'Fixed';
elems(22).DataType = 'double';
elems(22).SampleTime = -1;
elems(22).Complexity = 'real';
elems(22).SamplingMode = 'Sample based';
elems(22).Min = [];
elems(22).Max = [];
elems(22).DocUnits = '';
elems(22).Description = '';

elems(23) = Simulink.BusElement;
elems(23).Name = 'mpc_hold_offboard_xy';
elems(23).Dimensions = [1 1];
elems(23).DimensionsMode = 'Fixed';
elems(23).DataType = 'double';
elems(23).SampleTime = -1;
elems(23).Complexity = 'real';
elems(23).SamplingMode = 'Sample based';
elems(23).Min = [];
elems(23).Max = [];
elems(23).DocUnits = '';
elems(23).Description = '';

elems(24) = Simulink.BusElement;
elems(24).Name = 'mpc_hold_offboard_z';
elems(24).Dimensions = [1 1];
elems(24).DimensionsMode = 'Fixed';
elems(24).DataType = 'double';
elems(24).SampleTime = -1;
elems(24).Complexity = 'real';
elems(24).SamplingMode = 'Sample based';
elems(24).Min = [];
elems(24).Max = [];
elems(24).DocUnits = '';
elems(24).Description = '';

elems(25) = Simulink.BusElement;
elems(25).Name = 'mpc_hrt_absolute_time';
elems(25).Dimensions = [1 1];
elems(25).DimensionsMode = 'Fixed';
elems(25).DataType = 'double';
elems(25).SampleTime = -1;
elems(25).Complexity = 'real';
elems(25).SamplingMode = 'Sample based';
elems(25).Min = [];
elems(25).Max = [];
elems(25).DocUnits = '';
elems(25).Description = '';

elems(26) = Simulink.BusElement;
elems(26).Name = 'mpc_in_landing';
elems(26).Dimensions = [1 1];
elems(26).DimensionsMode = 'Fixed';
elems(26).DataType = 'double';
elems(26).SampleTime = -1;
elems(26).Complexity = 'real';
elems(26).SamplingMode = 'Sample based';
elems(26).Min = [];
elems(26).Max = [];
elems(26).DocUnits = '';
elems(26).Description = '';

elems(27) = Simulink.BusElement;
elems(27).Name = 'mpc_initialized';
elems(27).Dimensions = [1 1];
elems(27).DimensionsMode = 'Fixed';
elems(27).DataType = 'double';
elems(27).SampleTime = -1;
elems(27).Complexity = 'real';
elems(27).SamplingMode = 'Sample based';
elems(27).Min = [];
elems(27).Max = [];
elems(27).DocUnits = '';
elems(27).Description = '';

elems(28) = Simulink.BusElement;
elems(28).Name = 'mpc_lnd_reached_ground';
elems(28).Dimensions = [1 1];
elems(28).DimensionsMode = 'Fixed';
elems(28).DataType = 'double';
elems(28).SampleTime = -1;
elems(28).Complexity = 'real';
elems(28).SamplingMode = 'Sample based';
elems(28).Min = [];
elems(28).Max = [];
elems(28).DocUnits = '';
elems(28).Description = '';

elems(29) = Simulink.BusElement;
elems(29).Name = 'mpc_local_pos';
elems(29).Dimensions = 1;
elems(29).DimensionsMode = 'Fixed';
elems(29).DataType = 'mpc_local_pos';
elems(29).SampleTime = -1;
elems(29).Complexity = 'real';
elems(29).SamplingMode = 'Sample based';
elems(29).Min = [];
elems(29).Max = [];
elems(29).DocUnits = '';
elems(29).Description = '';

elems(30) = Simulink.BusElement;
elems(30).Name = 'mpc_local_pos_sp';
elems(30).Dimensions = 1;
elems(30).DimensionsMode = 'Fixed';
elems(30).DataType = 'mpc_local_pos_sp';
elems(30).SampleTime = -1;
elems(30).Complexity = 'real';
elems(30).SamplingMode = 'Sample based';
elems(30).Min = [];
elems(30).Max = [];
elems(30).DocUnits = '';
elems(30).Description = '';

elems(31) = Simulink.BusElement;
elems(31).Name = 'mpc_local_pos_sub';
elems(31).Dimensions = [1 1];
elems(31).DimensionsMode = 'Fixed';
elems(31).DataType = 'double';
elems(31).SampleTime = -1;
elems(31).Complexity = 'real';
elems(31).SamplingMode = 'Sample based';
elems(31).Min = [];
elems(31).Max = [];
elems(31).DocUnits = '';
elems(31).Description = '';

elems(32) = Simulink.BusElement;
elems(32).Name = 'mpc_manual';
elems(32).Dimensions = 1;
elems(32).DimensionsMode = 'Fixed';
elems(32).DataType = 'mpc_manual';
elems(32).SampleTime = -1;
elems(32).Complexity = 'real';
elems(32).SamplingMode = 'Sample based';
elems(32).Min = [];
elems(32).Max = [];
elems(32).DocUnits = '';
elems(32).Description = '';

elems(33) = Simulink.BusElement;
elems(33).Name = 'mpc_manual_control_setpoint_s';
elems(33).Dimensions = 1;
elems(33).DimensionsMode = 'Fixed';
elems(33).DataType = 'mpc_manual_control_setpoint_s';
elems(33).SampleTime = -1;
elems(33).Complexity = 'real';
elems(33).SamplingMode = 'Sample based';
elems(33).Min = [];
elems(33).Max = [];
elems(33).DocUnits = '';
elems(33).Description = '';

elems(34) = Simulink.BusElement;
elems(34).Name = 'mpc_manual_sub';
elems(34).Dimensions = [1 1];
elems(34).DimensionsMode = 'Fixed';
elems(34).DataType = 'double';
elems(34).SampleTime = -1;
elems(34).Complexity = 'real';
elems(34).SamplingMode = 'Sample based';
elems(34).Min = [];
elems(34).Max = [];
elems(34).DocUnits = '';
elems(34).Description = '';

elems(35) = Simulink.BusElement;
elems(35).Name = 'mpc_manual_thr_max';
elems(35).Dimensions = [1 1];
elems(35).DimensionsMode = 'Fixed';
elems(35).DataType = 'double';
elems(35).SampleTime = -1;
elems(35).Complexity = 'real';
elems(35).SamplingMode = 'Sample based';
elems(35).Min = [];
elems(35).Max = [];
elems(35).DocUnits = '';
elems(35).Description = '';

elems(36) = Simulink.BusElement;
elems(36).Name = 'mpc_manual_thr_min';
elems(36).Dimensions = [1 1];
elems(36).DimensionsMode = 'Fixed';
elems(36).DataType = 'double';
elems(36).SampleTime = -1;
elems(36).Complexity = 'real';
elems(36).SamplingMode = 'Sample based';
elems(36).Min = [];
elems(36).Max = [];
elems(36).DocUnits = '';
elems(36).Description = '';

elems(37) = Simulink.BusElement;
elems(37).Name = 'mpc_mode_auto';
elems(37).Dimensions = [1 1];
elems(37).DimensionsMode = 'Fixed';
elems(37).DataType = 'double';
elems(37).SampleTime = -1;
elems(37).Complexity = 'real';
elems(37).SamplingMode = 'Sample based';
elems(37).Min = [];
elems(37).Max = [];
elems(37).DocUnits = '';
elems(37).Description = '';

elems(38) = Simulink.BusElement;
elems(38).Name = 'mpc_param_find';
elems(38).Dimensions = 1;
elems(38).DimensionsMode = 'Fixed';
elems(38).DataType = 'mpc_param_find';
elems(38).SampleTime = -1;
elems(38).Complexity = 'real';
elems(38).SamplingMode = 'Sample based';
elems(38).Min = [];
elems(38).Max = [];
elems(38).DocUnits = '';
elems(38).Description = '';

elems(39) = Simulink.BusElement;
elems(39).Name = 'mpc_params';
elems(39).Dimensions = 1;
elems(39).DimensionsMode = 'Fixed';
elems(39).DataType = 'mpc_params';
elems(39).SampleTime = -1;
elems(39).Complexity = 'real';
elems(39).SamplingMode = 'Sample based';
elems(39).Min = [];
elems(39).Max = [];
elems(39).DocUnits = '';
elems(39).Description = '';

elems(40) = Simulink.BusElement;
elems(40).Name = 'mpc_params_handles';
elems(40).Dimensions = 1;
elems(40).DimensionsMode = 'Fixed';
elems(40).DataType = 'mpc_params_handles';
elems(40).SampleTime = -1;
elems(40).Complexity = 'real';
elems(40).SamplingMode = 'Sample based';
elems(40).Min = [];
elems(40).Max = [];
elems(40).DocUnits = '';
elems(40).Description = '';

elems(41) = Simulink.BusElement;
elems(41).Name = 'mpc_params_sub';
elems(41).Dimensions = [1 1];
elems(41).DimensionsMode = 'Fixed';
elems(41).DataType = 'double';
elems(41).SampleTime = -1;
elems(41).Complexity = 'real';
elems(41).SamplingMode = 'Sample based';
elems(41).Min = [];
elems(41).Max = [];
elems(41).DocUnits = '';
elems(41).Description = '';

elems(42) = Simulink.BusElement;
elems(42).Name = 'mpc_pos';
elems(42).Dimensions = [3 1];
elems(42).DimensionsMode = 'Fixed';
elems(42).DataType = 'double';
elems(42).SampleTime = -1;
elems(42).Complexity = 'real';
elems(42).SamplingMode = 'Sample based';
elems(42).Min = [];
elems(42).Max = [];
elems(42).DocUnits = '';
elems(42).Description = '';

elems(43) = Simulink.BusElement;
elems(43).Name = 'mpc_pos_hold_engaged';
elems(43).Dimensions = [1 1];
elems(43).DimensionsMode = 'Fixed';
elems(43).DataType = 'double';
elems(43).SampleTime = -1;
elems(43).Complexity = 'real';
elems(43).SamplingMode = 'Sample based';
elems(43).Min = [];
elems(43).Max = [];
elems(43).DocUnits = '';
elems(43).Description = '';

elems(44) = Simulink.BusElement;
elems(44).Name = 'mpc_pos_sp';
elems(44).Dimensions = [3 1];
elems(44).DimensionsMode = 'Fixed';
elems(44).DataType = 'double';
elems(44).SampleTime = -1;
elems(44).Complexity = 'real';
elems(44).SamplingMode = 'Sample based';
elems(44).Min = [];
elems(44).Max = [];
elems(44).DocUnits = '';
elems(44).Description = '';

elems(45) = Simulink.BusElement;
elems(45).Name = 'mpc_pos_sp_triplet';
elems(45).Dimensions = 1;
elems(45).DimensionsMode = 'Fixed';
elems(45).DataType = 'mpc_pos_sp_triplet';
elems(45).SampleTime = -1;
elems(45).Complexity = 'real';
elems(45).SamplingMode = 'Sample based';
elems(45).Min = [];
elems(45).Max = [];
elems(45).DocUnits = '';
elems(45).Description = '';

elems(46) = Simulink.BusElement;
elems(46).Name = 'mpc_pos_sp_triplet_sub';
elems(46).Dimensions = [1 1];
elems(46).DimensionsMode = 'Fixed';
elems(46).DataType = 'double';
elems(46).SampleTime = -1;
elems(46).Complexity = 'real';
elems(46).SamplingMode = 'Sample based';
elems(46).Min = [];
elems(46).Max = [];
elems(46).DocUnits = '';
elems(46).Description = '';

elems(47) = Simulink.BusElement;
elems(47).Name = 'mpc_position_setpoint_s';
elems(47).Dimensions = 1;
elems(47).DimensionsMode = 'Fixed';
elems(47).DataType = 'mpc_position_setpoint_s';
elems(47).SampleTime = -1;
elems(47).Complexity = 'real';
elems(47).SamplingMode = 'Sample based';
elems(47).Min = [];
elems(47).Max = [];
elems(47).DocUnits = '';
elems(47).Description = '';

elems(48) = Simulink.BusElement;
elems(48).Name = 'mpc_ref_alt';
elems(48).Dimensions = [1 1];
elems(48).DimensionsMode = 'Fixed';
elems(48).DataType = 'double';
elems(48).SampleTime = -1;
elems(48).Complexity = 'real';
elems(48).SamplingMode = 'Sample based';
elems(48).Min = [];
elems(48).Max = [];
elems(48).DocUnits = '';
elems(48).Description = '';

elems(49) = Simulink.BusElement;
elems(49).Name = 'mpc_ref_pos';
elems(49).Dimensions = 1;
elems(49).DimensionsMode = 'Fixed';
elems(49).DataType = 'mpc_ref_pos';
elems(49).SampleTime = -1;
elems(49).Complexity = 'real';
elems(49).SamplingMode = 'Sample based';
elems(49).Min = [];
elems(49).Max = [];
elems(49).DocUnits = '';
elems(49).Description = '';

elems(50) = Simulink.BusElement;
elems(50).Name = 'mpc_ref_timestamp';
elems(50).Dimensions = [1 1];
elems(50).DimensionsMode = 'Fixed';
elems(50).DataType = 'double';
elems(50).SampleTime = -1;
elems(50).Complexity = 'real';
elems(50).SamplingMode = 'Sample based';
elems(50).Min = [];
elems(50).Max = [];
elems(50).DocUnits = '';
elems(50).Description = '';

elems(51) = Simulink.BusElement;
elems(51).Name = 'mpc_reset_alt_sp';
elems(51).Dimensions = [1 1];
elems(51).DimensionsMode = 'Fixed';
elems(51).DataType = 'double';
elems(51).SampleTime = -1;
elems(51).Complexity = 'real';
elems(51).SamplingMode = 'Sample based';
elems(51).Min = [];
elems(51).Max = [];
elems(51).DocUnits = '';
elems(51).Description = '';

elems(52) = Simulink.BusElement;
elems(52).Name = 'mpc_reset_int_xy';
elems(52).Dimensions = [1 1];
elems(52).DimensionsMode = 'Fixed';
elems(52).DataType = 'double';
elems(52).SampleTime = -1;
elems(52).Complexity = 'real';
elems(52).SamplingMode = 'Sample based';
elems(52).Min = [];
elems(52).Max = [];
elems(52).DocUnits = '';
elems(52).Description = '';

elems(53) = Simulink.BusElement;
elems(53).Name = 'mpc_reset_int_z';
elems(53).Dimensions = [1 1];
elems(53).DimensionsMode = 'Fixed';
elems(53).DataType = 'double';
elems(53).SampleTime = -1;
elems(53).Complexity = 'real';
elems(53).SamplingMode = 'Sample based';
elems(53).Min = [];
elems(53).Max = [];
elems(53).DocUnits = '';
elems(53).Description = '';

elems(54) = Simulink.BusElement;
elems(54).Name = 'mpc_reset_int_z_manual';
elems(54).Dimensions = [1 1];
elems(54).DimensionsMode = 'Fixed';
elems(54).DataType = 'double';
elems(54).SampleTime = -1;
elems(54).Complexity = 'real';
elems(54).SamplingMode = 'Sample based';
elems(54).Min = [];
elems(54).Max = [];
elems(54).DocUnits = '';
elems(54).Description = '';

elems(55) = Simulink.BusElement;
elems(55).Name = 'mpc_reset_pos_sp';
elems(55).Dimensions = [1 1];
elems(55).DimensionsMode = 'Fixed';
elems(55).DataType = 'double';
elems(55).SampleTime = -1;
elems(55).Complexity = 'real';
elems(55).SamplingMode = 'Sample based';
elems(55).Min = [];
elems(55).Max = [];
elems(55).DocUnits = '';
elems(55).Description = '';

elems(56) = Simulink.BusElement;
elems(56).Name = 'mpc_reset_yaw_sp';
elems(56).Dimensions = [1 1];
elems(56).DimensionsMode = 'Fixed';
elems(56).DataType = 'double';
elems(56).SampleTime = -1;
elems(56).Complexity = 'real';
elems(56).SamplingMode = 'Sample based';
elems(56).Min = [];
elems(56).Max = [];
elems(56).DocUnits = '';
elems(56).Description = '';

elems(57) = Simulink.BusElement;
elems(57).Name = 'mpc_run_alt_control';
elems(57).Dimensions = [1 1];
elems(57).DimensionsMode = 'Fixed';
elems(57).DataType = 'double';
elems(57).SampleTime = -1;
elems(57).Complexity = 'real';
elems(57).SamplingMode = 'Sample based';
elems(57).Min = [];
elems(57).Max = [];
elems(57).DocUnits = '';
elems(57).Description = '';

elems(58) = Simulink.BusElement;
elems(58).Name = 'mpc_run_pos_control';
elems(58).Dimensions = [1 1];
elems(58).DimensionsMode = 'Fixed';
elems(58).DataType = 'double';
elems(58).SampleTime = -1;
elems(58).Complexity = 'real';
elems(58).SamplingMode = 'Sample based';
elems(58).Min = [];
elems(58).Max = [];
elems(58).DocUnits = '';
elems(58).Description = '';

elems(59) = Simulink.BusElement;
elems(59).Name = 'mpc_t';
elems(59).Dimensions = [1 1];
elems(59).DimensionsMode = 'Fixed';
elems(59).DataType = 'double';
elems(59).SampleTime = -1;
elems(59).Complexity = 'real';
elems(59).SamplingMode = 'Sample based';
elems(59).Min = [];
elems(59).Max = [];
elems(59).DocUnits = '';
elems(59).Description = '';

elems(60) = Simulink.BusElement;
elems(60).Name = 'mpc_t_prev';
elems(60).Dimensions = [1 1];
elems(60).DimensionsMode = 'Fixed';
elems(60).DataType = 'double';
elems(60).SampleTime = -1;
elems(60).Complexity = 'real';
elems(60).SamplingMode = 'Sample based';
elems(60).Min = [];
elems(60).Max = [];
elems(60).DocUnits = '';
elems(60).Description = '';

elems(61) = Simulink.BusElement;
elems(61).Name = 'mpc_takeoff_jumped';
elems(61).Dimensions = [1 1];
elems(61).DimensionsMode = 'Fixed';
elems(61).DataType = 'double';
elems(61).SampleTime = -1;
elems(61).Complexity = 'real';
elems(61).SamplingMode = 'Sample based';
elems(61).Min = [];
elems(61).Max = [];
elems(61).DocUnits = '';
elems(61).Description = '';

elems(62) = Simulink.BusElement;
elems(62).Name = 'mpc_takeoff_thrust_sp';
elems(62).Dimensions = [1 1];
elems(62).DimensionsMode = 'Fixed';
elems(62).DataType = 'double';
elems(62).SampleTime = -1;
elems(62).Complexity = 'real';
elems(62).SamplingMode = 'Sample based';
elems(62).Min = [];
elems(62).Max = [];
elems(62).DocUnits = '';
elems(62).Description = '';

elems(63) = Simulink.BusElement;
elems(63).Name = 'mpc_thrust_int';
elems(63).Dimensions = [3 1];
elems(63).DimensionsMode = 'Fixed';
elems(63).DataType = 'double';
elems(63).SampleTime = -1;
elems(63).Complexity = 'real';
elems(63).SamplingMode = 'Sample based';
elems(63).Min = [];
elems(63).Max = [];
elems(63).DocUnits = '';
elems(63).Description = '';

elems(64) = Simulink.BusElement;
elems(64).Name = 'mpc_vehicle_land_detected';
elems(64).Dimensions = 1;
elems(64).DimensionsMode = 'Fixed';
elems(64).DataType = 'mpc_vehicle_land_detected';
elems(64).SampleTime = -1;
elems(64).Complexity = 'real';
elems(64).SamplingMode = 'Sample based';
elems(64).Min = [];
elems(64).Max = [];
elems(64).DocUnits = '';
elems(64).Description = '';

elems(65) = Simulink.BusElement;
elems(65).Name = 'mpc_vehicle_land_detected_sub';
elems(65).Dimensions = [1 1];
elems(65).DimensionsMode = 'Fixed';
elems(65).DataType = 'double';
elems(65).SampleTime = -1;
elems(65).Complexity = 'real';
elems(65).SamplingMode = 'Sample based';
elems(65).Min = [];
elems(65).Max = [];
elems(65).DocUnits = '';
elems(65).Description = '';

elems(66) = Simulink.BusElement;
elems(66).Name = 'mpc_vehicle_status';
elems(66).Dimensions = 1;
elems(66).DimensionsMode = 'Fixed';
elems(66).DataType = 'mpc_vehicle_status';
elems(66).SampleTime = -1;
elems(66).Complexity = 'real';
elems(66).SamplingMode = 'Sample based';
elems(66).Min = [];
elems(66).Max = [];
elems(66).DocUnits = '';
elems(66).Description = '';

elems(67) = Simulink.BusElement;
elems(67).Name = 'mpc_vehicle_status_sub';
elems(67).Dimensions = [1 1];
elems(67).DimensionsMode = 'Fixed';
elems(67).DataType = 'double';
elems(67).SampleTime = -1;
elems(67).Complexity = 'real';
elems(67).SamplingMode = 'Sample based';
elems(67).Min = [];
elems(67).Max = [];
elems(67).DocUnits = '';
elems(67).Description = '';

elems(68) = Simulink.BusElement;
elems(68).Name = 'mpc_vel';
elems(68).Dimensions = [3 1];
elems(68).DimensionsMode = 'Fixed';
elems(68).DataType = 'double';
elems(68).SampleTime = -1;
elems(68).Complexity = 'real';
elems(68).SamplingMode = 'Sample based';
elems(68).Min = [];
elems(68).Max = [];
elems(68).DocUnits = '';
elems(68).Description = '';

elems(69) = Simulink.BusElement;
elems(69).Name = 'mpc_vel_err_d';
elems(69).Dimensions = [3 1];
elems(69).DimensionsMode = 'Fixed';
elems(69).DataType = 'double';
elems(69).SampleTime = -1;
elems(69).Complexity = 'real';
elems(69).SamplingMode = 'Sample based';
elems(69).Min = [];
elems(69).Max = [];
elems(69).DocUnits = '';
elems(69).Description = '';

elems(70) = Simulink.BusElement;
elems(70).Name = 'mpc_vel_ff';
elems(70).Dimensions = [3 1];
elems(70).DimensionsMode = 'Fixed';
elems(70).DataType = 'double';
elems(70).SampleTime = -1;
elems(70).Complexity = 'real';
elems(70).SamplingMode = 'Sample based';
elems(70).Min = [];
elems(70).Max = [];
elems(70).DocUnits = '';
elems(70).Description = '';

elems(71) = Simulink.BusElement;
elems(71).Name = 'mpc_vel_prev';
elems(71).Dimensions = [3 1];
elems(71).DimensionsMode = 'Fixed';
elems(71).DataType = 'double';
elems(71).SampleTime = -1;
elems(71).Complexity = 'real';
elems(71).SamplingMode = 'Sample based';
elems(71).Min = [];
elems(71).Max = [];
elems(71).DocUnits = '';
elems(71).Description = '';

elems(72) = Simulink.BusElement;
elems(72).Name = 'mpc_vel_sp';
elems(72).Dimensions = [3 1];
elems(72).DimensionsMode = 'Fixed';
elems(72).DataType = 'double';
elems(72).SampleTime = -1;
elems(72).Complexity = 'real';
elems(72).SamplingMode = 'Sample based';
elems(72).Min = [];
elems(72).Max = [];
elems(72).DocUnits = '';
elems(72).Description = '';

elems(73) = Simulink.BusElement;
elems(73).Name = 'mpc_vel_sp_prev';
elems(73).Dimensions = [3 1];
elems(73).DimensionsMode = 'Fixed';
elems(73).DataType = 'double';
elems(73).SampleTime = -1;
elems(73).Complexity = 'real';
elems(73).SamplingMode = 'Sample based';
elems(73).Min = [];
elems(73).Max = [];
elems(73).DocUnits = '';
elems(73).Description = '';

elems(74) = Simulink.BusElement;
elems(74).Name = 'mpc_vel_x_deriv';
elems(74).Dimensions = 1;
elems(74).DimensionsMode = 'Fixed';
elems(74).DataType = 'mpc_vel_x_deriv';
elems(74).SampleTime = -1;
elems(74).Complexity = 'real';
elems(74).SamplingMode = 'Sample based';
elems(74).Min = [];
elems(74).Max = [];
elems(74).DocUnits = '';
elems(74).Description = '';

elems(75) = Simulink.BusElement;
elems(75).Name = 'mpc_vel_y_deriv';
elems(75).Dimensions = 1;
elems(75).DimensionsMode = 'Fixed';
elems(75).DataType = 'mpc_vel_y_deriv';
elems(75).SampleTime = -1;
elems(75).Complexity = 'real';
elems(75).SamplingMode = 'Sample based';
elems(75).Min = [];
elems(75).Max = [];
elems(75).DocUnits = '';
elems(75).Description = '';

elems(76) = Simulink.BusElement;
elems(76).Name = 'mpc_vel_z_deriv';
elems(76).Dimensions = 1;
elems(76).DimensionsMode = 'Fixed';
elems(76).DataType = 'mpc_vel_z_deriv';
elems(76).SampleTime = -1;
elems(76).Complexity = 'real';
elems(76).SamplingMode = 'Sample based';
elems(76).Min = [];
elems(76).Max = [];
elems(76).DocUnits = '';
elems(76).Description = '';

elems(77) = Simulink.BusElement;
elems(77).Name = 'mpc_vel_z_lp';
elems(77).Dimensions = [1 1];
elems(77).DimensionsMode = 'Fixed';
elems(77).DataType = 'double';
elems(77).SampleTime = -1;
elems(77).Complexity = 'real';
elems(77).SamplingMode = 'Sample based';
elems(77).Min = [];
elems(77).Max = [];
elems(77).DocUnits = '';
elems(77).Description = '';

elems(78) = Simulink.BusElement;
elems(78).Name = 'mpc_vxy_reset_counter';
elems(78).Dimensions = [1 1];
elems(78).DimensionsMode = 'Fixed';
elems(78).DataType = 'double';
elems(78).SampleTime = -1;
elems(78).Complexity = 'real';
elems(78).SamplingMode = 'Sample based';
elems(78).Min = [];
elems(78).Max = [];
elems(78).DocUnits = '';
elems(78).Description = '';

elems(79) = Simulink.BusElement;
elems(79).Name = 'mpc_vz_reset_counter';
elems(79).Dimensions = [1 1];
elems(79).DimensionsMode = 'Fixed';
elems(79).DataType = 'double';
elems(79).SampleTime = -1;
elems(79).Complexity = 'real';
elems(79).SamplingMode = 'Sample based';
elems(79).Min = [];
elems(79).Max = [];
elems(79).DocUnits = '';
elems(79).Description = '';

elems(80) = Simulink.BusElement;
elems(80).Name = 'mpc_was_armed';
elems(80).Dimensions = [1 1];
elems(80).DimensionsMode = 'Fixed';
elems(80).DataType = 'double';
elems(80).SampleTime = -1;
elems(80).Complexity = 'real';
elems(80).SamplingMode = 'Sample based';
elems(80).Min = [];
elems(80).Max = [];
elems(80).DocUnits = '';
elems(80).Description = '';

elems(81) = Simulink.BusElement;
elems(81).Name = 'mpc_xy_reset_counter';
elems(81).Dimensions = [1 1];
elems(81).DimensionsMode = 'Fixed';
elems(81).DataType = 'double';
elems(81).SampleTime = -1;
elems(81).Complexity = 'real';
elems(81).SamplingMode = 'Sample based';
elems(81).Min = [];
elems(81).Max = [];
elems(81).DocUnits = '';
elems(81).Description = '';

elems(82) = Simulink.BusElement;
elems(82).Name = 'mpc_yaw';
elems(82).Dimensions = [1 1];
elems(82).DimensionsMode = 'Fixed';
elems(82).DataType = 'double';
elems(82).SampleTime = -1;
elems(82).Complexity = 'real';
elems(82).SamplingMode = 'Sample based';
elems(82).Min = [];
elems(82).Max = [];
elems(82).DocUnits = '';
elems(82).Description = '';

elems(83) = Simulink.BusElement;
elems(83).Name = 'mpc_z_reset_counter';
elems(83).Dimensions = [1 1];
elems(83).DimensionsMode = 'Fixed';
elems(83).DataType = 'double';
elems(83).SampleTime = -1;
elems(83).Complexity = 'real';
elems(83).SamplingMode = 'Sample based';
elems(83).Min = [];
elems(83).Max = [];
elems(83).DocUnits = '';
elems(83).Description = '';

mpc_self = Simulink.Bus;
mpc_self.HeaderFile = '';
mpc_self.Description = 'Bus for PX4 position controller';
mpc_self.DataScope = 'Auto';
mpc_self.Alignment = -1;
mpc_self.Elements = elems;
clear elems;
assignin('base','mpc_self', mpc_self);
