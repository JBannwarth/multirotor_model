function self = mc_pos_control( self )
% MC_POS_CONTROL
% /****************************************************************************
% 
%   Copyright (c) 2013 - 2016 PX4 Development Team. All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
% 
% 1. Redistributions of source code must retain the above copyright
%    notice, this list of conditions and the following disclaimer.
% 2. Redistributions in binary form must reproduce the above copyright
%    notice, this list of conditions and the following disclaimer in
%    the documentation and/or other materials provided with the
%    distribution.
% 3. Neither the name PX4 nor the names of its contributors may be
%    used to endorse or promote products derived from this software
%    without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
% COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
% OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
% AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.
% 
% ***************************************************************************/
%
% @file mc_pos_control_main.cpp
% Multicopter position controller.
% 
% Original publication for the desired attitude generation:
% Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
% Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
% 
% Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
% 
% The controller has two loops: P loop for position error and PID loop for velocity error.
% Output of velocity controller is thrust vector that splitted to thrust direction
% (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
% Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
% 
% @author Anton Babushkin <anton.babushkin@me.com>
    
    self.mpc_TILT_COS_MAX = 0.7;
    self.mpc_SIGMA = 0.000001;
    self.mpc_MIN_DIST = 0.01;
    self.mpc_MANUAL_THROTTLE_MAX_MULTICOPTER = 0.9;
    self.mpc_ONE_G = 9.8066;
    
    self.mpc_position_setpoint_s.SETPOINT_TYPE_FOLLOW_TARGET = 0;
    self.mpc_position_setpoint_s.SETPOINT_TYPE_IDLE = 0;
    self.mpc_position_setpoint_s.SETPOINT_TYPE_LAND = 0;
    self.mpc_position_setpoint_s.SETPOINT_TYPE_LOITER = 0;
    self.mpc_position_setpoint_s.SETPOINT_TYPE_POSITION = 0;
    self.mpc_position_setpoint_s.SETPOINT_TYPE_TAKEOFF = 0;
    self.mpc_position_setpoint_s.VELOCITY_FRAME_BODY_NED = 0;
    self.mpc_position_setpoint_s.VELOCITY_FRAME_LOCAL_NED = 0;
    
    self = task_main(self);
    
end

%% Class methods and variables
% class MulticopterPositionControl : public control::SuperBlock
% {
% public:
%     % /**
% % Constructor
%     
%     MulticopterPositionControl();
% 
%     % /**
% % Destructor, also kills task.
%     
%     ~MulticopterPositionControl();
% 
%     % /**
% % Start task.
%      *
% % @return        OK on success.
%     
%     int        start();
% 
%     bool        cross_sphere_line(const math::Vector<3> &sphere_c, sphere_r,
%                       const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res);
% 
% private:
%     bool        self.mpc_task_should_exit;        % /**< if true, task should exit
%     int        self.mpc_control_task;            % /**< task handle for task
%     orb_advert_t    self.mpc_mavlink_log_pub;        % /**< mavlink log advert
% 
%     int        self.mpc_vehicle_status_sub;        % /**< vehicle status subscription
%     int        self.mpc_vehicle_land_detected_sub;    % /**< vehicle land detected subscription
%     int        self.mpc_ctrl_state_sub;        % /**< control state subscription
%     int        self.mpc_att_sp_sub;            % /**< vehicle attitude setpoint
%     int        self.mpc_control_mode_sub;        % /**< vehicle control mode subscription
%     int        self.mpc_params_sub;            % /**< notification of parameter updates
%     int        self.mpc_manual_sub;            % /**< notification of manual control updates
%     int        self.mpc_arming_sub;            % /**< arming status of outputs
%     int        self.mpc_local_pos_sub;            % /**< vehicle local position
%     int        self.mpc_pos_sp_triplet_sub;        % /**< position setpoint triplet
%     int        self.mpc_local_pos_sp_sub;        % /**< offboard local position setpoint
%     int        self.mpc_global_vel_sp_sub;        % /**< offboard global velocity setpoint
% 
%     orb_advert_t    self.mpc_att_sp_pub;            % /**< attitude setpoint publication
%     orb_advert_t    self.mpc_local_pos_sp_pub;        % /**< vehicle local position setpoint publication
%     orb_advert_t    self.mpc_global_vel_sp_pub;        % /**< vehicle global velocity setpoint publication
% 
%     orb_id_t self.mpc_attitude_setpoint_id;
% 
%     struct vehicle_status_s             self.mpc_vehicle_status;     % /**< vehicle status
%     struct vehicle_land_detected_s             self.mpc_vehicle_land_detected;    % /**< vehicle land detected
%     struct control_state_s                self.mpc_ctrl_state;        % /**< vehicle attitude
%     struct vehicle_attitude_setpoint_s        self.mpc_att_sp;        % /**< vehicle attitude setpoint
%     struct manual_control_setpoint_s        self.mpc_manual;        % /**< r/c channel data
%     struct vehicle_control_mode_s            self.mpc_control_mode;        % /**< vehicle control mode
%     struct actuator_armed_s                self.mpc_arming;        % /**< actuator arming status
%     struct vehicle_local_position_s            self.mpc_local_pos;        % /**< vehicle local position
%     struct position_setpoint_triplet_s        self.mpc_pos_sp_triplet;    % /**< vehicle global position setpoint triplet
%     struct vehicle_local_position_setpoint_s    self.mpc_local_pos_sp;        % /**< vehicle local position setpoint
%     struct vehicle_global_velocity_setpoint_s    self.mpc_global_vel_sp;        % /**< vehicle global velocity setpoint
% 
%     control::BlockParamFloat self.mpc_manual_thr_min;
%     control::BlockParamFloat self.mpc_manual_thr_max;
% 
%     control::BlockDerivative self.mpc_vel_x_deriv;
%     control::BlockDerivative self.mpc_vel_y_deriv;
%     control::BlockDerivative self.mpc_vel_z_deriv;
% 
%     struct {
%         param_t thr_min;
%         param_t thr_max;
%         param_t thr_hover;
%         param_t alt_ctl_dz;
%         param_t alt_ctl_dy;
%         param_t z_p;
%         param_t z_vel_p;
%         param_t z_vel_i;
%         param_t z_vel_d;
%         param_t z_vel_max_up;
%         param_t z_vel_max_down;
%         param_t z_ff;
%         param_t xy_p;
%         param_t xy_vel_p;
%         param_t xy_vel_i;
%         param_t xy_vel_d;
%         param_t xy_vel_max;
%         param_t xy_vel_cruise;
%         param_t xy_ff;
%         param_t tilt_max_air;
%         param_t land_speed;
%         param_t tko_speed;
%         param_t tilt_max_land;
%         param_t man_roll_max;
%         param_t man_pitch_max;
%         param_t man_yaw_max;
%         param_t global_yaw_max;
%         param_t mc_att_yaw_p;
%         param_t hold_xy_dz;
%         param_t hold_max_xy;
%         param_t hold_max_z;
%         param_t acc_hor_max;
%         param_t alt_mode;
%         param_t opt_recover;
% 
%     }        self.mpc_params_handles;        % /**< handles for interesting parameters
% 
%     struct {
%         float thr_min;
%         float thr_max;
%         float thr_hover;
%         float alt_ctl_dz;
%         float alt_ctl_dy;
%         float tilt_max_air;
%         float land_speed;
%         float tko_speed;
%         float tilt_max_land;
%         float man_roll_max;
%         float man_pitch_max;
%         float man_yaw_max;
%         float global_yaw_max;
%         float mc_att_yaw_p;
%         float hold_xy_dz;
%         float hold_max_xy;
%         float hold_max_z;
%         float acc_hor_max;
%         float vel_max_up;
%         float vel_max_down;
%         uint32_t alt_mode;
% 
%         int opt_recover;
% 
%         math::Vector<3> pos_p;
%         math::Vector<3> vel_p;
%         math::Vector<3> vel_i;
%         math::Vector<3> vel_d;
%         math::Vector<3> vel_ff;
%         math::Vector<3> vel_max;
%         math::Vector<3> vel_cruise;
%         math::Vector<3> sp_offs_max;
%     }        self.mpc_params;
% 
%     struct map_projection_reference_s self.mpc_ref_pos;
%     float self.mpc_ref_alt;
%     hrt_abstime self.mpc_ref_timestamp;
% 
%     bool self.mpc_reset_pos_sp;
%     bool self.mpc_reset_alt_sp;
%     bool self.mpc_do_reset_alt_pos_flag;
%     bool self.mpc_mode_auto;
%     bool self.mpc_pos_hold_engaged;
%     bool self.mpc_alt_hold_engaged;
%     bool self.mpc_run_pos_control;
%     bool self.mpc_run_alt_control;
% 
%     bool self.mpc_reset_int_z = true;
%     bool self.mpc_reset_int_xy = true;
%     bool self.mpc_reset_int_z_manual = false;
%     bool self.mpc_reset_yaw_sp = true;
% 
%     bool self.mpc_hold_offboard_xy = false;
%     bool self.mpc_hold_offboard_z = false;
% 
%     math::Vector<3> self.mpc_thrust_int;
% 
%     math::Vector<3> self.mpc_pos;
%     math::Vector<3> self.mpc_pos_sp;
%     math::Vector<3> self.mpc_vel;
%     math::Vector<3> self.mpc_vel_sp;
%     math::Vector<3> self.mpc_vel_prev;            % /**< velocity on previous step
%     math::Vector<3> self.mpc_vel_ff;
%     math::Vector<3> self.mpc_vel_sp_prev;
%     math::Vector<3> self.mpc_vel_err_d;        % /**< derivative of current velocity
% 
%     math::Matrix<3, 3> self.mpc_R;            % /**< rotation matrix from attitude quaternions
%     float self.mpc_yaw;                % /**< yaw angle (euler)
%     bool self.mpc_in_landing;    % /**< the vehicle is in the landing descent
%     bool self.mpc_lnd_reached_ground; % /**< controller assumes the vehicle has reached the ground after landing
%     bool self.mpc_takeoff_jumped;
%     float self.mpc_vel_z_lp;
%     float self.mpc_acc_z_lp;
%     float self.mpc_takeoff_thrust_sp;
%     bool control_vel_enabled_prev;    % /**< previous loop was in velocity controlled mode (control_state.flag_control_velocity_enabled)
% 
%     % counters for reset events on position and velocity states
%     % they are used to identify a reset event
%     uint8_t self.mpc_z_reset_counter;
%     uint8_t self.mpc_xy_reset_counter;
%     uint8_t self.mpc_vz_reset_counter;
%     uint8_t self.mpc_vxy_reset_counter;
%     uint8_t self.mpc_heading_reset_counter;
% 
%     matrix::Dcmf self.mpc_R_setpoint;
% 
%     % /**
% % Update our local parameter cache.
%     
%     int        parameters_update(bool force);
% 
%     % /**
% % Update control outputs
%     
%     void        control_update();
% 
%     % /**
% % Check for changes in subscribed topics.
%     
%     void        poll_subscriptions();
% 
%     static float    scale_control(float ctl, float end, float dz, float dy);
%     static float    throttle_curve(float ctl, float ctr);
% 
%     % /**
% % Update reference for local position projection
%     
%     void        update_ref();
%     % /**
% % Reset position setpoint to current position.
%      *
% % This reset will only occur if the self.mpc_reset_pos_sp flag has been set.
% % The general logic is to first "activate" the flag in the flight
% % regime where a switch to a position control mode should hold the
% % very last position. Once switching to a position control mode
% % the last position is stored once.
%     
%     void        reset_pos_sp();
% 
%     % /**
% % Reset altitude setpoint to current altitude.
%      *
% % This reset will only occur if the self.mpc_reset_alt_sp flag has been set.
% % The general logic follows the reset_pos_sp() architecture.
%     
%     void        reset_alt_sp();
% 
%     % /**
% % Check if position setpoint is too far from current position and adjust it if needed.
%     
%     void        limit_pos_sp_offset();
% 
%     % /**
% % Set position setpoint using manual control
%     
%     void        control_manual(float dt);
% 
%     void control_non_manual(float dt);
% 
%     % /**
% % Set position setpoint using offboard control
%     
%     void        control_offboard(float dt);
% 
%     % /**
% % Set position setpoint for AUTO
%     
%     void        control_auto(float dt);
% 
%     void control_position(float dt);
% 
%     void limit_acceleration(float dt);
% 
%     % /**
% % Select between barometric and global (AMSL) altitudes
%     
%     void        select_alt(bool global);
% 
%     void update_velocity_derivative();
% 
%     void do_control(float dt);
% 
%     void generate_attitude_setpoint(float dt);
% 
%     % /**
% % Shim for calling task_main from task_create.
%     
%     static void    task_main_trampoline(int argc, char *argv[]);
% 
%     % /**
% % Main sensor collection task.
%     
%     void        task_main();
% };
% 
% namespace pos_control
% {
% 
% MulticopterPositionControl    *g_control;
% }

%% Multicopter position control constructor
function self = constructor( self )
    self.mpc_ctrl_state.q(1) = 1.0;
    self.mpc_R = eye(3);

    self.mpc_R_setpoint = eye(3);

    self.mpc_params_handles.thr_min        = self.mpc_param_find.MPC_THR_MIN;
    self.mpc_params_handles.thr_max        = self.mpc_param_find.MPC_THR_MAX;
    self.mpc_params_handles.thr_hover      = self.mpc_param_find.MPC_THR_HOVER;
    self.mpc_params_handles.alt_ctl_dz     = self.mpc_param_find.MPC_ALTCTL_DZ;
    self.mpc_params_handles.alt_ctl_dy     = self.mpc_param_find.MPC_ALTCTL_DY;
    self.mpc_params_handles.z_p            = self.mpc_param_find.MPC_Z_P;
    self.mpc_params_handles.z_vel_p        = self.mpc_param_find.MPC_Z_VEL_P;
    self.mpc_params_handles.z_vel_i        = self.mpc_param_find.MPC_Z_VEL_I;
    self.mpc_params_handles.z_vel_d        = self.mpc_param_find.MPC_Z_VEL_D;
    self.mpc_params_handles.z_vel_max_up   = self.mpc_param_find.MPC_Z_VEL_MAX_UP;
    self.mpc_params_handles.z_vel_max_down = self.mpc_param_find.MPC_Z_VEL_MAX;

    % Transitional support: Copy param values from max to down
    % param so that max param can be renamed in 1-2 releases
    % (currently at 1.3.0)
    self.mpc_param_find.MPC_Z_VEL_MAX_DN  = self.mpc_param_find.MPC_Z_VEL_MAX;

    self.mpc_params_handles.z_ff           = self.mpc_param_find.MPC_Z_FF;
    self.mpc_params_handles.xy_p           = self.mpc_param_find.MPC_XY_P;
    self.mpc_params_handles.xy_vel_p       = self.mpc_param_find.MPC_XY_VEL_P;
    self.mpc_params_handles.xy_vel_i       = self.mpc_param_find.MPC_XY_VEL_I;
    self.mpc_params_handles.xy_vel_d       = self.mpc_param_find.MPC_XY_VEL_D;
    self.mpc_params_handles.xy_vel_max     = self.mpc_param_find.MPC_XY_VEL_MAX;
    self.mpc_params_handles.xy_vel_cruise  = self.mpc_param_find.MPC_XY_CRUISE;
    self.mpc_params_handles.xy_ff          = self.mpc_param_find.MPC_XY_FF;
    self.mpc_params_handles.tilt_max_air   = self.mpc_param_find.MPC_TILTMAX_AIR;
    self.mpc_params_handles.land_speed     = self.mpc_param_find.MPC_LAND_SPEED;
    self.mpc_params_handles.tko_speed      = self.mpc_param_find.MPC_TKO_SPEED;
    self.mpc_params_handles.tilt_max_land  = self.mpc_param_find.MPC_TILTMAX_LND;
    self.mpc_params_handles.man_roll_max   = self.mpc_param_find.MPC_MAN_R_MAX;
    self.mpc_params_handles.man_pitch_max  = self.mpc_param_find.MPC_MAN_P_MAX;
    self.mpc_params_handles.man_yaw_max    = self.mpc_param_find.MPC_MAN_Y_MAX;
    self.mpc_params_handles.global_yaw_max = self.mpc_param_find.MC_YAWRATE_MAX;
    self.mpc_params_handles.mc_att_yaw_p   = self.mpc_param_find.MC_YAW_P;
    self.mpc_params_handles.hold_xy_dz     = self.mpc_param_find.MPC_HOLD_XY_DZ;
    self.mpc_params_handles.hold_max_xy    = self.mpc_param_find.MPC_HOLD_MAX_XY;
    self.mpc_params_handles.hold_max_z     = self.mpc_param_find.MPC_HOLD_MAX_Z;
    self.mpc_params_handles.acc_hor_max    = self.mpc_param_find.MPC_ACC_HOR_MAX;
    self.mpc_params_handles.alt_mode       = self.mpc_param_find.MPC_ALT_MODE;
    self.mpc_params_handles.opt_recover    = self.mpc_param_find.VT_OPT_RECOV_EN;

    % Fetch initial parameter values
    parameters_update(true);
end

%% Multicopter position control destructor
% MulticopterPositionControl::~MulticopterPositionControl()
% {
%     if (self.mpc_control_task ~= -1) {
% % task wakes up every 100ms or so at the longest
%         self.mpc_task_should_exit = true;

% % wait for a second for the task to quit at our request
%         unsigned i = 0;

%         do {
% % wait 20ms
%             usleep(20000);

% % if we have given up, kill it
%             if (++i > 50) {
%                 px4_task_delete(self.mpc_control_task);
%                 break;
%             }
%         } while (self.mpc_control_task ~= -1);
%     }

%     pos_control::g_control = nullptr;
% }

%% Parameters update
function self = parameters_update( force, self )

    updated = self.mpc_params_sub; % ???? check
    
    if (updated)
        self.mpc_params_sub = false;
    end

    if (updated || force)
        % Update C++ param system
        % self = updateParams( self );

        % Update legacy C interface params
        self.mpc_params.thr_min       = self.mpc_params_handles.thr_min;
        self.mpc_params.thr_max       = self.mpc_params_handles.thr_max;
        self.mpc_params.thr_hover     = self.mpc_params_handles.thr_hover;
        self.mpc_params.alt_ctl_dz    = self.mpc_params_handles.alt_ctl_dz;
        self.mpc_params.alt_ctl_dy    = self.mpc_params_handles.alt_ctl_dy;
        self.mpc_params.tilt_max_air  = self.mpc_params_handles.tilt_max_air;
        self.mpc_params.tilt_max_air  = radians( self.mpc_params.tilt_max_air );
        self.mpc_params.land_speed    = self.mpc_params_handles.land_speed;
        self.mpc_params.tko_speed     = self.mpc_params_handles.tko_speed;
        self.mpc_params.tilt_max_land = self.mpc_params_handles.tilt_max_land;
        self.mpc_params.tilt_max_land = radians( self.mpc_params.tilt_max_land );
        
        self.mpc_params.pos_p(1)      = self.mpc_params_handles.xy_p;
        self.mpc_params.pos_p(2)      = self.mpc_params_handles.xy_p;
        self.mpc_params.pos_p(3)      = self.mpc_params_handles.z_p;
        self.mpc_params.vel_p(1)      = self.mpc_params_handles.xy_vel_p;
        self.mpc_params.vel_p(2)      = self.mpc_params_handles.xy_vel_p;
        self.mpc_params.vel_p(3)      = self.mpc_params_handles.z_vel_p;
        self.mpc_params.vel_i(1)      = self.mpc_params_handles.xy_vel_i;
        self.mpc_params.vel_i(2)      = self.mpc_params_handles.xy_vel_i;
        self.mpc_params.vel_i(3)      = self.mpc_params_handles.z_vel_i;
        self.mpc_params.vel_d(1)      = self.mpc_params_handles.xy_vel_d;
        self.mpc_params.vel_d(2)      = self.mpc_params_handles.xy_vel_d;
        self.mpc_params.vel_d(3)      = self.mpc_params_handles.z_vel_d;
        self.mpc_params.vel_max(1)    = self.mpc_params_handles.xy_vel_max;
        self.mpc_params.vel_max(2)    = self.mpc_params_handles.xy_vel_max;
        self.mpc_params.vel_max_up    = self.mpc_params_handles.z_vel_max_up;
        self.mpc_params.vel_max(3)    = self.mpc_params_handles.z_vel_max_up;
        self.mpc_params.vel_max_down  = self.mpc_params_handles.z_vel_max_down;
        self.mpc_params.vel_cruise(1) = self.mpc_params_handles.xy_vel_cruise;
        self.mpc_params.vel_cruise(2) = self.mpc_params_handles.xy_vel_cruise;
        % Using Z max up for now
        self.mpc_params.vel_cruise(3) = self.mpc_params_handles.z_vel_max_up;
        v = constrain(self.mpc_params_handles.xy_ff, 0.0, 1.0);
        self.mpc_params.vel_ff(1)  = v;
        self.mpc_params.vel_ff(2)  = v;
        v = constrain(self.mpc_params_handles.z_ff, 0.0, 1.0);
        self.mpc_params.vel_ff(3)  = v;
        v = constrain(self.mpc_params_handles.hold_xy_dz, 0.0, 1.0);
        self.mpc_params.hold_xy_dz = v;
        if (self.mpc_params_handles.hold_max_xy < 0.0)
            self.mpc_params.hold_max_xy = 0;
        else
            self.mpc_params.hold_max_xy = self.mpc_params_handles.hold_max_xy;
        end
        if (self.mpc_params_handles.hold_max_z < 0.0)
            self.mpc_params.hold_max_z = 0;
        else
            self.mpc_params.hold_max_z = self.mpc_params_handles.hold_max_z;
        end
        self.mpc_params.acc_hor_max = self.mpc_params_handles.acc_hor_max;

        % Increase the maximum horizontal acceleration such that stopping
        % within 1 s from full speed is feasible
        self.mpc_params.acc_hor_max = max(self.mpc_params.vel_cruise(1), self.mpc_params.acc_hor_max);
        self.mpc_params.alt_mode    = self.mpc_params_handles.alt_mode;
        self.mpc_params.opt_recover = self.mpc_params_handles.opt_recover;
        
        self.mpc_params.sp_offs_max = (self.mpc_params.vel_cruise ./ self.mpc_params.pos_p) * 2.0;

        % MC attitude control parameters
        % Manual control scale
        self.mpc_params.man_roll_max   = self.mpc_params_handles.man_roll_max;
        self.mpc_params.man_pitch_max  = self.mpc_params_handles.man_pitch_max;
        self.mpc_params.man_yaw_max    = self.mpc_params_handles.man_yaw_max;
        self.mpc_params.global_yaw_max = self.mpc_params_handles.global_yaw_max;
        self.mpc_params.man_roll_max   = radians( self.mpc_params.man_roll_max );
        self.mpc_params.man_pitch_max  = radians( self.mpc_params.man_pitch_max );
        self.mpc_params.man_yaw_max    = radians( self.mpc_params.man_yaw_max );
        self.mpc_params.global_yaw_max = radians( self.mpc_params.global_yaw_max );

        self.mpc_params.mc_att_yaw_p   = self.mpc_params_handles.mc_att_yaw_p;

        % Takeoff and land velocities should not exceed maximum
        self.mpc_params.tko_speed  = min(self.mpc_params.tko_speed, self.mpc_params.vel_max_up);
        self.mpc_params.land_speed = min(self.mpc_params.land_speed, self.mpc_params.vel_max_down);
    end
end

%% Poll subscriptions
function self = poll_subscriptions( self )

    updated = true; % ORB CHECK self.mpc_vehicle_status_sub
    
    if (updated)
        self.mpc_vehicle_status = self.mpc_vehicle_status_sub; % ORB_ID(vehicle_status)
        
        % Set correct uORB ID, depending on if vehicle is VTOL or not
%         if (~self.mpc_attitude_setpoint_id)
%             if (self.mpc_vehicle_status.is_vtol)
%                 self.mpc_attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);
%             else
%                 self.mpc_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
%             end
%         end
    end

    % updated = self.mpc_vehicle_land_detected_sub; % ORB_CHECK

    if (updated)
        self.mpc_vehicle_land_detected = self.mpc_vehicle_land_detected_sub; % ORB_ID(vehicle_land_detected)
    end
    
    % updated = self.mpc_ctrl_state_sub; % ORB_CHECK

    if (updated)
        
        self.mpc_ctrl_state = self.mpc_ctrl_state_sub; % ORB_ID(control_state)

        % Get current rotation matrix and euler angles from control state quaternions
        q_att = [ self.mpc_ctrl_state.q(1);
                  self.mpc_ctrl_state.q(2);
                  self.mpc_ctrl_state.q(3);
                  self.mpc_ctrl_state.q(4) ];
        self.mpc_R = to_dcm( q_att );
        euler_angles = dcm_to_euler( self.mpc_R ); % math::Vector<3> 
        self.mpc_yaw = euler_angles(3);

        if (self.mpc_control_mode.flag_control_manual_enabled)
            if (self.mpc_heading_reset_counter ~= self.mpc_ctrl_state.quat_reset_counter)
                self.mpc_heading_reset_counter = self.mpc_ctrl_state.quat_reset_counter;
                delta_q = [ self.mpc_ctrl_state.delta_q_reset(1);
                            self.mpc_ctrl_state.delta_q_reset(2);
                            self.mpc_ctrl_state.delta_q_reset(3);
                            self.mpc_ctrl_state.delta_q_reset(4) ];

                % We only extract the heading change from the delta quaternion
                delta_euler = quat_to_euler( delta_q );
                self.mpc_att_sp.yaw_body = self.mpc_att_sp.yaw_body + delta_euler(3);
            end
        end

    end
    
    % updated = self.mpc_att_sp_sub;

    if (updated)
        self.mpc_att_sp = self.mpc_att_sp_sub; % ORB_ID(vehicle_attitude_setpoint)
    end

    % updated = self.mpc_control_mode_sub; % ORB_CHECK
    if (updated)
        self.mpc_control_mode = self.mpc_control_mode_sub;
    end

    % updated = self.mpc_manual_sub; % ORB_CHECK
    if (updated)
        self.mpc_manual = self.mpc_manual_sub;
    end

    % updated = self.mpc_arming_sub; % ORB_CHECK
    if (updated)
        self.mpc_arming = self.mpc_arming_sub;
    end

    % updated = self.mpc_local_pos_sub; % ORB_CHECK
    
    if (updated)
        self.mpc_local_pos = self.mpc_local_pos_sub; % ORB_ID(vehicle_local_position)

        % Check if a reset event has happened
        % If the vehicle is in manual mode we will shift the setpoints of the
        % states which were reset. In auto mode we do not shift the setpoints
        % since we want the vehicle to track the original state.
        if (self.mpc_control_mode.flag_control_manual_enabled)
            if (self.mpc_z_reset_counter ~= self.mpc_local_pos.z_reset_counter)
                self.mpc_pos_sp(3) = self.mpc_pos_sp(3) + self.mpc_local_pos.delta_z;
            end

            if (self.mpc_xy_reset_counter ~= self.mpc_local_pos.xy_reset_counter)
                self.mpc_pos_sp(1) = self.mpc_pos_sp(1) + self.mpc_local_pos.delta_xy(1);
                self.mpc_pos_sp(2) = self.mpc_pos_sp(2) + self.mpc_local_pos.delta_xy(2);
            end

            if (self.mpc_vz_reset_counter ~= self.mpc_local_pos.vz_reset_counter)
                self.mpc_vel_sp(3) = self.mpc_vel_sp(3) + self.mpc_local_pos.delta_vz;
                self.mpc_vel_sp_prev(3) = self.mpc_vel_sp_prev(3) +  self.mpc_local_pos.delta_vz;
            end

            if (self.mpc_vxy_reset_counter ~= self.mpc_local_pos.vxy_reset_counter)
                self.mpc_vel_sp(1) = self.mpc_vel_sp(1) + self.mpc_local_pos.delta_vxy(1);
                self.mpc_vel_sp(2) = self.mpc_vel_sp(2) + self.mpc_local_pos.delta_vxy(2);
                self.mpc_vel_sp_prev(1) = self.mpc_vel_sp_prev(1) + self.mpc_local_pos.delta_vxy(1);
                self.mpc_vel_sp_prev(2) = self.mpc_vel_sp_prev(2) + self.mpc_local_pos.delta_vxy(2);
            end
        end

        % update the reset counters in any case
        self.mpc_z_reset_counter = self.mpc_local_pos.z_reset_counter;
        self.mpc_xy_reset_counter = self.mpc_local_pos.xy_reset_counter;
        self.mpc_vz_reset_counter = self.mpc_local_pos.vz_reset_counter;
        self.mpc_vxy_reset_counter = self.mpc_local_pos.vxy_reset_counter;
    end
    
    % updated = self.mpc_pos_sp_triplet_sub; % ORB_CHECK

    if (updated)
        self.mpc_pos_sp_triplet = self.mpc_pos_sp_triplet_sub; % ORB_ID(position_setpoint_triplet)
        % Make sure that the position setpoint is valid
        if (isinf(self.mpc_pos_sp_triplet.current.lat) || ...
            isinf(self.mpc_pos_sp_triplet.current.lon) || ...
            isinf(self.mpc_pos_sp_triplet.current.alt) )
            self.mpc_pos_sp_triplet.current.valid = false;
        end
    end
end

%% Scale control
function output = scale_control(ctl, end1, dz, dy)
    if (ctl > dz)
        output = dy + (ctl - dz) * (1.0 - dy) / (end1 - dz);
    elseif (ctl < -dz)
        output = -dy + (ctl + dz) * (1.0 - dy) / (end1 - dz);
    else
        output = ctl * (dy / dz);
    end
end

%% Throttle curve
function output = throttle_curve(ctl, ctr)
    % Piecewise linear mapping: 0:ctr -> 0:0.5
    % and ctr:1 -> 0.5:1
    if (ctl < 0.5)
        output = 2 * ctl * ctr;
    else
        output = ctr + 2 * (ctl - 0.5) * (1.0 - ctr);
    end
end

%% Update reference
function self = update_ref(self)
    if (self.mpc_local_pos.ref_timestamp ~= self.mpc_ref_timestamp)
        % lat_sp, lon_sp;
        alt_sp = 0.0;

        if (self.mpc_ref_timestamp ~= 0)
            % Calculate current position setpoint in global frame
            map_projection_reproject(self.mpc_ref_pos, self.mpc_pos_sp(1), self.mpc_pos_sp(2), lat_sp, lon_sp);
            alt_sp = self.mpc_ref_alt - self.mpc_pos_sp(3);
        end

        % Update local projection reference
        map_projection_init(self.mpc_ref_pos, self.mpc_local_pos.ref_lat, self.mpc_local_pos.ref_lon);
        self.mpc_ref_alt = self.mpc_local_pos.ref_alt;

        if (self.mpc_ref_timestamp ~= 0)
            % Reproject position setpoint to new reference
            [~, self.mpc_pos_sp(1), self.mpc_pos_sp(2)] = map_projection_project(self.mpc_ref_pos, lat_sp, lon_sp, self.mpc_pos_sp(1), self.mpc_pos_sp(2));
            self.mpc_pos_sp(3) = -(alt_sp - self.mpc_ref_alt);
        end

        self.mpc_ref_timestamp = self.mpc_local_pos.ref_timestamp;
    end
end

%% Reset position setpoint
function self = reset_pos_sp( self )
% RESET_POS_SP
    if (self.mpc_reset_pos_sp)
        self.mpc_reset_pos_sp = false;

        % We have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
        % continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
        % position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
        self.mpc_pos_sp(1) = self.mpc_pos(1);
        self.mpc_pos_sp(2) = self.mpc_pos(2);
    end
end

%% Reset altitude setpoint
function self = reset_alt_sp( self )
    if (self.mpc_reset_alt_sp)
        self.mpc_reset_alt_sp = false;

        % We have logic in the main function which choosed the velocity setpoint such that the attitude setpoint is
        % continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
        % altitude in a special way
        self.mpc_pos_sp(3) = self.mpc_pos(3);
    end
end

%% limit_pos_sp_offset
function self = limit_pos_sp_offset( self )
    pos_sp_offs = zeros(3,1);

    if (self.mpc_control_mode.flag_control_position_enabled)
        pos_sp_offs(1) = (self.mpc_pos_sp(1) - self.mpc_pos(1)) / self.mpc_params.sp_offs_max(1);
        pos_sp_offs(2) = (self.mpc_pos_sp(2) - self.mpc_pos(2)) / self.mpc_params.sp_offs_max(2);
    end

    if (self.mpc_control_mode.flag_control_altitude_enabled)
        pos_sp_offs(3) = (self.mpc_pos_sp(3) - self.mpc_pos(3)) / self.mpc_params.sp_offs_max(3);
    end

    pos_sp_offs_norm = norm( pos_sp_offs );

    if ( pos_sp_offs_norm > 1.0 )
        pos_sp_offs = pos_sp_offs / pos_sp_offs_norm;
        self.mpc_pos_sp = self.mpc_pos + pos_sp_offs .* self.mpc_params.sp_offs_max;
    end
end

%% control_manual(dt, self)
function self = control_manual(dt, self)
    % Entering manual control from non-manual control mode, reset alt/pos setpoints
    if (self.mpc_mode_auto)
        self.mpc_mode_auto = false;

    % Reset alt pos flags if resetting is enabled
        if (self.mpc_do_reset_alt_pos_flag)
            self.mpc_reset_pos_sp = true;
            self.mpc_reset_alt_sp = true;
        end
    end

    % math::Vector<3> req_vel_sp; % % X,Y in local frame and Z in global (D), in [-1,1] normalized range
    req_vel_sp = zeros(3,1);

    if (self.mpc_control_mode.flag_control_altitude_enabled)
        % set vertical velocity setpoint with throttle stick
        req_vel_sp(3) = - scale_control(self.mpc_manual.z - 0.5, 0.5, self.mpc_params.alt_ctl_dz, self.mpc_params.alt_ctl_dy); % D
    end

    if (self.mpc_control_mode.flag_control_position_enabled)
        % set horizontal velocity setpoint with roll/pitch stick
        req_vel_sp(1) = self.mpc_manual.x;
        req_vel_sp(2) = self.mpc_manual.y;
    end

    if (self.mpc_control_mode.flag_control_altitude_enabled)
        % reset alt setpoint to current altitude if needed
        self = reset_alt_sp( self );
    end

    if (self.mpc_control_mode.flag_control_position_enabled)
        % reset position setpoint to current position if needed
        self = reset_pos_sp( self );
    end

    % limit velocity setpoint
    req_vel_sp_norm = norm( req_vel_sp ); % ~~~

    if (req_vel_sp_norm > 1.0)
        req_vel_sp = req_vel_sp / req_vel_sp_norm;
    end

    % _req_vel_sp scaled to 0..1, scale it to max speed and rotate around yaw
    %math::Matrix<3, 3> R_yaw_sp;
    R_yaw_sp = from_euler(0.0, 0.0, self.mpc_att_sp.yaw_body);
    req_vel_sp_scaled = R_yaw_sp * req_vel_sp .* ...
            self.mpc_params.vel_cruise; % in NED and scaled to actual velocity

    % Assisted velocity mode: user controls velocity, but if    velocity is small enough, position
    % hold is activated for the corresponding axis

    % Horizontal axes
    if (self.mpc_control_mode.flag_control_position_enabled)
        % Check for pos. hold
        if ( (abs(req_vel_sp(1)) < self.mpc_params.hold_xy_dz) && (abs(req_vel_sp(2)) < self.mpc_params.hold_xy_dz))
            if (~self.mpc_pos_hold_engaged)

                vel_xy_mag = sqrt( self.mpc_vel(1) * self.mpc_vel(1) + self.mpc_vel(2) * self.mpc_vel(2) );

                if ( (self.mpc_params.hold_max_xy < FLT_EPSILON) || (vel_xy_mag < self.mpc_params.hold_max_xy) )
                    % Reset position setpoint to have smooth transition from velocity control to position control
                    self.mpc_pos_hold_engaged = true;
                    self.mpc_pos_sp(1) = self.mpc_pos(1);
                    self.mpc_pos_sp(2) = self.mpc_pos(2);
                else
                    self.mpc_pos_hold_engaged = false;
                end
            end
        else
            self.mpc_pos_hold_engaged = false;
        end

        % Set requested velocity setpoint
        if (~self.mpc_pos_hold_engaged)
            self.mpc_pos_sp(1) = self.mpc_pos(1);
            self.mpc_pos_sp(2) = self.mpc_pos(2);
            self.mpc_run_pos_control = false; % request velocity setpoint to be used, instead of position setpoint
            self.mpc_vel_sp(1) = req_vel_sp_scaled(1);
            self.mpc_vel_sp(2) = req_vel_sp_scaled(2);
        end
    end

    % vertical axis
    if (self.mpc_control_mode.flag_control_altitude_enabled)
        % check for pos. hold
        if ( abs( req_vel_sp(3) ) < self.mpc_FLT_EPSILON )
            if (~ self.mpc_alt_hold_engaged )
                if ( (self.mpc_params.hold_max_z < self.mpc_FLT_EPSILON) || (abs( self.mpc_vel(3) ) < self.mpc_params.hold_max_z) )
                    % reset position setpoint to have smooth transition from velocity control to position control
                    self.mpc_alt_hold_engaged = true;
                    self.mpc_pos_sp(3) = self.mpc_pos(3);
                else
                    self.mpc_alt_hold_engaged = false;
                end
            end
        else
            self.mpc_alt_hold_engaged = false;
            self.mpc_pos_sp(3) = self.mpc_pos(3);
        end

        % set requested velocity setpoint
        if (~self.mpc_alt_hold_engaged)
            self.mpc_run_alt_control = false; % request velocity setpoint to be used, instead of altitude setpoint
            self.mpc_vel_sp(3) = req_vel_sp_scaled(3);
        end
    end

    if (self.mpc_vehicle_land_detected.landed)
        % don't run controller when landed
        self.mpc_reset_pos_sp = true;
        self.mpc_reset_alt_sp = true;
        self.mpc_mode_auto = false;
        self.mpc_reset_int_z = true;
        self.mpc_reset_int_xy = true;

        self.mpc_R_setpoint = eye(3,3);

        self.mpc_att_sp.roll_body = 0.0;
        self.mpc_att_sp.pitch_body = 0.0;
        self.mpc_att_sp.yaw_body = self.mpc_yaw;
        self.mpc_att_sp.thrust = 0.0;

        self.mpc_att_sp.timestamp = self.mpc_hrt_absolute_time;

        % publish attitude setpoint
        % if (self.mpc_att_sp_pub ~= nullptr) {
        %     orb_publish(self.mpc_attitude_setpoint_id, self.mpc_att_sp_pub, &self.mpc_att_sp);

        % elseif (self.mpc_attitude_setpoint_id) {
        %     self.mpc_att_sp_pub = orb_advertise(self.mpc_attitude_setpoint_id, &self.mpc_att_sp);
        % }
    else
        self = control_position(dt, self);
    end
end

%% Control non-manual
function self = control_non_manual(dt, self)
    % select control source
    if (self.mpc_control_mode.flag_control_offboard_enabled)
        % offboard control
        self = control_offboard(dt, self);
        self.mpc_mode_auto = false;
    else
        self.mpc_hold_offboard_xy = false;
        self.mpc_hold_offboard_z = false;

        % AUTO
        self = control_auto(dt, self);
    end

    % weather-vane mode for vtol: disable yaw control
    if (self.mpc_pos_sp_triplet.current.disable_mc_yaw_control == true)
        self.mpc_att_sp.disable_mc_yaw_control = true;
    else
        % reset in case of setpoint updates
        self.mpc_att_sp.disable_mc_yaw_control = false;
    end

    % guard against any bad velocity values

    velocity_valid = isfinite(self.mpc_pos_sp_triplet.current.vx) && ...
                  isfinite(self.mpc_pos_sp_triplet.current.vy) && ...
                  self.mpc_pos_sp_triplet.current.velocity_valid;

    % do not go slower than the follow target velocity when position tracking is active (set to valid)
    
    % Look at this later
    if ( self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_FOLLOW_TARGET && ...
        velocity_valid && self.mpc_pos_sp_triplet.current.position_valid)
        ft_vel = [self.mpc_pos_sp_triplet.current.vx;
                  self.mpc_pos_sp_triplet.current.vy;
                  0];

         cos_ratio = dot(ft_vel, self.mpc_vel_sp) / ( norm( ft_vel ) * norm( self.mpc_vel_sp ) );

        % only override velocity set points when uav is traveling in same direction as target and vector component
        % is greater than calculated position set point velocity component

        if (cos_ratio > 0)
            ft_vel = ft_vel * (cos_ratio);
            % min speed a little faster than target vel
            ft_vel = ft_vel + normalize(ft_vel) * 1.5;
        else
            ft_vel = ft_vel .* 0; %.zero();
        end

        if ( abs(ft_vel(1)) > abs(self.mpc_vel_sp(1)) )
            self.mpc_vel_sp(1) = ft_vel(1);
        else
            self.mpc_vel_sp(1) = self.mpc_vel_sp(1);
        end
        
        if ( abs(ft_vel(2)) > abs(self.mpc_vel_sp(2)) )
            self.mpc_vel_sp(2) = ft_vel(2);
        else
            self.mpc_vel_sp(2) = self.mpc_vel_sp(2);
        end

        % track target using velocity only

    % Look at this later
    elseif (self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_FOLLOW_TARGET && ...
           velocity_valid)

        self.mpc_vel_sp(1) = self.mpc_pos_sp_triplet.current.vx;
        self.mpc_vel_sp(2) = self.mpc_pos_sp_triplet.current.vy;
    end

    % use constant descend rate when landing, ignore altitude setpoint
    if (self.mpc_pos_sp_triplet.current.valid ...
        && self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_LAND)
        self.mpc_vel_sp(3) = self.mpc_params.land_speed;
        self.mpc_run_alt_control = false;
    end

    % special thrust setpoint generation for takeoff from ground
    if (self.mpc_pos_sp_triplet.current.valid ...
        && self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_TAKEOFF ...
        && self.mpc_control_mode.flag_armed)

        % check if we are not already in air.
        % if yes then we don't need a jumped takeoff anymore
        if (~self.mpc_takeoff_jumped && ~self.mpc_vehicle_land_detected.landed && (abs(self.mpc_takeoff_thrust_sp) < self.mpc_FLT_EPSILON))
            self.mpc_takeoff_jumped = true;
        end

        if (~self.mpc_takeoff_jumped)
            % ramp thrust setpoint up
            if (self.mpc_vel(3) > -(self.mpc_params.tko_speed / 2.0))
                self.mpc_takeoff_thrust_sp = self.mpc_takeoff_thrust_sp + 0.5 * dt;
                self.mpc_vel_sp = self.mpc_vel_sp .* 0;
                self.mpc_vel_prev = self.mpc_vel_prev .* 0;
            else
                % copter has reached our takeoff speed. split the thrust setpoint up
                % into an integral part and into a P part
                self.mpc_thrust_int(3) = self.mpc_takeoff_thrust_sp - self.mpc_params.vel_p(3) * abs(self.mpc_vel(3));
                self.mpc_thrust_int(3) = -constrain( self.mpc_thrust_int(3), self.mpc_params.thr_min, self.mpc_params.thr_max );
                self.mpc_vel_sp_prev(3) = -self.mpc_params.tko_speed;
                self.mpc_takeoff_jumped = true;
                self.mpc_reset_int_z = false;
            end
        end

        if (self.mpc_takeoff_jumped)
            self.mpc_vel_sp(3) = -self.mpc_params.tko_speed;
        end

    else
        self.mpc_takeoff_jumped = false;
        self.mpc_takeoff_thrust_sp = 0.0;
    end

    if (self.mpc_pos_sp_triplet.current.valid ...
        && self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_IDLE)
        % Idle state, don't run controller and set zero thrust
        self.mpc_R_setpoint = eye(3,3);

        qd = from_dcm(self.mpc_R_setpoint); % matrix::Quatf
        self.mpc_att_sp.q_d = qd; % memcpy(&_att_sp.q_d[0], qd.data(), sizeof(self.mpc_att_sp.q_d));
        self.mpc_att_sp.q_d_valid = true;

        self.mpc_att_sp.roll_body = 0.0;
        self.mpc_att_sp.pitch_body = 0.0;
        self.mpc_att_sp.yaw_body = self.mpc_yaw;
        self.mpc_att_sp.thrust = 0.0;

        self.mpc_att_sp.timestamp = self.mpc_hrt_absolute_time;

        % publish attitude setpoint
%         if (self.mpc_att_sp_pub ~= nullptr)
%             orb_publish(self.mpc_attitude_setpoint_id, self.mpc_att_sp_pub, &self.mpc_att_sp);
% 
%         elseif (self.mpc_attitude_setpoint_id)
%             self.mpc_att_sp_pub = orb_advertise(self.mpc_attitude_setpoint_id, &self.mpc_att_sp);
%         end

    else
        self = control_position(dt, self);
    end
end

%% Control offboard
function self = control_offboard(dt, self)
% CONTROL_OFFBOARD
    if (self.mpc_pos_sp_triplet.current.valid)

        if (self.mpc_control_mode.flag_control_position_enabled && self.mpc_pos_sp_triplet.current.position_valid)
            % Control position
            self.mpc_pos_sp(1) = self.mpc_pos_sp_triplet.current.x;
            self.mpc_pos_sp(2) = self.mpc_pos_sp_triplet.current.y;
            self.mpc_run_pos_control = true;

            self.mpc_hold_offboard_xy = false;

        elseif (self.mpc_control_mode.flag_control_velocity_enabled && self.mpc_pos_sp_triplet.current.velocity_valid)
            % Control velocity

            % Reset position setpoint to current position if needed
            self = reset_pos_sp(self);

            if ( (abs(self.mpc_pos_sp_triplet.current.vx) <= self.mpc_FLT_EPSILON) && ...
                 (abs(self.mpc_pos_sp_triplet.current.vy) <= self.mpc_FLT_EPSILON) && ...
                  self.mpc_local_pos.xy_valid )

                if (~self.mpc_hold_offboard_xy)
                    self.mpc_pos_sp(1) = self.mpc_pos(1);
                    self.mpc_pos_sp(2) = self.mpc_pos(2);
                    self.mpc_hold_offboard_xy = true;
                end

                self.mpc_run_pos_control = true;

            else

                if (self.mpc_pos_sp_triplet.current.velocity_frame == self.mpc_position_setpoint_s.VELOCITY_FRAME_LOCAL_NED)
                    % Set position setpoint move rate
                    self.mpc_vel_sp(1) = self.mpc_pos_sp_triplet.current.vx;
                    self.mpc_vel_sp(2) = self.mpc_pos_sp_triplet.current.vy;

                elseif (self.mpc_pos_sp_triplet.current.velocity_frame == self.mpc_position_setpoint_s.VELOCITY_FRAME_BODY_NED)
                    % Transform velocity command from body frame to NED frame
                    self.mpc_vel_sp(1) = cos(self.mpc_yaw) * self.mpc_pos_sp_triplet.current.vx - sin(self.mpc_yaw) * self.mpc_pos_sp_triplet.current.vy;
                    self.mpc_vel_sp(2) = sin(self.mpc_yaw) * self.mpc_pos_sp_triplet.current.vx + cos(self.mpc_yaw) * self.mpc_pos_sp_triplet.current.vy;
                else
                    warning('Unknown velocity offboard coordinate frame');
                end

                self.mpc_run_pos_control = false;

                self.mpc_hold_offboard_xy = false;
            end

        end

        if (self.mpc_control_mode.flag_control_altitude_enabled && self.mpc_pos_sp_triplet.current.alt_valid)
            % Control altitude as it is enabled
            self.mpc_pos_sp(3) = self.mpc_pos_sp_triplet.current.z;
            self.mpc_run_alt_control = true;

            self.mpc_hold_offboard_z = false;

        elseif (self.mpc_control_mode.flag_control_climb_rate_enabled && self.mpc_pos_sp_triplet.current.velocity_valid)

            % Reset alt setpoint to current altitude if needed
            self = reset_alt_sp(self);

            if ( (abs(self.mpc_pos_sp_triplet.current.vz) <= self.mpc_FLT_EPSILON) && ...
                  self.mpc_local_pos.z_valid )

                if (~self.mpc_hold_offboard_z)
                    self.mpc_pos_sp(3) = self.mpc_pos(3);
                    self.mpc_hold_offboard_z = true;
                end

                self.mpc_run_alt_control = true;
            else
                % Set position setpoint move rate
                self.mpc_vel_sp(3) = self.mpc_pos_sp_triplet.current.vz;
                self.mpc_run_alt_control = false;

                self.mpc_hold_offboard_z = false;
            end
        end

        if (self.mpc_pos_sp_triplet.current.yaw_valid)
            self.mpc_att_sp.yaw_body = self.mpc_pos_sp_triplet.current.yaw;
        elseif (self.mpc_pos_sp_triplet.current.yawspeed_valid)
            self.mpc_att_sp.yaw_body = self.mpc_att_sp.yaw_body + self.mpc_pos_sp_triplet.current.yawspeed * dt;
        end

    else
        self.mpc_hold_offboard_xy = false;
        self.mpc_hold_offboard_z = false;
        self = reset_pos_sp(self);
        self = reset_alt_sp(self);
    end
end

%% Limit acceleration
function self = limit_acceleration(dt, self)
    % Limit total horizontal acceleration
    acc_hor = zeros(2, 1);
    acc_hor(1) = (self.mpc_vel_sp(1) - self.mpc_vel_sp_prev(1)) / dt;
    acc_hor(2) = (self.mpc_vel_sp(2) - self.mpc_vel_sp_prev(2)) / dt;

    if ( norm(acc_hor) > self.mpc_params.acc_hor_max)
        acc_hor = normalize( acc_hor );
        acc_hor = acc_hor * self.mpc_params.acc_hor_max;
        vel_sp_hor_prev = [self.mpc_vel_sp_prev(1); self.mpc_vel_sp_prev(2)];
        vel_sp_hor = acc_hor .* dt + vel_sp_hor_prev;
        self.mpc_vel_sp(1) = vel_sp_hor(1);
        self.mpc_vel_sp(2) = vel_sp_hor(2);
    end

    % limit vertical acceleration
    acc_v = (self.mpc_vel_sp(3) - self.mpc_vel_sp_prev(3)) / dt;

    % TODO: vertical acceleration is not just 2 * horizontal acceleration
    if ( abs(acc_v) > (2 * self.mpc_params.acc_hor_max) )
        acc_v = acc_v / abs(acc_v);
        self.mpc_vel_sp(2) = acc_v * 2 * self.mpc_params.acc_hor_max * dt + self.mpc_vel_sp_prev(2);
    end
end

%% Cross sphere line
function [output, res] = cross_sphere_line(sphere_c, sphere_r, line_a, line_b, res)
% CROSS_SPHERE_LINE
    % Project center of sphere on line
    % Normalized AB
    ab_norm = line_b - line_a;
    ab_norm = normalize(ab_norm);
    d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
    cd_len = norm( sphere_c - d );

    if (sphere_r > cd_len)
        % We have triangle CDX with known CD and CX = R, find DX
        dx_len = sqrt(sphere_r * sphere_r - cd_len * cd_len);

        if ( ((sphere_c - line_b) * ab_norm) > 0.0)
            % Target waypoint is already behind us
            res = line_b;
        else
            % Target is in front of us
            res = d + ab_norm * dx_len; % vector A->B on line
        end

        output = true;

    else
        % Have no roots, return D
        res = d; % Go directly to line
        
        % Previous waypoint is still in front of us
        if ( ((sphere_c - line_a) * ab_norm) < 0.0)
            res = line_a;
        end

        % Target waypoint is already behind us
        if ( ((sphere_c - line_b) * ab_norm) > 0.0)
            res = line_b;
        end

        output = false;
    end
end

%% Control auto
function self = control_auto(dt, self)
% CONTROL_AUTO
    % reset position setpoint on AUTO mode activation or if we are not in MC mode
    if (~self.mpc_mode_auto || ~self.mpc_vehicle_status.is_rotary_wing)
        if (~self.mpc_mode_auto)
            self.mpc_mode_auto = true;
        end

        self.mpc_reset_pos_sp = true;
        self.mpc_reset_alt_sp = true;
    end

    % Always check reset state of altitude and position control flags in auto
    reset_pos_sp();
    reset_alt_sp();

    current_setpoint_valid = false;
    previous_setpoint_valid = false;

    prev_sp = zeros(3,1);
    curr_sp = zeros(3,1);

    if (self.mpc_pos_sp_triplet.current.valid)

        % Project setpoint to local frame ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CHECK THIS
        [~, curr_sp(1), curr_sp(2)] = map_projection_project( self.mpc_ref_pos, ...
                                        self.mpc_pos_sp_triplet.current.lat, ...
                                        self.mpc_pos_sp_triplet.current.lon, ...
                                        curr_sp(1), curr_sp(2) );
        curr_sp(3) = -( self.mpc_pos_sp_triplet.current.alt - self.mpc_ref_alt );

        if (isfinite( curr_sp(1) ) && ...
            isfinite( curr_sp(2) ) && ...
            isfinite( curr_sp(3) ))
            current_setpoint_valid = true;
        end
    end

    if (self.mpc_pos_sp_triplet.previous.valid)
        [~, prev_sp(1), prev_sp(2)] = map_projection_project( self.mpc_ref_pos, ...
                                       self.mpc_pos_sp_triplet.previous.lat, ...
                                       self.mpc_pos_sp_triplet.previous.lon, ...
                                       prev_sp(1), prev_sp(2) );
        prev_sp(3) = -(self.mpc_pos_sp_triplet.previous.alt - self.mpc_ref_alt);

        if (isfinite( prev_sp(1) ) && ...
            isfinite( prev_sp(2) ) && ...
            isfinite( prev_sp(3) ))
            previous_setpoint_valid = true;
        end
    end

    if (current_setpoint_valid && ...
        (self.mpc_pos_sp_triplet.current.type ~= self.mpc_position_setpoint_s.SETPOINT_TYPE_IDLE))

        % Scaled space: 1 == position error resulting max allowed speed

        cruising_speed = self.mpc_params.vel_cruise;

        if (isfinite(self.mpc_pos_sp_triplet.current.cruising_speed) && ...
            (self.mpc_pos_sp_triplet.current.cruising_speed > 0.1) )
            cruising_speed(1) = self.mpc_pos_sp_triplet.current.cruising_speed;
            cruising_speed(2) = self.mpc_pos_sp_triplet.current.cruising_speed;
        end

        scale = self.mpc_params.pos_p ./ cruising_speed;

        % Convert current setpoint to scaled space
        curr_sp_s = curr_sp .* scale;

        % By default use current setpoint as is
        pos_sp_s = curr_sp_s;

        if ((self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_POSITION  || ...
             self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_FOLLOW_TARGET) && ...
            previous_setpoint_valid)

            % Follow "previous - current" line

            if ( norm(curr_sp - prev_sp) > self.mpc_MIN_DIST)

                % Find X - cross point of unit sphere and trajectory
                pos_s = self.mpc_pos .* scale;
                prev_sp_s = prev_sp .* scale;
                prev_curr_s = curr_sp_s - prev_sp_s;
                curr_pos_s = pos_s - curr_sp_s;
                curr_pos_s_len = norm(curr_pos_s);

                if (curr_pos_s_len < 1.0)
                    % Copter is closer to waypoint than unit radius
                    % Check next waypoint and use it to avoid slowing down when passing via waypoint
                    if (self.mpc_pos_sp_triplet.next.valid)
                        next_sp = zeros(3,1);
                        [~, next_sp(1), net_sp(2)] = map_projection_project(self.mpc_ref_pos, ...
                                                    self.mpc_pos_sp_triplet.next.lat, ...
                                                    self.mpc_pos_sp_triplet.next.lon, ...
                                                    next_sp(1), next_sp(2) );
                        next_sp(3) = -(self.mpc_pos_sp_triplet.next.alt - self.mpc_ref_alt);

                        if (norm(next_sp - curr_sp) > self.mpc_MIN_DIST)
                            next_sp_s = next_sp .* scale;

                            % Calculate angle prev - curr - next
                            curr_next_s = next_sp_s - curr_sp_s;
                            prev_curr_s_norm = normalize(prev_curr_s);

                            % cos(a) * curr_next, a = angle between current
                            % and next trajectory segments
                            cos_a_curr_next = dot(prev_curr_s_norm, curr_next_s);

                            % cos(b), b = angle pos - curr_sp - prev_sp
                            cos_b = - dot(curr_pos_s, prev_curr_s_norm) / curr_pos_s_len;

                            if ( (cos_a_curr_next > 0.0) && (cos_b > 0.0) )
                                curr_next_s_len = norm(curr_next_s);

                                % If curr - next distance is larger than
                                % unit radius, limit it
                                if (curr_next_s_len > 1.0)
                                    cos_a_curr_next = cos_a_curr_next / curr_next_s_len;
                                end

                                % Feed forward position setpoint offset
                                pos_ff = prev_curr_s_norm * ...
                                             cos_a_curr_next * cos_b * cos_b * (1.0 - curr_pos_s_len) * ...
                                             (1.0 - exp(-curr_pos_s_len * curr_pos_s_len * 20.0));
                                pos_sp_s = pos_sp_s + pos_ff;
                            end
                        end
                    end

                else
                    near = cross_sphere_line(pos_s, 1.0, prev_sp_s, curr_sp_s, pos_sp_s);

                    if (~near)
                        % We're far away from trajectory, pos_sp_s is set
                        % to the nearest point on the trajectory
                        pos_sp_s = pos_s + normalize(pos_sp_s - pos_s);
                    end
                end
            end
        end

        % Move setpoint not faster than max allowed speed
        pos_sp_old_s = self.mpc_pos_sp .* scale;

        % Difference between current and desired position setpoints, 1 = max speed
        d_pos_m = (pos_sp_s - pos_sp_old_s) ./ self.mpc_params.pos_p;
        d_pos_m_len = size(d_pos_m);

        if (d_pos_m_len > dt)
            pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt) .* self.mpc_params.pos_p;
        end

        % Scale result back to normal space
        self.mpc_pos_sp = pos_sp_s ./ scale;

        % Update yaw setpoint if needed

        if (self.mpc_pos_sp_triplet.current.yawspeed_valid ...
            && self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_FOLLOW_TARGET)
            self.mpc_att_sp.yaw_body = self.mpc_att_sp.yaw_body + ...
                self.mpc_pos_sp_triplet.current.yawspeed * dt;
        elseif ( isfinite(self.mpc_pos_sp_triplet.current.yaw) )
            self.mpc_att_sp.yaw_body = self.mpc_pos_sp_triplet.current.yaw;
        end

        % If we're already near the current takeoff setpoint don't reset in
        % case we switch back to posctl. This makes the takeoff finish smoothly.
        if ((self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_TAKEOFF ...
             || self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_LOITER) ...
             && (self.mpc_pos_sp_triplet.current.acceptance_radius > 0.0) ...
             ...% need to detect we're close a bit before the navigator switches
             ...% from takeoff to next waypoint
             && (norm(self.mpc_pos - self.mpc_pos_sp) < self.mpc_pos_sp_triplet.current.acceptance_radius * 1.2) )
             
            self.mpc_do_reset_alt_pos_flag = false;

            % otherwise: in case of interrupted mission don't go to
            % waypoint but stay at current position
        else
            self.mpc_do_reset_alt_pos_flag = true;
        end

        % During a mission or in loiter it's safe to retract the landing gear.
        if ((self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_POSITION || ...
             self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_LOITER) && ...
            ~self.mpc_vehicle_land_detected.landed )
            self.mpc_att_sp.landing_gear = 1.0;

            % During takeoff and landing, we better put it down again.

        elseif (self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_TAKEOFF || ...
               self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_LAND)
            self.mpc_att_sp.landing_gear = -1.0;
        else
            % For the rest of the setpoint types, just leave it as is.
        end

    else
        % No waypoint, do nothing, setpoint was already reset
    end
end

%% Update velocity derivative
function self = update_velocity_derivative(self)
% UPDATE_VELOCITY_DERIVATIVE
    % Update velocity derivative,
    % independent of the current flight mode
    if (self.mpc_local_pos.timestamp == 0)
        return;
    end

    % TODO: this logic should be in the estimator, not the controller~
    if (isfinite(self.mpc_local_pos.x) && ...
        isfinite(self.mpc_local_pos.y) && ...
        isfinite(self.mpc_local_pos.z))

        self.mpc_pos(1) = self.mpc_local_pos.x;
        self.mpc_pos(2) = self.mpc_local_pos.y;

        if ( (self.mpc_params.alt_mode == 1) && (self.mpc_local_pos.dist_bottom_valid) )
            self.mpc_pos(3) = -self.mpc_local_pos.dist_bottom;
        else
            self.mpc_pos(3) = self.mpc_local_pos.z;
        end
    end

    if (isfinite(self.mpc_local_pos.vx) && ...
        isfinite(self.mpc_local_pos.vy) && ...
        isfinite(self.mpc_local_pos.vz))

        self.mpc_vel(1) = self.mpc_local_pos.vx;
        self.mpc_vel(2) = self.mpc_local_pos.vy;

        if (self.mpc_params.alt_mode == 1 && self.mpc_local_pos.dist_bottom_valid)
            self.mpc_vel(3) = -self.mpc_local_pos.dist_bottom_rate;
        else
            self.mpc_vel(3) = self.mpc_local_pos.vz;
        end
    end

    [self.mpc_vel_err_d(1), self.mpc_vel_x_deriv] = block_derivative_update( -self.mpc_vel(1), self.mpc_vel_x_deriv );
    [self.mpc_vel_err_d(2), self.mpc_vel_y_deriv] = block_derivative_update( -self.mpc_vel(2), self.mpc_vel_y_deriv );
    [self.mpc_vel_err_d(3), self.mpc_vel_z_deriv] = block_derivative_update( -self.mpc_vel(3), self.mpc_vel_z_deriv );
end

%% Do control
function self = do_control(dt, self)
% DO_CONTROL
    self.mpc_vel_ff = self.mpc_vel_ff .* 0;

    % Functions
    % Can disable this and run velocity controllers directly in this cycle
    self.mpc_run_pos_control = true;
    self.mpc_run_alt_control = true;

    if (self.mpc_control_mode.flag_control_manual_enabled)
        % Manual control
        self = control_manual(dt, self);
        self.mpc_mode_auto = false;

        self.mpc_hold_offboard_xy = false;
        self.mpc_hold_offboard_z = false;
    else
        self = control_non_manual(dt, self);
    end
end

%% Control position
function self = control_position(dt, self)
% CONTROL_POSITION
% Run position & altitude controllers, if enabled (otherwise use already
% computed velocity setpoints)
    if (self.mpc_run_pos_control)
        self.mpc_vel_sp(1) = (self.mpc_pos_sp(1) -self.mpc_pos(1)) * self.mpc_params.pos_p(1);
        self.mpc_vel_sp(2) = (self.mpc_pos_sp(2) -self.mpc_pos(2)) * self.mpc_params.pos_p(2);
    end

    if (self.mpc_run_alt_control)
        self.mpc_vel_sp(3) = (self.mpc_pos_sp(3) -self.mpc_pos(3)) * self.mpc_params.pos_p(3);
    end

    % Make sure velocity setpoint is saturated in xy
    vel_norm_xy = sqrt(self.mpc_vel_sp(1) * self.mpc_vel_sp(1) + ...
                       self.mpc_vel_sp(2) * self.mpc_vel_sp(2) );
                  
    if (vel_norm_xy > self.mpc_params.vel_max(1))
        % Note assumes: vel_max(1) == vel_max(2)
        self.mpc_vel_sp(1) = self.mpc_vel_sp(1) * self.mpc_params.vel_max(1) / vel_norm_xy;
        self.mpc_vel_sp(2) = self.mpc_vel_sp(2) * self.mpc_params.vel_max(2) / vel_norm_xy;
    end

    % Make sure velocity setpoint is saturated in z
    if(self.mpc_vel_sp(3) < (-1.0 * self.mpc_params.vel_max_up) )
       self.mpc_vel_sp(3) = -1.0 *self.mpc_params.vel_max_up;
    end

    if (self.mpc_vel_sp(3) >  self.mpc_params.vel_max_down)
        self.mpc_vel_sp(3) = self.mpc_params.vel_max_down;
    end

    if (~self.mpc_control_mode.flag_control_position_enabled)
        self.mpc_reset_pos_sp = true;
    end

    if (~self.mpc_control_mode.flag_control_altitude_enabled)
        self.mpc_reset_alt_sp = true;
    end

    if (~self.mpc_control_mode.flag_control_velocity_enabled)
        self.mpc_vel_sp_prev(1) = self.mpc_vel(1);
        self.mpc_vel_sp_prev(2) = self.mpc_vel(2);
        self.mpc_vel_sp(1) = 0.0;
        self.mpc_vel_sp(2) = 0.0;
        control_vel_enabled_prev = false;
    end

    if (~self.mpc_control_mode.flag_control_climb_rate_enabled)
        self.mpc_vel_sp(3) = 0.0;
    end

    % TODO: remove this is a pathetic leftover, it's here just to make sure that
    % _takeoff_jumped flags are reset
    if (self.mpc_control_mode.flag_control_manual_enabled || ~self.mpc_pos_sp_triplet.current.valid ...
        || self.mpc_pos_sp_triplet.current.type ~= self.mpc_position_setpoint_s.SETPOINT_TYPE_TAKEOFF ...
        || ~self.mpc_control_mode.flag_armed)

        self.mpc_takeoff_jumped = false;
        self.mpc_takeoff_thrust_sp = 0.0;
    end

    self = limit_acceleration(dt, self);

    self.mpc_vel_sp_prev = self.mpc_vel_sp;

    self.mpc_global_vel_sp.vx = self.mpc_vel_sp(1);
    self.mpc_global_vel_sp.vy = self.mpc_vel_sp(2);
    self.mpc_global_vel_sp.vz = self.mpc_vel_sp(3);

    % Publish velocity setpoint
    % if (self.mpc_global_vel_sp_pub ~= nullptr)
        % orb_publish(ORB_ID(vehicle_global_velocity_setpoint), self.mpc_global_vel_sp_pub, &self.mpc_global_vel_sp);

    % else
        % self.mpc_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &self.mpc_global_vel_sp);
    % end

    if (self.mpc_control_mode.flag_control_climb_rate_enabled || self.mpc_control_mode.flag_control_velocity_enabled || ...
        self.mpc_control_mode.flag_control_acceleration_enabled)
        % Reset integrals if needed
        if (self.mpc_control_mode.flag_control_climb_rate_enabled)
            if (self.mpc_reset_int_z)
                self.mpc_reset_int_z = false;
                i = self.mpc_params.thr_min;

                if (self.mpc_reset_int_z_manual)
                    i = constrain(self.mpc_params.thr_hover, self.mpc_params.thr_min, self.mpc_params.thr_max);
                end

                self.mpc_thrust_int(2) = -i;
            end

        else
            self.mpc_reset_int_z = true;
        end

        if (self.mpc_control_mode.flag_control_velocity_enabled)
            if (self.mpc_reset_int_xy)
                self.mpc_reset_int_xy = false;
                self.mpc_thrust_int(0) = 0.0;
                self.mpc_thrust_int(1) = 0.0;
            end

        else
            self.mpc_reset_int_xy = true;
        end

        % Velocity error
        vel_err = self.mpc_vel_sp - self.mpc_vel;

        % Check if we have switched from a non-velocity controlled mode into a velocity controlled mode
        % if yes, then correct xy velocity setpoint such that the attitude setpoint is continuous
        if (~control_vel_enabled_prev && self.mpc_control_mode.flag_control_velocity_enabled)

            Rb = todcm( [self.mpc_att_sp.q_d(0); ...
                         self.mpc_att_sp.q_d(1), ...
                         self.mpc_att_sp.q_d(2), ...
                         self.mpc_att_sp.q_d(3)]);

            % Choose velocity xyz setpoint such that the resulting thrust
            % setpoint has the direction given by the last attitude setpoint
            self.mpc_vel_sp(1) = self.mpc_vel(1) + (-Rb(1, ...
                            3) * self.mpc_att_sp.thrust - self.mpc_thrust_int(1) - self.mpc_vel_err_d(1) * self.mpc_params.vel_d(1)) / self.mpc_params.vel_p(1);
            self.mpc_vel_sp(2) = self.mpc_vel(2) + (-Rb(2, ...
                            3) * self.mpc_att_sp.thrust - self.mpc_thrust_int(2) - self.mpc_vel_err_d(2) * self.mpc_params.vel_d(2)) / self.mpc_params.vel_p(2);
            self.mpc_vel_sp(3) = self.mpc_vel(3) + (-Rb(3, ...
                            3) * self.mpc_att_sp.thrust - self.mpc_thrust_int(3) - self.mpc_vel_err_d(3) * self.mpc_params.vel_d(3)) / self.mpc_params.vel_p(3);
            self.mpc_vel_sp_prev = self.mpc_vel_sp;
            control_vel_enabled_prev = true;

            % compute updated velocity error
            vel_err = self.mpc_vel_sp - self.mpc_vel;
        end

        % thrust vector in NED frame
        thrust_sp = zeros(3,1); %#ok<PREALL>

        if (self.mpc_control_mode.flag_control_acceleration_enabled && self.mpc_pos_sp_triplet.current.acceleration_valid)
            thrust_sp = [self.mpc_pos_sp_triplet.current.a_x;
                         self.mpc_pos_sp_triplet.current.a_y;
                         self.mpc_pos_sp_triplet.current.a_z];

        else
            thrust_sp = vel_err .* self.mpc_params.vel_p + self.mpc_vel_err_d .* self.mpc_params.vel_d + self.mpc_thrust_int;
        end

        if ( (self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_TAKEOFF) ...
            && ~self.mpc_takeoff_jumped && ~self.mpc_control_mode.flag_control_manual_enabled)
            % For jumped takeoffs use special thrust setpoint calculated above
            thrust_sp = thrust_sp .* 0;
            thrust_sp(3) = -self.mpc_takeoff_thrust_sp;
        end

        if (~self.mpc_control_mode.flag_control_velocity_enabled && ~self.mpc_control_mode.flag_control_acceleration_enabled)
            thrust_sp(1) = 0.0;
            thrust_sp(2) = 0.0;
        end

        if (~self.mpc_control_mode.flag_control_climb_rate_enabled && ~self.mpc_control_mode.flag_control_acceleration_enabled)
            thrust_sp(3) = 0.0;
        end

        % Limit thrust vector and check for saturation
        saturation_xy = false;
        saturation_z = false;

        % Limit min lift
        thr_min = self.mpc_params.thr_min;

        if (~self.mpc_control_mode.flag_control_velocity_enabled && (thr_min < 0.0))
            % Don't allow downside thrust direction in manual attitude mode
            thr_min = 0.0;
        end

        thrust_abs = norm(thrust_sp);
        tilt_max = self.mpc_params.tilt_max_air;
        thr_max = self.mpc_params.thr_max;
        
        % filter vel_z over 1/8sec
        self.mpc_vel_z_lp =self.mpc_vel_z_lp * (1.0 - dt * 8.0) + dt * 8.0 * self.mpc_vel(2);
        % filter vel_z change over 1/8sec
        vel_z_change = (self.mpc_vel(2) - self.mpc_vel_prev(2)) / dt;
        self.mpc_acc_z_lp =self.mpc_acc_z_lp * (1.0 - dt * 8.0) + dt * 8.0 * vel_z_change;

        % We can only run the control if we're already in-air, have a takeoff setpoint,
        % or if we're in offboard control.
        % Otherwise, we should just bail out
        got_takeoff_setpoint = (self.mpc_pos_sp_triplet.current.valid && ...
                           (self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_TAKEOFF)) || ...
                           self.mpc_control_mode.flag_control_offboard_enabled;

        if (self.mpc_vehicle_land_detected.landed && ~got_takeoff_setpoint)
            % Keep throttle low while still on ground.
            thr_max = 0.0;
        elseif (~self.mpc_control_mode.flag_control_manual_enabled && self.mpc_pos_sp_triplet.current.valid && ...
               (self.mpc_pos_sp_triplet.current.type == self.mpc_position_setpoint_s.SETPOINT_TYPE_LAND))

            % adjust limits for landing mode
            % limit max tilt and min lift when landing
            tilt_max = self.mpc_params.tilt_max_land;

            if (thr_min < 0.0)
                thr_min = 0.0;
            end

            % descend stabilized, we're landing
            if (~self.mpc_in_landing && ~self.mpc_lnd_reached_ground ...
                && (abs(self.mpc_acc_z_lp) < 0.1) ...
                && (self.mpc_vel_z_lp > (0.5 * self.mpc_params.land_speed) ))
                self.mpc_in_landing = true;
            end

            % assume ground, cut thrust
            if (self.mpc_in_landing ...
                && (self.mpc_vel_z_lp < 0.1))
                thr_max = 0.0;
                self.mpc_in_landing = false;
                self.mpc_lnd_reached_ground = true;
            end

            % Once we assumed to have reached the ground always cut the thrust.
            % Only free fall detection below can revoke this
            if (~self.mpc_in_landing && self.mpc_lnd_reached_ground)
                thr_max = 0.0;
            end

            % if we suddenly fall, reset landing logic and remove thrust limit
            if (self.mpc_lnd_reached_ground ...
                ... % XXX: magic value, assuming free fall above 4m/s2 acceleration
                && ( (self.mpc_acc_z_lp > 4.0) ...
                ||   (self.mpc_vel_z_lp > (2.0 * self.mpc_params.land_speed)) ))
                thr_max = self.mpc_params.thr_max;
                self.mpc_in_landing = true;
                self.mpc_lnd_reached_ground = false;
            end

        else
            self.mpc_in_landing = false;
            self.mpc_lnd_reached_ground = false;
        end

        % limit min lift
        if ( -thrust_sp(3) < thr_min )
             thrust_sp(3) = -thr_min;
            % Don't freeze altitude integral if it wants to throttle up
            %saturation_z = vel_err(3) > 0.0 ? true : saturation_z;
            if (vel_err(3) > 0.0)
                saturation_z = true;
            else
                saturation_z = saturation_z; %#ok<ASGSL>
            end
        end

        if (self.mpc_control_mode.flag_control_velocity_enabled || self.mpc_control_mode.flag_control_acceleration_enabled)

            % limit max tilt
            if ( (thr_min >= 0.0) && ( tilt_max < (3.14159265358979323846 / 2 - 0.05) ) )
                % absolute horizontal thrust
                thrust_sp_xy_len = norm( [thrust_sp(1); thrust_sp(2)] );

                if (thrust_sp_xy_len > 0.01)
                % max horizontal thrust for given vertical thrust
                    thrust_xy_max = -thrust_sp(3) * tan(tilt_max);

                    if (thrust_sp_xy_len > thrust_xy_max)
                        k = thrust_xy_max / thrust_sp_xy_len;
                        thrust_sp(1) = thrust_sp(1) * k;
                        thrust_sp(2) = thrust_sp(2) * k;
                        % Don't freeze x,y integrals if they both want to throttle down
                        % saturation_xy = ((vel_err(0) *self.mpc_vel_sp(0) < 0.0f) && (vel_err(1) *self.mpc_vel_sp(1) < 0.0f)) ? saturation_xy : true;
                        if ( ((vel_err(1) * self.mpc_vel_sp(1)) < 0.0) && ...
                             ((vel_err(2) * self.mpc_vel_sp(2)) < 0.0) )
                            saturation_xy = saturation_xy; %#ok<ASGSL>
                        else
                            saturation_xy = true;
                        end
                    end
                end
            end
        end

        if (self.mpc_control_mode.flag_control_climb_rate_enabled && ~self.mpc_control_mode.flag_control_velocity_enabled)
            % thrust compensation when vertical velocity but not horizontal velocity is controlled
            %att_comp = 0;

            if (self.mpc_R(3, 3) > self.mpc_TILT_COS_MAX)
                att_comp = 1.0 / self.mpc_R(2, 2);

            elseif (self.mpc_R(3, 3) > 0.0)
                att_comp = ((1.0 / self.mpc_TILT_COS_MAX - 1.0) / self.mpc_TILT_COS_MAX) * self.mpc_R(3, 3) + 1.0;
                saturation_z = true;

            else
                att_comp = 1.0;
                saturation_z = true;
            end

            thrust_sp(3) = thrust_sp(3) * att_comp;
        end

        % limit max thrust
        thrust_abs = norm(thrust_sp); % recalculate because it might have changed

        if (thrust_abs > thr_max)
            if (thrust_sp(3) < 0.0)
                if (-thrust_sp(3) > thr_max)
                    % thrust Z component is too large, limit it
                    thrust_sp(1) = 0.0;
                    thrust_sp(2) = 0.0;
                    thrust_sp(3) = -thr_max;
                    saturation_xy = true;
                    
                    % Don't freeze altitude integral if it wants to throttle down
                    if (vel_err(3) < 0.0)
                        saturation_z = true;
                    else
                        saturation_z = saturation_z; %#ok<ASGSL>
                    end

                else
                    % preserve thrust Z component and lower XY, keeping altitude is more important than position
                    thrust_xy_max = sqrt(thr_max * thr_max - thrust_sp(3) * thrust_sp(3));
                    thrust_xy_abs = norm([thrust_sp(1); thrust_sp(2)]);
                    k = thrust_xy_max / thrust_xy_abs;
                    thrust_sp(1) = thrust_sp(1) * k;
                    thrust_sp(2) = thrust_sp(2) * k;
                    % Don't freeze x,y integrals if they both want to throttle down
                    % saturation_xy = ((vel_err(1) *self.mpc_vel_sp(1) < 0.0) && (vel_err(2) *self.mpc_vel_sp(2) < 0.0)) ? saturation_xy : true;
                    if (((vel_err(1) * self.mpc_vel_sp(1)) < 0.0) && ((vel_err(2) * self.mpc_vel_sp(2)) < 0.0))
                        saturation_xy = saturation_xy;
                    else
                        saturation_xy = true;
                    end
                end

            else
                % Z component is negative, going down, simply limit thrust vector
                k = thr_max / thrust_abs;
                thrust_sp = thrust_sp .* k;
                saturation_xy = true;
                saturation_z = true;
            end

            thrust_abs = thr_max;
        end

        % update integrals
        if (self.mpc_control_mode.flag_control_velocity_enabled && ~saturation_xy)
            self.mpc_thrust_int(1) = self.mpc_thrust_int(1) + vel_err(1) * self.mpc_params.vel_i(1) * dt;
            self.mpc_thrust_int(2) = self.mpc_thrust_int(2) + vel_err(2) * self.mpc_params.vel_i(2) * dt;
        end

        if (self.mpc_control_mode.flag_control_climb_rate_enabled && ~saturation_z)
           self.mpc_thrust_int(3) = self.mpc_thrust_int(3) + vel_err(3) * self.mpc_params.vel_i(3) * dt;

            % protection against flipping on ground when landing
            if (self.mpc_thrust_int(3) > 0.0)
                self.mpc_thrust_int(3) = 0.0;
            end
        end

        % calculate attitude setpoint from thrust vector
        if (self.mpc_control_mode.flag_control_velocity_enabled || self.mpc_control_mode.flag_control_acceleration_enabled)
            % desired body_z axis = -normalize(thrust_vector)
            body_x = zeros(3,1);
            body_y = zeros(3,1);
            body_z = zeros(3,1);

            if (thrust_abs > self.mpc_SIGMA)
                body_z = -thrust_sp / thrust_abs;

            else
                % no thrust, set Z axis to safe value
                body_z = body_z .* 0;
                body_z(3) = 1.0;
            end

            % vector of desired yaw direction in XY plane, rotated by PI/2
            y_C = [-sin(self.mpc_att_sp.yaw_body);
                    cos(self.mpc_att_sp.yaw_body);
                    0.0];

            if ( abs(body_z(3)) > self.mpc_SIGMA )
                % desired body_x axis, orthogonal to body_z
                body_x = cross(y_C, body_z);

                % keep nose to front while inverted upside down
                if (body_z(3) < 0.0)
                    body_x = -body_x;
                end

                body_x = normalize(body_x);

            else
                % desired thrust is in XY plane, set X downside to construct correct matrix,
                % but yaw component will not be used actually
                body_x = body_x .* 0;
                body_x(3) = 1.0;
            end

            % desired body_y axis
            body_y = cross(body_z, body_x) ;

            % fill rotation matrix
            for i = 1:3
                self.mpc_R_setpoint(i, 1) = body_x(i);
                self.mpc_R_setpoint(i, 2) = body_y(i);
                self.mpc_R_setpoint(i, 3) = body_z(i);
            end

            % copy quaternion setpoint to attitude setpoint topic
            q_sp = self.mpc_R_setpoint;
            self.mpc_att_sp.q_d = q_sp;
            self.mpc_att_sp.q_d_valid = true;

            % calculate euler angles, for logging only, must not be used for control
            euler = self.mpc_R_setpoint;
            self.mpc_att_sp.roll_body  = euler(1);
            self.mpc_att_sp.pitch_body = euler(2);
            % yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity

        elseif (~self.mpc_control_mode.flag_control_manual_enabled)
            % autonomous altitude control without position control (failsafe landing),
            % force level attitude, don't change yaw
            self.mpc_R_setpoint = from_euler(0.0, 0.0, self.mpc_att_sp.yaw_body);

            % copy quaternion setpoint to attitude setpoint topic
            q_sp = from_dcm(self.mpc_R_setpoint); % quat
            self.mpc_att_sp.q_d =  q_sp;
            self.mpc_att_sp.q_d_valid = true;

            self.mpc_att_sp.roll_body = 0.0;
            self.mpc_att_sp.pitch_body = 0.0;
        end

        self.mpc_att_sp.thrust = thrust_abs;

        % save thrust setpoint for logging
        self.mpc_local_pos_sp.acc_x = thrust_sp(1) * ONE_G;
        self.mpc_local_pos_sp.acc_y = thrust_sp(2) * ONE_G;
        self.mpc_local_pos_sp.acc_z = thrust_sp(3) * ONE_G;

        self.mpc_att_sp.timestamp = self.mpc_hrt_absolute_time;


    else
        self.mpc_reset_int_z = true;
    end
end

%% Generate attitude setpoint
function self = generate_attitude_setpoint(dt, self)
    % reset yaw setpoint to current position if needed
    if (self.mpc_reset_yaw_sp)
        self.mpc_reset_yaw_sp = false;
        self.mpc_att_sp.yaw_body = self.mpc_yaw;

        % do not move yaw while sitting on the ground
    elseif (~self.mpc_vehicle_land_detected.landed && ...
         ~(~self.mpc_control_mode.flag_control_altitude_enabled && (self.mpc_manual.z < 0.1)))

        % we want to know the real constraint, and global overrides manual
        if (self.mpc_params.man_yaw_max < self.mpc_params.global_yaw_max)
            yaw_rate_max = self.mpc_params.man_yaw_max;
        else
            yaw_rate_max = self.mpc_params.global_yaw_max;
        end
        
        yaw_offset_max = yaw_rate_max / self.mpc_params.mc_att_yaw_p;

        self.mpc_att_sp.yaw_sp_move_rate = self.mpc_manual.r * yaw_rate_max;
        yaw_target = wrap_pi(self.mpc_att_sp.yaw_body + self.mpc_att_sp.yaw_sp_move_rate * dt);
        yaw_offs   = wrap_pi(yaw_target - self.mpc_yaw);

        % If the yaw offset became too big for the system to track stop
        % shifting it, only allow if it would make the offset smaller again.
        if (fabsf(yaw_offs) < yaw_offset_max || ...
            ((self.mpc_att_sp.yaw_sp_move_rate > 0) && (yaw_offs < 0)) || ...
            ((self.mpc_att_sp.yaw_sp_move_rate < 0) && (yaw_offs > 0)))
            self.mpc_att_sp.yaw_body = yaw_target;
        end
        end

    % control throttle directly if no climb rate controller is active
    if (~self.mpc_control_mode.flag_control_climb_rate_enabled)
        thr_val = throttle_curve(self.mpc_manual.z, self.mpc_params.thr_hover);
        self.mpc_att_sp.thrust = min(thr_val, self.mpc_manual_thr_max);

        % enforce minimum throttle if not landed
        if (~self.mpc_vehicle_land_detected.landed)
            self.mpc_att_sp.thrust = max( self.mpc_att_sp.thrust, self.mpc_manual_thr_min );
        end
    end

    % control roll and pitch directly if no aiding velocity controller is active
    if (~self.mpc_control_mode.flag_control_velocity_enabled)
        self.mpc_att_sp.roll_body  =  self.mpc_manual.y * self.mpc_params.man_roll_max;
        self.mpc_att_sp.pitch_body = -self.mpc_manual.x * self.mpc_params.man_pitch_max;

        % only if optimal recovery is not used, modify roll/pitch
        if (self.mpc_params.opt_recover <= 0)
            % construct attitude setpoint rotation matrix. modify the setpoints for roll
            % and pitch such that they reflect the user's intention even if a yaw error
            % (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
            % from the pure euler angle setpoints will lead to unexpected attitude behaviour from
            % the user's view as the euler angle sequence uses the  yaw setpoint and not the current
            % heading of the vehicle.

            % calculate our current yaw error
            yaw_error = wrap_pi(self.mpc_att_sp.yaw_body - self.mpc_yaw);

            % compute the vector obtained by rotating a z unit vector by the rotation
            % given by the roll and pitch commands of the user
            zB = [0; 0; 1];
            % R_sp_roll_pitch;
            R_sp_roll_pitch = from_euler(self.mpc_att_sp.roll_body, self.mpc_att_sp.pitch_body, 0);
            z_roll_pitch_sp = R_sp_roll_pitch * zB;

            % transform the vector into a new frame which is rotated around the z axis
            % by the current yaw error. this vector defines the desired tilt when we look
            % into the direction of the desired heading
            % R_yaw_correction;
            R_yaw_correction = from_euler(0.0, 0.0, -yaw_error);
            z_roll_pitch_sp  = R_yaw_correction * z_roll_pitch_sp;

            % use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
            % R_tilt is computed from_euler; only true if cos(roll) not equal zero
            % -> valid if roll is not +-pi/2;
            self.mpc_att_sp.roll_body = -asin(z_roll_pitch_sp(2));
            self.mpc_att_sp.pitch_body = atan2( z_roll_pitch_sp(1), z_roll_pitch_sp(3) );
        end

        % copy quaternion setpoint to attitude setpoint topic
        q_sp = quat_from_euler(self.mpc_att_sp.roll_body, self.mpc_att_sp.pitch_body, self.mpc_att_sp.yaw_body);
        self.mpc_att_sp.q_d = q_sp;
        self.mpc_att_sp.q_d_valid = true;
    end

    if (self.mpc_manual.gear_switch == self.mpc_manual_control_setpoint_s.SWITCH_POS_ON && ...
        ~self.mpc_vehicle_land_detected.landed)
        self.mpc_att_sp.landing_gear = 1.0;

    elseif (self.mpc_manual.gear_switch == self.mpc_manual_control_setpoint_s.SWITCH_POS_OFF)
        self.mpc_att_sp.landing_gear = -1.0;
    end

    self.mpc_att_sp.timestamp = self.mpc_hrt_absolute_time;
end

%% Main task
function self = task_main(self)
% TASK_MAIN
% do subscriptions
%     self.mpc_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
%     self.mpc_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
%     self.mpc_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
%     self.mpc_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
%     self.mpc_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
%     self.mpc_params_sub = orb_subscribe(ORB_ID(parameter_update));
%     self.mpc_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
%     self.mpc_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
%     self.mpc_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
%     self.mpc_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
%     self.mpc_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
%     self.mpc_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));

    if (~self.mpc_initialized)
        self = constructor(self);
        self = parameters_update(true, self);

        % initialize values of critical structs until first regular update
        self.mpc_arming.armed = false;

        % get an initial update for all sensor and status data
        poll_subscriptions();

        % We really need to know from the beginning if we're landed or in-air.
        % orb_copy(ORB_ID(vehicle_land_detected),
        self.mpc_vehicle_land_detected = self.mpc_vehicle_land_detected_sub;
        was_armed = false;

        t_prev = 0;

        % Let's be safe and have the landing gear down by default
        self.mpc_att_sp.landing_gear = -1.0;
        
        self.mpc_initialized = true;
    end

    % wakeup source
    %     px4_pollfd_struct_t fds[1];

    %fds[0].fd = self.mpc_local_pos_sub;
    %fds[0].events = POLLIN;

    %while (~self.mpc_task_should_exit)
    % wait for up to 20ms for data ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~?
    %pret = 1; % ??
    %pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

    % timed out - periodic check for self.mpc_task_should_exit
    %if (pret == 0)
        % Go through the loop anyway to copy manual input at 50 Hz.
    %end

    % this is undesirable but not much we can do
    %if (pret < 0)
        % warning('poll error %d, %d', pret, errno);
        % continue;
    %end

    self = poll_subscriptions(self);

    self = parameters_update(false, self);

    self.mpc_t = self.mpc_hrt_absolute_time;

    if (self.mpc_t_prev ~= 0)
        dt = (self.mpc_t - self.mpc_t_prev) / 1e6;
    else
        dt = 0.0;
    end
    self.mpc_t_prev = self.mpc_t;

    % set dt for control blocks
    % setDt(dt); ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ?

    if (self.mpc_control_mode.flag_armed && ~self.mpc_was_armed)
        % reset setpoints and integrals on arming
        self.mpc_reset_pos_sp = true;
        self.mpc_reset_alt_sp = true;
        self.mpc_do_reset_alt_pos_flag = true;
        self.mpc_vel_sp_prev = self.mpc_vel_sp_prev .* 0;
        self.mpc_reset_int_z = true;
        self.mpc_reset_int_xy = true;
        self.mpc_reset_yaw_sp = true;
    end

    % reset yaw and altitude setpoint for VTOL which are in fw mode
    if (self.mpc_vehicle_status.is_vtol && ~self.mpc_vehicle_status.is_rotary_wing)
        self.mpc_reset_yaw_sp = true;
        self.mpc_reset_alt_sp = true;
    end

    % Update previous arming state
    self.mpc_was_armed = self.mpc_control_mode.flag_armed;

    self = update_ref(self);


    self = update_velocity_derivative(self);

    % reset the horizontal and vertical position hold flags for non-manual modes
    % or if position / altitude is not controlled
    if (~self.mpc_control_mode.flag_control_position_enabled || ~self.mpc_control_mode.flag_control_manual_enabled)
        self.mpc_pos_hold_engaged = false;
    end

    if (~self.mpc_control_mode.flag_control_altitude_enabled || ~self.mpc_control_mode.flag_control_manual_enabled)
        self.mpc_alt_hold_engaged = false;
    end

    if (self.mpc_control_mode.flag_control_altitude_enabled   || ...
        self.mpc_control_mode.flag_control_position_enabled   || ...
        self.mpc_control_mode.flag_control_climb_rate_enabled || ...
        self.mpc_control_mode.flag_control_velocity_enabled   || ...
        self.mpc_control_mode.flag_control_acceleration_enabled)

        self = do_control(dt, self);

        % fill local position, velocity and thrust setpoint
        self.mpc_local_pos_sp.timestamp = self.mpc_hrt_absolute_time;
        self.mpc_local_pos_sp.x         = self.mpc_pos_sp(1);
        self.mpc_local_pos_sp.y         = self.mpc_pos_sp(2);
        self.mpc_local_pos_sp.z         = self.mpc_pos_sp(3);
        self.mpc_local_pos_sp.yaw       = self.mpc_att_sp.yaw_body;
        self.mpc_local_pos_sp.vx        = self.mpc_vel_sp(1);
        self.mpc_local_pos_sp.vy        = self.mpc_vel_sp(2);
        self.mpc_local_pos_sp.vz        = self.mpc_vel_sp(3);

        % publish local position setpoint
%             if (self.mpc_local_pos_sp_pub ~= nullptr)
%                 orb_publish(ORB_ID(vehicle_local_position_setpoint), self.mpc_local_pos_sp_pub, &self.mpc_local_pos_sp);
% 
%             else
%                 self.mpc_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &self.mpc_local_pos_sp);
%             end

    else
        % position controller disabled, reset setpoints
        self.mpc_reset_pos_sp = true;
        self.mpc_reset_alt_sp = true;
        self.mpc_do_reset_alt_pos_flag = true;
        self.mpc_mode_auto = false;
        self.mpc_reset_int_z = true;
        self.mpc_reset_int_xy = true;
        control_vel_enabled_prev = false;

        % store last velocity in case a mode switch to position control occurs
        self.mpc_vel_sp_prev = self.mpc_vel;
    end

    % generate attitude setpoint from manual controls
    if (self.mpc_control_mode.flag_control_manual_enabled && self.mpc_control_mode.flag_control_attitude_enabled)

        self = generate_attitude_setpoint(dt, self);

    else
        self.mpc_reset_yaw_sp = true;
        self.mpc_att_sp.yaw_sp_move_rate = 0.0;
    end

    % update previous velocity for velocity controller D part
    self.mpc_vel_prev = self.mpc_vel;

    % publish attitude setpoint
    % Do not publish if offboard is enabled but position/velocity/accel control is disabled,
    % in this case the attitude setpoint is published by the mavlink app. Also do not publish
    % if the vehicle is a VTOL and it's just doing a transition (the VTOL attitude control module will generate
    % attitude setpoints for the transition).
    if (~(self.mpc_control_mode.flag_control_offboard_enabled && ...
        ~(self.mpc_control_mode.flag_control_position_enabled || ...
          self.mpc_control_mode.flag_control_velocity_enabled || ...
          self.mpc_control_mode.flag_control_acceleration_enabled)))

%             if (self.mpc_att_sp_pub ~= nullptr)
%                 orb_publish(self.mpc_attitude_setpoint_id, self.mpc_att_sp_pub, &self.mpc_att_sp);
% 
%             elseif (self.mpc_attitude_setpoint_id)
%                 self.mpc_att_sp_pub = orb_advertise(self.mpc_attitude_setpoint_id, &self.mpc_att_sp);
%             end
    end

    % reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control
    self.mpc_reset_int_z_manual = self.mpc_control_mode.flag_armed && self.mpc_control_mode.flag_control_manual_enabled ...
                  && ~self.mpc_control_mode.flag_control_climb_rate_enabled;
    %end

%     mavlink_log_info(&self.mpc_mavlink_log_pub, '[mpc] stopped');

    % self.mpc_control_task = -1;
end

%% Unneeded functions
% function [output, self] = start(self)
% {
%     ASSERT(self.mpc_control_task == -1);
% 
% % start the task
%     self.mpc_control_task = px4_task_spawn_cmd("mc_pos_control",
%                        SCHED_DEFAULT,
%                        SCHED_PRIORITY_MAX - 5,
%                        1900,
%                        (px4_main_t)&MulticopterPositionControl::task_main_trampoline,
%                        nullptr);
% 
%     if (self.mpc_control_task < 0) {
%         warn("task start failed");
%         return -errno;
%     }
% 
%     return OK;
% }

% int mc_pos_control_main(int argc, char *argv[])
% {
%     if (argc < 2) {
%         warnx("usage: mc_pos_control {start|stop|status}");
%         return 1;
%     }
% 
%     if (~strcmp(argv[1], "start")) {
% 
%         if (pos_control::g_control ~= nullptr) {
%             warnx("already running");
%             return 1;
%         }
% 
%         pos_control::g_control = new MulticopterPositionControl;
% 
%         if (pos_control::g_control == nullptr) {
%             warnx("alloc failed");
%             return 1;
%         }
% 
%         if (OK ~= pos_control::g_control->start()) {
%             delete pos_control::g_control;
%             pos_control::g_control = nullptr;
%             warnx("start failed");
%             return 1;
%         }
% 
%         return 0;
%     }
% 
%     if (~strcmp(argv[1], "stop")) {
%         if (pos_control::g_control == nullptr) {
%             warnx("not running");
%             return 1;
%         }
% 
%         delete pos_control::g_control;
%         pos_control::g_control = nullptr;
%         return 0;
%     }
% 
%     if (~strcmp(argv[1], "status")) {
%         if (pos_control::g_control) {
%             warnx("running");
%             return 0;
% 
%         else
%             warnx("not running");
%             return 1;
%         }
%     }
% 
%     warnx("unrecognized command");
%     return 1;
% }

%% New useful functions
function y = constrain( x, x_min, x_max )
    y = max( min(x, x_max), x_min );
end

function out = radians(in)
    out = in .* pi ./ 180;
end

function R  = to_dcm( data )
% TO_DCM Convert quaternion to DCM
% From https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/Quaternion.hpp
    aSq = data(1) * data(1);
    bSq = data(2) * data(2);
    cSq = data(3) * data(3);
    dSq = data(4) * data(4);
    R = zeros(3,3);
    R(1,1) = aSq + bSq - cSq - dSq;
    R(1,2) = 2.0 * ( data(2) * data(3) - data(1) * data(4) );
    R(1,3) = 2.0 * ( data(1) * data(3) + data(2) * data(4) );
    R(2,1) = 2.0 * ( data(2) * data(3) + data(1) * data(4) );
    R(2,2) = aSq - bSq + cSq - dSq;
    R(2,3) = 2.0 * ( data(3) * data(4) - data(1) * data(2) );
    R(3,1) = 2.0 * ( data(2) * data(4) - data(1) * data(3) );
    R(3,2) = 2.0 * ( data(1) * data(2) + data(3) * data(4) );
    R(3,3) = aSq - bSq - cSq + dSq;
end

function data  = from_dcm( dcm )
% FROM_DCM Convert DCM to quaternion
% From https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/Quaternion.hpp
    data = zeros(4,1);
    tr = dcm(1,1) + dcm(2,2) + dcm(3,3);
    if (tr > 0.0)
        s = sqrt(tr + 1.0);
        data(1) = s * 0.5;
        s = 0.5 / s;
        data(2) = (dcm(3,2) - dcm(2,3) ) * s;
        data(3) = (dcm(1,3) - dcm(3,1) ) * s;
        data(4) = (dcm(2,1) - dcm(1,2) ) * s;
    else
        % Find maximum diagonal element in dcm
        % store index in dcm_i
        dcm_i = 1;
        for i = 2:3 % (int i = 1; i < 3; i++)
            if ( dcm(i,i) > dcm(dcm_i,dcm_i) )
                dcm_i = i;
            end
        end
        dcm_j = rem( ( (dcm_i-1) + 1), 3) + 1;
        dcm_k = rem( ( (dcm_i-1) + 2), 3) + 1;
        s = sqrt( (dcm(dcm_i, dcm_i) - dcm(dcm_j, dcm_j) - ...
            dcm(dcm_k,dcm_k)) + 1.0 );
        data(dcm_i + 1) = s * 0.5;
        s = 0.5 / s;
        data(dcm_j + 1) = ( dcm(dcm_i,dcm_j) + dcm(dcm_j,dcm_i) ) * s;
        data(dcm_k + 1) = ( dcm(dcm_k,dcm_i) + dcm(dcm_i,dcm_k) ) * s;
        data(1) = ( dcm(dcm_k,dcm_j) - dcm(dcm_j,dcm_k) ) * s;
    end
end

function data = from_euler(roll, pitch, yaw)
% FROM_EULER Create a rotation matrix from given euler angles
% Based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
    cp = cos(pitch);
    sp = sin(pitch);
    sr = sin(roll);
    cr = cos(roll);
    sy = sin(yaw);
    cy = cos(yaw);

    data(1,1) = cp * cy;
    data(1,2) = (sr * sp * cy) - (cr * sy);
    data(1,3) = (cr * sp * cy) + (sr * sy);
    data(2,1) = cp * sy;
    data(2,2) = (sr * sp * sy) + (cr * cy);
    data(2,3) = (cr * sp * sy) - (sr * cy);
    data(3,1) = -sp;
    data(3,2) = sr * cp;
    data(3,3) = cr * cp;
end

function euler = dcm_to_euler(data)
% DCM_TO_EULER get euler angles from rotation matrix
    euler = zeros(3,1);
    euler(2) = asin( -data(3,1) );

    if ( abs(euler.data(2) - 1.57079632) < 1.0e-3)
        euler(1) = 0.0;
        euler(3) = atan2(data(2,3) - data(1,2), data(1,3) + data(2,2)) + euler(1);

    elseif ( abs(euler.data(2) + 1.57079632) < 1.0e-3 )
        euler(1) = 0.0;
        euler(3) = atan2(data(2,3) - data(1,2), data(1,3) + data(2,2)) - euler(1);
    else
        euler(1) = atan2(data(3,1), data(3,3));
        euler(3) = atan2(data(2,1), data(1,1));
    end
end

function out = quat_to_euler(data)
% QUAT_TO_EULER Create Euler angles vector from the quaternion
    out = [atan2(2.0 * (data(1) * data(2) + data(3) * data(4)), 1.0 - 2.0 * (data(2) * data(2) + data(3) * data(3)));
    asin(2.0 * (data(1) * data(3) - data(4) * data(2)));
    atan2(2.0 * (data(1) * data(4) + data(2) * data(3)), 1.0 - 2.0 * (data(3) * data(3) + data(4) * data(4)))];
end

function quat = quat_from_euler(roll, pitch, yaw)
% QUAT_FROM_EULER Set quaternion to rotation defined by euler angles
     cosPhi_2   = cos(roll / 2.0);
     sinPhi_2   = sin(roll / 2.0);
     cosTheta_2 = cos(pitch / 2.0);
     sinTheta_2 = sin(pitch / 2.0);
     cosPsi_2   = cos(yaw / 2.0);
     sinPsi_2   = sin(yaw / 2.0);

    % operations executed in double to avoid loss of precision through
    % consecutive multiplications. Result stored as float.
    quat(1) = cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2;
    quat(2) = sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2;
    quat(3) = cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2;
    quat(4) = cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2;
end

function out = normalize(in)
    out = in ./ norm(in);
end

function [out, lat, lon] = map_projection_project(ref, x, y, lat, lon)
% MAP_PROJECTION_PROJECT
    CONSTANTS_RADIUS_OF_EARTH = 6371000;
    M_PI = 3.14159265358979323846;
    
    if ( ~map_projection_initialized(ref))
        out = -1;
        return
    end

    x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
    y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
    c = sqrt(x_rad * x_rad + y_rad * y_rad);
    
    sin_c = sin(c);
    cos_c = cos(c);

    if (fabs(c) > eps)
        lat_rad = asin(cos_c * ref.sin_lat + (x_rad * sin_c * ref.cos_lat) / c);
        lon_rad = (ref.lon_rad + atan2(y_rad * sin_c, c * ref.cos_lat * cos_c - x_rad * ref.sin_lat * sin_c));
    else
        lat_rad = ref.lat_rad;
        lon_rad = ref.lon_rad;
    end

    lat = lat_rad * 180.0 / M_PI;
    lon = lon_rad * 180.0 / M_PI;

end

function init = map_projection_initialized(ref)
% MAP_PROJECTION_INITIALIZED
    init = ref.init_done;
end

function bearing = wrap_pi(bearing)
    % value is inf or NaN
    if (~isfinite(bearing))
        return
    end

    c = 0;

    while (bearing >= 3.14159265)
        bearing = bearing - 6.28318531;

        if (c > 3)
            bearing = nan;
            return
        end
        c = c + 1;
    end

    c = 0;

    while (bearing < -3.14159265)
        bearing = bearing + 6.28318531;

        if (c > 3)
            bearing = nan;
            return
        end
        c = c + 1;
    end
end

function [out, vel_block] = block_derivative_update( input, vel_block )
    if (vel_block.initialized)
        [out, vel_block.lp_block] = block_lowpass_update( (input - vel_block.u ) / vel_block.dt, vel_block.lp_block);
    else
        % if this is the first call to update
        % we have no valid derivative
        % and so we use the assumption the
        % input value is not changing much,
        % which is the best we can do here.
        [~, vel_block.lp_block] = block_lowpass_update(0.0, vel_block.lp_block);
        out = 0.0;
        vel_block.initialized = true;
    end

    vel_block.u = input;
end

function [out, lp_block] = block_lowpass_update( input, lp_block )
    M_PI = 3.14159265358979323846;
    if (~isfinite(lp_block.state))
		lp_block.state = input;
    end

    b = 2 * M_PI * lp_block.fcut * lp_block.dt;
    a = b / (1 + b);
    lp_block.state = a * input + (1 - a) * lp_block.state;
    out = lp_block.state;
end