function self = mc_pos_control( self )
% MC_POS_CONTROL
% /****************************************************************************
% *
% *  Copyright (c) 2013 - 2016 PX4 Development Team. All rights reserved.
% *
% *Redistribution and use in source and binary forms, with or without
% *modification, are permitted provided that the following conditions
% *are met:
% *
% *1. Redistributions of source code must retain the above copyright
% *   notice, this list of conditions and the following disclaimer.
% *2. Redistributions in binary form must reproduce the above copyright
% *   notice, this list of conditions and the following disclaimer in
% *   the documentation and/or other materials provided with the
% *   distribution.
% *3. Neither the name PX4 nor the names of its contributors may be
% *   used to endorse or promote products derived from this software
% *   without specific prior written permission.
% *
% *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% *"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% *LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
% *FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
% *COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% *INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% *BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
% *OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
% *AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
% *LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
% *ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% *POSSIBILITY OF SUCH DAMAGE.
% *
% ****************************************************************************/

% /**
% *@file mc_pos_control_main.cpp
% *Multicopter position controller.
% *
% *Original publication for the desired attitude generation:
% *Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
% *Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
% *
% *Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
% *
% *The controller has two loops: P loop for position error and PID loop for velocity error.
% *Output of velocity controller is thrust vector that splitted to thrust direction
% *(i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
% *Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
% *
% *@author Anton Babushkin <anton.babushkin@me.com>
% */
    
    self.TILT_COS_MAX = 0.7;
    self.SIGMA = 0.000001;
    self.MIN_DIST = 0.01;
    self.MANUAL_THROTTLE_MAX_MULTICOPTER = 0.9;
    self.ONE_G = 9.8066;

end

% /**
% *Multicopter position control app start / stop handling function
% *
% *@ingroup apps
% */

% class MulticopterPositionControl : public control::SuperBlock
% {
% public:
%     % /**
% % *Constructor
%      */
%     MulticopterPositionControl();
% 
%     % /**
% % *Destructor, also kills task.
%      */
%     ~MulticopterPositionControl();
% 
%     % /**
% % *Start task.
%      *
% % *@return        OK on success.
%      */
%     int        start();
% 
%     bool        cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
%                       const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res);
% 
% private:
%     bool        self.task_should_exit;        % /**< if true, task should exit */
%     int        self.control_task;            % /**< task handle for task */
%     orb_advert_t    self.mavlink_log_pub;        % /**< mavlink log advert */
% 
%     int        self.vehicle_status_sub;        % /**< vehicle status subscription */
%     int        self.vehicle_land_detected_sub;    % /**< vehicle land detected subscription */
%     int        self.ctrl_state_sub;        % /**< control state subscription */
%     int        self.att_sp_sub;            % /**< vehicle attitude setpoint */
%     int        self.control_mode_sub;        % /**< vehicle control mode subscription */
%     int        self.params_sub;            % /**< notification of parameter updates */
%     int        self.manual_sub;            % /**< notification of manual control updates */
%     int        self.arming_sub;            % /**< arming status of outputs */
%     int        self.local_pos_sub;            % /**< vehicle local position */
%     int        self.pos_sp_triplet_sub;        % /**< position setpoint triplet */
%     int        self.local_pos_sp_sub;        % /**< offboard local position setpoint */
%     int        self.global_vel_sp_sub;        % /**< offboard global velocity setpoint */
% 
%     orb_advert_t    self.att_sp_pub;            % /**< attitude setpoint publication */
%     orb_advert_t    self.local_pos_sp_pub;        % /**< vehicle local position setpoint publication */
%     orb_advert_t    self.global_vel_sp_pub;        % /**< vehicle global velocity setpoint publication */
% 
%     orb_id_t self.attitude_setpoint_id;
% 
%     struct vehicle_status_s             self.vehicle_status;     % /**< vehicle status */
%     struct vehicle_land_detected_s             self.vehicle_land_detected;    % /**< vehicle land detected */
%     struct control_state_s                self.ctrl_state;        % /**< vehicle attitude */
%     struct vehicle_attitude_setpoint_s        self.att_sp;        % /**< vehicle attitude setpoint */
%     struct manual_control_setpoint_s        self.manual;        % /**< r/c channel data */
%     struct vehicle_control_mode_s            self.control_mode;        % /**< vehicle control mode */
%     struct actuator_armed_s                self.arming;        % /**< actuator arming status */
%     struct vehicle_local_position_s            self.local_pos;        % /**< vehicle local position */
%     struct position_setpoint_triplet_s        self.pos_sp_triplet;    % /**< vehicle global position setpoint triplet */
%     struct vehicle_local_position_setpoint_s    self.local_pos_sp;        % /**< vehicle local position setpoint */
%     struct vehicle_global_velocity_setpoint_s    self.global_vel_sp;        % /**< vehicle global velocity setpoint */
% 
%     control::BlockParamFloat self.manual_thr_min;
%     control::BlockParamFloat self.manual_thr_max;
% 
%     control::BlockDerivative self.vel_x_deriv;
%     control::BlockDerivative self.vel_y_deriv;
%     control::BlockDerivative self.vel_z_deriv;
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
%     }        self.params_handles;        % /**< handles for interesting parameters */
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
%     }        self.params;
% 
%     struct map_projection_reference_s self.ref_pos;
%     float self.ref_alt;
%     hrt_abstime self.ref_timestamp;
% 
%     bool self.reset_pos_sp;
%     bool self.reset_alt_sp;
%     bool self.do_reset_alt_pos_flag;
%     bool self.mode_auto;
%     bool self.pos_hold_engaged;
%     bool self.alt_hold_engaged;
%     bool self.run_pos_control;
%     bool self.run_alt_control;
% 
%     bool self.reset_int_z = true;
%     bool self.reset_int_xy = true;
%     bool self.reset_int_z_manual = false;
%     bool self.reset_yaw_sp = true;
% 
%     bool self.hold_offboard_xy = false;
%     bool self.hold_offboard_z = false;
% 
%     math::Vector<3> self.thrust_int;
% 
%     math::Vector<3> self.pos;
%     math::Vector<3> self.pos_sp;
%     math::Vector<3> self.vel;
%     math::Vector<3> self.vel_sp;
%     math::Vector<3> self.vel_prev;            % /**< velocity on previous step */
%     math::Vector<3> self.vel_ff;
%     math::Vector<3> self.vel_sp_prev;
%     math::Vector<3> self.vel_err_d;        % /**< derivative of current velocity */
% 
%     math::Matrix<3, 3> self.R;            % /**< rotation matrix from attitude quaternions */
%     float self.yaw;                % /**< yaw angle (euler) */
%     bool self.in_landing;    % /**< the vehicle is in the landing descent */
%     bool self.lnd_reached_ground; % /**< controller assumes the vehicle has reached the ground after landing */
%     bool self.takeoff_jumped;
%     float self.vel_z_lp;
%     float self.acc_z_lp;
%     float self.takeoff_thrust_sp;
%     bool control_vel_enabled_prev;    % /**< previous loop was in velocity controlled mode (control_state.flag_control_velocity_enabled) */
% 
%     % counters for reset events on position and velocity states
%     % they are used to identify a reset event
%     uint8_t self.z_reset_counter;
%     uint8_t self.xy_reset_counter;
%     uint8_t self.vz_reset_counter;
%     uint8_t self.vxy_reset_counter;
%     uint8_t self.heading_reset_counter;
% 
%     matrix::Dcmf self.R_setpoint;
% 
%     % /**
% % *Update our local parameter cache.
%      */
%     int        parameters_update(bool force);
% 
%     % /**
% % *Update control outputs
%      */
%     void        control_update();
% 
%     % /**
% % *Check for changes in subscribed topics.
%      */
%     void        poll_subscriptions();
% 
%     static float    scale_control(float ctl, float end, float dz, float dy);
%     static float    throttle_curve(float ctl, float ctr);
% 
%     % /**
% % *Update reference for local position projection
%      */
%     void        update_ref();
%     % /**
% % *Reset position setpoint to current position.
%      *
% % *This reset will only occur if the self.reset_pos_sp flag has been set.
% % *The general logic is to first "activate" the flag in the flight
% % *regime where a switch to a position control mode should hold the
% % *very last position. Once switching to a position control mode
% % *the last position is stored once.
%      */
%     void        reset_pos_sp();
% 
%     % /**
% % *Reset altitude setpoint to current altitude.
%      *
% % *This reset will only occur if the self.reset_alt_sp flag has been set.
% % *The general logic follows the reset_pos_sp() architecture.
%      */
%     void        reset_alt_sp();
% 
%     % /**
% % *Check if position setpoint is too far from current position and adjust it if needed.
%      */
%     void        limit_pos_sp_offset();
% 
%     % /**
% % *Set position setpoint using manual control
%      */
%     void        control_manual(float dt);
% 
%     void control_non_manual(float dt);
% 
%     % /**
% % *Set position setpoint using offboard control
%      */
%     void        control_offboard(float dt);
% 
%     % /**
% % *Set position setpoint for AUTO
%      */
%     void        control_auto(float dt);
% 
%     void control_position(float dt);
% 
%     void limit_acceleration(float dt);
% 
%     % /**
% % *Select between barometric and global (AMSL) altitudes
%      */
%     void        select_alt(bool global);
% 
%     void update_velocity_derivative();
% 
%     void do_control(float dt);
% 
%     void generate_attitude_setpoint(float dt);
% 
%     % /**
% % *Shim for calling task_main from task_create.
%      */
%     static void    task_main_trampoline(int argc, char *argv[]);
% 
%     % /**
% % *Main sensor collection task.
%      */
%     void        task_main();
% };
% 
% namespace pos_control
% {
% 
% MulticopterPositionControl    *g_control;
% }

% MulticopterPositionControl::MulticopterPositionControl() :
%     SuperBlock(NULL, "MPC"),
%     self.task_should_exit(false),
%     self.control_task(-1),
%     self.mavlink_log_pub(nullptr),

% % *subscriptions */
%     self.ctrl_state_sub(-1),
%     self.att_sp_sub(-1),
%     self.control_mode_sub(-1),
%     self.params_sub(-1),
%     self.manual_sub(-1),
%     self.arming_sub(-1),
%     self.local_pos_sub(-1),
%     self.pos_sp_triplet_sub(-1),
%     self.global_vel_sp_sub(-1),

% % *publications */
%     self.att_sp_pub(nullptr),
%     self.local_pos_sp_pub(nullptr),
%     self.global_vel_sp_pub(nullptr),
%     self.attitude_setpoint_id(0),
%     self.vehicle_status{},
%     self.vehicle_land_detected{},
%     self.ctrl_state{},
%     self.att_sp{},
%     self.manual{},
%     self.control_mode{},
%     self.arming{},
%     self.local_pos{},
%     self.pos_sp_triplet{},
%     self.local_pos_sp{},
%     self.global_vel_sp{},
%     self.manual_thr_min(this, "MANTHR_MIN"),
%     self.manual_thr_max(this, "MANTHR_MAX"),
%     self.vel_x_deriv(this, "VELD"),
%     self.vel_y_deriv(this, "VELD"),
%     self.vel_z_deriv(this, "VELD"),
%     self.ref_alt(0.0),
%     self.ref_timestamp(0),

%     self.reset_pos_sp(true),
%     self.reset_alt_sp(true),
%     self.do_reset_alt_pos_flag(true),
%     self.mode_auto(false),
%     self.pos_hold_engaged(false),
%     self.alt_hold_engaged(false),
%     self.run_pos_control(true),
%     self.run_alt_control(true),
%     self.yaw(0.0),
%     self.in_landing(false),
%     self.lnd_reached_ground(false),
%     self.takeoff_jumped(false),
%     self.vel_z_lp(0),
%     self.acc_z_lp(0),
%     self.takeoff_thrust_sp(0.0),
%     control_vel_enabled_prev(false),
%     self.z_reset_counter(0),
%     self.xy_reset_counter(0),
%     self.vz_reset_counter(0),
%     self.vxy_reset_counter(0),
%     self.heading_reset_counter(0)
% {
%     % Make the quaternion valid for control state
%     self.ctrl_state.q[0] = 1.0;

%     memset(&self.ref_pos, 0, sizeof(self.ref_pos));

%     self.params.pos_p.zero();
%     self.params.vel_p.zero();
%     self.params.vel_i.zero();
%     self.params.vel_d.zero();
%     self.params.vel_max.zero();
%     self.params.vel_cruise.zero();
%     self.params.vel_ff.zero();
%     self.params.sp_offs_max.zero();

%     self.pos.zero();
%     self.pos_sp.zero();
%     self.vel.zero();
%     self.vel_sp.zero();
%     self.vel_prev.zero();
%     self.vel_ff.zero();
%     self.vel_sp_prev.zero();
%     self.vel_err_d.zero();

%     self.R.identity();

%     self.R_setpoint.identity();

%     self.thrust_int.zero();

%     self.params_handles.thr_min        = param_find("MPC_THR_MIN");
%     self.params_handles.thr_max        = param_find("MPC_THR_MAX");
%     self.params_handles.thr_hover    = param_find("MPC_THR_HOVER");
%     self.params_handles.alt_ctl_dz    = param_find("MPC_ALTCTL_DZ");
%     self.params_handles.alt_ctl_dy    = param_find("MPC_ALTCTL_DY");
%     self.params_handles.z_p        = param_find("MPC_Z_P");
%     self.params_handles.z_vel_p        = param_find("MPC_Z_VEL_P");
%     self.params_handles.z_vel_i        = param_find("MPC_Z_VEL_I");
%     self.params_handles.z_vel_d        = param_find("MPC_Z_VEL_D");
%     self.params_handles.z_vel_max_up    = param_find("MPC_Z_VEL_MAX_UP");
%     self.params_handles.z_vel_max_down    = param_find("MPC_Z_VEL_MAX");

%     % transitional support: Copy param values from max to down
%     % param so that max param can be renamed in 1-2 releases
%     % (currently at 1.3.0)
%     float p;
%     param_get(param_find("MPC_Z_VEL_MAX"), &p);
%     param_set(param_find("MPC_Z_VEL_MAX_DN"), &p);

%     self.params_handles.z_ff        = param_find("MPC_Z_FF");
%     self.params_handles.xy_p        = param_find("MPC_XY_P");
%     self.params_handles.xy_vel_p    = param_find("MPC_XY_VEL_P");
%     self.params_handles.xy_vel_i    = param_find("MPC_XY_VEL_I");
%     self.params_handles.xy_vel_d    = param_find("MPC_XY_VEL_D");
%     self.params_handles.xy_vel_max    = param_find("MPC_XY_VEL_MAX");
%     self.params_handles.xy_vel_cruise    = param_find("MPC_XY_CRUISE");
%     self.params_handles.xy_ff        = param_find("MPC_XY_FF");
%     self.params_handles.tilt_max_air    = param_find("MPC_TILTMAX_AIR");
%     self.params_handles.land_speed    = param_find("MPC_LAND_SPEED");
%     self.params_handles.tko_speed    = param_find("MPC_TKO_SPEED");
%     self.params_handles.tilt_max_land    = param_find("MPC_TILTMAX_LND");
%     self.params_handles.man_roll_max = param_find("MPC_MAN_R_MAX");
%     self.params_handles.man_pitch_max = param_find("MPC_MAN_P_MAX");
%     self.params_handles.man_yaw_max = param_find("MPC_MAN_Y_MAX");
%     self.params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX");
%     self.params_handles.mc_att_yaw_p = param_find("MC_YAW_P");
%     self.params_handles.hold_xy_dz = param_find("MPC_HOLD_XY_DZ");
%     self.params_handles.hold_max_xy = param_find("MPC_HOLD_MAX_XY");
%     self.params_handles.hold_max_z = param_find("MPC_HOLD_MAX_Z");
%     self.params_handles.acc_hor_max = param_find("MPC_ACC_HOR_MAX");
%     self.params_handles.alt_mode = param_find("MPC_ALT_MODE");
%     self.params_handles.opt_recover = param_find("VT_OPT_RECOV_EN");

% % *fetch initial parameter values */
%     parameters_update(true);
% }

% MulticopterPositionControl::~MulticopterPositionControl()
% {
%     if (self.control_task ~= -1) {
% % *task wakes up every 100ms or so at the longest */
%         self.task_should_exit = true;

% % *wait for a second for the task to quit at our request */
%         unsigned i = 0;

%         do {
% % *wait 20ms */
%             usleep(20000);

% % *if we have given up, kill it */
%             if (++i > 50) {
%                 px4_task_delete(self.control_task);
%                 break;
%             }
%         } while (self.control_task ~= -1);
%     }

%     pos_control::g_control = nullptr;
% }

function self = parameters_update( force, self )

    updated = self.params_sub; % ???? check
    
    if (updated)
        self.params_sub;
    end

    if (updated || force)
        % Update C++ param system
        self = updateParams( self );

        % Update legacy C interface params
        self.params.thr_min = self.params_handles.thr_min;
        self.params.thr_max = self.params_handles.thr_max;
        self.params.thr_hover = self.params_handles.thr_hover;
        self.params.alt_ctl_dz = self.params_handles.alt_ctl_dz;
        self.params.alt_ctl_dy = self.params_handles.alt_ctl_dy;
        self.params.tilt_max_air = self.params_handles.tilt_max_air;
        self.params.tilt_max_air = radians( self.params.tilt_max_air );
        self.params.land_speed = self.params_handles.land_speed;
        self.params.tko_speed = self.params_handles.tko_speed;
        self.params.tilt_max_land = self.params_handles.tilt_max_land;
        self.params.tilt_max_land = radians( self.params.tilt_max_land );

        self.params.pos_p(1) = self.params_handles.xy_p;
        self.params.pos_p(2) = self.params_handles.xy_p;
        self.params.pos_p(3) = self.params_handles.z_p;
        self.params.vel_p(1) = self.params_handles.xy_vel_p;
        self.params.vel_p(2) = self.params_handles.xy_vel_p;
        self.params.vel_p(3) = self.params_handles.z_vel_p;
        self.params.vel_i(1) = self.params_handles.xy_vel_i;
        self.params.vel_i(2) = self.params_handles.xy_vel_i;
        self.params.vel_i(3) = self.params_handles.z_vel_i;
        self.params.vel_d(1) = self.params_handles.xy_vel_d;
        self.params.vel_d(2) = self.params_handles.xy_vel_d;
        self.params.vel_d(3) = self.params_handles.z_vel_d;
        self.params.vel_max(1) = self.params_handles.xy_vel_max;
        self.params.vel_max(2) = self.params_handles.xy_vel_max;
        self.params.vel_max_up = self.params_handles.z_vel_max_up;
        self.params.vel_max(3) = self.params_handles.z_vel_max_up;
        self.params.vel_max_down = self.params_handles.z_vel_max_down;
        self.params.vel_cruise(1) = self.params_handles.xy_vel_cruise;
        self.params.vel_cruise(2) = self.params_handles.xy_vel_cruise;
        % Using Z max up for now
        self.params.vel_cruise(3) = self.params_handles.z_vel_max_up;
        v = constrain(self.params_handles.xy_ff, 0.0, 1.0);
        self.params.vel_ff(1) = v;
        self.params.vel_ff(2) = v;
        v = constrain(self.params_handles.z_ff, 0.0, 1.0);
        self.params.vel_ff(3) = v;
        v = constrain(self.params_handles.hold_xy_dz, 0.0, 1.0);
        self.params.hold_xy_dz = v;
        if (self.params_handles.hold_max_xy < 0.0)
            self.params.hold_max_xy = 0;
        else
            self.params.hold_max_xy = self.params_handles.hold_max_xy;
        end
        if (self.params_handles.hold_max_z < 0.0)
            self.params.hold_max_z = 0;
        else
            self.params.hold_max_z = self.params_handles.hold_max_z;
        end
        self.params.acc_hor_max = self.params_handles.acc_hor_max;

        % Increase the maximum horizontal acceleration such that stopping
        % within 1 s from full speed is feasible
        self.params.acc_hor_max = max(self.params.vel_cruise(0), self.params.acc_hor_max);
        self.params.alt_mode = self.params_handles.alt_mode;
        self.params.opt_recover = self.params_handles.opt_recover;
        
        self.params.sp_offs_max = (self.params.vel_cruise ./ self.params.pos_p) * 2.0;

        % MC attitude control parameters
        % Manual control scale
       	self.params.man_roll_max = self.params_handles.man_roll_max;
        self.params.man_pitch_max = self.params_handles.man_pitch_max;
        self.params.man_yaw_max = self.params_handles.man_yaw_max;
        self.params.global_yaw_max = self.params_handles.global_yaw_max;
        self.params.man_roll_max = radians( self.params.man_roll_max );
        self.params.man_pitch_max = radians( self.params.man_pitch_max );
        self.params.man_yaw_max = radians( self.params.man_yaw_max );
        self.params.global_yaw_max = radians( self.params.global_yaw_max );
        
        self.params.mc_att_yaw_p = self.params_handles.mc_att_yaw_p;

        % Takeoff and land velocities should not exceed maximum
        self.params.tko_speed = min(self.params.tko_speed, self.params.vel_max_up);
        self.params.land_speed = min(self.params.land_speed, self.params.vel_max_down);
    end
end

function self = poll_subscriptions( self )

    updated = self.vehicle_status_sub; % ORB CHECK
    
    if (updated)
        self.vehicle_status = self.vehicle_status_sub; % ORB_ID(vehicle_status)
        
        % Set correct uORB ID, depending on if vehicle is VTOL or not
        if (~self.attitude_setpoint_id)
            if (self.vehicle_status.is_vtol)
                self.attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);
            else
                self.attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
            end
        end
    end

    updated = self.vehicle_land_detected_sub; % ORB_CHECK

    if (updated)
        self.vehicle_land_detected = self.vehicle_land_detected_sub; % ORB_ID(vehicle_land_detected)
    end
    
    updated = self.ctrl_state_sub; % ORB_CHECK

    if (updated)
        
        self.ctrl_state = self.ctrl_state_sub; % ORB_ID(control_state)

        % Get current rotation matrix and euler angles from control state quaternions
        q_att = [ self.ctrl_state.q(1);
                  self.ctrl_state.q(2);
                  self.ctrl_state.q(3);
                  self.ctrl_state.q(4) ];
        self.R = to_dcm( q_att );
        euler_angles = to_euler( self.R ); % math::Vector<3> 
        self.yaw = euler_angles(3);

        if (self.control_mode.flag_control_manual_enabled)
            if (self.heading_reset_counter ~= self.ctrl_state.quat_reset_counter)
                self.heading_reset_counter = self.ctrl_state.quat_reset_counter;
                delta_q = [ self.ctrl_state.delta_q_reset(1);
                            self.ctrl_state.delta_q_reset(2);
                            self.ctrl_state.delta_q_reset(3);
                            self.ctrl_state.delta_q_reset(4) ];

                % We only extract the heading change from the delta quaternion
                delta_euler = to_euler( delta_q );
                self.att_sp.yaw_body = self.att_sp.yaw_body + delta_euler(3);
            end
        end

    end
    
    updated = self.att_sp_sub;

    if (updated)
        self.att_sp = self.att_sp_sub; % ORB_ID(vehicle_attitude_setpoint)
    end

    updated = self.control_mode_sub; % ORB_CHECK
    if (updated)
        self.control_mode = self.control_mode_sub;
    end

    updated = self.manual_sub; % ORB_CHECK
    if (updated)
        self.manual = self.manual_sub;
    end

    updated = self.arming_sub; % ORB_CHECK
    if (updated)
        self.arming = self.arming_sub;
    end

    updated = self.local_pos_sub; % ORB_CHECK
    
    if (updated)
        self.local_pos = self.local_pos_sub; % ORB_ID(vehicle_local_position)

        % Check if a reset event has happened
        % If the vehicle is in manual mode we will shift the setpoints of the
        % states which were reset. In auto mode we do not shift the setpoints
        % since we want the vehicle to track the original state.
        if (self.control_mode.flag_control_manual_enabled)
            if (self.z_reset_counter ~= self.local_pos.z_reset_counter)
                self.pos_sp(3) = self.pos_sp(3) + self.local_pos.delta_z;
            end

            if (self.xy_reset_counter ~= self.local_pos.xy_reset_counter)
                self.pos_sp(1) = self.pos_sp(1) + self.local_pos.delta_xy(1);
                self.pos_sp(2) = self.pos_sp(2) + self.local_pos.delta_xy(2);
            end

            if (self.vz_reset_counter ~= self.local_pos.vz_reset_counter)
                self.vel_sp(3) = self.vel_sp(3) + self.local_pos.delta_vz;
                self.vel_sp_prev(3) = self.vel_sp_prev(3) +  self.local_pos.delta_vz;
            end

            if (self.vxy_reset_counter ~= self.local_pos.vxy_reset_counter)
                self.vel_sp(1) = self.vel_sp(1) + self.local_pos.delta_vxy(1);
                self.vel_sp(2) = self.vel_sp(2) + self.local_pos.delta_vxy(2);
                self.vel_sp_prev(1) = self.vel_sp_prev(1) + self.local_pos.delta_vxy(1);
                self.vel_sp_prev(2) = self.vel_sp_prev(2) + self.local_pos.delta_vxy(2);
            end
        end

        % update the reset counters in any case
        self.z_reset_counter = self.local_pos.z_reset_counter;
        self.xy_reset_counter = self.local_pos.xy_reset_counter;
        self.vz_reset_counter = self.local_pos.vz_reset_counter;
        self.vxy_reset_counter = self.local_pos.vxy_reset_counter;
    end
    
    updated = self.pos_sp_triplet_sub; % ORB_CHECK

    if (updated)
        self.pos_sp_triplet = self.pos_sp_triplet_sub; % ORB_ID(position_setpoint_triplet)
        % Make sure that the position setpoint is valid
        if (isinf(self.pos_sp_triplet.current.lat) || ...
            isinf(self.pos_sp_triplet.current.lon) || ...
            isinf(self.pos_sp_triplet.current.alt) )
            self.pos_sp_triplet.current.valid = false;
        end
    end
end

function output = scale_control(ctl, end1, dz, dy)
    if (ctl > dz)
        output = dy + (ctl - dz) * (1.0 - dy) / (end1 - dz);
    elseif (ctl < -dz)
        output = -dy + (ctl + dz) * (1.0 - dy) / (end1 - dz);
    else
        output = ctl * (dy / dz);
    end
end

function output = throttle_curve(ctl, ctr)
    % Piecewise linear mapping: 0:ctr -> 0:0.5
    % and ctr:1 -> 0.5:1 */
    if (ctl < 0.5)
        output = 2 * ctl * ctr;
    else
        output = ctr + 2 * (ctl - 0.5) * (1.0 - ctr);
    end
end

function self = update_ref(self)
    if (self.local_pos.ref_timestamp ~= self.ref_timestamp)
        % double lat_sp, lon_sp;
        alt_sp = 0.0;

        if (self.ref_timestamp ~= 0)
            % Calculate current position setpoint in global frame
            map_projection_reproject(self.ref_pos, self.pos_sp(1), self.pos_sp(2), lat_sp, lon_sp);
            alt_sp = self.ref_alt - self.pos_sp(3);
        end

        % Update local projection reference */
        map_projection_init(self.ref_pos, self.local_pos.ref_lat, self.local_pos.ref_lon);
        self.ref_alt = self.local_pos.ref_alt;

        if (self.ref_timestamp ~= 0)
            % Reproject position setpoint to new reference
            map_projection_project(self.ref_pos, lat_sp, lon_sp, self.pos_sp.data(1), self.pos_sp.data(2));
            self.pos_sp(3) = -(alt_sp - self.ref_alt);
        end

        self.ref_timestamp = self.local_pos.ref_timestamp;
    end
end

% !
function self = reset_pos_sp( self )
    if (self.reset_pos_sp)
        self.reset_pos_sp = false;

        % We have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
        % continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
        % position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
        self.pos_sp(1) = self.pos(1);
        self.pos_sp(2) = self.pos(2);
    end
end

% !
function self = reset_alt_sp( self )
    if (self.reset_alt_sp)
        self.reset_alt_sp = false;

        % We have logic in the main function which choosed the velocity setpoint such that the attitude setpoint is
        % continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
        % altitude in a special way
        self.pos_sp(3) = self.pos(3);
    end
end

%% limit_pos_sp_offset
function self = limit_pos_sp_offset( self )
    pos_sp_offs = zeros(3,1);

    if (self.control_mode.flag_control_position_enabled)
        pos_sp_offs(1) = (self.pos_sp(1) - self.pos(1)) / self.params.sp_offs_max(1);
        pos_sp_offs(2) = (self.pos_sp(2) - self.pos(2)) / self.params.sp_offs_max(2);
    end

    if (self.control_mode.flag_control_altitude_enabled)
        pos_sp_offs(3) = (self.pos_sp(3) - self.pos(3)) / self.params.sp_offs_max(3);
    end

    pos_sp_offs_norm = norm( pos_sp_offs );

    if ( pos_sp_offs_norm > 1.0 )
        pos_sp_offs = pos_sp_offs / pos_sp_offs_norm;
        self.pos_sp = self.pos + pos_sp_offs .* self.params.sp_offs_max;
    end
end

%% control_manual(dt, self)
function self = control_manual(dt, self)
    % *Entering manual control from non-manual control mode, reset alt/pos setpoints */
    if (self.mode_auto)
        self.mode_auto = false;

    % *Reset alt pos flags if resetting is enabled */
        if (self.do_reset_alt_pos_flag)
            self.reset_pos_sp = true;
            self.reset_alt_sp = true;
        end
    end

    % math::Vector<3> req_vel_sp; % % X,Y in local frame and Z in global (D), in [-1,1] normalized range
    req_vel_sp = zeros(3,1);

    if (self.control_mode.flag_control_altitude_enabled)
    % *set vertical velocity setpoint with throttle stick */
        req_vel_sp(3) = - scale_control(self.manual.z - 0.5, 0.5, self.params.alt_ctl_dz, self.params.alt_ctl_dy); % D
    end

    if (self.control_mode.flag_control_position_enabled)
    % *set horizontal velocity setpoint with roll/pitch stick */
        req_vel_sp(1) = self.manual.x;
        req_vel_sp(2) = self.manual.y;
    end

    if (self.control_mode.flag_control_altitude_enabled)
    % *reset alt setpoint to current altitude if needed */
        self = reset_alt_sp( self );
    end

    if (self.control_mode.flag_control_position_enabled)
    % *reset position setpoint to current position if needed */
        self = reset_pos_sp( self );
    end

    % *limit velocity setpoint */
    req_vel_sp_norm = norm( req_vel_sp ); % !!!

    if (req_vel_sp_norm > 1.0)
        req_vel_sp = req_vel_sp / req_vel_sp_norm;
    end

    % *_req_vel_sp scaled to 0..1, scale it to max speed and rotate around yaw */
    %math::Matrix<3, 3> R_yaw_sp;
    R_yaw_sp = from_euler(0.0, 0.0, self.att_sp.yaw_body);
    req_vel_sp_scaled = R_yaw_sp * req_vel_sp .* ...
            self.params.vel_cruise; % in NED and scaled to actual velocity

    % Assisted velocity mode: user controls velocity, but if    velocity is small enough, position
    % hold is activated for the corresponding axis

    % Horizontal axes */
    if (self.control_mode.flag_control_position_enabled)
        % Check for pos. hold
        if ( abs(req_vel_sp(1)) < self.params.hold_xy_dz && fabsf(req_vel_sp(2)) < self.params.hold_xy_dz)
            if (~self.pos_hold_engaged)

                vel_xy_mag = sqrt( self.vel(1) * self.vel(1) + self.vel(2) * self.vel(2) );

                if ( (self.params.hold_max_xy < FLT_EPSILON) || (vel_xy_mag < self.params.hold_max_xy) )
                    % Reset position setpoint to have smooth transition from velocity control to position control
                    self.pos_hold_engaged = true;
                    self.pos_sp(1) = self.pos(1);
                    self.pos_sp(2) = self.pos(2);
                else
                    self.pos_hold_engaged = false;
                end
            end
        else
            self.pos_hold_engaged = false;
        end

        % Set requested velocity setpoint
        if (~self.pos_hold_engaged)
            self.pos_sp(1) = self.pos(1);
            self.pos_sp(2) = self.pos(2);
            self.run_pos_control = false; % *request velocity setpoint to be used, instead of position setpoint */
            self.vel_sp(1) = req_vel_sp_scaled(1);
            self.vel_sp(2) = req_vel_sp_scaled(2);
        end
    end

    % *vertical axis */
    if (self.control_mode.flag_control_altitude_enabled)
        % *check for pos. hold */
        if ( abs( req_vel_sp(3) ) < self.FLT_EPSILON )
            if (~ self.alt_hold_engaged )
                if ( (self.params.hold_max_z < self.FLT_EPSILON) || (abs( self.vel(3) ) < self.params.hold_max_z) )
                    % *reset position setpoint to have smooth transition from velocity control to position control */
                    self.alt_hold_engaged = true;
                    self.pos_sp(3) = self.pos(3);
                else
                    self.alt_hold_engaged = false;
                end
            end
        else
            self.alt_hold_engaged = false;
            self.pos_sp(3) = self.pos(3);
        end

        % *set requested velocity setpoint */
        if (~self.alt_hold_engaged)
            self.run_alt_control = false; % *request velocity setpoint to be used, instead of altitude setpoint */
            self.vel_sp(3) = req_vel_sp_scaled(3);
        end
    end

    if (self.vehicle_land_detected.landed)
        % *don't run controller when landed */
        self.reset_pos_sp = true;
        self.reset_alt_sp = true;
        self.mode_auto = false;
        self.reset_int_z = true;
        self.reset_int_xy = true;

        self.R_setpoint = eye(3,3);

        self.att_sp.roll_body = 0.0;
        self.att_sp.pitch_body = 0.0;
        self.att_sp.yaw_body = self.yaw;
        self.att_sp.thrust = 0.0;

        self.att_sp.timestamp = self.hrt_absolute_time;

        % *publish attitude setpoint */
        % if (self.att_sp_pub ~= nullptr) {
        %     orb_publish(self.attitude_setpoint_id, self.att_sp_pub, &self.att_sp);

        % } else if (self.attitude_setpoint_id) {
        %     self.att_sp_pub = orb_advertise(self.attitude_setpoint_id, &self.att_sp);
        % }
    else
        self = control_position(dt, self);
    end
end

function self = control_non_manual(dt, self)
    % *select control source */
    if (self.control_mode.flag_control_offboard_enabled)
        % *offboard control */
        self = control_offboard(dt, self);
        self.mode_auto = false;
    else
        self.hold_offboard_xy = false;
        self.hold_offboard_z = false;

        % *AUTO */
        self = control_auto(dt, self);
    end

    % *weather-vane mode for vtol: disable yaw control */
    if (self.pos_sp_triplet.current.disable_mc_yaw_control == true)
        self.att_sp.disable_mc_yaw_control = true;
    else
        % *reset in case of setpoint updates */
        self.att_sp.disable_mc_yaw_control = false;
    end

    % guard against any bad velocity values

    velocity_valid = isfinite(self.pos_sp_triplet.current.vx) && ...
                  isfinite(self.pos_sp_triplet.current.vy) && ...
                  self.pos_sp_triplet.current.velocity_valid;

    % do not go slower than the follow target velocity when position tracking is active (set to valid)
    
    %% Look at this later
    if ( self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET && ...
        velocity_valid && ...
        self.pos_sp_triplet.current.position_valid)

        ft_vel = [self.pos_sp_triplet.current.vx;
                  self.pos_sp_triplet.current.vy;
                  0];

         cos_ratio = (ft_vel * self.vel_sp) / ( norm( ft_vel ) * norm( self.vel_sp.length ) );

        % only override velocity set points when uav is traveling in same direction as target and vector component
        % is greater than calculated position set point velocity component

        if (cos_ratio > 0)
            ft_vel = ft_vel * (cos_ratio);
            % min speed a little faster than target vel
            ft_vel = ft_vel + normalized(ft_vel) * 1.5;
        else
            ft_vel = ft_vel .* 0; %.zero();
        end

        if ( abs(ft_vel(1)) > abs(self.vel_sp(1)) )
            self.vel_sp(1) = ft_vel(1);
        else
            self.vel_sp(1) = self.vel_sp(1);
        end
        
        if ( abs(ft_vel(2)) > abs(self.vel_sp(2)) )
            self.vel_sp(2) = ft_vel(2);
        else
            self.vel_sp(2) = self.vel_sp(2);
        end

        % track target using velocity only

    %% Look at this later
    elseif (self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET && ...
           velocity_valid)

        self.vel_sp(1) = self.pos_sp_triplet.current.vx;
        self.vel_sp(2) = self.pos_sp_triplet.current.vy;
    end

    % *use constant descend rate when landing, ignore altitude setpoint */
    if (self.pos_sp_triplet.current.valid ...
        && self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND)
        self.vel_sp(3) = self.params.land_speed;
        self.run_alt_control = false;
    end

    % *special thrust setpoint generation for takeoff from ground */
    if (self.pos_sp_triplet.current.valid ...
        && self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ...
        && self.control_mode.flag_armed)

        % check if we are not already in air.
        % if yes then we don't need a jumped takeoff anymore
        if (~self.takeoff_jumped && ~self.vehicle_land_detected.landed && (abs(self.takeoff_thrust_sp) < self.FLT_EPSILON))
            self.takeoff_jumped = true;
        end

        if (~self.takeoff_jumped)
            % ramp thrust setpoint up
            if (self.vel(3) > -(self.params.tko_speed / 2.0))
                self.takeoff_thrust_sp = self.takeoff_thrust_sp + 0.5 * dt;
                self.vel_sp = self.vel_sp .* 0;
                self.vel_prev = self.vel_prev .* 0;
            else
                % copter has reached our takeoff speed. split the thrust setpoint up
                % into an integral part and into a P part
                self.thrust_int(3) = self.takeoff_thrust_sp - self.params.vel_p(3) * abs(self.vel(3));
                self.thrust_int(3) = -constrain( self.thrust_int(3), self.params.thr_min, self.params.thr_max );
                self.vel_sp_prev(3) = -self.params.tko_speed;
                self.takeoff_jumped = true;
                self.reset_int_z = false;
            end
        end

        if (self.takeoff_jumped)
            self.vel_sp(3) = -self.params.tko_speed;
        end

    else
        self.takeoff_jumped = false;
        self.takeoff_thrust_sp = 0.0;
    end

    if (self.pos_sp_triplet.current.valid ...
        && self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE)
        % Idle state, don't run controller and set zero thrust
        self.R_setpoint = eye(3,3);


        qd = self.R_setpoint; % matrix::Quatf
        self.att_sp.q_d(0) = qd.data(); % memcpy(&_att_sp.q_d[0], qd.data(), sizeof(_att_sp.q_d));
        self.att_sp.q_d_valid = true;

        self.att_sp.roll_body = 0.0;
        self.att_sp.pitch_body = 0.0;
        self.att_sp.yaw_body = self.yaw;
        self.att_sp.thrust = 0.0;

        self.att_sp.timestamp = self.hrt_absolute_time;

        % *publish attitude setpoint */
%         if (self.att_sp_pub ~= nullptr)
%             orb_publish(self.attitude_setpoint_id, self.att_sp_pub, &self.att_sp);
% 
%         elseif (self.attitude_setpoint_id)
%             self.att_sp_pub = orb_advertise(self.attitude_setpoint_id, &self.att_sp);
%         end

    else
        self = control_position(dt, self);
    end
end

function self = control_offboard(dt, self)
% CONTROL_OFFBOARD
    if (self.pos_sp_triplet.current.valid)

        if (self.control_mode.flag_control_position_enabled && self.pos_sp_triplet.current.position_valid)
            % Control position
            self.pos_sp(1) = self.pos_sp_triplet.current.x;
            self.pos_sp(2) = self.pos_sp_triplet.current.y;
            self.run_pos_control = true;

            self.hold_offboard_xy = false;

        elseif (self.control_mode.flag_control_velocity_enabled && self.pos_sp_triplet.current.velocity_valid)
            % Control velocity

            % Reset position setpoint to current position if needed
            self = reset_pos_sp(self);

            if ( (abs(self.pos_sp_triplet.current.vx) <= self.FLT_EPSILON) && ...
                 (abs(self.pos_sp_triplet.current.vy) <= self.FLT_EPSILON) && ...
                  self.local_pos.xy_valid )

                if (~self.hold_offboard_xy)
                    self.pos_sp(1) = self.pos(1);
                    self.pos_sp(2) = self.pos(2);
                    self.hold_offboard_xy = true;
                end

                self.run_pos_control = true;

            else

                if (self.pos_sp_triplet.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_LOCAL_NED)
                    % Set position setpoint move rate
                    self.vel_sp(1) = self.pos_sp_triplet.current.vx;
                    self.vel_sp(2) = self.pos_sp_triplet.current.vy;

                elseif (self.pos_sp_triplet.current.velocity_frame == position_setpoint_s::VELOCITY_FRAME_BODY_NED)
                    % Transform velocity command from body frame to NED frame
                    self.vel_sp(1) = cos(self.yaw) * self.pos_sp_triplet.current.vx - sin(self.yaw) * self.pos_sp_triplet.current.vy;
					self.vel_sp(2) = sin(self.yaw) * self.pos_sp_triplet.current.vx + cos(self.yaw) * self.pos_sp_triplet.current.vy;
                else
                    warning('Unknown velocity offboard coordinate frame');
                end

                self.run_pos_control = false;

                self.hold_offboard_xy = false;
            end

        end

        if (self.control_mode.flag_control_altitude_enabled && self.pos_sp_triplet.current.alt_valid)
            % Control altitude as it is enabled
            self.pos_sp(3) = self.pos_sp_triplet.current.z;
            self.run_alt_control = true;

            self.hold_offboard_z = false;

        elseif (self.control_mode.flag_control_climb_rate_enabled && self.pos_sp_triplet.current.velocity_valid)

            % Reset alt setpoint to current altitude if needed
            self = reset_alt_sp(self);

            if ( (abs(self.pos_sp_triplet.current.vz) <= self.FLT_EPSILON) && ...
                  self.local_pos.z_valid )

                if (~self.hold_offboard_z)
                    self.pos_sp(3) = self.pos(3);
                    self.hold_offboard_z = true;
                end

                self.run_alt_control = true;
            else
                % Set position setpoint move rate
                self.vel_sp(3) = self.pos_sp_triplet.current.vz;
                self.run_alt_control = false;

                self.hold_offboard_z = false;
            end
        end

        if (self.pos_sp_triplet.current.yaw_valid)
            self.att_sp.yaw_body = self.pos_sp_triplet.current.yaw;
        elseif (self.pos_sp_triplet.current.yawspeed_valid)
            self.att_sp.yaw_body = self.att_sp.yaw_body + self.pos_sp_triplet.current.yawspeed * dt;
        end

    else
        self.hold_offboard_xy = false;
        self.hold_offboard_z = false;
        self = reset_pos_sp(self);
        self = reset_alt_sp(self);
    end
end

%% Limit acceleration
function self = limit_acceleration(dt, self)
    % Limit total horizontal acceleration
    acc_hor = zeros(2, 1);
    acc_hor(1) = (self.vel_sp(1) - self.vel_sp_prev(1)) / dt;
    acc_hor(2) = (self.vel_sp(2) - self.vel_sp_prev(2)) / dt;

    if ( norm(acc_hor) > self.params.acc_hor_max)
        acc_hor = normalize( acc_hor );
        acc_hor = acc_hor * self.params.acc_hor_max;
        vel_sp_hor_prev = [self.vel_sp_prev(1); self.vel_sp_prev(2)];
        vel_sp_hor = acc_hor .* dt + vel_sp_hor_prev;
        self.vel_sp(1) = vel_sp_hor(1);
        self.vel_sp(2) = vel_sp_hor(2);
    end

    % limit vertical acceleration
    acc_v = (self.vel_sp(3) - self.vel_sp_prev(3)) / dt;

    % TODO: vertical acceleration is not just 2 * horizontal acceleration
    if ( abs(acc_v) > (2 * self.params.acc_hor_max) )
        acc_v = acc_v / abs(acc_v);
        self.vel_sp(2) = acc_v * 2 * self.params.acc_hor_max * dt + self.vel_sp_prev(2);
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
        dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

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
    % *reset position setpoint on AUTO mode activation or if we are not in MC mode */
    if (~self.mode_auto || ~self.vehicle_status.is_rotary_wing)
        if (~self.mode_auto)
            self.mode_auto = true;
        end

        self.reset_pos_sp = true;
        self.reset_alt_sp = true;
    end

    % Always check reset state of altitude and position control flags in auto
    reset_pos_sp();
    reset_alt_sp();

    current_setpoint_valid = false;
    previous_setpoint_valid = false;

    prev_sp = zeros(3,1);
    curr_sp = zeros(3,1);

    if (self.pos_sp_triplet.current.valid)

        % Project setpoint to local frame ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CHECK THIS
        [curr_sp(1), curr_sp(2)] = map_projection_project( self.ref_pos, ...
                                        self.pos_sp_triplet.current.lat, ...
                                        self.pos_sp_triplet.current.lon, ...
                                        curr_sp(1), curr_sp(2) );
        curr_sp(3) = -( self.pos_sp_triplet.current.alt - self.ref_alt );

        if (isfinite( curr_sp(1) ) && ...
            isfinite( curr_sp(2) ) && ...
            isfinite( curr_sp(3) ))
            current_setpoint_valid = true;
        end
    end

    if (self.pos_sp_triplet.previous.valid)
        [prev_sp(1), prev_sp(2)] = map_projection_project( self.ref_pos, ...
                                       self.pos_sp_triplet.previous.lat, ...
                                       self.pos_sp_triplet.previous.lon, ...
                                       prev_sp(1), prev_sp.data(2) );
        prev_sp(3) = -(self.pos_sp_triplet.previous.alt - self.ref_alt);

        if (isfinite( prev_sp(1) ) && ...
            isfinite( prev_sp(2) ) && ...
            isfinite( prev_sp(3) ))
            previous_setpoint_valid = true;
        end
    end

    if (current_setpoint_valid && ...
        (self.pos_sp_triplet.current.type ~= position_setpoint_s::SETPOINT_TYPE_IDLE))

        % Scaled space: 1 == position error resulting max allowed speed

        cruising_speed = self.params.vel_cruise;

        if (isfinite(self.pos_sp_triplet.current.cruising_speed) && ...
            (self.pos_sp_triplet.current.cruising_speed > 0.1) )
            cruising_speed(1) = self.pos_sp_triplet.current.cruising_speed;
            cruising_speed(2) = self.pos_sp_triplet.current.cruising_speed;
        end

        scale = self.params.pos_p ./ cruising_speed;

        % Convert current setpoint to scaled space
        curr_sp_s = curr_sp .* scale;

        % By default use current setpoint as is
        pos_sp_s = curr_sp_s;

        if ((self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION  || ...
             self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) && ...
            previous_setpoint_valid) {

            % Follow "previous - current" line

            if ( norm(curr_sp - prev_sp) > self.MIN_DIST) {

                % Find X - cross point of unit sphere and trajectory
                pos_s = self.pos .* scale;
                prev_sp_s = prev_sp .* scale;
                prev_curr_s = curr_sp_s - prev_sp_s;
                curr_pos_s = pos_s - curr_sp_s;
                curr_pos_s_len = norm(curr_pos_s);

                if (curr_pos_s_len < 1.0) {
                    % Copter is closer to waypoint than unit radius */
                    % Check next waypoint and use it to avoid slowing down when passing via waypoint */
                    if (self.pos_sp_triplet.next.valid)
                        next_sp = zeros(3,1);
                        [next_sp(1), net_sp(2)] = map_projection_project(self.ref_pos,
                                                    self.pos_sp_triplet.next.lat, ...
                                                    self.pos_sp_triplet.next.lon, ...
                                                    next_sp(1), next_sp.data(2) );
                        next_sp(3) = -(self.pos_sp_triplet.next.alt - self.ref_alt);

                        if ((next_sp - curr_sp).length() > self.MIN_DIST)
                            next_sp_s = next_sp .* scale;

                            % Calculate angle prev - curr - next
                            curr_next_s = next_sp_s - curr_sp_s;
                            prev_curr_s_norm = normalize(prev_curr_s);

                            % cos(a) * curr_next, a = angle between current
                            % and next trajectory segments
							cos_a_curr_next = prev_curr_s_norm * curr_next_s;

                            % cos(b), b = angle pos - curr_sp - prev_sp
                            cos_b = -curr_pos_s * prev_curr_s_norm / curr_pos_s_len;

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
                        pos_sp_s = pos_s + (pos_sp_s - pos_s).normalized();
                    end
                end
            end
        end

        % Move setpoint not faster than max allowed speed
        pos_sp_old_s = self.pos_sp .* scale;

        % Difference between current and desired position setpoints, 1 = max speed
        d_pos_m = (pos_sp_s - pos_sp_old_s) ./ self.params.pos_p;
        d_pos_m_len = size(d_pos_m);

        if (d_pos_m_len > dt)
            pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt) .* self.params.pos_p;
        end

        % Scale result back to normal space
        self.pos_sp = pos_sp_s ./ scale;

        % Update yaw setpoint if needed

        if (self.pos_sp_triplet.current.yawspeed_valid ...
            && self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET)
            self.att_sp.yaw_body = self.att_sp.yaw_body + ...
                self.pos_sp_triplet.current.yawspeed * dt;
        elseif ( isfinite(self.pos_sp_triplet.current.yaw) )
            self.att_sp.yaw_body = self.pos_sp_triplet.current.yaw;
        end

        % If we're already near the current takeoff setpoint don't reset in
        % case we switch back to posctl. This makes the takeoff finish smoothly.
        if ((self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ...
             || self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) ...
             && (self.pos_sp_triplet.current.acceptance_radius > 0.0) ...
             % need to detect we're close a bit before the navigator switches
             % from takeoff to next waypoint
             && (norm(self.pos - self.pos_sp) < self.pos_sp_triplet.current.acceptance_radius * 1.2) )
             
            self.do_reset_alt_pos_flag = false;

            % *otherwise: in case of interrupted mission don't go to
            % waypoint but stay at current position
        else
            self.do_reset_alt_pos_flag = true;
        end

        % During a mission or in loiter it's safe to retract the landing gear.
        if ((self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION || ...
             self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) && ...
            ~self.vehicle_land_detected.landed )
            self.att_sp.landing_gear = 1.0;

            % During takeoff and landing, we better put it down again.

        elseif (self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF || ...
               self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND)
            self.att_sp.landing_gear = -1.0;
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
    if (self.local_pos.timestamp == 0)
        return;
    end

    % TODO: this logic should be in the estimator, not the controller!
    if (isfinite(self.local_pos.x) && ...
        isfinite(self.local_pos.y) && ...
        isfinite(self.local_pos.z))

        self.pos(1) = self.local_pos.x;
        self.pos(2) = self.local_pos.y;

        if ( (self.params.alt_mode == 1) && (self.local_pos.dist_bottom_valid) )
            self.pos(3) = -self.local_pos.dist_bottom;
        else
            self.pos(3) = self.local_pos.z;
        end
    end

    if (isfinite(self.local_pos.vx) && ...
        isfinite(self.local_pos.vy) && ...
        isfinite(self.local_pos.vz))

        self.vel(1) = self.local_pos.vx;
        self.vel(2) = self.local_pos.vy;

        if (self.params.alt_mode == 1 && self.local_pos.dist_bottom_valid)
            self.vel(3) = -self.local_pos.dist_bottom_rate;
        else
            self.vel(3) = self.local_pos.vz;
        end
    end

    self.vel_err_d(1) = self.vel_x_deriv.update( -self.vel(1) ); % ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CHANGE THIS
    self.vel_err_d(2) = self.vel_y_deriv.update( -self.vel(2) );
    self.vel_err_d(3) = self.vel_z_deriv.update( -self.vel(3) );
end

%% Do control
function self = do_control(dt, self)
% DO_CONTROL
    self.vel_ff = self.vel_ff .* 0;

    % Functions
    % Can disable this and run velocity controllers directly in this cycle
    self.run_pos_control = true;
    self.run_alt_control = true;

    if (self.control_mode.flag_control_manual_enabled)
        % Manual control
        self = control_manual(dt, self);
        self.mode_auto = false;

        self.hold_offboard_xy = false;
        self.hold_offboard_z = false;
    else
        self = control_non_manual(dt, self);
    end
end

%% Control position
function self = control_position(dt, self)
% CONTROL_POSITION
% *run position & altitude controllers, if enabled (otherwise use already computed velocity setpoints) */
    if (self.run_pos_control) {
% *_params.pos_p(0);
% *_params.pos_p(1);
    }

    if (self.run_alt_control) {
% *_params.pos_p(2);
    }

% *make sure velocity setpoint is saturated in xy*/
% *_vel_sp(0) +
% *_vel_sp(1));

    if (vel_norm_xy > self.params.vel_max(0)) {
% *note assumes vel_max(0) == vel_max(1) */
% *_params.vel_max(0) / vel_norm_xy;
% *_params.vel_max(1) / vel_norm_xy;
    }

% *make sure velocity setpoint is saturated in z*/
% *_params.vel_max_up) {
% *_params.vel_max_up;
    }

    if (self.vel_sp(2) >  self.params.vel_max_down) {
        self.vel_sp(2) = self.params.vel_max_down;
    }

    if (!self.control_mode.flag_control_position_enabled) {
        self.reset_pos_sp = true;
    }

    if (!self.control_mode.flag_control_altitude_enabled) {
        self.reset_alt_sp = true;
    }

    if (!self.control_mode.flag_control_velocity_enabled) {
        self.vel_sp_prev(0) = self.vel(0);
        self.vel_sp_prev(1) = self.vel(1);
        self.vel_sp(0) = 0.0;
        self.vel_sp(1) = 0.0;
        control_vel_enabled_prev = false;
    }

    if (!self.control_mode.flag_control_climb_rate_enabled) {
        self.vel_sp(2) = 0.0;
    }

% *TODO: remove this is a pathetic leftover, it's here just to make sure that
% *_takeoff_jumped flags are reset */
    if (self.control_mode.flag_control_manual_enabled || !self.pos_sp_triplet.current.valid
        || self.pos_sp_triplet.current.type ~= position_setpoint_s::SETPOINT_TYPE_TAKEOFF
        || !self.control_mode.flag_armed) {

        self.takeoff_jumped = false;
        self.takeoff_thrust_sp = 0.0;
    }

    limit_acceleration(dt);

    self.vel_sp_prev = self.vel_sp;

    self.global_vel_sp.vx = self.vel_sp(0);
    self.global_vel_sp.vy = self.vel_sp(1);
    self.global_vel_sp.vz = self.vel_sp(2);

% *publish velocity setpoint */
    if (self.global_vel_sp_pub ~= nullptr) {
        orb_publish(ORB_ID(vehicle_global_velocity_setpoint), self.global_vel_sp_pub, &self.global_vel_sp);

    } else {
        self.global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &self.global_vel_sp);
    }

    if (self.control_mode.flag_control_climb_rate_enabled || self.control_mode.flag_control_velocity_enabled ||
        self.control_mode.flag_control_acceleration_enabled) {
% *reset integrals if needed */
        if (self.control_mode.flag_control_climb_rate_enabled) {
            if (self.reset_int_z) {
                self.reset_int_z = false;
                float i = self.params.thr_min;

                if (self.reset_int_z_manual) {
                    i = constrain(self.params.thr_hover, self.params.thr_min, self.params.thr_max);
                }

                self.thrust_int(2) = -i;
            }

        } else {
            self.reset_int_z = true;
        }

        if (self.control_mode.flag_control_velocity_enabled) {
            if (self.reset_int_xy) {
                self.reset_int_xy = false;
                self.thrust_int(0) = 0.0;
                self.thrust_int(1) = 0.0;
            }

        } else {
            self.reset_int_xy = true;
        }

% *velocity error */
        math::Vector<3> vel_err = self.vel_sp - self.vel;

        % check if we have switched from a non-velocity controlled mode into a velocity controlled mode
        % if yes, then correct xy velocity setpoint such that the attitude setpoint is continuous
        if (!control_vel_enabled_prev && self.control_mode.flag_control_velocity_enabled) {

            matrix::Dcmf Rb = matrix::Quatf(self.att_sp.q_d[0], self.att_sp.q_d[1], self.att_sp.q_d[2], self.att_sp.q_d[3]);

            % choose velocity xyz setpoint such that the resulting thrust setpoint has the direction
            % given by the last attitude setpoint
            self.vel_sp(0) = self.vel(0) + (-Rb(0,
% *_params.vel_d(0)) / self.params.vel_p(0);
            self.vel_sp(1) = self.vel(1) + (-Rb(1,
% *_params.vel_d(1)) / self.params.vel_p(1);
            self.vel_sp(2) = self.vel(2) + (-Rb(2,
% *_params.vel_d(2)) / self.params.vel_p(2);
            self.vel_sp_prev = self.vel_sp;
            control_vel_enabled_prev = true;

            % compute updated velocity error
            vel_err = self.vel_sp - self.vel;
        }

% *thrust vector in NED frame */
        math::Vector<3> thrust_sp;

        if (self.control_mode.flag_control_acceleration_enabled && self.pos_sp_triplet.current.acceleration_valid) {
            thrust_sp = math::Vector<3>(self.pos_sp_triplet.current.a_x, self.pos_sp_triplet.current.a_y, self.pos_sp_triplet.current.a_z);

        } else {
            thrust_sp = vel_err.emult(self.params.vel_p) + self.vel_err_d.emult(self.params.vel_d) + self.thrust_int;
        }

        if (self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
            && !self.takeoff_jumped && !self.control_mode.flag_control_manual_enabled) {
            % for jumped takeoffs use special thrust setpoint calculated above
            thrust_sp.zero();
            thrust_sp(2) = -_takeoff_thrust_sp;
        }

        if (!self.control_mode.flag_control_velocity_enabled && !self.control_mode.flag_control_acceleration_enabled) {
            thrust_sp(0) = 0.0;
            thrust_sp(1) = 0.0;
        }

        if (!self.control_mode.flag_control_climb_rate_enabled && !self.control_mode.flag_control_acceleration_enabled) {
            thrust_sp(2) = 0.0;
        }

% *limit thrust vector and check for saturation */
        bool saturation_xy = false;
        bool saturation_z = false;

% *limit min lift */
        float thr_min = self.params.thr_min;

        if (!self.control_mode.flag_control_velocity_enabled && thr_min < 0.0) {
% *don't allow downside thrust direction in manual attitude mode */
            thr_min = 0.0;
        }

        float thrust_abs = thrust_sp.length();
        float tilt_max = self.params.tilt_max_air;
        float thr_max = self.params.thr_max;
% *filter vel_z over 1/8sec */
% *_vel(2);
% *filter vel_z change over 1/8sec */
        float vel_z_change = (self.vel(2) - self.vel_prev(2)) / dt;
% *vel_z_change;

        % We can only run the control if we're already in-air, have a takeoff setpoint,
        % or if we're in offboard control.
        % Otherwise, we should just bail out
        const bool got_takeoff_setpoint = (self.pos_sp_triplet.current.valid &&
                           self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) ||
                          self.control_mode.flag_control_offboard_enabled;

        if (self.vehicle_land_detected.landed && !got_takeoff_setpoint) {
            % Keep throttle low while still on ground.
            thr_max = 0.0;

        } else if (!self.control_mode.flag_control_manual_enabled && self.pos_sp_triplet.current.valid &&
               self.pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

% *adjust limits for landing mode */
% *limit max tilt and min lift when landing */
            tilt_max = self.params.tilt_max_land;

            if (thr_min < 0.0) {
                thr_min = 0.0;
            }

% *descend stabilized, we're landing */
            if (!self.in_landing && !self.lnd_reached_ground
                && (float)fabsf(self.acc_z_lp) < 0.1f
% *_params.land_speed) {
                self.in_landing = true;
            }

% *assume ground, cut thrust */
            if (self.in_landing
                && self.vel_z_lp < 0.1f) {
                thr_max = 0.0;
                self.in_landing = false;
                self.lnd_reached_ground = true;
            }

% *once we assumed to have reached the ground always cut the thrust.
                Only free fall detection below can revoke this
            */
            if (!self.in_landing && self.lnd_reached_ground) {
                thr_max = 0.0;
            }

% *if we suddenly fall, reset landing logic and remove thrust limit */
            if (self.lnd_reached_ground
% *XXX: magic value, assuming free fall above 4m/s2 acceleration */
                && (self.acc_z_lp > 4.0
% *_params.land_speed)) {
                thr_max = self.params.thr_max;
                self.in_landing = true;
                self.lnd_reached_ground = false;
            }

        } else {
            self.in_landing = false;
            self.lnd_reached_ground = false;
        }

% *limit min lift */
        if (-thrust_sp(2) < thr_min) {
            thrust_sp(2) = -thr_min;
% *Don't freeze altitude integral if it wants to throttle up */
            saturation_z = vel_err(2) > 0.0 ? true : saturation_z;
        }

        if (self.control_mode.flag_control_velocity_enabled || self.control_mode.flag_control_acceleration_enabled) {

% *limit max tilt */
            if (thr_min >= 0.0 && tilt_max < M_PI_F / 2 - 0.05f) {
% *absolute horizontal thrust */
                float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

                if (thrust_sp_xy_len > 0.01f) {
% *max horizontal thrust for given vertical thrust*/
% *tanf(tilt_max);

                    if (thrust_sp_xy_len > thrust_xy_max) {
                        float k = thrust_xy_max / thrust_sp_xy_len;
                        thrust_sp(0) *= k;
                        thrust_sp(1) *= k;
% *Don't freeze x,y integrals if they both want to throttle down */
% *_vel_sp(1) < 0.0)) ? saturation_xy : true;
                    }
                }
            }
        }

        if (self.control_mode.flag_control_climb_rate_enabled && !self.control_mode.flag_control_velocity_enabled) {
% *thrust compensation when vertical velocity but not horizontal velocity is controlled */
            float att_comp;

            if (self.R(2, 2) > self.TILT_COS_MAX) {
                att_comp = 1.0 / self.R(2, 2);

            } else if (self.R(2, 2) > 0.0) {
% *_R(2, 2) + 1.0;
                saturation_z = true;

            } else {
                att_comp = 1.0;
                saturation_z = true;
            }

            thrust_sp(2) *= att_comp;
        }

% *limit max thrust */
% *recalculate because it might have changed */

        if (thrust_abs > thr_max) {
            if (thrust_sp(2) < 0.0) {
                if (-thrust_sp(2) > thr_max) {
% *thrust Z component is too large, limit it */
                    thrust_sp(0) = 0.0;
                    thrust_sp(1) = 0.0;
                    thrust_sp(2) = -thr_max;
                    saturation_xy = true;
% *Don't freeze altitude integral if it wants to throttle down */
                    saturation_z = vel_err(2) < 0.0 ? true : saturation_z;

                } else {
% *preserve thrust Z component and lower XY, keeping altitude is more important than position */
% *thrust_sp(2));
                    float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
                    float k = thrust_xy_max / thrust_xy_abs;
                    thrust_sp(0) *= k;
                    thrust_sp(1) *= k;
% *Don't freeze x,y integrals if they both want to throttle down */
% *_vel_sp(1) < 0.0)) ? saturation_xy : true;
                }

            } else {
% *Z component is negative, going down, simply limit thrust vector */
                float k = thr_max / thrust_abs;
                thrust_sp *= k;
                saturation_xy = true;
                saturation_z = true;
            }

            thrust_abs = thr_max;
        }

% *update integrals */
        if (self.control_mode.flag_control_velocity_enabled && !saturation_xy) {
% *dt;
% *dt;
        }

        if (self.control_mode.flag_control_climb_rate_enabled && !saturation_z) {
% *dt;

% *protection against flipping on ground when landing */
            if (self.thrust_int(2) > 0.0) {
                self.thrust_int(2) = 0.0;
            }
        }

% *calculate attitude setpoint from thrust vector */
        if (self.control_mode.flag_control_velocity_enabled || self.control_mode.flag_control_acceleration_enabled) {
% *desired body_z axis = -normalize(thrust_vector) */
            math::Vector<3> body_x;
            math::Vector<3> body_y;
            math::Vector<3> body_z;

            if (thrust_abs > self.SIGMA) {
                body_z = -thrust_sp / thrust_abs;

            } else {
% *no thrust, set Z axis to safe value */
                body_z.zero();
                body_z(2) = 1.0;
            }

% *vector of desired yaw direction in XY plane, rotated by PI/2 */
            math::Vector<3> y_C(-sinf(self.att_sp.yaw_body), cosf(self.att_sp.yaw_body), 0.0);

            if (fabsf(body_z(2)) > self.SIGMA) {
% *desired body_x axis, orthogonal to body_z */
                body_x = y_C % body_z;

% *keep nose to front while inverted upside down */
                if (body_z(2) < 0.0) {
                    body_x = -body_x;
                }

                body_x.normalize();

            } else {
% *desired thrust is in XY plane, set X downside to construct correct matrix,
% *but yaw component will not be used actually */
                body_x.zero();
                body_x(2) = 1.0;
            }

% *desired body_y axis */
            body_y = body_z % body_x;

% *fill rotation matrix */
            for (int i = 0; i < 3; i++) {
                self.R_setpoint(i, 0) = body_x(i);
                self.R_setpoint(i, 1) = body_y(i);
                self.R_setpoint(i, 2) = body_z(i);
            }

% *copy quaternion setpoint to attitude setpoint topic */
            matrix::Quatf q_sp = self.R_setpoint;
            memcpy(&self.att_sp.q_d[0], q_sp.data(), sizeof(self.att_sp.q_d));
            self.att_sp.q_d_valid = true;

% *calculate euler angles, for logging only, must not be used for control */
            matrix::Eulerf euler = self.R_setpoint;
            self.att_sp.roll_body = euler(0);
            self.att_sp.pitch_body = euler(1);
% *yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */

        } else if (!self.control_mode.flag_control_manual_enabled) {
% *autonomous altitude control without position control (failsafe landing),
% *force level attitude, don't change yaw */
            self.R_setpoint = matrix::Eulerf(0.0, 0.0, self.att_sp.yaw_body);

% *copy quaternion setpoint to attitude setpoint topic */
            matrix::Quatf q_sp = self.R_setpoint;
            memcpy(&self.att_sp.q_d[0], q_sp.data(), sizeof(self.att_sp.q_d));
            self.att_sp.q_d_valid = true;

            self.att_sp.roll_body = 0.0;
            self.att_sp.pitch_body = 0.0;
        }

        self.att_sp.thrust = thrust_abs;

% *save thrust setpoint for logging */
% *self.ONE_G;
% *self.ONE_G;
% *self.ONE_G;

        self.att_sp.timestamp = hrt_absolute_time();


    } else {
        self.reset_int_z = true;
    }
}

function self = generate_attitude_setpoint(dt, self)
{
% *reset yaw setpoint to current position if needed */
    if (self.reset_yaw_sp) {
        self.reset_yaw_sp = false;
        self.att_sp.yaw_body = self.yaw;
    }

% *do not move yaw while sitting on the ground */
    else if (!self.vehicle_land_detected.landed &&
         !(!self.control_mode.flag_control_altitude_enabled && self.manual.z < 0.1f)) {

% *we want to know the real constraint, and global overrides manual */
        const float yaw_rate_max = (self.params.man_yaw_max < self.params.global_yaw_max) ? self.params.man_yaw_max :
                       self.params.global_yaw_max;
        const float yaw_offset_max = yaw_rate_max / self.params.mc_att_yaw_p;

% *yaw_rate_max;
% *dt);
        float yaw_offs = self.wrap_pi(yaw_target - self.yaw);

        % If the yaw offset became too big for the system to track stop
        % shifting it, only allow if it would make the offset smaller again.
        if (fabsf(yaw_offs) < yaw_offset_max ||
            (self.att_sp.yaw_sp_move_rate > 0 && yaw_offs < 0) ||
            (self.att_sp.yaw_sp_move_rate < 0 && yaw_offs > 0)) {
            self.att_sp.yaw_body = yaw_target;
        }
    }

% *control throttle directly if no climb rate controller is active */
    if (!self.control_mode.flag_control_climb_rate_enabled) {
        float thr_val = throttle_curve(self.manual.z, self.params.thr_hover);
        self.att_sp.thrust = math::min(thr_val, self.manual_thr_max.get());

% *enforce minimum throttle if not landed */
        if (!self.vehicle_land_detected.landed) {
            self.att_sp.thrust = max(self.att_sp.thrust, self.manual_thr_min.get());
        }
    }

% *control roll and pitch directly if no aiding velocity controller is active */
    if (!self.control_mode.flag_control_velocity_enabled) {
% *_params.man_roll_max;
% *_params.man_pitch_max;

% *only if optimal recovery is not used, modify roll/pitch */
        if (self.params.opt_recover <= 0) {
            % construct attitude setpoint rotation matrix. modify the setpoints for roll
            % and pitch such that they reflect the user's intention even if a yaw error
            % (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
            % from the pure euler angle setpoints will lead to unexpected attitude behaviour from
            % the user's view as the euler angle sequence uses the  yaw setpoint and not the current
            % heading of the vehicle.

            % calculate our current yaw error
            float yaw_error = self.wrap_pi(self.att_sp.yaw_body - self.yaw);

            % compute the vector obtained by rotating a z unit vector by the rotation
            % given by the roll and pitch commands of the user
            math::Vector<3> zB = {0, 0, 1};
            math::Matrix<3, 3> R_sp_roll_pitch;
            R_sp_roll_pitch.from_euler(self.att_sp.roll_body, self.att_sp.pitch_body, 0);
% *zB;


            % transform the vector into a new frame which is rotated around the z axis
            % by the current yaw error. this vector defines the desired tilt when we look
            % into the direction of the desired heading
            math::Matrix<3, 3> R_yaw_correction;
            R_yaw_correction.from_euler(0.0, 0.0, -yaw_error);
% *z_roll_pitch_sp;

% *[0;0;1]
            % R_tilt is computed from_euler; only true if cos(roll) not equal zero
            % -> valid if roll is not +-pi/2;
            self.att_sp.roll_body = -asinf(z_roll_pitch_sp(1));
            self.att_sp.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
        }

% *copy quaternion setpoint to attitude setpoint topic */
        matrix::Quatf q_sp = matrix::Eulerf(self.att_sp.roll_body, self.att_sp.pitch_body, self.att_sp.yaw_body);
        memcpy(&self.att_sp.q_d[0], q_sp.data(), sizeof(self.att_sp.q_d));
        self.att_sp.q_d_valid = true;
    }

    if (self.manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON &&
        !self.vehicle_land_detected.landed) {
        self.att_sp.landing_gear = 1.0;

    } else if (self.manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
        self.att_sp.landing_gear = -1.0;
    }

    self.att_sp.timestamp = hrt_absolute_time();
}

function self = task_main(self)
{

    % /*
% *do subscriptions
%     */
    self.vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    self.vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
    self.ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
    self.att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    self.control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    self.params_sub = orb_subscribe(ORB_ID(parameter_update));
    self.manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    self.arming_sub = orb_subscribe(ORB_ID(actuator_armed));
    self.local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    self.pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
    self.local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
    self.global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));

    parameters_update(true);

% *initialize values of critical structs until first regular update */
    self.arming.armed = false;

% *get an initial update for all sensor and status data */
    poll_subscriptions();

% *We really need to know from the beginning if we're landed or in-air. */
    orb_copy(ORB_ID(vehicle_land_detected), self.vehicle_land_detected_sub, &self.vehicle_land_detected);

    bool was_armed = false;

    hrt_abstime t_prev = 0;

    % Let's be safe and have the landing gear down by default
    self.att_sp.landing_gear = -1.0;


% *wakeup source */
    px4_pollfd_struct_t fds[1];

    fds[0].fd = self.local_pos_sub;
    fds[0].events = POLLIN;

    while (!self.task_should_exit) {
% *wait for up to 20ms for data */
        int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

% *timed out - periodic check for self.task_should_exit */
        if (pret == 0) {
            % Go through the loop anyway to copy manual input at 50 Hz.
        }

% *this is undesirable but not much we can do */
        if (pret < 0) {
            warn("poll error %d, %d", pret, errno);
            continue;
        }

        poll_subscriptions();

        parameters_update(false);

        hrt_abstime t = hrt_absolute_time();
        float dt = t_prev ~= 0 ? (t - t_prev) / 1e6f : 0.0;
        t_prev = t;

        % set dt for control blocks
        setDt(dt);

        if (self.control_mode.flag_armed && !was_armed) {
% *reset setpoints and integrals on arming */
            self.reset_pos_sp = true;
            self.reset_alt_sp = true;
            self.do_reset_alt_pos_flag = true;
            self.vel_sp_prev.zero();
            self.reset_int_z = true;
            self.reset_int_xy = true;
            self.reset_yaw_sp = true;
        }

% *reset yaw and altitude setpoint for VTOL which are in fw mode */
        if (self.vehicle_status.is_vtol && !self.vehicle_status.is_rotary_wing) {
            self.reset_yaw_sp = true;
            self.reset_alt_sp = true;
        }

        //Update previous arming state
        was_armed = self.control_mode.flag_armed;

        update_ref();


        update_velocity_derivative();

        % reset the horizontal and vertical position hold flags for non-manual modes
        % or if position / altitude is not controlled
        if (!self.control_mode.flag_control_position_enabled || !self.control_mode.flag_control_manual_enabled) {
            self.pos_hold_engaged = false;
        }

        if (!self.control_mode.flag_control_altitude_enabled || !self.control_mode.flag_control_manual_enabled) {
            self.alt_hold_engaged = false;
        }

        if (self.control_mode.flag_control_altitude_enabled ||
            self.control_mode.flag_control_position_enabled ||
            self.control_mode.flag_control_climb_rate_enabled ||
            self.control_mode.flag_control_velocity_enabled ||
            self.control_mode.flag_control_acceleration_enabled) {

            do_control(dt);

% *fill local position, velocity and thrust setpoint */
            self.local_pos_sp.timestamp = hrt_absolute_time();
            self.local_pos_sp.x = self.pos_sp(0);
            self.local_pos_sp.y = self.pos_sp(1);
            self.local_pos_sp.z = self.pos_sp(2);
            self.local_pos_sp.yaw = self.att_sp.yaw_body;
            self.local_pos_sp.vx = self.vel_sp(0);
            self.local_pos_sp.vy = self.vel_sp(1);
            self.local_pos_sp.vz = self.vel_sp(2);

% *publish local position setpoint */
            if (self.local_pos_sp_pub ~= nullptr) {
                orb_publish(ORB_ID(vehicle_local_position_setpoint), self.local_pos_sp_pub, &self.local_pos_sp);

            } else {
                self.local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &self.local_pos_sp);
            }

        } else {
% *position controller disabled, reset setpoints */
            self.reset_pos_sp = true;
            self.reset_alt_sp = true;
            self.do_reset_alt_pos_flag = true;
            self.mode_auto = false;
            self.reset_int_z = true;
            self.reset_int_xy = true;
            control_vel_enabled_prev = false;

% *store last velocity in case a mode switch to position control occurs */
            self.vel_sp_prev = self.vel;
        }

% *generate attitude setpoint from manual controls */
        if (self.control_mode.flag_control_manual_enabled && self.control_mode.flag_control_attitude_enabled) {

            generate_attitude_setpoint(dt);

        } else {
            self.reset_yaw_sp = true;
            self.att_sp.yaw_sp_move_rate = 0.0;
        }

% *update previous velocity for velocity controller D part */
        self.vel_prev = self.vel;

% *publish attitude setpoint
% *Do not publish if offboard is enabled but position/velocity/accel control is disabled,
% *in this case the attitude setpoint is published by the mavlink app. Also do not publish
% *if the vehicle is a VTOL and it's just doing a transition (the VTOL attitude control module will generate
% *attitude setpoints for the transition).
         */
        if (!(self.control_mode.flag_control_offboard_enabled &&
              !(self.control_mode.flag_control_position_enabled ||
            self.control_mode.flag_control_velocity_enabled ||
            self.control_mode.flag_control_acceleration_enabled))) {

            if (self.att_sp_pub ~= nullptr) {
                orb_publish(self.attitude_setpoint_id, self.att_sp_pub, &self.att_sp);

            } else if (self.attitude_setpoint_id) {
                self.att_sp_pub = orb_advertise(self.attitude_setpoint_id, &self.att_sp);
            }
        }

% *reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
        self.reset_int_z_manual = self.control_mode.flag_armed && self.control_mode.flag_control_manual_enabled
                      && !self.control_mode.flag_control_climb_rate_enabled;
    }

    mavlink_log_info(&self.mavlink_log_pub, "[mpc] stopped");

    self.control_task = -1;
}

% function [output, self] = start(self)
% {
%     ASSERT(self.control_task == -1);
% 
% % *start the task */
%     self.control_task = px4_task_spawn_cmd("mc_pos_control",
%                        SCHED_DEFAULT,
%                        SCHED_PRIORITY_MAX - 5,
%                        1900,
%                        (px4_main_t)&MulticopterPositionControl::task_main_trampoline,
%                        nullptr);
% 
%     if (self.control_task < 0) {
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
%     if (!strcmp(argv[1], "start")) {
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
%     if (!strcmp(argv[1], "stop")) {
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
%     if (!strcmp(argv[1], "status")) {
%         if (pos_control::g_control) {
%             warnx("running");
%             return 0;
% 
%         } else {
%             warnx("not running");
%             return 1;
%         }
%     }
% 
%     warnx("unrecognized command");
%     return 1;
% }

%% New useful functions
radians
to_dcm
normalized
map_projection_project