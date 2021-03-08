classdef MulticopterPositionControl < matlab.System & matlab.system.mixin.CustomIcon & matlab.system.mixin.SampleTime & matlab.system.mixin.Propagates
%MULTICOPTERPOSITIONCONTROL PX4 Firmware position controller (v1.82)
%   Original description:
%   ---------------------
%   Original publication for the desired attitude generation:
%   Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation
%   and Control for Quadrotors. Int. Conf. on Robotics and Automation,
%   Shanghai, China, May 2011.
%
%   Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
%
%   The controller has two loops: P loop for position error and PID loop
%   for velocity error. Output of velocity controller is thrust vector
%   that splitted to thrust direction (i.e. rotation matrix for
%   multicopter orientation) and thrust module (i.e. multicopter thrust
%   itself). Controller doesn't use Euler angles for work, they generated
%   only for more human-friendly control and logging.
%
%   Author: Anton Babushkin <anton.babushkin@me.com>
%   ---------------------
%
%   Original copyright:
%   ---------------------
%   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
%
%   Redistribution and use in source and binary forms, with or without
%   modification, are permitted provided that the following conditions
%   are met:
%
%   1. Redistributions of source code must retain the above copyright
%      notice, this list of conditions and the following disclaimer.
%   2. Redistributions in binary form must reproduce the above copyright
%      notice, this list of conditions and the following disclaimer in
%      the documentation and/or other materials provided with the
%      distribution.
%   3. Neither the name PX4 nor the names of its contributors may be
%      used to endorse or promote products derived from this software
%      without specific prior written permission.
%
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
%   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
%   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
%   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
%   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
%   OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
%   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
%   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
%   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%   POSSIBILITY OF SUCH DAMAGE.
%   ---------------------
%
%   Based on code from PX4 Firmware (retrieved 2019/01/15, v1.82):
%       https://github.com/PX4/PX4-Autopilot/blob/v1.8.2/src/modules/mc_pos_control/mc_pos_control_main.cpp
%
%   See also MULTICOPTERATTITUDECONTROL.
%
%   Written: 2021/03/05, J.X.J. Bannwarth

    %% Public properties (equivalent to those available on PX4)
    properties (Nontunable)
        global_yaw_max_deg     (1,1) {mustBeGreaterThanOrEqual(global_yaw_max_deg     ,    0), mustBeLessThanOrEqual(global_yaw_max_deg     ,1000)} =  200; % MC_YAWRATE_MAX [0-1000]
        mc_att_yaw_p           (1,1) {mustBeGreaterThanOrEqual(mc_att_yaw_p           ,    0), mustBeLessThanOrEqual(mc_att_yaw_p           ,   5)} =  2.8; % MC_YAW_P [0-5]
        mis_yaw_error          (1,1) {mustBeGreaterThanOrEqual(mis_yaw_error          ,    0), mustBeLessThanOrEqual(mis_yaw_error          ,  90)} =   12; % MIS_YAW_ERR [0-90]
        acceleration_z_max_down(1,1) {mustBeGreaterThanOrEqual(acceleration_z_max_down,    2), mustBeLessThanOrEqual(acceleration_z_max_down,  15)} =   10; % MPC_ACC_DOWN_MAX [2-15]
        acceleration_hor       (1,1) {mustBeGreaterThanOrEqual(acceleration_hor       ,    2), mustBeLessThanOrEqual(acceleration_hor       ,  15)} =    5; % MPC_ACC_HOR [2-15]
        acc_max_estimator_xy   (1,1) {mustBeGreaterThanOrEqual(acc_max_estimator_xy   ,  0.2), mustBeLessThanOrEqual(acc_max_estimator_xy   ,   2)} =  0.5; % MPC_ACC_HOR_ESTM [0.2-2]
        acceleration_hor_max   (1,1) {mustBeGreaterThanOrEqual(acceleration_hor_max   ,    2), mustBeLessThanOrEqual(acceleration_hor_max   ,  15)} =   10; % MPC_ACC_HOR_MAX [2-15]
        acceleration_z_max_up  (1,1) {mustBeGreaterThanOrEqual(acceleration_z_max_up  ,    2), mustBeLessThanOrEqual(acceleration_z_max_up  ,  15)} =   10; % MPC_ACC_UP_MAX [2-15]
        alt_mode               (1,1) {mustBeGreaterThanOrEqual(alt_mode               ,    0), mustBeLessThanOrEqual(alt_mode               ,   1)} =    0; % MPC_ALT_MODE [0-1]
        cruise_speed_90        (1,1) {mustBeGreaterThanOrEqual(cruise_speed_90        ,    1), mustBeLessThanOrEqual(cruise_speed_90        ,  20)} =    3; % MPC_CRUISE_90 [1-20]
        deceleration_hor_slow  (1,1) {mustBeGreaterThanOrEqual(deceleration_hor_slow  ,  0.5), mustBeLessThanOrEqual(deceleration_hor_slow  ,  10)} =    5; % MPC_DEC_HOR_SLOW [0.5-10]
        test_flight_tasks      (1,1) {mustBeGreaterThanOrEqual(test_flight_tasks      ,    0), mustBeLessThanOrEqual(test_flight_tasks      ,   1)} =    0; % MPC_FLT_TSK [0-1]
        hold_dz                (1,1) {mustBeGreaterThanOrEqual(hold_dz                ,    0), mustBeLessThanOrEqual(hold_dz                ,   1)} =  0.1; % MPC_HOLD_DZ [0-1]
        hold_max_xy            (1,1) {mustBeGreaterThanOrEqual(hold_max_xy            ,    0), mustBeLessThanOrEqual(hold_max_xy            ,   3)} =  0.8; % MPC_HOLD_MAX_XY [0-3]
        hold_max_z             (1,1) {mustBeGreaterThanOrEqual(hold_max_z             ,    0), mustBeLessThanOrEqual(hold_max_z             ,   3)} =  0.6; % MPC_HOLD_MAX_Z [0-3]
        jerk_hor_max           (1,1) {mustBeGreaterThanOrEqual(jerk_hor_max           ,    0), mustBeLessThanOrEqual(jerk_hor_max           ,  15)} =    0; % MPC_JERK_MAX [0-15]
        jerk_hor_min           (1,1) {mustBeGreaterThanOrEqual(jerk_hor_min           ,  0.5), mustBeLessThanOrEqual(jerk_hor_min           ,  10)} =    1; % MPC_JERK_MIN [0.5-10]
        slow_land_alt1         (1,1) {mustBeGreaterThanOrEqual(slow_land_alt1         ,    0), mustBeLessThanOrEqual(slow_land_alt1         , 122)} =   10; % MPC_LAND_ALT1 [0-122]
        slow_land_alt2         (1,1) {mustBeGreaterThanOrEqual(slow_land_alt2         ,    0), mustBeLessThanOrEqual(slow_land_alt2         , 122)} =    5; % MPC_LAND_ALT2 [0-122]
        land_speed             (1,1) {mustBeGreaterThanOrEqual(land_speed             ,  0.6)                                                     } =  0.7; % MPC_LAND_SPEED [>0.6]
        manual_thr_max         (1,1) {mustBeGreaterThanOrEqual(manual_thr_max         ,    0), mustBeLessThanOrEqual(manual_thr_max         ,   1)} =    1; % MPC_MANTHR_MAX [0-1]
        manual_thr_min         (1,1) {mustBeGreaterThanOrEqual(manual_thr_min         ,    0), mustBeLessThanOrEqual(manual_thr_min         ,   1)} = 0.08; % MPC_MANTHR_MIN [0-1]
        man_tilt_max_deg       (1,1) {mustBeGreaterThanOrEqual(man_tilt_max_deg       ,    0), mustBeLessThanOrEqual(man_tilt_max_deg       ,  90)} =   35; % MPC_MAN_TILT_MAX [0-90]
        man_yaw_max_deg        (1,1) {mustBeGreaterThanOrEqual(man_yaw_max_deg        ,    0), mustBeLessThanOrEqual(man_yaw_max_deg        , 400)} =  200; % MPC_MAN_Y_MAX [0-400]
        thr_hover              (1,1) {mustBeGreaterThanOrEqual(thr_hover              ,  0.2), mustBeLessThanOrEqual(thr_hover              , 0.8)} =  0.5; % MPC_THR_HOVER [0.2-0.8]
        thr_max                (1,1) {mustBeGreaterThanOrEqual(thr_max                ,    0), mustBeLessThanOrEqual(thr_max                ,   1)} =    1; % MPC_THR_MAX [0-1]
        thr_min                (1,1) {mustBeGreaterThanOrEqual(thr_min                , 0.05), mustBeLessThanOrEqual(thr_min                ,   1)} = 0.12; % MPC_THR_MIN [0.05-1]
        tilt_max_air_deg       (1,1) {mustBeGreaterThanOrEqual(tilt_max_air_deg       ,    0), mustBeLessThanOrEqual(tilt_max_air_deg       ,  90)} =   45; % MPC_TILTMAX_AIR [0-90]
        tilt_max_land_deg      (1,1) {mustBeGreaterThanOrEqual(tilt_max_land_deg      ,    0), mustBeLessThanOrEqual(tilt_max_land_deg      ,  90)} =   12; % MPC_TILTMAX_LND [0-90]
        takeoff_ramp_time      (1,1) {mustBeGreaterThanOrEqual(takeoff_ramp_time      ,  0.1), mustBeLessThanOrEqual(takeoff_ramp_time      ,   1)} =  0.4; % MPC_TKO_RAMP_T [0.1-1]
        tko_speed              (1,1) {mustBeGreaterThanOrEqual(tko_speed              ,    1), mustBeLessThanOrEqual(tko_speed              ,   5)} =  1.5; % MPC_TKO_SPEED [1-5]
        veld_lp                (1,1) {mustBeGreaterThanOrEqual(veld_lp                ,    0), mustBeLessThanOrEqual(veld_lp                ,  10)} =    5; % MPC_VELD_LP [0-10]
        velocity_hor_manual    (1,1) {mustBeGreaterThanOrEqual(velocity_hor_manual    ,    3), mustBeLessThanOrEqual(velocity_hor_manual    ,  20)} =   10; % MPC_VEL_MANUAL [3-20]
        vel_cruise_xy          (1,1) {mustBeGreaterThanOrEqual(vel_cruise_xy          ,    3), mustBeLessThanOrEqual(vel_cruise_xy          ,  20)} =    5; % MPC_XY_CRUISE [3-20]
        xy_vel_man_expo        (1,1) {mustBeGreaterThanOrEqual(xy_vel_man_expo        ,    0), mustBeLessThanOrEqual(xy_vel_man_expo        ,   1)} =    0; % MPC_XY_MAN_EXPO [0-1]
        xy_p                   (1,1) {mustBeGreaterThanOrEqual(xy_p                   ,    0), mustBeLessThanOrEqual(xy_p                   ,   2)} = 0.95; % MPC_XY_P [0-2]
        xy_vel_d               (1,1) {mustBeGreaterThanOrEqual(xy_vel_d               ,0.005), mustBeLessThanOrEqual(xy_vel_d               , 0.1)} = 0.01; % MPC_XY_VEL_D [0.005-0.1]
        xy_vel_i               (1,1) {mustBeGreaterThanOrEqual(xy_vel_i               ,    0), mustBeLessThanOrEqual(xy_vel_i               , 0.1)} = 0.02; % MPC_XY_VEL_I [0-0.1]
        vel_max_xy_param       (1,1) {mustBeGreaterThanOrEqual(vel_max_xy_param       ,    0), mustBeLessThanOrEqual(vel_max_xy_param       ,  20)} =   12; % MPC_XY_VEL_MAX [0-20]
        xy_vel_p               (1,1) {mustBeGreaterThanOrEqual(xy_vel_p               , 0.06), mustBeLessThanOrEqual(xy_vel_p               ,0.15)} = 0.09; % MPC_XY_VEL_P [0.06-0.15]
        z_vel_man_expo         (1,1) {mustBeGreaterThanOrEqual(z_vel_man_expo         ,    0), mustBeLessThanOrEqual(z_vel_man_expo         ,   1)} =    0; % MPC_Z_MAN_EXPO [0-1]
        z_p                    (1,1) {mustBeGreaterThanOrEqual(z_p                    ,    0), mustBeLessThanOrEqual(z_p                    , 1.5)} =    1; % MPC_Z_P [0-1.5]
        z_vel_d                (1,1) {mustBeGreaterThanOrEqual(z_vel_d                ,    0), mustBeLessThanOrEqual(z_vel_d                , 0.1)} =    0; % MPC_Z_VEL_D [0-0.1]
        z_vel_i                (1,1) {mustBeGreaterThanOrEqual(z_vel_i                , 0.01), mustBeLessThanOrEqual(z_vel_i                , 0.1)} = 0.02; % MPC_Z_VEL_I [0.01-0.1]
        vel_max_down           (1,1) {mustBeGreaterThanOrEqual(vel_max_down           ,  0.5), mustBeLessThanOrEqual(vel_max_down           ,   4)} =    1; % MPC_Z_VEL_MAX_DN [0.5-4]
        vel_max_up             (1,1) {mustBeGreaterThanOrEqual(vel_max_up             ,  0.5), mustBeLessThanOrEqual(vel_max_up             ,   8)} =    3; % MPC_Z_VEL_MAX_UP [0.5-8]
        z_vel_p                (1,1) {mustBeGreaterThanOrEqual(z_vel_p                ,  0.1), mustBeLessThanOrEqual(z_vel_p                , 0.4)} =  0.2; % MPC_Z_VEL_P [0.1-0.4]
        nav_rad                (1,1) {mustBeGreaterThanOrEqual(nav_rad                , 0.05), mustBeLessThanOrEqual(nav_rad                , 200)} =   10; % NAV_ACC_RAD [0.05-200]
        rc_flt_cutoff          (1,1) {mustBeGreaterThanOrEqual(rc_flt_cutoff          ,    0)                                                     } =   10; % RC_FLT_CUTOFF [>0]
        rc_flt_smp_rate        (1,1) {mustBeGreaterThanOrEqual(rc_flt_smp_rate        ,    1)                                                     } =   50; % RC_FLT_SMP_RATE [>1]
        loop_update_rate_hz    (1,1) {mustBeGreaterThanOrEqual(loop_update_rate_hz    ,   50), mustBeLessThanOrEqual(loop_update_rate_hz    ,1000)} =   50; % Loop update rate Hz [50-1000]
        simple_mode            (1,1) {mustBeGreaterThanOrEqual(simple_mode            ,    0), mustBeLessThanOrEqual(simple_mode            ,   1), mustBeInteger(simple_mode )} = 1; % Simple mode [0-1]
        start_landed           (1,1) {mustBeGreaterThanOrEqual(start_landed           ,    0), mustBeLessThanOrEqual(start_landed           ,   1), mustBeInteger(start_landed)} = 0; % Start landed [0-1]
        ref_lat_init           (1,1) {mustBeGreaterThanOrEqual(ref_lat_init           ,  -90), mustBeLessThanOrEqual(ref_lat_init           ,  90)} = -36.8509; % Ref. latitude [-90-90]
        ref_lon_init           (1,1) {mustBeGreaterThanOrEqual(ref_lon_init           , -180), mustBeLessThanOrEqual(ref_lon_init           , 180)} = 174.7645; % Ref. longitude [-180-180]
        ref_alt_init           (1,1)                                                                                                                = 19      ; % Ref. altitude [-]
    end

    %% Pre-computed constants
    properties(Access = private)
        % Gains
        pos_p           (3,1) = zeros(3,1); % P gain for position control
        vel_p           (3,1) = zeros(3,1); % P gain for velocity control
        vel_i           (3,1) = zeros(3,1); % I gain for velocity control
        vel_d           (3,1) = zeros(3,1); % D gain for velocity control
        
        % Limits
        tilt_max_air    (1,1) = 0; % maximum tilt angle [rad]
        tilt_max_land   (1,1) = 0; % maximum tilt angle during landing [rad]
        man_tilt_max    (1,1) = 0;
        man_yaw_max     (1,1) = 0;
        global_yaw_max  (1,1) = 0;
        sigma_single_op (1,1) = 0.000001;
        sigma_norm      (1,1) = 0.001;
        flt_epsilon     (1,1) = 1.19209e-07;

        % Sampling rate
        dt              (1,1) = 0; 
        
        % Filter
        lp_filters_d_a1               (1,1) = 0; % Filter constants
        lp_filters_d_a2               (1,1) = 0; % Filter constants
        lp_filters_d_b0               (1,1) = 0; % Filter constants
        lp_filters_d_b1               (1,1) = 0; % Filter constants
        lp_filters_d_b2               (1,1) = 0; % Filter constants
        lp_filters_d_cutoff_freq      (1,1) = 0; % Filter constants
    end
    
    %% Discrete states
    properties(DiscreteState)
        % Manual roll/pitch filter delays
        lp_filters_d_delay_element_1 % Delay 1
        lp_filters_d_delay_element_2 % Delay 2
        
        % Flags
        task_should_exit       % 1 if task should exit
        gear_state_initialized % 1 if the gear state has been initialized
        reset_pos_sp           % 1 if position setpoint needs a reset
        reset_alt_sp           % 1 if altitude setpoint needs a reset
        do_reset_alt_pos_flag  % TODO: check if we need this
        mode_auto              % 1 if in auot mode
        pos_hold_engaged       % 1 if hold positon in xy desired
        alt_hold_engaged       % 1 if hold in z desired
        run_pos_control        % 1 if position controller should be used
        run_alt_control        % 1 if altitude controller should be used
        reset_int_z            % 1 if reset integral in z
        reset_int_xy           % 1 if reset integral in xy
        reset_yaw_sp           % 1 if reset yaw setpoint
        hold_offboard_xy       % TODO : check if we need this extra hold_offboard flag
        hold_offboard_z
        in_smooth_takeoff      % 1 if takeoff ramp is applied
        in_landing             % 1 if landing descent (only used in auto)
        lnd_reached_ground     % 1 if controller assumes the vehicle has reached the ground after landing
        triplet_lat_lon_finite % 1 if triplets current is non-finite
        terrain_follow         % 1 is the position controller is controlling height above ground
        ref_alt_is_global      % 1 when the reference altitude is defined in a global reference frame
        vel_sp_significant     % 1 when the velocity setpoint is over 50% of the obj.vel_max_xy limit

        % Scalars
        ref_alt
        ref_timestamp
        yaw_takeoff                      % Home yaw angle present when vehicle was taking off (euler)
        man_yaw_offset                   % Current yaw offset in manual mode
        vel_max_xy                       % equal to vel_max except in auto mode when close to target
        acc_state_dependent_xy           % acceleration limit applied in manual mode
        acc_state_dependent_z            % acceleration limit applied in manual mode in z
        manual_jerk_limit_xy             % jerk limit in manual mode dependent on stick input
        manual_jerk_limit_z              % jerk limit in manual mode in z
        z_derivative                     % velocity in z that agrees with position rate
        takeoff_vel_limit                % velocity limit value which gets ramped up

        % Counters for reset events
        z_reset_counter
        xy_reset_counter
        heading_reset_counter

        % 3x1 vectors
        thrust_int
        pos
        pos_sp
        vel
        vel_sp
        vel_prev            % Velocity on previous step
        vel_sp_prev
        vel_err_d           % Derivative of current velocity
        curr_pos_sp         % Current setpoint of the triplets
        prev_pos_sp         % Previous setpoint of the triples
        stick_input_xy_prev % For manual controlled mode to detect direction change

        % Matrices
        R_setpoint

        % Structures
        att_sp
        local_pos_sp
        ref_pos
        vel_deriv
        hysteresis

        % Char arrays
        user_intention_xy
        user_intention_z

        % Originally local variables
        was_landed
        t_prev
    end

    methods(Access = protected)
        function setupImpl( obj )
        %SETUPIMPL Perform one-time calculations, such as computing constants
        %   Written: 2021/03/05, J.X.J. Bannwarth
            % Simulation uses a fixed sampling rate, as opposed to the PX4
            % implementation
            obj.dt = 1 / obj.loop_update_rate_hz;
            
            % Update parameters
            parameters_update( obj, 1 );

            % Local variables defined in task_main
            % We really need to know from the beginning if we're landed or
            % in-air
            obj.was_landed = obj.start_landed;
            obj.t_prev = 0;
        end
        
        function resetImpl( obj )
        %RESETIMPL Initialize / reset discrete-state properties
        %   Written: 2021/03/05, J.X.J. Bannwarth

            % Lowpass filters for manual roll and pitch
            obj.lp_filters_d_delay_element_1 = zeros(3,1);
            obj.lp_filters_d_delay_element_2 = zeros(3,1);

            % Variables in task_main
            obj.was_landed = 0;
            obj.t_prev = 0;

            % Flags
            obj.task_should_exit       = 0;
            obj.gear_state_initialized = 0;
            obj.reset_pos_sp           = 1;
            obj.reset_alt_sp           = 1;
            obj.do_reset_alt_pos_flag  = 1;
            obj.mode_auto              = 0;
            obj.pos_hold_engaged       = 0;
            obj.alt_hold_engaged       = 0;
            obj.run_pos_control        = 1;
            obj.run_alt_control        = 1;
            obj.reset_int_z            = 1;
            obj.reset_int_xy           = 1;
            obj.reset_yaw_sp           = 1; 
            obj.hold_offboard_xy       = 0;
            obj.hold_offboard_z        = 0;
            obj.in_smooth_takeoff      = 0;
            obj.in_landing             = 0;
            obj.lnd_reached_ground     = 0;
            obj.triplet_lat_lon_finite = 1;
            obj.terrain_follow         = 0;
            obj.ref_alt_is_global      = 0;
            obj.vel_sp_significant     = 0;

            % Scalars
            obj.ref_alt                         = obj.ref_alt_init;
            obj.ref_timestamp                   = 1;
            obj.yaw_takeoff                     = 0;
            obj.man_yaw_offset                  = 0;
            obj.vel_max_xy                      = 0;
            obj.acc_state_dependent_xy = 0;
            obj.acc_state_dependent_z  = 0;
            obj.manual_jerk_limit_xy            = 1;
            obj.manual_jerk_limit_z             = 1;
            obj.z_derivative                    = 0;
            obj.takeoff_vel_limit               = 0;

            % Counters
            obj.z_reset_counter                 = 1;
            obj.xy_reset_counter                = 1;
            obj.heading_reset_counter           = 1;

            % Vectors
            obj.thrust_int          = zeros(3,1);
            obj.pos                 = zeros(3,1);
            obj.pos_sp              = zeros(3,1);
            obj.vel                 = zeros(3,1);
            obj.vel_sp              = zeros(3,1);
            obj.vel_prev            = zeros(3,1);
            obj.vel_sp_prev         = zeros(3,1);
            obj.vel_err_d           = zeros(3,1);
            obj.curr_pos_sp         = zeros(3,1);
            obj.prev_pos_sp         = zeros(3,1);
            obj.stick_input_xy_prev = zeros(3,1);

            % Matrices
            obj.R_setpoint = zeros(3,3);

            % Structures saved as arrays
            obj.ref_pos      = zeros(6,1);
            obj.att_sp       = zeros(12,1);
            obj.att_sp(5)    = -1; % Landing gear down
            obj.local_pos_sp = zeros(13,1);
            obj.vel_deriv    = [ inf; inf; inf; 0; 0; 0; obj.veld_lp; obj.dt ];
            obj.hysteresis   = [ 0; 1; 0; 0; 100000 ];

            % Char arrays
            obj.user_intention_xy = 1; % Brake
            obj.user_intention_z  = 1; % Brake
        end

        %% Input specifications
        function varargout = getInputNamesImpl( obj )
        %GETINPUTNAMESIMPL Return input port names for System block
        %   Written: 2021/03/05, J.X.J. Bannwarth
            if obj.simple_mode
                varargout = { ...
                    'xiDes'        , ...
                    'xiMeas'   , ...
                    'xiDotMeas', ...
                    'qMeas'    , ...
                    };
            else
                % Not implemented yet
                varargout = { ...
                    'vehicle_status'           , ...
                    'vehicle_land_detected'    , ...
                    'vehicle_attitude'         , ...
                    'vehicle_control_mode'     , ...
                    'manual_control_setpoint'  , ...
                    'vehicle_local_position'   , ...
                    'position_setpoint_triplet', ...
                    'home_position'            , ...
                    };
            end
        end
        
        function num = getNumInputsImpl( obj )
        %GETNUMINPUTSIMPL Return number of inputs
        %   Input subscriptions in PX4 implementation:
        %       - [x] vehicle_status
        %       - [x] vehicle_land_detected
        %       - [s] vehicle_attitude
        %       - [x] vehicle_control_mode
        %       - [ ] parameter_update
        %       - [x] manual_control_setpoint
        %       - [s] vehicle_local_position
        %       - [s] position_setpoint_triplet
        %       - [x] home_position
        %   Legend for simulation:
        %       - s: available in simple mode
        %       - x: available in full mode
        %   Written: 2021/03/05, J.X.J. Bannwarth
            if obj.simple_mode
                num = 4;
            else
                num = 8;
            end
        end

        %% Output specifications
        function varargout = getOutputNamesImpl( obj )
        %GETOUTPUTNAMESIMPL Return input port names for System block
        %   Written: 2021/03/05, J.X.J. Bannwarth
            varargout = { ...
                'qDes'      ;
                'thrustDes' ;
                'yawRateDes';
                'xiDotDes'  ;
                };
        end

        function num = getNumOutputsImpl( obj )
        %GETNUMOUTPUTSIMPL Return number of outputs
        %   Written: 2021/03/05, J.X.J. Bannwarth
            num = 4;
        end

        function varargout = getOutputSizeImpl( obj )
        %GETOUTPUTSIZEIMPL Return output size(s)
        %   Written: 2021/03/05, J.X.J. Bannwarth
            varargout = { ...
                         [4 1];
                         [1 1];
                         [1 1];
                         [3 1];
                         };
        end
        
        function varargout = getOutputDataTypeImpl( obj )
        %GETOUTPUTDATATYPEIMPL Return output data type(s)
        %   Written: 2021/03/05, J.X.J. Bannwarth
            varargout = { ...
                            'double';
                            'double';
                            'double';
                            'double';
                            };
        end
        
        function varargout = isOutputComplexImpl( obj )
        %ISOUTPUTCOMPLEXIMPL Return whether output is complex
        %   Written: 2021/03/05, J.X.J. Bannwarth
            varargout = { ...
                         0;
                         0;
                         0;
                         0;
                         };
        end
        
        function varargout = isOutputFixedSizeImpl( obj )
        %ISOUTPUTFIXEDSIZEIMPL Return whether output is fixed size
        %   Written: 2021/03/05, J.X.J. Bannwarth
            varargout = { ...
                        1;
                        1;
                        1;
                        1;
                        };
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl( ~, name )
        %GETDISCRETESTATESPECIFICATIONIMPL Return specs of des. state
        % Return size, data type, and complexity of discrete-state
        % specified in name
        %   Written: 2021/03/05, J.X.J. Bannwarth
            switch name
                % Manual roll/pitch filter delays
                case 'lp_filters_d_delay_element_1'
                    sz = [3 1];
                    dt = "double";
                    cp = 0;
                case 'lp_filters_d_delay_element_2'
                    sz = [3 1];
                    dt = "double";
                    cp = 0;
                % Flags
                case 'task_should_exit'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'gear_state_initialized'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'reset_pos_sp'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'reset_alt_sp'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'do_reset_alt_pos_flag'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'mode_auto'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'pos_hold_engaged'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'alt_hold_engaged'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'run_pos_control'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'run_alt_control'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'reset_int_z'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'reset_int_xy'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'reset_yaw_sp'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'hold_offboard_xy'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'hold_offboard_z'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'in_smooth_takeoff'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'in_landing'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'lnd_reached_ground'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'triplet_lat_lon_finite'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'terrain_follow'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'ref_alt_is_global'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'vel_sp_significant'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                % Scalars
                case 'ref_alt'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'ref_timestamp'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'yaw_takeoff'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'man_yaw_offset'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'vel_max_xy'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'acc_state_dependent_xy'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'acc_state_dependent_z'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'manual_jerk_limit_xy'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'manual_jerk_limit_z'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'z_derivative'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'takeoff_vel_limit'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                % Counters
                case 'z_reset_counter'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'xy_reset_counter'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'heading_reset_counter'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                % 3x1 vectors
                case 'thrust_int'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                case 'pos'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                case 'pos_sp'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                case 'vel'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                case 'vel_sp'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                case 'vel_prev'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                case 'vel_sp_prev'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                case 'vel_err_d'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                case 'curr_pos_sp'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                case 'prev_pos_sp'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                case 'stick_input_xy_prev'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                % Matrix
                case 'R_setpoint'
                    sz = [3 3];
                    dt = 'double';
                    cp = 0;
                % Structures
                case 'att_sp'
                    sz = [12 1];
                    dt = 'double';
                    cp = 0;
                case 'local_pos_sp'
                    sz = [13 1];
                    dt = 'double';
                    cp = 0;
                case 'ref_pos'
                    sz = [6 1];
                    dt = 'double';
                    cp = 0;
                case 'vel_deriv'
                    sz = [8 1];
                    dt = 'double';
                    cp = 0;
                case 'hysteresis'
                    sz = [5 1];
                    dt = 'double';
                    cp = 0;
                % Char arrays
                case 'user_intention_xy'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 'user_intention_z'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                % Originally local variables
                case 'was_landed'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                case 't_prev'
                    sz = [1 1];
                    dt = 'double';
                    cp = 0;
                otherwise
                    error(['Error: Incorrect State Name: ', name.']);
            end
        end
        
        function icon = getIconImpl( ~ )
        %GETICONIMPL Define icon for System block
        %   Written: 2021/03/05, J.X.J. Bannwarth
            icon = ["mc_pos_control","","v1.82"];
        end

        function sts = getSampleTimeImpl( obj )
        %GETSAMPLETIMEIMPL Return sample time of the controller
        %   Written: 2021/03/05, J.X.J. Bannwarth
            sts = createSampleTime( obj, ...
                'Type', 'Discrete', ...
                'SampleTime', 1/obj.loop_update_rate_hz, ...
                'OffsetTime', 0 ...
                );
        end
    
        %% Main function
        function varargout = stepImpl( obj, varargin )
        %STEPIMPL Main function executed at each time step
        %
        %   See also TASK_MAIN.
        %
        %   Written: 2021/03/05, J.X.J. Bannwarth
        
            % [0] Structure states decoding
            % Necessary because it is not possible to have structures as
            % discrete-state properties
            extra_states = discrete_states_to_structure( obj );

            % [1] Pre-process input - equivalent to poll_subscriptions()
            if obj.simple_mode
                % Assign inputs
                xiDes         = varargin{1};
                xiMeasured    = varargin{2};
                xiDotMeasured = varargin{3};
                qMeasured     = varargin{4};

                % (1.1) vehicle_status
                in.vehicle_status.arming_state   = 2;
                in.vehicle_status.is_rotary_wing = 1;
                in.vehicle_status.is_vtol        = 1;
                in.vehicle_status.nav_state      = 'NAVIGATION_STATE_POSCTL';

                % (1.2) vehicle_land_detected
                in.vehicle_land_detected.alt_max        = 200;
                in.vehicle_land_detected.ground_contact =   0;
                in.vehicle_land_detected.landed         =   0;
                in.vehicle_land_detected.maybe_landed   =   0;

                % (1.3) vehicle_control_mode
                in.control_mode.flag_armed                        = 1;
                in.control_mode.flag_control_acceleration_enabled = 0;
                in.control_mode.flag_control_altitude_enabled     = 1;
                in.control_mode.flag_control_attitude_enabled     = 1;
                in.control_mode.flag_control_auto_enabled         = 0;
                in.control_mode.flag_control_climb_rate_enabled   = 1;
                in.control_mode.flag_control_fixed_hdg_enabled    = 1;
                in.control_mode.flag_control_manual_enabled       = 0;
                in.control_mode.flag_control_offboard_enabled     = 1;
                in.control_mode.flag_control_position_enabled     = 1;
                in.control_mode.flag_control_velocity_enabled     = 1;

                % (1.4) vehicle_attitude
                att.q = qMeasured;
                in.R = QuatToDcm( att.q );
                eul = DcmToEuler( in.R );
                in.yaw = eul(3);

                if in.control_mode.flag_control_manual_enabled
                    if obj.heading_reset_counter ~= obj.att.quat_reset_counter
                        obj.heading_reset_counter = obj.att.quat_reset_counter;
        
                        % We only extract the heading change from the delta quaternion
                        eul_sp = QuatToEuler( att.delta_q_reset );
                        extra_states.att_sp.yaw_body = extra_states.att_sp.yaw_body + eul_sp(3);
                    end
                end

                % (1.5) manual_control_setpoint
                in.manual.gear_switch = 0;
                in.manual.r           = 0;
                in.manual.timestamp   = getCurrentTime( obj );
                in.manual.x           = 0;
                in.manual.y           = 0;
                in.manual.z           = 0.5;

                % (1.6) vehicle_local_position
                in.local_pos.dist_bottom       = -xiMeasured(3);
                in.local_pos.dist_bottom_rate  = -xiDotMeasured(3);
                in.local_pos.dist_bottom_valid = isfinite( xiMeasured(3) );
                in.local_pos.hagl_max          = 999999;
                in.local_pos.hagl_min          = 0;
                in.local_pos.ref_alt           = obj.ref_alt_init;
                in.local_pos.ref_lat           = obj.ref_lat_init;
                in.local_pos.ref_lon           = obj.ref_lon_init;
                in.local_pos.ref_timestamp     = getCurrentTime( obj );
                in.local_pos.timestamp         = getCurrentTime( obj );
                in.local_pos.vx                = xiDotMeasured(1);
                in.local_pos.vxy_max           = 0;
                in.local_pos.vy                = xiDotMeasured(2);
                in.local_pos.vz                = xiDotMeasured(3);
                in.local_pos.x                 = xiMeasured(1);
                in.local_pos.xy_reset_counter  = 0;
                in.local_pos.xy_valid          = sum( isfinite(xiMeasured(1:2)) ) == 2;
                in.local_pos.y                 = xiMeasured(2);
                in.local_pos.z                 = xiMeasured(3);
                in.local_pos.z_deriv           = xiDotMeasured(3);
                in.local_pos.z_global          = 0;
                in.local_pos.z_reset_counter   = 0;
                in.local_pos.z_valid           = isfinite( xiMeasured(3) );

                % Check if a reset event has happened
                % If the vehicle is in manual mode we will shift the setpoints of the
                % states which were reset. In auto mode we do not shift the setpoints
                % since we want the vehicle to track the original state.
                if in.control_mode.flag_control_manual_enabled
                    if obj.z_reset_counter ~= in.local_pos.z_reset_counter
                        obj.pos_sp(3) = in.local_pos.z;
                    end

                    if obj.xy_reset_counter ~= in.local_pos.xy_reset_counter
                        obj.pos_sp(1) = in.local_pos.x;
                        obj.pos_sp(2) = in.local_pos.y;
                    end
                end
            
                % update the reset counters in any case
                obj.z_reset_counter = in.local_pos.z_reset_counter;
                obj.xy_reset_counter = in.local_pos.xy_reset_counter;

                % (1.7) position_setpoint_triplet
                in.pos_sp_triplet.current.a_x                = 0;
                in.pos_sp_triplet.current.a_y                = 0;
                in.pos_sp_triplet.current.a_z                = 0;
                in.pos_sp_triplet.current.acceleration_valid = 1;
                in.pos_sp_triplet.current.acceptance_radius  = 0;
                in.pos_sp_triplet.current.alt                = obj.ref_alt-xiDes(3);
                in.pos_sp_triplet.current.alt_valid          = isfinite(xiDes(3));
                in.pos_sp_triplet.current.cruising_speed     = 0;
                in.pos_sp_triplet.current.lat                = obj.ref_lat_init;
                in.pos_sp_triplet.current.lon                = obj.ref_lon_init;
                in.pos_sp_triplet.current.position_valid     = sum(isfinite(xiDes)) == 3;
                in.pos_sp_triplet.current.type               = 'SETPOINT_TYPE_POSITION';
                in.pos_sp_triplet.current.valid              = 1;
                in.pos_sp_triplet.current.velocity_frame     = 'VELOCITY_FRAME_LOCAL_NED';
                in.pos_sp_triplet.current.velocity_valid     = 1;
                in.pos_sp_triplet.current.vx                 = 0;
                in.pos_sp_triplet.current.vy                 = 0;
                in.pos_sp_triplet.current.vz                 = 0;
                in.pos_sp_triplet.current.x                  = xiDes(1);
                in.pos_sp_triplet.current.y                  = xiDes(2);
                in.pos_sp_triplet.current.yaw                = 0;
                in.pos_sp_triplet.current.yaw_valid          = 1;
                in.pos_sp_triplet.current.yawspeed           = 0;
                in.pos_sp_triplet.current.yawspeed_valid     = 0;
                in.pos_sp_triplet.current.z                  = xiDes(3);

                % We are station keeping so all waypoints are the same
                in.pos_sp_triplet.next     = in.pos_sp_triplet.current;
                in.pos_sp_triplet.previous = in.pos_sp_triplet.current;

                % To be a valid current triplet, altitude has to be finite
                if ~isfinite( in.pos_sp_triplet.current.alt )
                    in.pos_sp_triplet.current.valid = 0;
                end

                % To be a valid previous triplet, lat/lon/alt has to be finite
                if ( ~isfinite( in.pos_sp_triplet.previous.lat ) || ...
                    ~isfinite( in.pos_sp_triplet.previous.lon ) || ...
                    ~isfinite( in.pos_sp_triplet.previous.alt ) )
                    in.pos_sp_triplet.previous.valid = 0;
                end
                
                % (1.8) home_position
                in.home_pos.z = 0;
            else
                error( 'Full mode not implemented yet' )
            end

            % [2] Execute main code
            extra_states = task_main( obj, in, extra_states );

            % [3] Post-process output
            varargout{1} = extra_states.att_sp.q_d;
            varargout{2} = extra_states.att_sp.thrust;
            varargout{3} = extra_states.att_sp.yaw_sp_move_rate;
            varargout{4} = [ extra_states.local_pos_sp.vx;
                             extra_states.local_pos_sp.vy;
                             extra_states.local_pos_sp.vz ];
            
            % [4] Re-encode structures
            structure_to_discrete_states( obj, extra_states );
        end

        %% mc_pos_control_main.cpp - Main functions
        function extra_states = task_main( obj, in, extra_states )
        %TASK_MAIN Main task, run at every time step
        %   In the original code, this calls the inital set-up functions and
        %   then run in an infinite while loop. Here, we moved all the
        %   initialisation code to other functions and only left the contents of
        %   the while loop.
        %
        %   See also SETUPIMPL, STEPIMPL.
        %
        %   Written:  2021/03/05, J.X.J. Bannwarth
        
            % Skip update of parameters and topics, as we assume the former
            % don't change and the latter are handled in stepImpl
        
            % Get time
            t = getCurrentTime( obj );
            obj.t_prev = t;
        
            % Update velocity limits. Use the minimum of the estimator demanded and
            % vehicle limits, and allow a minimum of 0.3 m/s for repositioning
            if isfinite( in.local_pos.vxy_max )
                obj.vel_max_xy = max( min(obj.vel_max_xy_param, in.local_pos.vxy_max), 0.3 );
            else
                % If the estimator stopped demanding a limit, release the limit gradually
                if obj.vel_sp_significant
                    if obj.vel_max_xy < obj.vel_max_xy_param
                        obj.vel_max_xy = obj.vel_max_xy + obj.dt*obj.acc_max_estimator_xy;
                    else
                        obj.vel_max_xy = obj.vel_max_xy_param;
                    end
                end
            end
        
            if in.vehicle_land_detected.landed
                % Reset flags when landed
                obj.reset_pos_sp = 1;
                obj.reset_alt_sp = 1;
                obj.do_reset_alt_pos_flag = 1;
                obj.mode_auto = 0;
                obj.pos_hold_engaged = 0;
                obj.alt_hold_engaged = 0;
                obj.run_pos_control = 1;
                obj.run_alt_control = 1;
                obj.reset_int_z = 1;
                obj.reset_int_xy = 1;
                obj.reset_yaw_sp = 1;
                obj.hold_offboard_xy = 0;
                obj.hold_offboard_z = 0;
                obj.in_landing = 0;
                obj.lnd_reached_ground = 0;
        
                % Also reset previous setpoints
                obj.yaw_takeoff = obj.yaw;
                obj.vel_sp_prev = zeros(3,1);
                obj.vel_prev    = zeros(3,1);
        
                % Make sure attitude setpoint output "disables" attitude control
                % TODO: we need a defined setpoint to do this properly especially
                %       when adjusting the mixer
                extra_states.att_sp.thrust = 0;
                extra_states.att_sp.timestamp = getCurrentTime( obj );
        
                % Reset velocity limit
                obj.vel_max_xy = obj.vel_max_xy_param;
            end
        
            % Reset setpoints and integrators VTOL in FW mode
            if ( in.vehicle_status.is_vtol && ~in.vehicle_status.is_rotary_wing )
                obj.reset_alt_sp = 1;
                obj.reset_int_xy = 1;
                obj.reset_int_z  = 1;
                obj.reset_pos_sp = 1;
                obj.reset_yaw_sp = 1;
                obj.vel_sp_prev  = obj.vel;
            end
        
            if ( ~obj.in_smooth_takeoff && in.vehicle_land_detected.landed && ...
                    in.control_mode.flag_armed && ...
                    ( in_auto_takeoff(obj, in) || manual_wants_takeoff(obj, in)) )
                obj.in_smooth_takeoff = 1;
                % This ramp starts negative and goes to positive later because we
                % want to be as smooth as possible. If we start at 0, we alrady jump
                % to hover throttle
                obj.takeoff_vel_limit = -0.5;
            elseif ~in.control_mode.flag_armed
                % If we're disarmed and for some reason were in a smooth takeoff, we
                % reset that
                obj.in_smooth_takeoff = 0;
            end
        
            % Set triplets to invalid if we just landed
            if (in.vehicle_land_detected.landed && ~obj.was_landed)
                in.pos_sp_triplet.current.valid = 0;
            end
        
            obj.was_landed = in.vehicle_land_detected.landed;
        
            extra_states = update_ref( obj, in, extra_states );
        
            extra_states = update_velocity_derivative( obj, in, extra_states );
        
            % Reset the horizontal and vertical position hold flags for non-manual
            % modes or if position / altitude is not controlled
            if ( ~in.control_mode.flag_control_position_enabled || ...
                    ~in.control_mode.flag_control_manual_enabled )
                obj.pos_hold_engaged = 0;
            end
        
            if ( ~in.control_mode.flag_control_altitude_enabled || ...
                    ~in.control_mode.flag_control_manual_enabled )
                obj.alt_hold_engaged = 0;
            end
        
            % Note: Flight tasks are not activated by default, and therefore are not
            %       supported in this Simulink implementation. This code is not working
            %       but left here in case it is needed in the future.
            if obj.test_flight_tasks
                error( 'Flight tasks not implemented' )
                switch in.vehicle_status.nav_state
                    case 'NAVIGATION_STATE_ALTCTL'
                        obj.flight_tasks.switchTask( FlightTaskIndex_Altitude );
                    case 'NAVIGATION_STATE_POSCTL'
                        obj.flight_tasks.switchTask( FlightTaskIndex_Position );
                    case 'NAVIGATION_STATE_MANUAL'
                        obj.flight_tasks.switchTask( FlightTaskIndex_Stabilized );
                    otherwise
                        % Not supported yet
                        obj.flight_tasks.switchTask( FlightTaskIndex_None );
                end
            else
                % Make sure to disable any task when we are not testing them
%                 obj.flight_tasks.switchTask( FlightTaskIndex_None );
            end
        
            if ( obj.test_flight_tasks && obj.flight_tasks.isAnyTaskActive() )
        
                obj.flight_tasks.update();
        
                % Get Flighttask setpoints
                setpoint = obj.flight_tasks.getPositionSetpoint();
        
                % Get obj.constraints depending on flight mode
                % This logic will be set by FlightTasks
                constraints = updateConstraints( obj, constraints );
        
                % For takeoff we adjust the velocity setpoint in the z-direction
                if obj.in_smooth_takeoff
                    % Adjust velocity setpoint in z if we are in smooth takeoff
                    set_takeoff_velocity( obj, setpoint.vz);
                end
        
                % This logic is only temporary
                % Mode switch related things will be handled within Flighttask
                % activate method
                if strcmp( in.vehicle_status.nav_state, 'NAVIGATION_STATE_MANUAL' )
                    % We set triplets to 0
                    % This ensures that when switching to auto, the position
                    % controller will not use the old triplets but waits until
                    % triplets have been updated
                    obj.mode_auto = 0;
                    in.pos_sp_triplet.current.valid = 0;
                    in.pos_sp_triplet.previous.valid = 0;
                    obj.hold_offboard_xy = 0;
                    obj.hold_offboard_z = 0;
                end
        
                % We can only run the control if we're already in-air, have a
                % takeoff setpoint, or if we're in offboard control.
                % Otherwise, we should just bail out
                if ( in.vehicle_land_detected.landed && ~in_auto_takeoff(obj, in) && ...
                        ~manual_wants_takeoff(obj, in) )
                    % Keep throttle low while still on ground.
                    extra_states = set_idle_state( obj, in, extra_states );
                elseif ( strcmp( in.vehicle_status.nav_state, 'NAVIGATION_STATE_MANUAL' ) || ...
                            strcmp( in.vehicle_status.nav_state, 'NAVIGATION_STATE_POSCTL' ) || ...
                            strcmp( in.vehicle_status.nav_state, 'NAVIGATION_STATE_ALTCTL' ) )
                    obj.control.updateState( in.local_pos, obj.vel_err_d );
                    obj.control.updateSetpoint( setpoint );
                    obj.control.updateConstraints( constraints );
                    obj.control.generateThrustYawSetpoint( obj.dt );
        
                    % fill local position, velocity and thrust setpoint
                    extra_states.local_pos_sp.timestamp = getCurrentTime( obj );
                    extra_states.local_pos_sp.x = obj.control.getPosSp(1);
                    extra_states.local_pos_sp.y = obj.control.getPosSp(2);
                    extra_states.local_pos_sp.z = obj.control.getPosSp(3);
                    extra_states.local_pos_sp.yaw = obj.control.getYawSetpoint();
                    extra_states.local_pos_sp.yawspeed = obj.control.getYawspeedSetpoint();
                    extra_states.local_pos_sp.vx = obj.control.getVelSp(1);
                    extra_states.local_pos_sp.vy = obj.control.getVelSp(2);
                    extra_states.local_pos_sp.vz = obj.control.getVelSp(3);
                    extra_states.local_pos_sp.thrust = obj.control.getThrustSetpoint();
        
                    % We adjust thrust setpoint based on landdetector
                    thr_sp = obj.control.getThrustSetpoint();
        
                        % TODO: only do that if not in pure manual
                    landdetection_thrust_limit( obj, thr_sp );
        
                    extra_states.att_sp = thrustToAttitude( thr_sp, ...
                        obj.control.getYawSetpoint() );
                    extra_states.att_sp.yaw_sp_move_rate = obj.control.getYawspeedSetpoint();
                end
        
                % In the original code, attitude and local_pos_sp are published
                % here
                extra_states.att_sp.timestamp = getCurrentTime( obj );
            else
                % Note: this is the only path that is implemented
                if ( in.control_mode.flag_control_altitude_enabled || ...
                        in.control_mode.flag_control_position_enabled || ...
                        in.control_mode.flag_control_climb_rate_enabled || ...
                        in.control_mode.flag_control_velocity_enabled || ...
                        in.control_mode.flag_control_acceleration_enabled )
        
                    extra_states = do_control( obj, in, extra_states );
        
                    % fill local position, velocity and thrust setpoint
                    extra_states.local_pos_sp.timestamp = getCurrentTime( obj );
                    extra_states.local_pos_sp.x = obj.pos_sp(1);
                    extra_states.local_pos_sp.y = obj.pos_sp(2);
                    extra_states.local_pos_sp.z = obj.pos_sp(3);
                    extra_states.local_pos_sp.yaw = extra_states.att_sp.yaw_body;
                    extra_states.local_pos_sp.vx = obj.vel_sp(1);
                    extra_states.local_pos_sp.vy = obj.vel_sp(2);
                    extra_states.local_pos_sp.vz = obj.vel_sp(3);

                    % In the original code, local position setpoint is published
                    % here
                else
                    % Position controller disabled, reset setpoints
                    obj.reset_pos_sp = 1;
                    obj.reset_alt_sp = 1;
                    obj.do_reset_alt_pos_flag = 1;
                    obj.mode_auto = 0;
                    obj.reset_int_z = 1;
                    obj.reset_int_xy = 1;
        
                    % Store last velocity in case a mode switch to position
                    % control occurs
                    obj.vel_sp_prev = obj.vel;
                end
        
                % Generate attitude setpoint from manual controls
                if ( in.control_mode.flag_control_manual_enabled && ...
                        in.control_mode.flag_control_attitude_enabled )
                    generate_attitude_setpoint( obj, in, extra_states );
                else
                    obj.reset_yaw_sp = 1;
                    extra_states.att_sp.yaw_sp_move_rate = 0;
                end
        
                % Update previous velocity for velocity controller D part
                obj.vel_prev = obj.vel;
        
                % In the original code, attitude is published here
                extra_states.att_sp.timestamp = getCurrentTime( obj );
            end
        end
        
        function out = parameters_update( obj, force )
        %PARAMETERS_UPDATE Update our local parameter cache
        %   We don't want to have too many states in our MATLAB system, so this
        %   function should only be called once in setupImpl. This means this
        %   MATLAB port does not support real time parameter tuning.
        %	Written:  2021/03/05, J.X.J. Bannwarth
            updated = 1;
        
            if (updated || force)
                % Initialize vectors from params and enforce constraints
                obj.pos_p = [ obj.xy_p    ; obj.xy_p    ; obj.z_p     ];
                obj.vel_p = [ obj.xy_vel_p; obj.xy_vel_p; obj.z_vel_p ];
                obj.vel_i = [ obj.xy_vel_i; obj.xy_vel_i; obj.z_vel_i ];
                obj.vel_d = [ obj.xy_vel_d; obj.xy_vel_d; obj.z_vel_d ];
        
                obj.thr_hover = constrain(obj.thr_hover, obj.thr_min, obj.thr_max);
        
                obj.tilt_max_air  = deg2rad( obj.tilt_max_air_deg );
                obj.tilt_max_land = deg2rad( obj.tilt_max_land_deg );
        
                obj.hold_max_xy     = max( 0, obj.hold_max_xy );
                obj.hold_max_z      = max( 0, obj.hold_max_z );
                obj.rc_flt_smp_rate = max(1, obj.rc_flt_smp_rate );
        
                % Make sure the filter is in its stable region -> fc < fs/2
                obj.rc_flt_cutoff = min( obj.rc_flt_cutoff, (obj.rc_flt_smp_rate/2 - 1) );
        
                % Update filters
                LowPassFilter2pVectorSetCutOffFrequency( obj, ...
                                                            obj.rc_flt_smp_rate, ...
                                                            obj.rc_flt_cutoff );
                LowPassFilter2pVectorReset( obj, zeros(3,1) );
        
                % Make sure that vel_cruise_xy is always smaller than vel_max
                obj.vel_cruise_xy = min( obj.vel_cruise_xy, obj.vel_max_xy_param );
        
                % mc attitude control parameters
                obj.slow_land_alt1 = max( obj.slow_land_alt1, obj.slow_land_alt2 );
        
                % manual control scale
                obj.man_tilt_max   = deg2rad( obj.man_tilt_max_deg );
                obj.man_yaw_max    = deg2rad( obj.man_yaw_max_deg );
                obj.global_yaw_max = deg2rad( obj.global_yaw_max_deg );
        
                % takeoff and land velocities should not exceed maximum
                obj.tko_speed  = min( obj.tko_speed, obj.vel_max_up );
                obj.land_speed = min( obj.land_speed, obj.vel_max_down );
        
                % default limit for acceleration and manual jerk
                obj.acc_state_dependent_xy = obj.acceleration_hor_max;
                obj.manual_jerk_limit_xy            = obj.jerk_hor_max;
        
                % acceleration up must be larger than acceleration down
                if obj.acceleration_z_max_up < obj.acceleration_z_max_down
                    obj.acceleration_z_max_up =  obj.acceleration_z_max_down ;
                end
        
                % acceleration horizontal max > deceleration hor
                if obj.acceleration_hor_max < obj.deceleration_hor_slow
                    obj.acceleration_hor_max =  obj.deceleration_hor_slow ;
                end
        
                % For z direction we use fixed jerk for now
                % TODO: check if other jerk value is required
                obj.acc_state_dependent_z = obj.acceleration_z_max_up;
                
                % We only use jerk for braking if jerk_hor_max > jerk_hor_min; otherwise
                % just set jerk very large
                if obj.jerk_hor_max > obj.jerk_hor_min
                    obj.manual_jerk_limit_z = obj.jerk_hor_max;
                else
                    obj.manual_jerk_limit_z = 1000000;
                end
        
            end
        
            out = -1;
        end

        function extra_states = update_ref( obj, in, extra_states )
        %UPDATE_REF Update reference for local position projection
        %   The reference point is only allowed to change when the vehicle is in
        %   standby state which is the normal state when the estimator origin is
        %   set. Changing reference point in flight causes large controller
        %   setpoint changes. Changing reference point in other arming states is
        %   untested and shoud not be performed.
        %
        %   See also MAP_PROJECTION_PROJECT, MAP_PROJECTION_REPROJECT.
        %
        %   Written:  2021/03/05, J.X.J. Bannwarth
        
            if ( (in.local_pos.ref_timestamp ~= obj.ref_timestamp) && ...
                 ( strcmp(in.vehicle_status.arming_state, 'ARMING_STATE_STANDBY') || ...
                 (~obj.ref_alt_is_global && in.local_pos.z_global) ) )
                lat_sp = 0;
                lon_sp = 0;
                alt_sp = 0;
        
                if obj.ref_timestamp ~= 0
                    % calculate current position setpoint in global frame
                    [ ~, lat_sp, lon_sp ] = map_projection_reproject( extra_states.ref_pos, ...
                        obj.pos_sp(1), obj.pos_sp(2) );
        
                    % The altitude setpoint is the reference altitude (Z up)
                    % plus the (Z down) NED setpoint, multiplied out to minus
                    alt_sp = obj.ref_alt - obj.pos_sp(3);
                end
        
                % Update local projection reference including altitude
                [ ~, extra_states.ref_pos ] = map_projection_init_timestamped( extra_states.ref_pos, ...
                    in.local_pos.ref_lat, in.local_pos.ref_lon, in.local_pos.timestamp );
                obj.ref_alt = in.local_pos.ref_alt;
        
                if in.local_pos.z_global
                    obj.ref_alt_is_global = 1;
                end
        
                if obj.ref_timestamp ~= 0
                    % Reproject position setpoint to new reference
                    % This effectively adjusts the position setpoint to keep the
                    % vehicle in its current local position. It would only
                    % change its global position on the next setpoint update.
                    [ ~, obj.pos_sp(1), obj.pos_sp(2) ] = map_projection_project( ...
                        extra_states.ref_pos, lat_sp, lon_sp );
                    obj.pos_sp(3) = -(alt_sp - obj.ref_alt);
                end
        
                obj.ref_timestamp = in.local_pos.ref_timestamp;
            end
        end

        function extra_states = update_velocity_derivative( obj, in, extra_states )
        %UPDATE_VELOCITY_DERIVATIVE Update velocity derivative
        %   Independent of the current flight mode
        %   Written:  2021/03/05, J.X.J. Bannwarth
            if in.local_pos.timestamp == 0
                % No position data
                return
            end
        
            % TODO: this logic should be in the estimator, not the controller
            if ( isfinite(in.local_pos.x) && ...
                 isfinite(in.local_pos.y) && ...
                 isfinite(in.local_pos.z) )
        
                obj.pos(1) = in.local_pos.x;
                obj.pos(2) = in.local_pos.y;
        
                if (obj.alt_mode == 1) && in.local_pos.dist_bottom_valid
                    if ~obj.terrain_follow
                        obj.terrain_follow = 1;
                        obj.reset_alt_sp   = 1;
                        reset_altitude_sp( obj );
                    end
        
                    obj.pos(3) = -in.local_pos.dist_bottom;
                else
                    if obj.terrain_follow
                        obj.terrain_follow = 0;
                        obj.reset_alt_sp = 1;
                        reset_altitude_sp( obj );
                    end
        
                    obj.pos(3) = in.local_pos.z;
                end
            end
        
            if ( isfinite(in.local_pos.vx) && ...
                 isfinite(in.local_pos.vy) && ...
                 isfinite(in.local_pos.vz) )
        
                obj.vel(1) = in.local_pos.vx;
                obj.vel(2) = in.local_pos.vy;
        
                if (obj.terrain_follow)
                    obj.vel(3) = -in.local_pos.dist_bottom_rate;
                else
                    obj.vel(3) = in.local_pos.vz;
                end
        
                if ~obj.run_alt_control
                    % Set velocity to the derivative of position because it has less
                    % bias but blend it in across the landing speed range
                    weighting = min( abs(obj.vel_sp(3)) / obj.land_speed, 1.0 );
                    obj.vel(3) = obj.z_derivative * weighting + ...
                        obj.vel(3) * (1.0 - weighting);
                end
        
            end
        
            if isfinite(in.local_pos.z_deriv)
                obj.z_derivative = in.local_pos.z_deriv;
            end
        
            [ extra_states.vel_deriv, obj.vel_err_d ] = BlockDerivativeUpdate( ...
                extra_states.vel_deriv, -obj.vel );
        end

        function extra_states = do_control( obj, in, extra_states )
        %DO_CONTROL
        %   By default, run position/altitude controller. The control_* functions
        %   can disable this and run velocity controllers directly in this cycle.
        %   Written:  2021/03/05, J.X.J. Bannwarth
            obj.run_pos_control = 1;
            obj.run_alt_control = 1;
        
            if in.control_mode.flag_control_manual_enabled
                % Manual control
                extra_states = control_manual( obj, in, extra_states );
                obj.mode_auto = 0;
        
                % We set triplets to 0
                % This ensures that when switching to auto, the position ontroller
                % will not use the old triplets but waits until triplets have been
                % updated
                in.pos_sp_triplet.current.valid  = 0;
                in.pos_sp_triplet.previous.valid = 0;
                obj.curr_pos_sp = nan(3, 1);
        
                obj.hold_offboard_xy = 0;
                obj.hold_offboard_z = 0;
            else
                % Reset acceleration to default
                obj.acc_state_dependent_xy = obj.acceleration_hor_max;
                obj.acc_state_dependent_z = obj.acceleration_z_max_up;
                extra_states = control_non_manual( obj, in, extra_states );
            end
        end

        function extra_states = set_manual_acceleration_xy( obj, stick_xy, extra_states )
        %SET_MANUAL_ACCELERATION_XY Set manual horizontal acceleration
        %   In manual mode we consider four states with different acceleration handling:
        %   1. user wants to stop
        %   2. user wants to quickly change direction
        %   3. user wants to accelerate
        %   4. user wants to decelerate
        %    Written:  2021/03/05, J.X.J. Bannwarth
        
            % Get normalized stick input vector
            if norm(stick_xy) > 0
                stick_xy_norm = normalize(stick_xy);
            else
                stick_xy_norm = stick_xy;
            end
        
            if norm(obj.stick_input_xy_prev) > 0
                stick_xy_prev_norm = normalize(obj.stick_input_xy_prev);
            else
                stick_xy_prev_norm = obj.stick_input_xy_prev;
            end
        
            % check if stick direction and current velocity are within 60angle
            is_aligned = (stick_xy_norm * stick_xy_prev_norm) > 0.5;
        
            % check if zero input stick
            is_prev_zero = abs(norm(obj.stick_input_xy_prev)) <= obj.flt_epsilon;
            is_current_zero = abs(norm(stick_xy)) <= obj.flt_epsilon;
        
            % check acceleration
            do_acceleration = is_prev_zero || (is_aligned && ...
                ((norm(stick_xy) > norm(obj.stick_input_xy_prev)) || ...
                (abs(norm(stick_xy) - 1) < obj.flt_epsilon)));
        
            do_deceleration = ( is_aligned && ...
                (norm(stick_xy) <= norm(obj.stick_input_xy_prev)) );
        
            do_direction_change = ~is_aligned;
        
            if is_current_zero
                % We want to stop
                intention = 'brake';
            elseif do_acceleration
                % We do manual acceleration
                intention = 'acceleration';
            elseif do_deceleration
                % We do manual deceleration
                intention = 'deceleration';
            elseif do_direction_change
                % We have a direction change
                intention = 'direction_change';
            else
                % Catchall: acceleration
                intention = 'acceleration';
            end
        
            % Update user intention
            % We always want to break starting with slow deceleration
            if ( ~strcmp(extra_states.user_intention_xy, 'brake') && strcmp(intention, 'brake') )
        
                if (obj.jerk_hor_max > obj.jerk_hor_min)
                    obj.manual_jerk_limit_xy = (obj.jerk_hor_max - obj.jerk_hor_min) / ...
                        obj.velocity_hor_manual * sqrt( obj.vel(1) * obj.vel(1) + ...
                        obj.vel(2) * obj.vel(2) ) + obj.jerk_hor_min;
        
                    % We start braking with lowest accleration
                    obj.acc_state_dependent_xy = obj.deceleration_hor_slow;
                else
                    % Set the jerk limit large since we don't know it better*/
                    obj.manual_jerk_limit_xy = 1000000;
        
                    % At brake we use max acceleration
                    obj.acc_state_dependent_xy = obj.acceleration_hor_max;
                end
        
                % Reset slew rate
                obj.vel_sp_prev(1) = obj.vel(1);
                obj.vel_sp_prev(2) = obj.vel(2);
            end
        
            switch (extra_states.user_intention_xy)
                case 'brake'
                    if ~strcmp(intention, 'brake')
                        extra_states.user_intention_xy = 'acceleration';
                        % We initialize with lowest acceleration
                        obj.acc_state_dependent_xy = obj.deceleration_hor_slow;
                    end
                case 'direction_change'
                    % Only exit direction change if brake or aligned
                    vel_xy = [obj.vel(1); obj.vel(2)];
        
                    if norm(vel_xy) > 0
                        vel_xy_norm = normalize(vel_xy);
                    end
        
                    stick_vel_aligned = (vel_xy_norm * stick_xy_norm) > 0;
        
                    % Update manual direction change hysteresis
                    extra_states.hysteresis = hysteresis_set_state_and_update( extra_states.hysteresis, ...
                        ~stick_vel_aligned, getCurrentTime(obj)*1E6 );
        
                    % Exit direction change if one of the condition is met
                    if strcmp(intention, 'brake')
                        extra_states.user_intention_xy = intention;
                    elseif stick_vel_aligned
                        extra_states.user_intention_xy = 'acceleration';
                    elseif extra_states.hysteresis.state
                        % TODO: Find conditions which are always continuous
                        %       Only if stick input is large
                        if norm(stick_xy) > 0.6
                            obj.acc_state_dependent_xy = obj.acceleration_hor_max;
                        end
                    end
                case 'acceleration'
                    extra_states.user_intention_xy = intention;
        
                    if strcmp(extra_states.user_intention_xy, 'direction_change')
                        obj.vel_sp_prev(1) = obj.vel(1);
                        obj.vel_sp_prev(2) = obj.vel(2);
                    end
                case 'deceleration'
                    extra_states.user_intention_xy = intention;
        
                    if strcmp(extra_states.user_intention_xy, 'direction_change')
                        obj.vel_sp_prev(1) = obj.vel(1);
                        obj.vel_sp_prev(2) = obj.vel(2);
                    end
            end
        
            % Apply acceleration based on state
            switch extra_states.user_intention_xy
                case 'brake'
                    % Limit jerk when braking to zero
                    jerk = ( obj.acceleration_hor_max - ...
                                obj.acc_state_dependent_xy ) / obj.dt;
        
                    if (jerk > obj.manual_jerk_limit_xy)
                        obj.acc_state_dependent_xy = obj.manual_jerk_limit_xy * ...
                            obj.dt + obj.acc_state_dependent_xy;
                    else
                        obj.acc_state_dependent_xy = obj.acceleration_hor_max;
                    end
                case 'direction_change'
                    % Limit acceleration linearly on stick input
                    obj.acc_state_dependent_xy = ( obj.acceleration_hor - ...
                        obj.deceleration_hor_slow ) * norm(stick_xy) + ...
                        obj.deceleration_hor_slow;
                case 'acceleration'
                    % Limit acceleration linearly on stick input
                    acc_limit  = (obj.acceleration_hor - obj.deceleration_hor_slow) * ...
                        norm(stick_xy) + obj.deceleration_hor_slow;
        
                    if obj.acc_state_dependent_xy > acc_limit
                        acc_limit = obj.acc_state_dependent_xy;
                    end
        
                    obj.acc_state_dependent_xy = acc_limit;
                case 'deceleration'
                    obj.acc_state_dependent_xy = obj.deceleration_hor_slow;
                otherwise
                    warning( 'User intention not recognized' );
                    obj.acc_state_dependent_xy = obj.acceleration_hor_max;
            end
        
            % Update previous stick input
            stick_input_xy_prev3 = LowPassFilter2pVector3Apply( obj, [ stick_xy; 0 ] );
            obj.stick_input_xy_prev = stick_input_xy_prev3(1:2);
        
            if norm(obj.stick_input_xy_prev) > 1
                obj.stick_input_xy_prev = normalize(obj.stick_input_xy_prev);
            end
        end

        function [ extra_states, max_acceleration ] = set_manual_acceleration_z( obj, stick_z, extra_states )
        %SET_MANUAL_ACCELERATION_Z Set manual vertical acceleration
        %   In manual altitude control apply acceleration limit based on stick input
        %   we consider two states
        %   1.) brake
        %   2.) accelerate
        %   Written:  2021/03/05, J.X.J. Bannwarth
        
            % Check if zero input stick
            is_current_zero = abs(stick_z) <= obj.flt_epsilon;
        
            % default is acceleration
            intention = 'acceleration';
        
            % check zero input stick
            if is_current_zero
                intention = 'brake';
            end
        
            % Get max and min acceleration where min acceleration is just 1/5 of max
            % acceleration
            if stick_z <= 0
                max_acceleration = obj.acceleration_z_max_up;
            else
                max_acceleration = obj.acceleration_z_max_down;
            end
        
            % Update user input
            if ~strcmp(extra_states.user_intention_z, 'brake') && strcmp(intention, 'brake')
        
                % we start with lowest acceleration
                obj.acc_state_dependent_z = obj.acceleration_z_max_down;
        
                % reset slew rate
                obj.vel_sp_prev(3) = obj.vel(3);
                extra_states.user_intention_z = 'brake';
            end
        
            extra_states.user_intention_z = intention;
        
            % Apply acceleration depending on state
            if strcmp(extra_states.user_intention_z, 'brake')
        
                % Limit jerk when braking to zero
                jerk = (obj.acceleration_z_max_up - obj.acc_state_dependent_z) ...
                    / obj.dt;
        
                if (jerk > obj.manual_jerk_limit_z)
                    obj.acc_state_dependent_z = obj.manual_jerk_limit_z * obj.dt + ...
                        obj.acc_state_dependent_z;
                else
                    obj.acc_state_dependent_z = obj.acceleration_z_max_up;
                end
            end
        
            if strcmp(extra_states.user_intention_z, 'acceleration')
                obj.acc_state_dependent_z = ( max_acceleration - ...
                    obj.acceleration_z_max_down ) * abs(stick_z) + ...
                    obj.acceleration_z_max_down;
            end
        end

        function extra_states = control_manual( obj, in, extra_states )
        % CONTROL_MANUAL Set position setpoint using manual control
        %    Written:  2021/03/05, J.X.J. Bannwarth
            
            % Entering manual control from non-manual control mode, reset alt/pos
            % setpoints
            if obj.mode_auto
                obj.mode_auto = 0;
        
                % Reset alt pos flags if resetting is enabled
                if (obj.do_reset_alt_pos_flag)
                    obj.reset_pos_sp = 1;
                    obj.reset_alt_sp = 1;
                end
            end
        
            % Map from stick input to velocity setpoint
            % Velocity setpoint commanded by user stick input
            man_vel_sp = zeros(3,1);
        
            if in.control_mode.flag_control_altitude_enabled
                % Set vertical velocity setpoint with throttle stick, remapping of
                % manual.z [0,1] to up and down command [-1,1]
                man_vel_sp(3) = -expo_deadzone( (obj.manual.z - 0.5) * 2, ...
                                                obj.z_vel_man_expo, obj.hold_dz );
        
                % Reset alt setpoint to current altitude if needed
                reset_altitude_sp( obj );
            end
        
            if in.control_mode.flag_control_position_enabled
                % Set horizontal velocity setpoint with roll/pitch stick
                man_vel_sp(1) = expo_deadzone( obj.manual.x, ...
                    obj.xy_vel_man_expo, obj.hold_dz );
                man_vel_sp(2) = expo_deadzone( obj.manual.y, ...
                    obj.xy_vel_man_expo, obj.hold_dz );
        
                man_vel_hor_length = norm( man_vel_sp(1:2) );
        
                % Saturate such that magnitude is never larger than 1
                if man_vel_hor_length > 1
                    man_vel_sp(1) = man_vel_sp(1) / man_vel_hor_length;
                    man_vel_sp(2) = man_vel_sp(2) / man_vel_hor_length;
                end
        
                % Reset position setpoint to current position if needed
                reset_position_sp( obj );
            end
        
            % prepare yaw to rotate into NED frame
            if in.control_mode.flag_control_fixed_hdg_enabled
                yaw_input_frame = obj.yaw_takeoff;
            else
                yaw_input_frame = extra_states.att_sp.yaw_body;
            end
        
            % Setpoint in NED frame
            man_vel_sp = EulerToDcm( [0; 0; yaw_input_frame] ) * man_vel_sp;
        
            % Adjust acceleration based on stick input
            stick_xy = [ man_vel_sp(1); man_vel_sp(2) ];
            extra_states = set_manual_acceleration_xy( obj, stick_xy, ...
                extra_states );
            stick_z = man_vel_sp(3);
            [ extra_states, max_acc_z ] = set_manual_acceleration_z( obj, stick_z, extra_states );
        
            % Prepare cruise speed (m/s) vector to scale the velocity setpoint
            if obj.velocity_hor_manual < obj.vel_max_xy
                vel_mag = obj.velocity_hor_manual;
            else
                vel_mag = obj.vel_max_xy;
            end
        
            if man_vel_sp(3) > 0
                vel_cruise_scale = [ vel_mag; vel_mag; obj.vel_max_down ];
            else
                vel_cruise_scale = [ vel_mag; vel_mag; obj.vel_max_up ];
            end
        
            % Setpoint scaled to cruise speed
            man_vel_sp = man_vel_sp .* vel_cruise_scale;
        
            % Assisted velocity mode: user controls velocity, but if velocity is small
            % enough, position hold is activated for the corresponding axis
        
            % Want to get/stay in altitude hold if user has z stick in the middle
            % (accounted for deadzone already)
            alt_hold_desired = in.control_mode.flag_control_altitude_enabled && ...
                strcmp(extra_states.user_intention_z, 'brake');
        
            % Want to get/stay in position hold if user has xy stick in the middle
            % (accounted for deadzone already)
            pos_hold_desired = in.control_mode.flag_control_position_enabled && ...
                strcmp(extra_states.user_intention_xy, 'brake');
        
            % Check vertical hold engaged flag
            if (obj.alt_hold_engaged)
                obj.alt_hold_engaged = alt_hold_desired;
        
            else
                % Check if we switch to alt_hold_engaged
                smooth_alt_transition = alt_hold_desired && ...
                    ( (max_acc_z - obj.acc_state_dependent_z) < obj.flt_epsilon ) && ...
                    (obj.hold_max_z < obj.flt_epsilon || abs(obj.vel(3)) < obj.hold_max_z);
        
                % During transition predict setpoint forward
                if smooth_alt_transition
        
                    % Time to travel from current velocity to zero velocity
                    delta_t = abs(obj.vel(3) / max_acc_z);
        
                    % Set desired position setpoint assuming max acceleration
                    obj.pos_sp(3) = obj.pos(3) + obj.vel(3) * delta_t ...
                        + 0.5 * max_acc_z * delta_t *delta_t;
        
                    obj.alt_hold_engaged = 1;
                end
            end
        
            % Check horizontal hold engaged flag
            if obj.pos_hold_engaged
                % Check if contition still 1
                obj.pos_hold_engaged = pos_hold_desired;
        
                % Use max acceleration
                if obj.pos_hold_engaged
                    obj.acc_state_dependent_xy = obj.acceleration_hor_max;
                end
            else
                % Check if we switch to pos_hold_engaged
                vel_xy_mag = sqrt( obj.vel(1) * obj.vel(1) + obj.vel(2) * obj.vel(2) );
                smooth_pos_transition = pos_hold_desired && ...
                    ( abs(obj.acceleration_hor_max - obj.acc_state_dependent_xy) < obj.flt_epsilon ) && ...
                    (obj.hold_max_xy < obj.flt_epsilon || vel_xy_mag < obj.hold_max_xy);
        
                % During transition predict setpoint forward
                if smooth_pos_transition
                    % Time to travel from current velocity to zero velocity
                    delta_t = sqrt(obj.vel(1) * obj.vel(1) + obj.vel(2) * obj.vel(2)) ...
                        / obj.acceleration_hor_max;
        
                    % p pos_sp in xy from max acceleration and current velocity
                    pos = [obj.pos(1); obj.pos(2)];
                    vel = [obj.vel(1); obj.vel(2)];
                    pos_sp = pos + vel * delta_t - ...
                        normalize(vel) * 0.5 * obj.acceleration_hor_max * delta_t * delta_t;
                    obj.pos_sp(1) = pos_sp(1);
                    obj.pos_sp(2) = pos_sp(2);
        
                    obj.pos_hold_engaged = 1;
                end
            end
        
            % Set requested velocity setpoints
            if ~obj.alt_hold_engaged
                obj.pos_sp(3) = obj.pos(3);
                % Request velocity setpoint to be used, instead of altitude setpoint
                obj.run_alt_control = 0;
                obj.vel_sp(3) = man_vel_sp(3);
            end
        
            if ~obj.pos_hold_engaged
                obj.pos_sp(1) = obj.pos(1);
                obj.pos_sp(2) = obj.pos(2);
                % Request velocity setpoint to be used, instead of position setpoint
                obj.run_pos_control = 0;
                obj.vel_sp(1) = man_vel_sp(1);
                obj.vel_sp(2) = man_vel_sp(2);
            end
        
            extra_states = control_position( obj, in, extra_states );
        end

        function extra_states = control_non_manual( obj, in, extra_states )
        %CONTROL_NON_MANUAL Set position setpoint using non-manual control
        %    Written:  2021/03/05, J.X.J. Bannwarth

            % Select control source
            if in.control_mode.flag_control_offboard_enabled
                % Offboard control
                extra_states = control_offboard( obj, in, extra_states );
                obj.mode_auto = 0;
            else
                obj.hold_offboard_xy = 0;
                obj.hold_offboard_z = 0;
        
                % AUTO mode
                extra_states = control_auto( obj, in, extra_states );
            end
        
            % Guard against any bad velocity values
            velocity_valid = isfinite(in.pos_sp_triplet.current.vx) && ...
                                isfinite(in.pos_sp_triplet.current.vy) && ...
                                in.pos_sp_triplet.current.velocity_valid;
        
            % Do not go slower than the follow target velocity when position tracking is
            % active (set to valid)
            if ( strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_FOLLOW_TARGET') && ...
                velocity_valid && in.pos_sp_triplet.current.position_valid )
        
                ft_vel = [ in.pos_sp_triplet.current.vx;
                            in.pos_sp_triplet.current.vy;
                            0 ];
        
                cos_ratio = (ft_vel * obj.vel_sp) / ( norm(ft_vel) * norm(obj.vel_sp) );
        
                % Only override velocity set points when uav is traveling in same direction as target and vector component
                % is greater than calculated position set point velocity component
                if cos_ratio > 0
                    ft_vel = ft_vel * (cos_ratio);
                    % Min speed a little faster than target vel
                    ft_vel = ft_vel + normalize(ft_vel) * 1.5;
                else
                    ft_vel = ft_vel .* 0;
                end
        
                if abs(ft_vel(1)) > abs(obj.vel_sp(1))
                    obj.vel_sp(1) =  ft_vel(1);
                else
                    obj.vel_sp(1) = obj.vel_sp(1);
                end
        
                if abs(ft_vel(2)) > abs(obj.vel_sp(2))
                    obj.vel_sp(2) = ft_vel(2);
                else
                    obj.vel_sp(2) = obj.vel_sp(2);
                end
            elseif ( strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_FOLLOW_TARGET') && ...
                velocity_valid )
                % Track target using velocity only
                obj.vel_sp(1) = in.pos_sp_triplet.current.vx;
                obj.vel_sp(2) = in.pos_sp_triplet.current.vy;
            end
        
            % Use constant descend rate when landing, ignore altitude setpoint
            if ( in.pos_sp_triplet.current.valid && ...
                    strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_LAND') )
                obj.vel_sp(3) = obj.land_speed;
                obj.run_alt_control = 0;
            end
        
            if ( in.pos_sp_triplet.current.valid && ...
                    strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_IDLE' ) )
                % Idle state, don't run controller and set zero thrust
                obj.R_setpoint = eye(3);
        
                qd = DcmToQuat( obj.R_setpoint );
                extra_states.att_sp.q_d = qd;
                extra_states.att_sp.q_d_valid = 1;
        
                extra_states.att_sp.roll_body = 0;
                extra_states.att_sp.pitch_body = 0;
                extra_states.att_sp.yaw_body = obj.yaw;
                extra_states.att_sp.thrust = 0;
        
                extra_states.att_sp.timestamp = getCurrentTime( obj );
        
            else
                extra_states = control_position( obj, in, extra_states );
            end
        end

        function extra_states = control_offboard( obj, in, extra_states )
        %CONTROL_OFFBOARD Set position setpoint using offboard control
        %    Written:  2021/03/05, J.X.J. Bannwarth
            if in.pos_sp_triplet.current.valid
        
                if ( in.control_mode.flag_control_position_enabled && ...
                    in.pos_sp_triplet.current.position_valid )
                    % Control position
                    obj.pos_sp(1) = in.pos_sp_triplet.current.x;
                    obj.pos_sp(2) = in.pos_sp_triplet.current.y;
                    obj.run_pos_control = 1;
        
                    obj.hold_offboard_xy = 0;
                elseif ( in.control_mode.flag_control_velocity_enabled && ...
                            in.pos_sp_triplet.current.velocity_valid )
                    % Control velocity
                    % Reset position setpoint to current position if needed
                    reset_position_sp( obj );
        
                    if ( abs(in.pos_sp_triplet.current.vx) <= obj.flt_epsilon && ...
                            abs(in.pos_sp_triplet.current.vy) <= obj.flt_epsilon && ...
                            in.local_pos.xy_valid )
        
                        if ~obj.hold_offboard_xy
                            obj.pos_sp(1) = obj.pos(1);
                            obj.pos_sp(2) = obj.pos(2);
                            obj.hold_offboard_xy = 1;
                        end
        
                        obj.run_pos_control = 1;
                    else
                        if strcmp( in.pos_sp_triplet.current.velocity_frame, 'VELOCITY_FRAME_LOCAL_NED' )
                            % Set position setpoint move rate
                            obj.vel_sp(1) = in.pos_sp_triplet.current.vx;
                            obj.vel_sp(2) = in.pos_sp_triplet.current.vy;
                        elseif strcmp( in.pos_sp_triplet.current.velocity_frame, 'VELOCITY_FRAME_BODY_NED' )
                            % Transform velocity command from body frame to NED frame
                            obj.vel_sp(1) = cos(obj.yaw) * in.pos_sp_triplet.current.vx ...
                                - sin(obj.yaw) * in.pos_sp_triplet.current.vy;
                            obj.vel_sp(2) = sin(obj.yaw) * in.pos_sp_triplet.current.vx ...
                            + cos(obj.yaw) * in.pos_sp_triplet.current.vy;
                        else
                            warning( 'Unknown velocity offboard coordinate frame' );
                        end
        
                        obj.run_pos_control = 0;
                        obj.hold_offboard_xy = 0;
                    end
                end
        
                if ( in.control_mode.flag_control_altitude_enabled && ...
                        in.pos_sp_triplet.current.alt_valid )
                    % Control altitude as it is enabled
                    obj.pos_sp(3) = in.pos_sp_triplet.current.z;
                    obj.run_alt_control = 1;
        
                    obj.hold_offboard_z = 0;
                elseif ( in.control_mode.flag_control_climb_rate_enabled && ...
                    in.pos_sp_triplet.current.velocity_valid )
        
                    % Reset alt setpoint to current altitude if needed
                    reset_altitude_sp( obj );
        
                    if ( abs(in.pos_sp_triplet.current.vz) <= obj.flt_epsilon && ...
                            in.local_pos.z_valid )
        
                        if (~obj.hold_offboard_z)
                            obj.pos_sp(3) = obj.pos(3);
                            obj.hold_offboard_z = 1;
                        end
        
                        obj.run_alt_control = 1;
                    else
                        % Set position setpoint move rate
                        obj.vel_sp(3) = in.pos_sp_triplet.current.vz;
                        obj.run_alt_control = 0;
                        obj.hold_offboard_z = 0;
                    end
                end
        
                if in.pos_sp_triplet.current.yaw_valid
                    extra_states.att_sp.yaw_body = in.pos_sp_triplet.current.yaw;
                elseif in.pos_sp_triplet.current.yawspeed_valid
                    yaw_target = WrapPi( extra_states.att_sp.yaw_body ...
                        + in.pos_sp_triplet.current.yawspeed * obj.dt );
                    yaw_offs = WrapPi( yaw_target - obj.yaw );
        
                    if obj.man_yaw_max < obj.global_yaw_max
                        yaw_rate_max = obj.man_yaw_max;
                    else
                        yaw_rate_max = obj.global_yaw_max;
                    end
                    yaw_offset_max = yaw_rate_max / obj.mc_att_yaw_p;
        
                    % If the yaw offset became too big for the system to track stop
                    % shifting it, only allow if it would make the offset smaller again.
                    if ( (abs(yaw_offs) < yaw_offset_max) || ...
                        (in.pos_sp_triplet.current.yawspeed > 0 && yaw_offs < 0) || ...
                        (in.pos_sp_triplet.current.yawspeed < 0 && yaw_offs > 0) )
                        extra_states.att_sp.yaw_body = yaw_target;
                    end
                end
        
            else
                obj.hold_offboard_xy = 0;
                obj.hold_offboard_z = 0;
                reset_position_sp( obj );
                reset_altitude_sp( obj );
            end
        end

        function extra_states = control_auto( obj, in, extra_states )
        %CONTROL_AUTO Set position setpoint for AUTO mode
        %   Written: 2021/03/06, J.X.J. Bannwarth
        
            % reset position setpoint on AUTO mode activation or if we are not in MC mode
            if ( ~obj.mode_auto || ~in.vehicle_status.is_rotary_wing )
                if ~obj.mode_auto
                    obj.mode_auto = 1;
        
                    % Set obj.triplet_lat_lon_finite 1 once switch to AUTO
                    % (e.g. LAND)
                    obj.triplet_lat_lon_finite = 1;
                end
        
                obj.reset_pos_sp = 1;
                obj.reset_alt_sp = 1;
            end
        
            % Always check reset state of altitude and position control flags in auto
            reset_position_sp( obj );
            reset_altitude_sp( obj );
        
            current_setpoint_valid = 0;
            previous_setpoint_valid = 0;
            next_setpoint_valid = 0;
            triplet_updated = 0;
        
            prev_sp = zeros(3, 1);
            next_sp = zeros(3, 1);
        
            if in.pos_sp_triplet.current.valid
                curr_pos_sp = obj.curr_pos_sp;
        
                % Only project setpoints if they are finite, else use current position
                if ( isfinite(in.pos_sp_triplet.current.lat) && ...
                        isfinite(in.pos_sp_triplet.current.lon) )
                    % Project setpoint to local frame
                    map_projection_project( extra_states.ref_pos, ...
                                    in.pos_sp_triplet.current.lat, ...
                                    in.pos_sp_triplet.current.lon, ...
                                    curr_pos_sp(1), curr_pos_sp(2) );
        
                    obj.triplet_lat_lon_finite = 1;
        
                else
                    % Use current position if NAN -> e.g. land
                    if (obj.triplet_lat_lon_finite)
                        curr_pos_sp(1) = obj.pos(1);
                        curr_pos_sp(2) = obj.pos(2);
                        obj.triplet_lat_lon_finite = 0;
                    end
                end
        
                % Only project setpoints if they are finite, else use current position
                if isfinite(in.pos_sp_triplet.current.alt)
                    curr_pos_sp(3) = -( in.pos_sp_triplet.current.alt - obj.ref_alt );
                end
        
        
                % Sanity check
                if ( isfinite(obj.curr_pos_sp(1)) && ...
                        isfinite(obj.curr_pos_sp(2)) && ...
                        isfinite(obj.curr_pos_sp(3)) )
                    current_setpoint_valid = 1;
                end
        
                % Check if triplets have been updated
                % Note: we only can look at xy since navigator applies slewrate to z
                if (obj.triplet_lat_lon_finite)
                    diff = norm( [ (obj.curr_pos_sp(1) - curr_pos_sp(1)) ;
                                    (obj.curr_pos_sp(2) - curr_pos_sp(2)) ] );
        
                else
                    diff = abs(obj.curr_pos_sp(3) - curr_pos_sp(3));
                end
        
                if (diff > obj.flt_epsilon || ~isfinite(diff))
                    triplet_updated = 1;
                end
        
                % we need to update obj.curr_pos_sp always since navigator applies slew
                % rate on z
                obj.curr_pos_sp = curr_pos_sp;
            end
        
            if in.pos_sp_triplet.previous.valid
                [ ~, prev_sp(1), prev_sp(2) ] = map_projection_project( extra_states.ref_pos, ...
                    in.pos_sp_triplet.previous.lat, ...
                    in.pos_sp_triplet.previous.lon );
                prev_sp(3) = -(in.pos_sp_triplet.previous.alt - obj.ref_alt);
        
                if ( isfinite(prev_sp(1)) && ...
                        isfinite(prev_sp(2)) && ...
                        isfinite(prev_sp(3)) )
                    obj.prev_pos_sp = prev_sp;
                    previous_setpoint_valid = 1;
                end
            end
        
            % Set previous setpoint to current position if no previous setpoint available
            if (~previous_setpoint_valid && triplet_updated)
                obj.prev_pos_sp = obj.pos;
                % Currently not necessary to set to 1 since not used
                previous_setpoint_valid = 1;
            end
        
            if in.pos_sp_triplet.next.valid
                [ ~, next_sp(1), next_sp(2) ] = map_projection_project( extra_states.ref_pos, ...
                                        in.pos_sp_triplet.next.lat, ...
                                        in.pos_sp_triplet.next.lon );
        
                next_sp(3) = -(in.pos_sp_triplet.next.alt - obj.ref_alt);
        
                if ( isfinite(next_sp(1)) && ...
                        isfinite(next_sp(2)) && ...
                        isfinite(next_sp(3)) )
                    next_setpoint_valid = 1;
                end
            end
        
            % Auto logic:
            % The vehicle should follow the line previous-current.
            % - if there is no next setpoint or the current is a loiter point, then
            %   slowly approach the current along the line
            % - if there is a next setpoint, then the velocity is adjusted depending on
            %   the angle of the corner prev-current-next.
            % When following the line, the pos_sp is computed from the orthogonal
            % distance to the closest point on line and the desired cruise speed along
            % the track.
        
            % Create new obj.pos_sp from triplets
            if (current_setpoint_valid && ...
                ~strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_IDLE') )
        
                % Update yaw setpoint if needed
                if ( in.pos_sp_triplet.current.yawspeed_valid && ...
                    strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_FOLLOW_TARGET') )
                    extra_states.att_sp.yaw_body = extra_states.att_sp.yaw_body + ...
                        in.pos_sp_triplet.current.yawspeed * obj.dt;
        
                elseif isfinite(in.pos_sp_triplet.current.yaw)
                    extra_states.att_sp.yaw_body = in.pos_sp_triplet.current.yaw;
                end
        
                yaw_diff = WrapPi(extra_states.att_sp.yaw_body - obj.yaw);
        
                % Only follow previous-current-line for specific triplet type
                if ( strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_POSITION') || ...
                        strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_LOITER')   || ...
                        strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_FOLLOW_TARGET') )
        
                    % By default use current setpoint as is
                    pos_sp = obj.curr_pos_sp;
        
                    % Z-DIRECTION
                    
                    % Get various distances
                    total_dist_z = abs(obj.curr_pos_sp(3) - obj.prev_pos_sp(3));
                    dist_to_prev_z = abs(obj.pos(3) - obj.prev_pos_sp(3));
                    dist_to_current_z = abs(obj.curr_pos_sp(3) - obj.pos(3));
        
                    % If pos_sp has not reached target setpoint (=curr_pos_sp(3)),
                    % then compute setpoint depending on vel_max
                    if ( (total_dist_z >  obj.sigma_norm) && ...
                            (abs(obj.pos_sp(3) - obj.curr_pos_sp(3)) > obj.sigma_norm) )
        
                        % Check sign
                        flying_upward = obj.curr_pos_sp(3) < obj.pos(3);
        
                        % Final_vel_z is the max velocity which depends on the distance
                        % of total_dist_z with default params.vel_max_up/down
                        if flying_upward
                            final_vel_z = obj.vel_max_up;
                        else
                            final_vel_z = obj.vel_max_down;
                        end
        
                        % Target threshold defines the distance to obj.curr_pos_sp(3) at
                        % which the vehicle starts to slow down to approach the target
                        % smoothly
                        target_threshold_z = final_vel_z * 1.5;
        
                        % If the total distance in z is NOT 2x distance of
                        % target_threshold, we will need to adjust the final_vel_z
                        is_2_target_threshold_z = total_dist_z >= (2*target_threshold_z);
                        
                        % Define the the acceleration when slowing down
                        slope = (final_vel_z) / (target_threshold_z);
        
                        % Minimum velocity: this is needed since estimation is not
                        % perfect
                        min_vel_z = 0.2;
        
                        if ~is_2_target_threshold_z
                            % Adjust final_vel_z since we are already very close to
                            % current and therefore it is not necessary to accelerate up
                            % to full speed (=final_vel_z)
                            target_threshold_z = total_dist_z * 0.5;
        
                            % get the velocity at target_threshold_z
                            final_vel_z_tmp = slope * target_threshold_z + min_vel_z;
        
                            % Make sure that final_vel_z is never smaller than 0.5 of
                            % the default final_vel_z this is mainly done because the
                            % estimation in z is not perfect and therefore it is
                            % necessary to have a minimum speed
                            final_vel_z = constrain( final_vel_z_tmp, ...
                                                        final_vel_z * 0.5, ...
                                                        final_vel_z );
                        end
        
                        vel_sp_z = final_vel_z;
        
                        % We want to slow down
                        if dist_to_current_z < target_threshold_z
                            vel_sp_z = slope * dist_to_current_z + min_vel_z;
                        elseif dist_to_prev_z < target_threshold_z
                            % We want to accelerate
                            acc_z = ( vel_sp_z - abs(obj.vel_sp(3)) ) / obj.dt;
        
                            if flying_upward
                                acc_max = (obj.acceleration_z_max_up * 0.5);
                            else
                                acc_max = (obj.acceleration_z_max_down * 0.5);
                            end
        
                            if acc_z > acc_max
                                vel_sp_z = obj.acceleration_z_max_up * obj.dt ...
                                    + abs(obj.vel_sp(3));
                            end
                        end
        
                        % If we already close to current, then just take over the
                        % velocity that we would have computed if going directly to the
                        % current setpoint
                        if vel_sp_z >= (dist_to_current_z * obj.pos_p(3))
                            vel_sp_z = dist_to_current_z * obj.pos_p(3);
                        end
        
                        % Make sure vel_sp_z is always positive
                        vel_sp_z = constrain( vel_sp_z, 0.0, final_vel_z );
        
                        % Get the sign of vel_sp_z
                        if flying_upward
                            vel_sp_z = -vel_sp_z;
                        end
        
                        % Compute pos_sp(3)
                        pos_sp(3) = obj.pos(3) + vel_sp_z / obj.pos_p(3);
                    end
        
                    % XY-DIRECTION
                    % Line from previous to current and from pos to current
                    vec_prev_to_current = [ obj.curr_pos_sp(1) - obj.prev_pos_sp(1) ;
                                            obj.curr_pos_sp(2) - obj.prev_pos_sp(2) ];
                    vec_pos_to_current =  [ obj.curr_pos_sp(1) - obj.pos(1) ;
                                            obj.curr_pos_sp(2) - obj.pos(2) ];
        
                    % Check if we just want to stay at current position
                    pos_sp_diff = [ obj.curr_pos_sp(1) - obj.pos_sp(1) ;
                                    obj.curr_pos_sp(2) - obj.pos_sp(2) ];
                    
                    stay_at_current_pos = ( strcmp(in.pos_sp_triplet.current.type, ...
                        'SETPOINT_TYPE_LOITER') || ~next_setpoint_valid ) && ...
                        ( norm(pos_sp_diff) < obj.sigma_norm );
        
                    % Only follow line if previous to current has a minimum distance
                    if ( norm(vec_prev_to_current) > obj.nav_rad) && ~stay_at_current_pos
        
                        % Normalize prev-current line (always > nav_rad)
                        unit_prev_to_current = normalize( vec_prev_to_current );
        
                        % Unit vector from current to next
                        unit_current_to_next = [0; 0];
        
                        if (next_setpoint_valid)
                            unit_current_to_next =[ (next_sp(1) - pos_sp(1)) ;
                                                    (next_sp(2) - pos_sp(2)) ];
                            
                            if norm(unit_current_to_next) > obj.sigma_norm
                                unit_current_to_next = normalize( unit_current_to_next );
                            end
                        end
        
                        % Point on line closest to pos
                        closest_point = [obj.prev_pos_sp(1); obj.prev_pos_sp(2)] + ...
                                            dot(unit_prev_to_current, unit_prev_to_current) * ...
                                            [ (obj.pos(1) - obj.prev_pos_sp(1)) ;
                                            (obj.pos(2) - obj.prev_pos_sp(2)) ];
        
                        vec_closest_to_current = [ (obj.curr_pos_sp(1) - closest_point(1)) ;
                                                    (obj.curr_pos_sp(2) - closest_point(2)) ];
        
                        % Compute vector from position-current and previous-position
                        vec_prev_to_pos = [ (obj.pos(1) - obj.prev_pos_sp(1)) ;
                                            (obj.pos(2) - obj.prev_pos_sp(2)) ];
        
                        % Current velocity along track
                        vel_sp_along_track_prev = [obj.vel_sp(1); obj.vel_sp(2)] ...
                            * unit_prev_to_current;
        
                        % Distance to target when brake should occur
                        target_threshold_xy = 1.5 * get_cruising_speed_xy(obj);
        
                        close_to_current = norm(vec_pos_to_current) < target_threshold_xy;
                        close_to_prev = ( norm(vec_prev_to_pos) < target_threshold_xy ) && ...
                                        ( norm(vec_prev_to_pos) < norm(vec_pos_to_current) );
        
                        % Indicates if we are at least half the distance from previous to
                        % current close to previous
                        is_2_target_threshold = norm(vec_prev_to_current) ...
                            >= (2.0 * target_threshold_xy);
        
                        % Check if the current setpoint is behind
                        current_behind = dot(-vec_pos_to_current, unit_prev_to_current) > 0;
        
                        % Check if the previous is in front
                        previous_in_front = dot(vec_prev_to_pos, unit_prev_to_current) < 0;
        
                        % Default velocity along line prev-current
                        vel_sp_along_track = get_cruising_speed_xy( obj );
        
                        % Compute velocity setpoint along track
                        % Only go directly to previous setpoint if more than 5m away and
                        % previous in front
                        if ( previous_in_front && (norm(vec_prev_to_pos) > 5) )
        
                            % Just use the default velocity along track
                            vel_sp_along_track = norm(vec_prev_to_pos) * obj.pos_p(1);
        
                            if (vel_sp_along_track > get_cruising_speed_xy(obj))
                                vel_sp_along_track = get_cruising_speed_xy(obj);
                            end
        
                        elseif current_behind
                            % Go directly to current setpoint
                            vel_sp_along_track = norm(vec_pos_to_current) * obj.pos_p(1);
                            if ~( vel_sp_along_track < get_cruising_speed_xy(obj) )
                                vel_sp_along_track = get_cruising_speed_xy( obj );
                            end
        
                        elseif (close_to_prev)
                            % Accelerate from previous setpoint towards current setpoint
                            % We are close to previous and current setpoint
                            % We first compute the start velocity when close to current
                            % septoint and use this velocity as final velocity when
                            % transition occurs from acceleration to deceleration.
                            % This ensures smooth transition
                            final_cruise_speed = get_cruising_speed_xy( obj );
        
                            if ~is_2_target_threshold
                                % Set target threshold to half dist pre-current
                                target_threshold_tmp = target_threshold_xy;
                                target_threshold_xy = norm(vec_prev_to_current) * 0.5;
        
                                if ((target_threshold_xy - obj.nav_rad) < obj.sigma_norm)
                                    target_threshold_xy = obj.nav_rad;
                                end
        
                                % Velocity close to current setpoint with default zero
                                % if no next setpoint is available
                                vel_close = 0;
                                acceptance_radius = 0;
        
                                % We want to pass and need to compute the desired
                                % velocity close to current setpoint
                                if ( next_setpoint_valid &&  ...
                                        ~strcmp(in.pos_sp_triplet.current.type, ...
                                        'SETPOINT_TYPE_LOITER') )
                                    % Get velocity close to current that depends on
                                    % angle between prev-current and current-next line
                                    vel_close = get_vel_close( obj, ...
                                                                unit_prev_to_current, ...
                                                                unit_current_to_next );
                                    acceptance_radius = obj.nav_rad;
                                end
        
                                % Compute velocity at transition where vehicle switches
                                % from acceleration to deceleration
                                if (target_threshold_tmp - acceptance_radius) < obj.sigma_norm
                                    final_cruise_speed = vel_close;
        
                                else
                                    slope = (get_cruising_speed_xy(obj) - vel_close) / ...
                                        (target_threshold_tmp - acceptance_radius);
                                    final_cruise_speed = vel_close + slope * ...
                                        (target_threshold_xy - acceptance_radius);
                                    
                                    if (final_cruise_speed <= vel_close)
                                        final_cruise_speed = vel_close;
                                    end
                                end
                            end
        
                            % Make sure final cruise speed is larger than 0
                            if final_cruise_speed <= obj.sigma_norm
                                final_cruise_speed = obj.sigma_norm;
                            end
                            vel_sp_along_track = final_cruise_speed;
        
                            % We want to accelerate not too fast
                            % TODO: change the name acceleration_hor_man to something
                            % that can be used by auto and manual
                            acc_track = (final_cruise_speed - vel_sp_along_track_prev) / obj.dt;
        
                            % If yaw offset is large, only accelerate with 0.5m/s^2
                            if abs(yaw_diff) >  deg2rad(obj.mis_yaw_error)
                                acc = 0.5;
                            else
                                acc = obj.acceleration_hor;
                            end
        
                            if (acc_track > acc)
                                vel_sp_along_track = acc * obj.dt + vel_sp_along_track_prev;
                            end
        
                            % Enforce minimum cruise speed
                            vel_sp_along_track  = constrain( vel_sp_along_track, ...
                                                                obj.sigma_norm, ...
                                                                final_cruise_speed );
        
                        elseif close_to_current
                            % Slow down when close to current setpoint
        
                            % Check if altidue is within acceptance radius
                            if dist_to_current_z < obj.nav_rad
                                reached_altitude = 1;
                            else
                                reached_altitude = 0;
                            end
        
                            if ( reached_altitude && next_setpoint_valid && ...
                                    ~strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_LOITER') )
                                % Since we have a next setpoint use the angle
                                % prev-current-next to compute velocity setpoint limit
        
                                % Get velocity close to current that depends on angle
                                % between prev-current and current-next line
                                vel_close = get_vel_close( obj, unit_prev_to_current, ...
                                                            unit_current_to_next );
        
                                % Compute velocity along line which depends on distance
                                % to current setpoint
                                if ( norm(vec_closest_to_current) < obj.nav_rad )
                                    vel_sp_along_track = vel_close;
                                else
                                    if (target_threshold_xy - obj.nav_rad) < obj.sigma_norm
                                        vel_sp_along_track = vel_close;
                                    else
                                        slope = ( get_cruising_speed_xy(obj) - vel_close ) / ...
                                            ( target_threshold_xy - obj.nav_rad ) ;
                                        vel_sp_along_track = vel_close + slope * ...
                                            ( norm(vec_closest_to_current) - obj.nav_rad );
                                    end
                                end
        
                                % Since we want to slow down take over previous velocity
                                % setpoint along track if it was lower
                                if ( (vel_sp_along_track_prev < vel_sp_along_track) && ...
                                        (vel_sp_along_track * vel_sp_along_track_prev > 0) )
                                    vel_sp_along_track = vel_sp_along_track_prev;
                                end
        
                                % If we are close to target and the previous velocity
                                % setpoints was smaller than vel_sp_along_track, then
                                % take over the previous one
                                % This ensures smoothness since we anyway want to slow
                                % down
                                if ( (vel_sp_along_track_prev < vel_sp_along_track) && ...
                                        (vel_sp_along_track * vel_sp_along_track_prev > 0) && ...
                                        (vel_sp_along_track_prev > vel_close) )
                                    vel_sp_along_track = vel_sp_along_track_prev;
                                end
        
                                % Make sure that vel_sp_along track is at least min
                                if vel_sp_along_track < vel_close
                                    vel_sp_along_track = vel_close;
                                end
                            else
                                % We want to stop at current setpoint
                                slope = get_cruising_speed_xy(obj) / target_threshold_xy;
                                vel_sp_along_track =  slope * norm(vec_closest_to_current);
        
                                % Since we want to slow down take over previous velocity
                                % setpoint along track if it was lower but ensure its
                                % not zero
                                if ( (vel_sp_along_track_prev < vel_sp_along_track) && ...
                                        (vel_sp_along_track * vel_sp_along_track_prev > 0) && ...
                                        (vel_sp_along_track_prev > 0.5) )
                                    vel_sp_along_track = vel_sp_along_track_prev;
                                end
                            end
                        end
        
                        % Compute velocity orthogonal to prev-current-line to position
                        vec_pos_to_closest = closest_point - [obj.pos(1); obj.pos(2)];
                        vel_sp_orthogonal = norm(vec_pos_to_closest) * obj.pos_p(1);
        
                        % Compute the cruise speed from velocity along line and
                        % orthogonal velocity setpoint
                        cruise_sp_mag = sqrt( vel_sp_orthogonal * vel_sp_orthogonal + ...
                            vel_sp_along_track * vel_sp_along_track );
        
                        % Sanity check
                        if ~isfinite(cruise_sp_mag)
                            cruise_sp_mag = vel_sp_orthogonal;
                        end
        
                        % Orthogonal velocity setpoint is smaller than cruise speed
                        if ( vel_sp_orthogonal < get_cruising_speed_xy(obj) ) && ...
                            ~current_behind
        
                            % We need to limit vel_sp_along_track such that cruise speed
                            % is never exceeded but still can keep velocity orthogonal
                            % to track
                            if cruise_sp_mag > get_cruising_speed_xy(obj)
                                vel_sp_along_track = sqrt( ...
                                    get_cruising_speed_xy(obj) * get_cruising_speed_xy(obj) ...
                                    - vel_sp_orthogonal * vel_sp_orthogonal );
                            end
        
                            pos_sp(1) = closest_point(1) + ...
                                unit_prev_to_current(1) * vel_sp_along_track / obj.pos_p(1);
                            pos_sp(2) = closest_point(2) + ...
                                unit_prev_to_current(2) * vel_sp_along_track / obj.pos_p(2);
        
                        elseif current_behind
                            % Current is behind
                            if ( norm(vec_pos_to_current)  > 0.01 )
                                pos_sp(1) = obj.pos(1) + vec_pos_to_current(1) / ...
                                    norm(vec_pos_to_current) * vel_sp_along_track / obj.pos_p(1);
                                pos_sp(2) = obj.pos(2) + vec_pos_to_current(2) / ...
                                    norm(vec_pos_to_current) * vel_sp_along_track / obj.pos_p(2);
                            else
                                pos_sp(1) = obj.curr_pos_sp(1);
                                pos_sp(2) = obj.curr_pos_sp(2);
                            end
        
                        else
                            % We are more than cruise_speed away from track
                            % If previous is in front just go directly to previous point
                            if previous_in_front
                                vec_pos_to_closest(1) = obj.prev_pos_sp(1) - obj.pos(1);
                                vec_pos_to_closest(2) = obj.prev_pos_sp(2) - obj.pos(2);
                            end
        
                            % Make sure that we never exceed maximum cruise speed
                            cruise_sp = norm(vec_pos_to_closest) * obj.pos_p(1);
        
                            if cruise_sp > get_cruising_speed_xy( obj )
                                cruise_sp = get_cruising_speed_xy( obj );
                            end
        
                            % Sanity check: don't divide by zero
                            if ( norm(vec_pos_to_closest) > obj.sigma_norm )
                                pos_sp(1) = obj.pos(1) + vec_pos_to_closest(1) / ...
                                    norm(vec_pos_to_closest) * cruise_sp / obj.pos_p(1);
                                pos_sp(2) = obj.pos(2) + vec_pos_to_closest(2) / ...
                                    norm(vec_pos_to_closest) * cruise_sp / obj.pos_p(2);
                            else
                                pos_sp(1) = closest_point(1);
                                pos_sp(2) = closest_point(2);
                            end
                        end
                    end
                    obj.pos_sp = pos_sp;
                elseif strcmp( in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_VELOCITY' )
                    vel_xy_mag = sqrt(obj.vel(1) * obj.vel(1) + obj.vel(2) * obj.vel(2));
        
                    if vel_xy_mag > obj.sigma_norm
                        obj.vel_sp(1) = obj.vel(1) / vel_xy_mag * get_cruising_speed_xy(obj);
                        obj.vel_sp(2) = obj.vel(2) / vel_xy_mag * get_cruising_speed_xy(obj);
                    else
                        % TODO: We should go in the direction we are heading if current
                        %       velocity is zero
                        obj.vel_sp(1) = 0;
                        obj.vel_sp(2) = 0;
                    end
        
                    obj.run_pos_control = 0;
                else
                    % Just go to the target point
                    obj.pos_sp = obj.curr_pos_sp;
        
                    % Set max velocity to cruise
                    obj.vel_max_xy = get_cruising_speed_xy( obj );
                end
        
                % sanity check
                if ( ~(isfinite(obj.pos_sp(1)) && isfinite(obj.pos_sp(2)) && ...
                        isfinite(obj.pos_sp(3))) )
                    warning( 'Auto: Position setpoint not finite' );
                    obj.pos_sp = obj.curr_pos_sp;
                end
        
                % if we're already near the current takeoff setpoint don't reset in case
                % we switch back to posctl.
                % This makes the takeoff finish smoothly.
                
                % Need to detect we're close a bit before the navigator switches from
                % takeoff to next waypoint
                if ( ( strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_TAKEOFF') || ...
                        strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_LOITER') ) && ...
                        (in.pos_sp_triplet.current.acceptance_radius > 0.0) && ...
                    (norm(obj.pos - obj.pos_sp) < in.pos_sp_triplet.current.acceptance_radius * 1.2) )
                    obj.do_reset_alt_pos_flag = 0;
                else
                    % Otherwise: in case of interrupted mission don't go to waypoint but
                    % stay at current position
                    obj.do_reset_alt_pos_flag = 1;
                end
        
                % Handle the landing gear based on the manual landing alt
                high_enough_for_landing_gear = (-obj.pos(3) + in.home_pos.z > 2.0);
        
                % During a mission or in loiter it's safe to retract the landing gear.
                if ( strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_POSITION') || ...
                        strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_LOITER' ) && ...
                    ~in.vehicle_land_detected.landed && high_enough_for_landing_gear )
        
                    extra_states.att_sp.landing_gear = 1;
        
                elseif ( strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_TAKEOFF') || ...
                            strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_LAND') || ...
                        ~high_enough_for_landing_gear )
                    % During takeoff and landing, we better put it down again.
                    extra_states.att_sp.landing_gear = -1;
                    % For the rest of the setpoint types, just leave it as is.
                end
            else
                % Idle or triplet not valid, set velocity setpoint to zero
                obj.vel_sp = obj.vel_sp .* 0;
                obj.run_pos_control = 0;
                obj.run_alt_control = 0;
            end
        end

        function extra_states = control_position( obj, in, extra_states )
        %CONTROL_POSITION
        %    Written:  2021/03/05, J.X.J. Bannwarth
            extra_states = calculate_velocity_setpoint( obj, in, extra_states );
        
            if ( in.control_mode.flag_control_climb_rate_enabled || ...
                    in.control_mode.flag_control_velocity_enabled || ...
                    in.control_mode.flag_control_acceleration_enabled )
                extra_states = calculate_thrust_setpoint( obj, in, extra_states );
            else
                obj.reset_int_z = 1;
            end
        end

        function extra_states = calculate_velocity_setpoint( obj, in, extra_states )
        %CALCULATE_VELOCITY_SETPOINT Run position & altitude controllers if enabled
        %   Otherwise use already computed velocity setpoints.
        %    Written:  2021/03/05, J.X.J. Bannwarth
            if obj.run_pos_control
                % If for any reason, we get a NaN position setpoint, we better just stay
                % where we are.
                if ( isfinite(obj.pos_sp(1)) && isfinite(obj.pos_sp(2)) )
                    obj.vel_sp(1) = (obj.pos_sp(1) - obj.pos(1)) * obj.pos_p(1);
                    obj.vel_sp(2) = (obj.pos_sp(2) - obj.pos(2)) * obj.pos_p(2);
        
                else
                    obj.vel_sp(1) = 0;
                    obj.vel_sp(2) = 0;
                    warning( 'Caught invalid pos_sp in x and y' );
                end
            end
        
            % in auto the setpoint is already limited by the navigator
            if ~in.control_mode.flag_control_auto_enabled
                limit_altitude( obj, in );
            end
        
            if obj.run_alt_control
                if isfinite(obj.pos_sp(3))
                    obj.vel_sp(3) = (obj.pos_sp(3) - obj.pos(3)) * obj.pos_p(3);
                else
                    obj.vel_sp(3) = 0;
                    warning( 'Caught invalid pos_sp in z' );
                end
            end
        
            if ~in.control_mode.flag_control_position_enabled
                obj.reset_pos_sp = 1;
            end
        
            if ~in.control_mode.flag_control_altitude_enabled
                obj.reset_alt_sp = 1;
            end
        
            if ~in.control_mode.flag_control_velocity_enabled
                obj.vel_sp_prev(1) = obj.vel(1);
                obj.vel_sp_prev(2) = obj.vel(2);
                obj.vel_sp(1) = 0;
                obj.vel_sp(2) = 0;
            end
        
            if ~in.control_mode.flag_control_climb_rate_enabled
                obj.vel_sp(3) = 0;
            end
        
            % Limit vertical upwards speed in auto takeoff and close to ground
            altitude_above_home = -obj.pos(3) + in.home_pos.z;
        
            if ( in.pos_sp_triplet.current.valid && ...
                    strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_TAKEOFF') && ...
                ~in.control_mode.flag_control_manual_enabled )
                vel_limit = gradual( altitude_above_home, ...
                                obj.slow_land_alt2, obj.slow_land_alt1, ...
                                obj.tko_speed, obj.vel_max_up );
                obj.vel_sp(3) = max( obj.vel_sp(3), -vel_limit );
            end
        
            % Encourage pilot to respect estimator height limitations when in manually
            % controlled modes and not landing
            if ( isfinite(in.local_pos.hagl_min) && ...               % We need height limiting
                    in.control_mode.flag_control_manual_enabled   && ... % Vehicle is under manual control
                    in.control_mode.flag_control_altitude_enabled && ... % Altitude controller is running
                ~manual_wants_landing( obj ) )                        % Operator is not trying to land
                if in.local_pos.dist_bottom < in.local_pos.hagl_min
                    % If distance to ground is less than limit, increment setpoint
                    % upwards at up to the landing descent rate
                    climb_rate_bias = min( ...
                        1.5 * obj.pos_p(3) * (in.local_pos.hagl_min - in.local_pos.dist_bottom), ...
                        obj.land_speed );
                    obj.vel_sp(3) = obj.vel_sp(3) - climb_rate_bias;
                    obj.pos_sp(3) = obj.pos_sp(3) - climb_rate_bias * obj.dt;
                end
            end
        
            % Limit vertical downwards speed (positive z) close to ground for now we use
            % the altitude above home and assume that we want to land at same height as
            % we took off
            vel_limit = gradual( altitude_above_home, ...
                                    obj.slow_land_alt2, obj.slow_land_alt1, ...
                                    obj.land_speed, obj.vel_max_down );
        
            obj.vel_sp(3) = min(obj.vel_sp(3), vel_limit);
        
            % Apply slewrate (aka acceleration limit) for smooth flying
            if (~in.control_mode.flag_control_auto_enabled && ~obj.in_smooth_takeoff)
                vel_sp_slewrate( obj, in );
            end
        
            % Special velocity setpoint limitation for smooth takeoff (after slewrate!)
            if obj.in_smooth_takeoff
                obj.vel_sp(3) = set_takeoff_velocity( obj, obj.vel_sp(3) );
            end
        
            % Make sure velocity setpoint is constrained in all directions (xyz)
            vel_norm_xy = sqrt( obj.vel_sp(1) * obj.vel_sp(1) + obj.vel_sp(2) * obj.vel_sp(2) );
        
            % Check if the velocity demand is significant
            obj.vel_sp_significant = double(vel_norm_xy > (0.5 * obj.vel_max_xy));
        
            if vel_norm_xy > obj.vel_max_xy
                obj.vel_sp(1) = obj.vel_sp(1) * obj.vel_max_xy / vel_norm_xy;
                obj.vel_sp(2) = obj.vel_sp(2) * obj.vel_max_xy / vel_norm_xy;
            end
        
            obj.vel_sp(3) = constrain( obj.vel_sp(3), -obj.vel_max_up, obj.vel_max_down );
        
            obj.vel_sp_prev = obj.vel_sp;
        end

        function extra_states = calculate_thrust_setpoint( obj, in, extra_states )
        %CALCULATE_THRUST_SETPOINT Calculate thrust setpoint
        %    Written:  2021/03/05, J.X.J. Bannwarth
            % reset integrals if needed
            if in.control_mode.flag_control_climb_rate_enabled
                if obj.reset_int_z
                    obj.reset_int_z = 0;
                    obj.thrust_int(3) = 0;
                end
            else
                obj.reset_int_z = 1;
            end
        
            if in.control_mode.flag_control_velocity_enabled
                if obj.reset_int_xy
                    obj.reset_int_xy = 0;
                    obj.thrust_int(1) = 0;
                    obj.thrust_int(2) = 0;
                end
            else
                obj.reset_int_xy = 1;
            end
        
            % If any of the velocity setpoint is bogus, it's probably safest to command
            % no velocity at all.
            for i = 1:3
                if ~isfinite(obj.vel_sp(i))
                    obj.vel_sp(i) = 0;
                end
            end
        
            % Velocity error
            vel_err = obj.vel_sp - obj.vel;
        
            % Thrust vector in NED frame
            if ( in.control_mode.flag_control_acceleration_enabled && ...
                    in.pos_sp_triplet.current.acceleration_valid )
                thrust_sp = [ in.pos_sp_triplet.current.a_x;
                                in.pos_sp_triplet.current.a_y;
                                in.pos_sp_triplet.current.a_z ];
            else
                thrust_sp = vel_err .* obj.vel_p + obj.vel_err_d .* obj.vel_d ...
                        + obj.thrust_int - [ 0; 0; obj.thr_hover];
            end
        
            if ( ~in.control_mode.flag_control_velocity_enabled && ...
                    ~in.control_mode.flag_control_acceleration_enabled )
                thrust_sp(1) = 0;
                thrust_sp(2) = 0;
            end
        
            % Reduce thrust in early landing detection states to check if the vehicle
            % still moves
            thrust_sp = landdetection_thrust_limit( obj, in, thrust_sp );
        
            if ( ~in.control_mode.flag_control_climb_rate_enabled && ...
                    ~in.control_mode.flag_control_acceleration_enabled )
                thrust_sp(3) = 0;
            end
        
            % Limit thrust vector and check for saturation
            saturation_xy = 0;
            saturation_z = 0;
        
            % Limit min lift
            thr_min = obj.thr_min;
        
            if ( ~in.control_mode.flag_control_velocity_enabled && (thr_min < 0) )
                % Don't allow downside thrust direction in manual attitude mode
                thr_min = 0;
            end
        
            tilt_max = obj.tilt_max_air;
            thr_max = obj.thr_max;
        
            % We can only run the control if we're already in-air, have a takeoff
            % setpoint, or if we're in offboard control.
            % Otherwise, we should just bail out
            if ( in.vehicle_land_detected.landed && ~in_auto_takeoff( obj, in ) && ...
                    ~manual_wants_takeoff( obj, in ) )
                % Keep throttle low while still on ground.
                thr_max = 0;
            elseif ( ~in.control_mode.flag_control_manual_enabled && ...
                        in.pos_sp_triplet.current.valid && ...
                        strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_LAND') )
                % Adjust limits for landing mode
                % Limit max tilt and min lift when landing
                tilt_max = obj.tilt_max_land;
            end
        
            % Limit min lift
            if ( -thrust_sp(3) < thr_min )
                thrust_sp(3) = -thr_min;
        
                % Don't freeze altitude integral if it wants to throttle up
                if vel_err(3) > 0
                    saturation_z = 1;
                end
            end
        
            if ( in.control_mode.flag_control_velocity_enabled || ...
                    in.control_mode.flag_control_acceleration_enabled )
                % Limit max tilt
                if ( thr_min >= 0 ) && ( tilt_max < (pi/2 - 0.05) )
                    % Absolute horizontal thrust
                    thrust_sp_xy_len = norm( [ thrust_sp(1); thrust_sp(2) ] );
        
                    if (thrust_sp_xy_len > 0.01)
                        % Max horizontal thrust for given vertical thrust
                        thrust_xy_max = -thrust_sp(3) * tan(tilt_max);
        
                        if (thrust_sp_xy_len > thrust_xy_max)
                            k = thrust_xy_max / thrust_sp_xy_len;
                            thrust_sp(1) = thrust_sp(1) * k;
                            thrust_sp(2) = thrust_sp(2) * k;
        
                            % Don't freeze x,y integrals if they both want to throttle
                            % down
                            if ~( (vel_err(1) * obj.vel_sp(1) < 0) && ...
                                    (vel_err(2) * obj.vel_sp(2) < 0) )
                                saturation_xy = 1;
                            end
                        end
                    end
                end
            end
        
            if ( in.control_mode.flag_control_climb_rate_enabled && ...
                ~in.control_mode.flag_control_velocity_enabled )
                % Thrust compensation when vertical velocity but not horizontal velocity
                % is controlled
                tilt_cos_max = 0.7;
        
                if ( in.R(3,3) > tilt_cos_max )
                    att_comp = 1 / in.R(3, 3);
        
                elseif (in.R(3,3) > 0)
                    att_comp = ( (1 / tilt_cos_max - 1) / tilt_cos_max ) * in.R(3,3) + 1;
                    saturation_z = 1;
        
                else
                    att_comp = 1;
                    saturation_z = 1;
                end
        
                thrust_sp(3) = thrust_sp(3) * att_comp;
            end
        
            % Calculate desired total thrust amount in body z direction.
            % To compensate for excess thrust during attitude tracking errors we
            % project the desired thrust force vector F onto the real vehicle's thrust
            % axis in NED:
            % Body thrust axis [0,0,-1]' rotated by R is: R*[0,0,-1]' = -R_z
            R_z = [ in.R(1,3); in.R(2,3); in.R(3,3) ];
        
            % Recalculate because it might have changed
            thrust_body_z = dot( thrust_sp, -R_z );
        
            % Limit max thrust
            if abs(thrust_body_z) > thr_max
                if ( thrust_sp(3) < 0 )
                    if -thrust_sp(3) > thr_max
                        % Thrust Z component is too large, limit it
                        thrust_sp(1) = 0;
                        thrust_sp(2) = 0;
                        thrust_sp(3) = -thr_max;
                        saturation_xy = 1;
        
                        % Don't freeze altitude integral if it wants to throttle down
                        if vel_err(3) < 0.0
                            saturation_z = 1;
                        end
        
                    else
                        % Preserve thrust Z component and lower XY, keeping altitude is
                        % more important than position
                        thrust_xy_max = sqrt(thr_max * thr_max - thrust_sp(3) * thrust_sp(3));
                        thrust_xy_abs = norm( [thrust_sp(1); thrust_sp(2)] );
                        k = thrust_xy_max / thrust_xy_abs;
                        thrust_sp(1) = thrust_sp(1) * k;
                        thrust_sp(2) = thrust_sp(2) * k;
                        % Don't freeze x,y integrals if they both want to throttle down
                        if ~( (vel_err(1) * obj.vel_sp(1) < 0) && ...
                                (vel_err(2) * obj.vel_sp(2) < 0) )
                            saturation_xy = 1;
                        end
                    end
        
                else
                    % Z component is positive, going down (Z is positive down in NED),
                    % simply limit thrust vector
                    k = thr_max / abs( thrust_body_z );
                    thrust_sp = thrust_sp * k;
                    saturation_xy = 1;
                    saturation_z = 1;
                end
        
                thrust_body_z = thr_max;
            end
        
            % If any of the thrust setpoint is bogus, send out a warning
            if ( ~isfinite(thrust_sp(1)) || ...
                    ~isfinite(thrust_sp(2)) || ...
                    ~isfinite(thrust_sp(3)) )
                warning( 'Thrust setpoint not finite' );
            end
        
            extra_states.att_sp.thrust = max( thrust_body_z, thr_min );
        
            % Update integrals
            if ( in.control_mode.flag_control_velocity_enabled && ~saturation_xy )
                obj.thrust_int(1) = obj.thrust_int(1) + vel_err(1) * obj.vel_i(1) * obj.dt;
                obj.thrust_int(2) = obj.thrust_int(2) + vel_err(2) * obj.vel_i(2) * obj.dt;
            end
        
            if ( in.control_mode.flag_control_climb_rate_enabled && ~saturation_z )
                obj.thrust_int(3) = obj.thrust_int(3) + vel_err(3) * obj.vel_i(3) * obj.dt;
            end
        
            % Calculate attitude setpoint from thrust vector
            if ( in.control_mode.flag_control_velocity_enabled || ...
                    in.control_mode.flag_control_acceleration_enabled )
                % Desired body_z axis = -normalize(thrust_vector)
                if ( norm(thrust_sp) > obj.sigma_norm )
                    body_z = -normalize( thrust_sp );
                else
                    % No thrust, set Z axis to safe value
                    body_z = [ 0; 0; 1 ];
                end
        
                % Vector of desired yaw direction in XY plane, rotated by PI/2
                y_C = [ -sin(extra_states.att_sp.yaw_body); cos(extra_states.att_sp.yaw_body); 0 ];
        
                if ( abs(body_z(3)) > obj.sigma_single_op )
                    % Desired body_x axis, orthogonal to body_z
                    body_x = cross( y_C, body_z );
        
                    % Keep nose to front while inverted upside down
                    if (body_z(3) < 0)
                        body_x = -body_x;
                    end
        
                    body_x = normalize( body_x );
        
                else
                    % Desired thrust is in XY plane, set X downside to construct correct
                    % matrix, but yaw component will not be used actually
                    body_x = [0; 0; 1];
                end
        
                % Desired body_y axis
                body_y = cross( body_z, body_x );
        
                % Fill rotation matrix
                for ii = 1:3
                    obj.R_setpoint(ii, 1) = body_x(ii);
                    obj.R_setpoint(ii, 2) = body_y(ii);
                    obj.R_setpoint(ii, 3) = body_z(ii);
                end
        
                % Copy quaternion setpoint to attitude setpoint topic
                q_sp = DcmToQuat( obj.R_setpoint );
                extra_states.att_sp.q_d = q_sp;
                extra_states.att_sp.q_d_valid = 1;
        
                % Calculate euler angles, for logging only, must not be used for control
                euler = DcmToEuler( obj.R_setpoint );
                extra_states.att_sp.roll_body = euler(1);
                extra_states.att_sp.pitch_body = euler(2);
                % Yaw already used to construct rot matrix, but actual rotation matrix
                % can have different yaw near singularity
        
            elseif ~in.control_mode.flag_control_manual_enabled
                % Autonomous altitude control without position control (failsafe
                % landing), force level attitude, don't change yaw
                obj.R_setpoint = EulerToDcm( [0; 0; extra_states.att_sp.yaw_body] );
        
                % Copy quaternion setpoint to attitude setpoint topic
                q_sp = DcmToQuat( obj.R_setpoint );
                extra_states.att_sp.q_d = q_sp;
                extra_states.att_sp.q_d_valid = 1;
        
                extra_states.att_sp.roll_body = 0;
                extra_states.att_sp.pitch_body = 0;
            end
        
            % save thrust setpoint for logging
            extra_states.local_pos_sp.acc_x = thrust_sp(1) * 9.80665;
            extra_states.local_pos_sp.acc_y = thrust_sp(2) * 9.80665;
            extra_states.local_pos_sp.acc_z = thrust_sp(3) * 9.80665;
        
            extra_states.att_sp.timestamp = getCurrentTime( obj );
        end

        function extra_states = generate_attitude_setpoint( obj, in, extra_states )
        %GENERATE_ATTITUDE_SETPOINT Generate attitude setpoint
        %    Written:  2021/03/05, J.X.J. Bannwarth
        
            % Yaw setpoint is integrated over time, but we don't want to integrate the
            % offsets
            extra_states.att_sp.yaw_body = extra_states.att_sp.yaw_body - obj.man_yaw_offset;
            obj.man_yaw_offset = 0;
        
            % Reset yaw setpoint to current position if needed
            if obj.reset_yaw_sp
                obj.reset_yaw_sp = 0;
                extra_states.att_sp.yaw_body = obj.yaw;
            elseif ( ~in.vehicle_land_detected.landed && ...
                    ~(~in.control_mode.flag_control_altitude_enabled && obj.manual.z < 0.1) )
                % Do not move yaw while sitting on the ground
                % We want to know the real constraint, and global overrides manual
                if obj.man_yaw_max < obj.global_yaw_max
                    yaw_rate_max = obj.man_yaw_max;
                else
                    yaw_rate_max = obj.global_yaw_max;
                end
                yaw_offset_max = yaw_rate_max / obj.mc_att_yaw_p;
        
                extra_states.att_sp.yaw_sp_move_rate = obj.manual.r * yaw_rate_max;
        
                yaw_target = WrapPi( extra_states.att_sp.yaw_body ...
                    + extra_states.att_sp.yaw_sp_move_rate * obj.dt );
                yaw_offs = WrapPi( yaw_target - obj.yaw );
        
                % If the yaw offset became too big for the system to track stop
                % shifting it, only allow if it would make the offset smaller again.
                if ( (abs(yaw_offs) < yaw_offset_max) || ...
                        (extra_states.att_sp.yaw_sp_move_rate > 0 && yaw_offs < 0) || ...
                        (extra_states.att_sp.yaw_sp_move_rate < 0 && yaw_offs > 0) )
                    extra_states.att_sp.yaw_body = yaw_target;
                end
            end
        
            % Control throttle directly if no climb rate controller is active
            if ~in.control_mode.flag_control_climb_rate_enabled
                thr_val = throttle_curve( obj.manual.z, obj.thr_hover );
                extra_states.att_sp.thrust = min( thr_val, obj.manual_thr_max );
        
                % Enforce minimum throttle if not landed
                if ~in.vehicle_land_detected.landed
                    extra_states.att_sp.thrust = max( extra_states.att_sp.thrust, obj.manual_thr_min );
                end
            end
        
            % Control roll and pitch directly if no aiding velocity controller is active
            if ~in.control_mode.flag_control_velocity_enabled
                % Input mapping for roll & pitch setpoints
                % ----------------------------------------
                % This simplest thing to do is map the y & x inputs directly to roll and
                % pitch, and scale according to the max tilt angle.
                % But this has several issues:
                % - The maximum tilt angle cannot easily be restricted. By limiting the
                %   roll and pitch separately, it would be possible to get to a higher
                %   tilt angle by combining roll and pitch (the difference is around 15
                %   degrees maximum, so quite noticeable). Limiting this angle is not
                %   simple in roll-pitch-space, it requires to limit the tilt
                %   angle = acos(cos(roll)% cos(pitch)) in a meaningful way (eg. scaling
                %   both roll and pitch).
                % - Moving the stick diagonally, such that |x| = |y|, does not move the
                %   vehicle towards a 45 degrees angle.
                %   The direction angle towards the max tilt in the XY-plane is
                %   atan(1/cos(x)). Which means it even depends on the tilt angle (for a
                %   tilt angle of 35 degrees, it's off by about 5 degrees).
                %
                % So instead we control the following 2 angles:
                % - tilt angle, given by sqrt(x*x + y*y)
                % - the direction of the maximum tilt in the XY-plane, which also
                %   defines the direction of the motion
                %
                % This allows a simple limitation of the tilt angle, the vehicle flies
                % towards the direction that the stick points to, and changes of the
                % stick input are linear.
                
                x = obj.manual.x * obj.man_tilt_max;
                y = obj.manual.y * obj.man_tilt_max;
        
                % we want to fly towards the direction of (x, y), so we use a
                % perpendicular axis angle vector in the XY-plane
                v = [y; -x];
                v_norm = norm(v); % the norm of v defines the tilt angle
        
                if v_norm > obj.man_tilt_max
                    % Limit to the configured maximum tilt angle
                    v = v * obj.man_tilt_max / v_norm;
                end
        
                q_sp_rpy = AxisAngleToQuat( [v(1); v(2); 0] );
        
                % The axis angle can change the yaw as well (but only at higher tilt
                % angles. Note: we're talking about the world frame here, in terms of
                % body frame the yaw rate will be unaffected).
                % This the the formula by how much the yaw changes:
                %   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
                %   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
                euler_sp = QuatToEuler( q_sp_rpy );
        
                % Since the yaw setpoint is integrated, we extract the offset here,
                % so that we can remove it before the next iteration
                obj.man_yaw_offset = euler_sp(3);
        
                % Update the setpoints
                extra_states.att_sp.roll_body = euler_sp(1);
                extra_states.att_sp.pitch_body = euler_sp(2);
                extra_states.att_sp.yaw_body = extra_states.att_sp.yaw_body + euler_sp(3);
        
                % Only if we're a VTOL modify roll/pitch
                if in.vehicle_status.is_vtol
                    % Construct attitude setpoint rotation matrix. modify the setpoints
                    % for roll and pitch such that they reflect the user's intention
                    % even if a yaw error (yaw_sp - yaw) is present. In the presence of
                    % a yaw error constructing a rotation matrix from the pure euler
                    % angle setpoints will lead to unexpected attitude behaviour from
                    % the user's view as the euler angle sequence uses the yaw setpoint
                    % and not the current heading of the vehicle.
                    % The effect of that can be seen with:
                    % - Roll/pitch into one direction, keep it fixed (at high angle)
                    % - Apply a fast yaw rotation
                    % - Look at the roll and pitch angles: they should stay pretty much
                    %   the same as when not yawing
        
                    % Calculate our current yaw error
                    yaw_error = WrapPi( extra_states.att_sp.yaw_body - obj.yaw );
        
                    % compute the vector obtained by rotating a z unit vector by the rotation
                    % given by the roll and pitch commands of the user
                    zB = [0; 0; 1];
                    R_sp_roll_pitch = EulerToDcm( [extra_states.att_sp.roll_body; extra_states.att_sp.pitch_body; 0] );
                    z_roll_pitch_sp = R_sp_roll_pitch * zB;
        
                    % transform the vector into a new frame which is rotated around the z axis
                    % by the current yaw error. this vector defines the desired tilt when we look
                    % into the direction of the desired heading
                    R_yaw_correction = EulerToDcm( [0; 0; -yaw_error] );
                    z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;
        
                    % use the formula z_roll_pitch_sp = R_tilt * [0; 0; 1]
                    % R_tilt is computed from_euler; only 1 if cos(roll) not equal zero
                    % -> valid if roll is not +-pi/2;
                    extra_states.att_sp.roll_body = -asin( z_roll_pitch_sp(2) );
                    extra_states.att_sp.pitch_body = atan2( z_roll_pitch_sp(1), z_roll_pitch_sp(3) );
                end
        
                % copy quaternion setpoint to attitude setpoint topic
                q_sp = EulerToQuat( [ extra_states.att_sp.roll_body;
                                        extra_states.att_sp.pitch_body;
                                        extra_states.att_sp.yaw_body ] );
                extra_states.att_sp.q_d = q_sp;
                extra_states.att_sp.q_d_valid = 1;
            end
        
            % Only switch the landing gear up if we are not landed and if
            % the user switched from gear down to gear up.
            % If the user had the switch in the gear up position and took off ignore it
            % until he toggles the switch to avoid retracting the gear immediately on takeoff.
            if ( strcmp(obj.manual.gear_switch, 'SWITCH_POS_ON') && ...
                    obj.gear_state_initialized && ~in.vehicle_land_detected.landed )
                extra_states.att_sp.landing_gear = vehicle_attitude_setpoint_s_LANDING_GEAR_UP;
        
            elseif strcmp(obj.manual.gear_switch, 'SWITCH_POS_OFF')
                extra_states.att_sp.landing_gear = vehicle_attitude_setpoint_s_LANDING_GEAR_DOWN;
                % Switching the gear off does put it into a safe defined state
                obj.gear_state_initialized = 1;
            end
        
            extra_states.att_sp.timestamp = getCurrentTime( obj );
        end

        %% mc_pos_control_main.cpp - Secondary functions
        function reset_altitude_sp( obj )
        %RESET_ALTITUDE_SP Reset altitude setpoint to current altitude
        %
        %   This reset will only occur if the obj.reset_alt_sp flag has been set.
        %   The general logic follows the reset_pos_sp() architecture.
        %
        %   We have logic in the main function which choosed the velocity setpoint such
        %   that the attitude setpoint is continuous when switching into velocity
        %   controlled mode, therefore, we don't need to bother about resetting altitude
        %   in a special way.
        %
        %   Written:  2021/03/05, J.X.J. Bannwarth
            if obj.reset_alt_sp
                obj.reset_alt_sp = 0;
                obj.pos_sp(3) = obj.pos(3);
            end
        end

        function reset_position_sp( obj )
        %RESET_POSITION_SP  Reset position setpoint to current position
        %
        %   This reset will only occur if the obj.reset_pos_sp flag has been set.
        %   The general logic is to first "activate" the flag in the flight
        %   regime where a switch to a position control mode should hold the
        %   very last position. Once switching to a position control mode
        %   the last position is stored once.
        %
        %   We have logic in the main function which chooses the velocity setpoint
        %   such that the attitude setpoint is continuous when switching into
        %   velocity controlled mode, therefore, we don't need to bother about
        %   resetting position in a special way. In position control mode the
        %   position will be reset anyway until the vehicle has reduced speed.
        %
        %   Written:  2021/03/05, J.X.J. Bannwarth
            if obj.reset_pos_sp
                obj.reset_pos_sp = 0;
                obj.pos_sp(1) = obj.pos(1);
                obj.pos_sp(2) = obj.pos(2);
            end
        end

        function vel_close = get_vel_close( obj, unit_prev_to_current, unit_current_to_next )
        %GET_VEL_CLOSE Get close velocity
        %    Written:  2021/03/06, J.X.J. Bannwarth
        
            % Minimum cruise speed when passing waypoint
            min_cruise_speed = 1;
        
            % Make sure that cruise speed is larger than minimum*/
            if ( (get_cruising_speed_xy(obj) - min_cruise_speed) < obj.sigma_norm )
                vel_close = get_cruising_speed_xy(obj);
                return
            end
        
            % Middle cruise speed is a number between maximum cruising speed and minimum
            % cruising speed and corresponds to speed at angle of 90degrees it needs to
            % be always larger than minimum cruise speed
            middle_cruise_speed = obj.cruise_speed_90;
        
            if ( (middle_cruise_speed - min_cruise_speed) < obj.sigma_norm )
                middle_cruise_speed = min_cruise_speed + obj.sigma_norm;
            end
        
            if ( (get_cruising_speed_xy(obj) - middle_cruise_speed) < obj.sigma_norm )
                middle_cruise_speed = (get_cruising_speed_xy(obj) + min_cruise_speed) * 0.5;
            end
        
            % If middle cruise speed is exactly in the middle, then compute vel_close
            % linearly
            use_linear_approach = 0;
        
            if ( ( (get_cruising_speed_xy(obj) + min_cruise_speed) * 0.5 - ...
                middle_cruise_speed ) < obj.sigma_norm )
                use_linear_approach = 1;
            end
        
            % angle = cos(x) + 1.0
            % angle goes from 0 to 2 with 0 = large angle, 2 = small angle:
            % 0 = PI, 2 = PI*0
            angle = 2.0;
            if ( norm(unit_current_to_next) > obj.sigma_norm )
                angle = unit_current_to_next * (unit_prev_to_current * -1) + 1;
            end
        
            % Compute velocity target close to waypoint
            if use_linear_approach
                % Velocity close to target adjusted to angle, vel_close =  m*x+q
                slope = -( get_cruising_speed_xy(obj) - min_cruise_speed ) / 2.0;
                vel_close = slope * angle + get_cruising_speed_xy(obj);
            else
                % Velocity close to target adjusted to angle
                % vel_close = a *b ^x + c; where at angle = 0 -> vel_close = vel_cruise;
                % angle = 1 -> vel_close = middle_cruise_speed (this means that at
                % 90degrees the velocity at target is middle_cruise_speed);
                % angle = 2 -> vel_close = min_cruising_speed
        
                % From maximum cruise speed, minimum cruise speed and middle cruise
                % speed compute constants a, b and c
                a = -( (middle_cruise_speed -  get_cruising_speed_xy(obj)) * ...
                        (middle_cruise_speed -  get_cruising_speed_xy(obj)) ) / ...
                        ( 2.0 * middle_cruise_speed - get_cruising_speed_xy(obj) - ...
                        min_cruise_speed );
                c =  get_cruising_speed_xy(obj) - a;
                b = (middle_cruise_speed - c) / a;
                vel_close = a * b^angle + c;
            end
        
            % vel_close needs to be in between max and min
            vel_close = constrain( vel_close, min_cruise_speed, get_cruising_speed_xy(obj) );
        end

        function out = get_cruising_speed_xy( obj )
        %GET_CRUISING_SPEED_XY Get horizontal cruising speed
        %   In mission the user can choose cruising speed different to default
        %    Written:  2021/03/05, J.X.J. Bannwarth
            if ( isfinite(in.pos_sp_triplet.current.cruising_speed) && ...
                    ~(in.pos_sp_triplet.current.cruising_speed < 0) )
                out = in.pos_sp_triplet.current.cruising_speed;
            else
                out = obj.vel_cruise_xy;
            end
        end

        function limit_altitude( obj, in )
        %LIMIT_ALTITUDE Limit altitude based on several conditions
        %    Written:  2021/03/05, J.X.J. Bannwarth
        
            % Calculate vertical position limit
            if (obj.terrain_follow)
                poz_z_min_limit = max( -in.local_pos.hagl_max, ...
                                        -in.vehicle_land_detected.alt_max );
            else
                poz_z_min_limit = max( -in.local_pos.hagl_max + obj.pos(3) + ...
                                        in.local_pos.dist_bottom, ...
                                        -in.vehicle_land_detected.alt_max + ...
                                        in.home_pos.z );
            end
        
            % Don't allow the z position setpoint to exceed the limit
            if (obj.pos_sp(3) < poz_z_min_limit)
                obj.pos_sp(3) = poz_z_min_limit;
            end
        
            % Limit the velocity setpoint to not exceed a value that will allow
            % controlled deceleration to a stop at the height limit
            vel_z_sp_altctl = ( poz_z_min_limit - obj.pos(3) ) * obj.pos_p(3);
            vel_z_sp_altctl = min( vel_z_sp_altctl, obj.vel_max_down );
            obj.vel_sp(3) = max( obj.vel_sp(3), vel_z_sp_altctl );
        end

        function out = manual_wants_landing( obj, in )
        %MANUAL_WANTS_LANDING User wants manual landing
        %   Written: 2021/03/05, J.X.J. Bannwarth
            has_manual_control_present = in.control_mode.flag_control_manual_enabled && ...
                in.manual.timestamp > 0;
        
            % Operator is trying to land if the throttle stick is under 15%.
            out = ( has_manual_control_present && (in.manual.z < 0.15 || ...
                ~in.control_mode.flag_control_climb_rate_enabled) );
        end

        function out = manual_wants_takeoff( obj, in )
        %MANUAL_WANTS_TAKEOFF User wants manual takeoff
        %   Written: 2021/03/05, J.X.J. Bannwarth
            has_manual_control_present = in.control_mode.flag_control_manual_enabled && ...
                in.manual.timestamp > 0;
        
            % Manual takeoff is triggered if the throttle stick is above 65%.
            out = ( has_manual_control_present && (in.manual.z > 0.65 || ...
                ~in.control_mode.flag_control_climb_rate_enabled) );
        end

        function vel_sp_slewrate( obj, in )
        %VEL_SP_SLEWRATE
        %    Written:  2021/03/05, J.X.J. Bannwarth
            vel_sp_xy = [ obj.vel_sp(1); obj.vel_sp(2) ];
            vel_sp_prev_xy = [ obj.vel_sp_prev(1); obj.vel_sp_prev(2) ];
            acc_xy = (vel_sp_xy - vel_sp_prev_xy) / obj.dt;
        
            % limit total horizontal acceleration
            if ( norm( acc_xy ) > obj.acc_state_dependent_xy )
                vel_sp_xy = obj.acc_state_dependent_xy * normalize(acc_xy) * obj.dt ...
                    + vel_sp_prev_xy;
                obj.vel_sp(1) = vel_sp_xy(1);
                obj.vel_sp(2) = vel_sp_xy(2);
            end
        
            % limit vertical acceleration
            acc_z = (obj.vel_sp(3) - obj.vel_sp_prev(3)) / obj.dt;
        
            if (in.control_mode.flag_control_manual_enabled)
                if acc_z < 0
                    max_acc_z = -obj.acc_state_dependent_z;
                else
                    max_acc_z = obj.acc_state_dependent_z;
                end
        
            else
                if acc_z < 0
                    max_acc_z = -obj.acceleration_z_max_up;
                else
                    max_acc_z = obj.acceleration_z_max_down;
                end
            end
        
            if (abs(acc_z) > abs(max_acc_z))
                obj.vel_sp(3) = max_acc_z * obj.dt + obj.vel_sp_prev(3);
            end
        end

        function vel_sp_z = set_takeoff_velocity( obj, vel_sp_z )
        %SET_TAKEOFF_VELOCITY
        %    Written:  2021/03/05, J.X.J. Bannwarth
            obj.in_smooth_takeoff = obj.takeoff_vel_limit < -vel_sp_z;
            % Ramp vertical velocity limit up to takeoff speed
            obj.takeoff_vel_limit = obj.takeoff_vel_limit - ...
                vel_sp_z * obj.dt / obj.takeoff_ramp_time;
            % Limit vertical velocity to the current ramp value
            vel_sp_z = max( vel_sp_z, -obj.takeoff_vel_limit );
        end

        function extra_states = set_idle_state( obj, in, extra_states )
        %SET_IDLE_STATE Set idle state
        %    Written:  2021/03/05, J.X.J. Bannwarth
            extra_states.local_pos_sp.x = obj.pos(1);
            extra_states.local_pos_sp.y = obj.pos(2);
            extra_states.local_pos_sp.z = obj.pos(3) + 1.0; % 1 m into ground when idle
            extra_states.local_pos_sp.vx = 0;
            extra_states.local_pos_sp.vy = 0;
            extra_states.local_pos_sp.vz = 1.0; % 1 m/s into ground
            extra_states.local_pos_sp.yaw = in.yaw;
            extra_states.local_pos_sp.yawspeed = 0;
        
            extra_states.att_sp.roll_body = 0;
            extra_states.att_sp.pitch_body = 0;
            extra_states.att_sp.yaw_body = in.yaw;
            extra_states.att_sp.yaw_sp_move_rate = 0;
            q_sp = EulerToQuat( [0; 0; in.yaw] );
            extra_states.att_sp.q_d = q_sp;
            extra_states.att_sp.q_d_valid = 1;
            extra_states.att_sp.thrust = 0;
        end

        function thrust_sp = landdetection_thrust_limit( obj, in, thrust_sp )
        %LANDDETECTION_THRUST_LIMIT
        %    Written:  2021/03/05, J.X.J. Bannwarth
            if ~in_auto_takeoff(obj, in) && ~manual_wants_takeoff(obj, in)
                if in.vehicle_land_detected.ground_contact
                    % if still or already on ground command zero xy thrust_sp in body
                    % frame to consider uneven ground
        
                    % thrust setpoint in body frame
                    thrust_sp_body = in.R' * thrust_sp;
        
                    % we dont want to make any correction in body x and y
                    thrust_sp_body(1) = 0;
                    thrust_sp_body(2) = 0;
        
                    % make sure z component of thrust_sp_body is larger than 0 (positive
                    % thrust is downward)
                    if thrust_sp(3) > 0
                        thrust_sp_body(3) = thrust_sp(3);
                    else
                        thrust_sp_body(3) = 0;
                    end
        
                    % convert back to local frame (NED)
                    thrust_sp = in.R * thrust_sp_body;
                end
        
                if in.vehicle_land_detected.maybe_landed
                    % We set thrust to zero
                    % This will help to decide if we are actually landed or not
                    thrust_sp = thrust_sp .* 0;
                end
            end
        end

        function out = in_auto_takeoff( obj, in )
        %IN_AUTO_TAKEOFF In auto mode, check if we do a takeoff
        %    Written:  2021/03/05, J.X.J. Bannwarth
            out = ( in.pos_sp_triplet.current.valid && ...
                strcmp(in.pos_sp_triplet.current.type, 'SETPOINT_TYPE_TAKEOFF') ) || ...
                in.control_mode.flag_control_offboard_enabled;
        end

        %% Low-pass filter functions
        function output = LowPassFilter2pVector3Apply( obj, sample )
        %LOWPASSFILTER2PVECTOR3APPLY Apply low pass filter
        %   Based on code from PX4 Firmware:
        %       https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/filter/LowPassFilter2pVector3f.hpp
        %   Written:2019/01/16, J.X.J. Bannwarth 
            delay_element_0 = sample - obj.lp_filters_d_delay_element_1.*obj.lp_filters_d_a1 - ...
                obj.lp_filters_d_delay_element_2.*obj.lp_filters_d_a2;
            output = delay_element_0.*obj.lp_filters_d_b0 + ...
                obj.lp_filters_d_delay_element_1.*obj.lp_filters_d_b1 + ...
                obj.lp_filters_d_delay_element_2*obj.lp_filters_d_b2;
            obj.lp_filters_d_delay_element_2 = obj.lp_filters_d_delay_element_1;
            obj.lp_filters_d_delay_element_1 = delay_element_0;
        end
        
        function LowPassFilter2pVectorSetCutOffFrequency( obj, sample_freq, cutoff_freq )
        %LOWPASSFILTER2PVECTORSETCUTOFFFREQUENCY Set cutoff frequency
        %   Based on code from PX4 Firmware:
        %       https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/filter/LowPassFilter2pVector3f.cpp
        %   Written: 2019/01/16, J.X.J. Bannwarth
            obj.lp_filters_d_cutoff_freq = cutoff_freq;
            
            % Reset delay elements on filter change
            obj.lp_filters_d_delay_element_1 = zeros(3,1);
            obj.lp_filters_d_delay_element_2 = zeros(3,1);
            
            if (obj.lp_filters_d_cutoff_freq <= 0.0)
                % No filtering
                obj.lp_filters_d_b0 = 1.0;
                obj.lp_filters_d_b1 = 0.0;
                obj.lp_filters_d_b2 = 0.0;
                
                obj.lp_filters_d_a1 = 0.0;
                obj.lp_filters_d_a2 = 0.0;
                return
            end
            
            fr = sample_freq / obj.lp_filters_d_cutoff_freq;
            ohm = tan( pi / fr);
            c = 1.0 + 2.0 * cos( pi / 4.0) * ohm + ohm * ohm;
            
            obj.lp_filters_d_b0 = ohm * ohm / c;
            obj.lp_filters_d_b1 = 2.0 * obj.lp_filters_d_b0;
            obj.lp_filters_d_b2 = obj.lp_filters_d_b0;
            
            obj.lp_filters_d_a1 = 2.0 * (ohm * ohm - 1.0) / c;
            obj.lp_filters_d_a2 = (1.0 - 2.0 * cos(pi / 4.0) * ohm + ohm * ohm) / c;
        end
        
        function output = LowPassFilter2pVectorReset( obj, sample )
        %LOWPASSFILTER2PVECTORRESET Reset low-pass filter
        %   Based on code from PX4 Firmware:
        %       https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/filter/LowPassFilter2pVector3f.cpp
        %   Written: 2019/01/16, J.X.J. Bannwarth
            dval = sample ./ (obj.lp_filters_d_b0 + obj.lp_filters_d_b1 ...
                + obj.lp_filters_d_b2);
            
            if ( isfinite( dval(1) ) && ...
                 isfinite( dval(2) ) && ...
                 isfinite( dval(3) ) )
                obj.lp_filters_d_delay_element_1 = dval;
                obj.lp_filters_d_delay_element_2 = dval;
            else
                obj.lp_filters_d_delay_element_1 = sample;
                obj.lp_filters_d_delay_element_2 = sample;
            end
            
            output = LowPassFilter2pVector3Apply( obj, sample );
        end

        %% Helper functions
        function extra_states = discrete_states_to_structure( obj )
        %DISCRETE_STATES_TO_STRUCTURE Decode discrete state arrays to structures
        %   Structs cannot be used for discrete states. As a consequence, it is
        %   necessary to store them as arrays. This function converts from array
        %   to structure representation
        %
        %   See also STRUCTURE_TO_DISCRETE_STATES.
        %
        %   Written: 2021/03/08, J.X.J. Bannwarth
    
            % Structures
            extra_states.ref_pos = struct( ...
                'lat_rad'   , obj.ref_pos(1), ...
                'lon_rad'   , obj.ref_pos(2), ...
                'sin_lat'   , obj.ref_pos(3), ...
                'cos_lat'   , obj.ref_pos(4), ...
                'timestamp' , obj.ref_pos(5), ...
                'init_done' , obj.ref_pos(6)  ...
                );
            extra_states.att_sp = struct( ...
                'q_d'              , obj.att_sp(1:4), ...
                'landing_gear'     , obj.att_sp(5)  , ...
                'pitch_body'       , obj.att_sp(6)  , ...
                'q_d_valid'        , obj.att_sp(7)  , ...
                'roll_body'        , obj.att_sp(8)  , ...
                'thrust'           , obj.att_sp(9)  , ...
                'timestamp'        , obj.att_sp(10) , ...
                'yaw_body'         , obj.att_sp(11) , ...
                'yaw_sp_move_rate' , obj.att_sp(12)   ...
                );
            extra_states.local_pos_sp = struct( ...
                'acc_x'     , obj.local_pos_sp(1) , ...
                'acc_y'     , obj.local_pos_sp(2) , ...
                'acc_z'     , obj.local_pos_sp(3) , ...
                'thrust'    , obj.local_pos_sp(4) , ...
                'timestamp' , obj.local_pos_sp(5) , ...
                'vx'        , obj.local_pos_sp(6) , ...
                'vy'        , obj.local_pos_sp(7) , ...
                'vz'        , obj.local_pos_sp(8) , ...
                'x'         , obj.local_pos_sp(9) , ...
                'y'         , obj.local_pos_sp(10), ...
                'yaw'       , obj.local_pos_sp(11), ...
                'yawspeed'  , obj.local_pos_sp(12), ...
                'z'         , obj.local_pos_sp(13)  ...
                );
            extra_states.vel_deriv = struct( ...
                'lp_state', obj.vel_deriv(1:3), ...
                'in_prev' , obj.vel_deriv(4:6), ...
                'f_cut'   , obj.vel_deriv(7), ...
                'dt'      , obj.vel_deriv(8)  ...
                );
            extra_states.hysteresis = struct( ...
                'state'                    , obj.hysteresis(1), ...
                'requested_state'          , obj.hysteresis(2), ...
                'last_time_to_change_state', obj.hysteresis(3), ...
                'time_from_1_us'           , obj.hysteresis(4), ...
                'time_from_0_us'           , obj.hysteresis(5)  ...
                );
            
            % Char arrays
            intentions = { 'brake'; 'direction_change'; 'acceleration'; 'deceleration' };
            extra_states.user_intention_xy = intentions{obj.user_intention_xy};
            extra_states.user_intention_z  = intentions{obj.user_intention_z};
        end
    
        function structure_to_discrete_states( obj, extra_states )
        %STRUCTURE_TO_DISCRETE_STATES Encode structures to discrete state arrays
        %
        %   See also DISCRETE_STATES_TO_STRUCTURE.
        %
        %   Written: 2021/03/08, J.X.J. Bannwarth
        
            % Structures
            obj.ref_pos = [ ...
                extra_states.ref_pos.lat_rad;
                extra_states.ref_pos.lon_rad;
                extra_states.ref_pos.sin_lat;
                extra_states.ref_pos.cos_lat;
                extra_states.ref_pos.timestamp;
                extra_states.ref_pos.init_done;
                ];
            obj.att_sp = [ ...
                extra_states.att_sp.q_d;
                extra_states.att_sp.landing_gear;
                extra_states.att_sp.pitch_body;
                extra_states.att_sp.q_d_valid;
                extra_states.att_sp.roll_body;
                extra_states.att_sp.thrust;
                extra_states.att_sp.timestamp;
                extra_states.att_sp.yaw_body;
                extra_states.att_sp.yaw_sp_move_rate;
                ];
            obj.local_pos_sp = [ ...
                extra_states.local_pos_sp.acc_x;
                extra_states.local_pos_sp.acc_y;
                extra_states.local_pos_sp.acc_z;
                extra_states.local_pos_sp.thrust;
                extra_states.local_pos_sp.timestamp;
                extra_states.local_pos_sp.vx;
                extra_states.local_pos_sp.vy;
                extra_states.local_pos_sp.vz;
                extra_states.local_pos_sp.x;
                extra_states.local_pos_sp.y;
                extra_states.local_pos_sp.yaw;
                extra_states.local_pos_sp.yawspeed;
                extra_states.local_pos_sp.z;
                ];
            obj.vel_deriv = [ ...
                extra_states.vel_deriv.lp_state;
                extra_states.vel_deriv.in_prev;
                extra_states.vel_deriv.f_cut;
                extra_states.vel_deriv.dt;
                ];
    
            % Char arrays
            intentions = { 'brake'; 'direction_change'; 'acceleration'; 'deceleration' };
            obj.user_intention_xy = find( strcmp( extra_states.user_intention_xy, ...
                intentions ) );
            obj.user_intention_z  = find( strcmp( extra_states.user_intention_z, ...
                intentions ) );
        end
    end
    
    %% Static functions
    methods(Static)
        % Static secondary functions
        function throttle = throttle_curve( ctl, ctr )
        %THROTTLE_CURVE Piecewise linear mapping
        %   0:ctr -> 0:0.5 and ctr:1 -> 0.5:1.
        %   Written: 2021/03/05, J.X.J. Bannwarth
            if (ctl < 0.5) 
                throttle = 2 * ctl * ctr;
            else 
                throttle = ctr + 2 * (ctl - 0.5) * (1.0 - ctr);
            end
        end
    end
end

% Helper functions
%% [1] Angle/orientation functions
function quat = DcmToQuat( dcm )
%DCMTOQUAT Convert rotation matrix to quaternion
%   Written: 2017/08/07, J.X.J. Bannwarth
    quat = zeros(4,1);
    tr = dcm(1,1) + dcm(2,2) + dcm(3,3);
    if (tr > 0.0)
        s = sqrt(tr + 1.0);
        quat(1) = s * 0.5;
        s = 0.5 / s;
        quat(2) = (dcm(3,2) - dcm(2,3) ) * s;
        quat(3) = (dcm(1,3) - dcm(3,1) ) * s;
        quat(4) = (dcm(2,1) - dcm(1,2) ) * s;
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
        quat(dcm_i + 1) = s * 0.5;
        s = 0.5 / s;
        quat(dcm_j + 1) = ( dcm(dcm_i,dcm_j) + dcm(dcm_j,dcm_i) ) * s;
        quat(dcm_k + 1) = ( dcm(dcm_k,dcm_i) + dcm(dcm_i,dcm_k) ) * s;
        quat(1) = ( dcm(dcm_k,dcm_j) - dcm(dcm_j,dcm_k) ) * s;
    end
end

function euler = QuatToEuler( quat )
%QUATTOEULER Convert quaternion to Euler angles
%   Written: 2021/03/05, J.X.J. Bannwarth
    quat_tmp = ones(4,1).*quat;
    euler = DcmToEuler( QuatToDcm( quat_tmp ) );
end

function euler = DcmToEuler( dcm )
%DCMTOEULER Convert DCM to Euler angles
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Euler.hpp
%   Written: 2021/03/05, J.X.J. Bannwarth
    phi_val   = atan2( dcm(3,2), dcm(3,3) );
    theta_val = asin( -dcm(3,1) );
    psi_val   = atan2( dcm(2,1), dcm(1,1) );
    if abs( theta_val - pi/2 ) < 1e-3
        phi_val = 0;
        psi_val = atan2( dcm(2,3), dcm(1,3) );
    elseif abs( theta_val + pi /2 ) < 1e-3
        phi_val = 0;
        psi_val = atan2( -dcm(2,3), -dcm(1,3) );
    end
    
    euler = [ phi_val; theta_val; psi_val ];
end

function dcm = QuatToDcm( quat )
%QUATTODCM Convert quaternion to DCM
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Dcm.hpp
%   Written: 2021/03/05, J.X.J. Bannwarth
    a = quat(1);
    b = quat(2);
    c = quat(3);
    d = quat(4);
    aa = a * a;
    ab = a * b;
    ac = a * c;
    ad = a * d;
    bb = b * b;
    bc = b * c;
    bd = b * d;
    cc = c * c;
    cd = c * d;
    dd = d * d;
    
    dcm = zeros(3, 3);
    dcm(1, 1) = aa + bb - cc - dd;
    dcm(1, 2) = 2 * (bc - ad);
    dcm(1, 3) = 2 * (ac + bd);
    dcm(2, 1) = 2 * (bc + ad);
    dcm(2, 2) = aa - bb + cc - dd;
    dcm(2, 3) = 2 * (cd - ab);
    dcm(3, 1) = 2 * (bd - ac);
    dcm(3, 2) = 2 * (ab + cd);
    dcm(3, 3) = aa - bb - cc + dd;
end

function dcmZ = QuatToDcmZ( quat )
%QUATTODCMZ Convert quaternion to corresponding body z-axis
%	Last orthogonal unit basis vector
%   == last column of the equivalent rotation matrix
%   but calculated more efficiently than a full conversion
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Dcm.hpp
%   Written: 2021/03/05, J.X.J. Bannwarth
    
    a = quat(1);
    b = quat(2);
    c = quat(3);
    d = quat(4);
    dcmZ = [ 2 * (a * c + b * d);
        2 * (c * d - a * b);
        a * a - b * b - c * c + d * d ];
end

function dcm = EulerToDcm( eul )
%EULERTODCM Convert Euler angles to a DCM
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Dcm.hpp
%   Written: 2021/03/05, J.X.J. Bannwarth
    phi   = eul(1);
    theta = eul(2);
    psi   = eul(3);
    sinPhi = sin(phi);
    cosPhi = cos(phi);
    cosThe = cos(theta);
    sinThe = sin(theta);
    sinPsi = sin(psi);
    cosPsi = cos(psi);
    
    dcm = zeros(3,3);
    dcm(1,1) = cosThe * cosPsi;
    dcm(1,2) = (sinPhi * sinThe * cosPsi) - (cosPhi * sinPsi);
    dcm(1,3) = (cosPhi * sinThe * cosPsi) + (sinPhi * sinPsi);
    dcm(2,1) = cosThe * sinPsi;
    dcm(2,2) = (sinPhi * sinThe * sinPsi) + (cosPhi * cosPsi);
    dcm(2,3) = (cosPhi * sinThe * sinPsi) - (sinPhi * cosPsi);
    dcm(3,1) = -sinThe;
    dcm(3,2) = sinPhi * cosThe;
    dcm(3,3) = cosPhi * cosThe;
end

function quat = EulerToQuat( eul )
%EULERTOQUAT Convert Euler angles to a quaternion
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Dcm.hpp
%   Written:       J.X.J. Bannwarth, 2019/01/15
%   Last modified: J.X.J. Bannwarth, 2019/01/22
    roll  = eul(1);
    pitch = eul(2);
    yaw   = eul(3);

    cosPhi_2   = cos(roll / 2.0);
    sinPhi_2   = sin(roll / 2.0);
    cosTheta_2 = cos(pitch / 2.0);
    sinTheta_2 = sin(pitch / 2.0);
    cosPsi_2   = cos(yaw / 2.0);
    sinPsi_2   = sin(yaw / 2.0);

    quat = [ cosPhi_2 .* cosTheta_2 .* cosPsi_2 + sinPhi_2 .* sinTheta_2 .* sinPsi_2;
                sinPhi_2 .* cosTheta_2 .* cosPsi_2 - cosPhi_2 .* sinTheta_2 .* sinPsi_2;
                cosPhi_2 .* sinTheta_2 .* cosPsi_2 + sinPhi_2 .* cosTheta_2 .* sinPsi_2;
                cosPhi_2 .* cosTheta_2 .* sinPsi_2 - sinPhi_2 .* sinTheta_2 .* cosPsi_2 ];
end

function quat = AxisAngleToQuat( axisAngle )
%AXISANGLETOQUAT Convert axis angle representation to quaternion
%   axisAngle = [x; y; z]*angle where [x; y; z] is a unit vector
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Quaternion.hpp
%   Written: 2021/03/05, J.X.J. Bannwarth
    
    angle = norm(axisAngle);
    axis = axisAngle / angle;
    quat = zeros(4,1);
    if (angle < 1e-10)
        quat(1) = 1.0;
        % quat(1) = quat(2) = quat(3) = 0;
    else
        magnitude = sin(angle / 2.0);
        quat(1) = cos(angle / 2.0);
        quat(2) = axis(1) * magnitude;
        quat(3) = axis(2) * magnitude;
        quat(4) = axis(3) * magnitude;
    end
end

function quat = VecsToQuat( src, dst )
%VECSTOQUAT Quaternion from two vectors
%   Generates shortest rotation from source to destination vector
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Quaternion.hpp
%   Written: 2021/03/05, J.X.J. Bannwarth
    cr = cross(src, dst);
    dt = dot(src, dst);
    quat = zeros(4,1);
    locEps = 1e-5; % Don't use eps as it's a reserved variable in MATLAB
    if (norm(cr) < locEps && dt < 0)
        % Handle corner cases with 180 degree rotations
        % If the two vectors are parallel, cross product is zero
        % If they point opposite, the dot product is negative
        cr = abs(src);
        if (cr(1) < cr(2))
            if (cr(1) < cr(3))
                cr = [1; 0; 0];
            else
                cr = [0; 0; 1];
            end
        else
            if (cr(2) < cr(3))
                cr = [0; 1; 0];
            else
                cr = [0; 0; 1];
            end
        end
        quat(1) = 0;
        cr = cross(src, cr);
    else
        % Normal case, do half-way quaternion solution
        quat(1) = dt + sqrt( norm(src)^2 * norm(dst)^2 );
    end
    quat(2) = cr(1);
    quat(3) = cr(2);
    quat(4) = cr(3);
    quat = normalize( quat );
end

function quatInv = InvertQuat( quat )
%INVERTQUAT Invert quaternion
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Quaternion.hpp
%   Written: 2021/03/05, J.X.J. Bannwarth
    normSq = dot( quat, quat );
    quatInv = ( quat .* [1;-1;-1;-1] ./ normSq );
end

function quatOut = QuatMult( q1, q2 )
%QUATMULT Multiply quaternion q1 by quaternion q2
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/Quaternion.hpp
%   Written by:    J.X.J. Bannwarth, 2017/03/28
%   Last modified: J.X.J. Bannwarth, 2019/01/15
    quatOut = [ q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3) - q1(4) * q2(4) ;
        q1(1) * q2(2) + q1(2) * q2(1) + q1(3) * q2(4) - q1(4) * q2(3) ;
        q1(1) * q2(3) - q1(2) * q2(4) + q1(3) * q2(1) + q1(4) * q2(2) ;
        q1(1) * q2(4) + q1(2) * q2(3) - q1(3) * q2(2) + q1(4) * q2(1) ];
end

function bearing = WrapPi( bearing )
%WRAPPI Wrap value to +-pi
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Matrix/blob/master/matrix/helper_functions.hpp
%   Written by:    J.X.J. Bannwarth, 2019/01/17
%   Last modified: J.X.J. Bannwarth, 2019/01/17

    % value is inf or NaN
    if (~isfinite(bearing))
        bearing = nan;
        return
    end
    
    c = 0;
    
    while (bearing >= pi)
        bearing = bearing - 2*pi;
        
        if (c > 100)
            bearing = nan;
            return
        end
        c = c + 1;
    end
    
    c = 0;
    
    while (bearing < -pi)
        bearing = bearing + 2*pi;
        
        if (c > 100)
            bearing = nan;
            return
        end
        c = c + 1;
    end
end

%% [2] Math functions
function out = normalize( in )
%NORMALIZE Normalise vector
%   Written: 2021/03/05, J.X.J. Bannwarth
    out = in ./ norm(in);
end

function y = constrain( x, x_min, x_max )
%CONSTRAIN Constrain value between minimum and maximum
%   Written: 2021/03/05, J.X.J. Bannwarth
    y = max( min(x, x_max), x_min );
end

function out = sign_no_zero( in )
%SIGN_NO_ZERO Signum function that returns 1 at in=0
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/Functions.hpp
%   Written: 2019/01/15, J.X.J. Bannwarth
    out = ( 0 <= in ) - ( in < 0 );
end

function out = super_expo( value, e, g )
%SUPER_EXPO So called SuperExpo function implementation
%   It is a 1/(1-x) function to further shape the rc input curve intuitively.
%   It is enhanced compared to other implementations to keep the scale between
%   [-1,1].
%   Input:
%       - value [-1,1] input value to function
%       - e [0,1] function parameter to set ratio between linear and cubic shape
%                 (see expo)
%       - g [0,1) function parameter to set SuperExpo shape
%           0 - pure expo function
%           0.99 - very strong bent curve, stays zero until maximum stick input
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/Functions.hpp
%   Written: 2019/01/17, J.X.J. Bannwarth
    x = constrain(value, - 1, 1);
    gc = constrain(g, 0, 0.99);
    out = expo(x, e) * (1 - gc) / (1 - abs(x) * gc);
end

function out = expo( value, e )
%EXPO So called exponential curve function implementation
%   It is essentially a linear combination between a linear and a cubic function.
%   Input:
%       - value [-1,1] input value to function
%       - e [0,1] function parameter to set ratio between linear and cubic shape
%           0 - pure linear function
%           1 - pure cubic function
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/PX4-Autopilot/blob/v1.8.2/src/lib/mathlib/math/Functions.hpp
%   Written: 2019/01/17, J.X.J. Bannwarth
    x = constrain( value, -1, 1 );
    ec = constrain( e, 0, 1 );
    out = (1 - ec) * x + ec * x * x * x;
end

function out = expo_deadzone( value, e, dz  )
%EXPO_DEADZONE Apply deadzone and exponential function one after the other
%   Written: 2021/03/06, J.X.J. Bannwarth
    out = expo( deadzone( value, dz ), e );
end

function out = deadzone( value, dz )
%DEADZONE Function being linear and continuous outside of the deadzone
%   1                ------
%                  /
%               --
%             /
%   -1 ------
%          -1 -dz +dz 1
%   Input:
%       - value [-1,1] input value to function
%       - dz [0,1) ratio between deazone and complete span
%           0    - no deadzone, linear -1 to 1
%           0.5  - deadzone is half of the span [-0.5,0.5]
%           0.99 - almost entire span is deadzone
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/PX4-Autopilot/blob/v1.8.2/src/lib/mathlib/math/Functions.hpp
%   Written: 2021/03/07, J.X.J. Bannwarth
    x   = constrain( value, -1, 1    );
    dzc = constrain( dz   , 0 , 0.99 );

    % Rescale the input such that we get a piecewise linear function that will be
    % continuous with applied deadzone
    out = (x - sign(x) * dzc) / (1 - dzc);

    % Apply the deadzone (values zero around the middle)
    if abs(x) <= dzc
        out = 0;
    end
end

function out = gradual( value, x_low, x_high, y_low, y_high )
%GRADUAL Constant, linear, constant function with the two corner points as params
%   y_high          -------
%                  /
%                 /
%                /
%   y_low -------
%           x_low   x_high
%   Based on code from PX4 Firmware:
%       https://github.com/PX4/PX4-Autopilot/blob/v1.8.2/src/lib/mathlib/math/Functions.hpp
%   Written: 2021/03/07, J.X.J. Bannwarth
    if value < x_low
        out = y_low;
    elseif value > x_high
        out = y_high;
    else
        % Linear function between the two points
        a = (y_high - y_low) / (x_high - x_low);
        b = y_low - a * x_low;
        out = a * value + b;
    end
end

%% [3] Projection functions
function [ out, lat, lon ] = map_projection_reproject( ref, x, y )
%MAP_PROJECTION_REPROJECT
%   Based on code from PX4-ECL library:
%       https://github.com/PX4/PX4-ECL/blob/master/geo/geo.cpp
%   Written: 2021/03/07, J.X.J. Bannwarth
    radius_earth = 6371000;
    
    if ~map_projection_initialized( ref )
        out = -1;
        return
    end

    x_rad = x / radius_earth;
    y_rad = y / radius_earth;
    c = sqrt(x_rad * x_rad + y_rad * y_rad);
    
    if abs(c) > eps
        sin_c = sin(c);
        cos_c = cos(c);

        lat_rad = asin(cos_c * ref.sin_lat + (x_rad * sin_c * ref.cos_lat) / c);
        lon_rad = ref.lon_rad + ...
            atan2( y_rad*sin_c, c*ref.cos_lat*cos_c - x_rad*ref.sin_lat*sin_c );
    else
        lat_rad = ref.lat_rad;
        lon_rad = ref.lon_rad;
    end

    lat = rad2deg( lat_rad );
    lon = rad2deg( lon_rad );
    
    out = 0;
end

function [ out, x, y ] = map_projection_project( ref, lat, lon )
%MAP_PROJECTION_PROJECT
%   Based on code from PX4-ECL library:
%       https://github.com/PX4/PX4-ECL/blob/master/geo/geo.cpp
%   Written: 2021/03/07, J.X.J. Bannwarth
    radius_earth = 6371000;
    
    if ~map_projection_initialized( ref )
        out = -1;
        return
    end
    
    lat_rad = deg2rad( lat );
    lon_rad = deg2rad( lon );

    sin_lat = sin(lat_rad);
    cos_lat = cos(lat_rad);
    cos_d_lon = cos(lon_rad - ref.lon_rad);

    arg = constrain( ref.sin_lat*sin_lat + ref.cos_lat*cos_lat*cos_d_lon, -1, 1 );
    c = acos( arg );
    
    if abs(c) > eps
        k = c / sin(c);
    else
        k = 1.0;
    end

    x = k * (ref.cos_lat*sin_lat - ref.sin_lat*cos_lat*cos_d_lon) * radius_earth;
    y = k * cos_lat * sin(lon_rad - ref.lon_rad) * radius_earth;

    out = 0;
end

function [ out, ref ] = map_projection_init_timestamped( ref, lat_0, lon_0, timestamp )
%MAP_PROJECTION_INIT
%  lat_0, lon_0 are expected to be in correct format:
%       -> 47.1234567 and not 471234567
%       https://github.com/PX4/PX4-ECL/blob/master/geo/geo.cpp
%   Written: 2021/03/07, J.X.J. Bannwarth
    ref.lat_rad = deg2rad(lat_0);
    ref.lon_rad = deg2rad(lon_0);
    ref.sin_lat = sin(ref.lat_rad);
    ref.cos_lat = cos(ref.lat_rad);

    ref.timestamp = timestamp;
    ref.init_done = 1;
    out = 0;
end

%% [4] BlockDerivative.cpp - Derivative functions
function [ block, out ] = BlockDerivativeUpdate( block, in )
%BLOCKDERIVATIVEUPDATE Update the state and get current derivative
%
%   This call updates the state and gets the current derivative. As the
%   derivative is only valid on the second call to update, it will
%   return no change (0) on the first. To get a closer estimate of the
%   derivative on the first call, call setU() one time step before using
%   the return value of update().
%
%   Combined with BlockLowPass to simplify code
%
%   Input:  -  in: the variable to calculate the derivative of
%   Output: - out: the current derivative
    if sum( ~isfinite( block.lp_state ) ) ~= 0
        block.lp_state = in;
    end

    delta_in = in - block.in_prev;

    b = 2 * pi * block.f_cut * block.dt;
    a = b / (1 + b);

    block.lp_state = a * delta_in + (1 - a)*block.lp_state;

    out = block.lp_state / block.dt;

    block.in_prev = in;
end

% [5] hysteresis.cpp - Hysteresis functions
function hysteresis = hysteresis_set_state_and_update( hysteresis, new_state, now_us )
%HYSTERESIS_SET_STATE_AND_UPDATE Set hysteresis state and update
%   Written: 2021/03/08, J.X.J. Bannwarth
    % Set state
    if new_state ~= hysteresis.state
        if new_state ~= hysteresis.requested_state
            hysteresis.requested_state = new_state;
            hysteresis.last_time_to_change_state = now_us;
        end
    else
        hysteresis.requested_state = hysteresis.state;
    end

    % Update
    if hysteresis.requested_state ~= hysteresis.state
        elapsed = now_us - hysteresis.last_time_to_change_state;

        if hysteresis.state && hysteresis.requested_state 
            % 1 -> 0
            if (elapsed >= hysteresis.time_from_1_us) 
                hysteresis.state = 0;
            end
        elseif (~hysteresis.state && hysteresis.requested_state) 
            % 0 -> 1
            if (elapsed >= hysteresis.time_from_0_us) 
                hysteresis.state = 1;
            end
        end
    end
end