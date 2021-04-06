classdef MulticopterPositionControlHorizontal < matlab.System & matlab.system.mixin.CustomIcon & matlab.system.mixin.SampleTime & matlab.system.mixin.Propagates
%MULTICOPTERPOSITIONCONTROLHORIZONTAL PX4 Firmware position controller (v1.82)
%   With horizontal thrust capability
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
        use_hor_thrust         (1,1) {mustBeGreaterThanOrEqual(use_hor_thrust         ,    0), mustBeLessThanOrEqual(use_hor_thrust         ,   1), mustBeInteger(use_hor_thrust)} = 1; % Horizontal controller [0-1]
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
        
        % Horizontal thrust controller
        hor_A                         (38,38) = zeros(38, 38);
        hor_B                         (38, 9) = zeros(38, 9 );
        hor_C                         (5 ,38) = zeros(5 , 38);
        hor_D                         (5 , 9) = zeros(5 , 9 );
        hor_op                        (5 , 1) = zeros(5 , 1 );
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
        
        % Horizontal thrust controller
        hor_state
        pos_err_int
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
            
            % Horizontal thrust controller
            obj.hor_A = [0.595462485377179,-4.53706638806801e-13,-0.000769304232201206,-0.103903436137142,-3.30819152553931e-12,0.0593937506355471,-0.259210822801781,-6.34283923592612e-12,0.157950082536683,1.55432741614393,-7.32908788491918e-12,-0.282368331999388,-6.33514169719861e-12,0.0495950325786188,-2.07692847235342e-12,-1.78523067753805e-14,0.00470409278260486,-3.81561937776373e-14,3.76623328283136e-06,-4.82157766417267e-06,9.64021247908613e-06,5.83678337849865e-07,5.83678339101114e-07,9.64021247922871e-06,-4.82157766425312e-06,3.76623328273369e-06,0.203879253466434,21.9836784178003,0.418399313134748,7.12356383564259e-13,-4.20194134909190e-09,2.16515904754374e-13,7.42474163708608e-14,-0.405503683667209,-3.15121748214981e-12,0.443917623503101,0.556046446466908,1.14192203668500e-11;-1.29694816194417e-12,0.592276048523296,5.09890696800932e-13,6.27437887154727e-12,0.0485411046693186,-3.29451491941200e-12,1.40306761719913e-11,0.111824093682850,-8.02288607214810e-12,-1.51025159088521e-12,-1.28051774339414,1.92268298840979e-11,-0.0225510399919514,-1.10703713948980e-11,-0.00878977746757981,0.00371455501509699,-8.66823962316403e-13,0.000134889578549294,6.81283016522120e-06,-1.66327237299343e-06,-1.75559790637057e-06,8.19851838481223e-06,-8.19851838449134e-06,1.75559791059381e-06,1.66327235748659e-06,-6.81283016598158e-06,-3.22367725407221e-11,6.66007161749933e-09,-1.30858039389906e-11,0.149955045878815,4.84689748268082,0.191679748076250,-0.00204732013865706,1.72556695533536e-11,0.154149982972634,-2.51758384506090e-11,5.75018893469713e-12,-0.153008886832281;0.0158984652156047,-1.84854815145175e-13,0.531559918496937,0.143442110744365,2.32097409509399e-13,-0.665521221368966,0.405981802693874,3.77431446234148e-13,-1.67024031633737,-2.00798222023529,-3.23058144622710e-11,7.46439137429281,-1.18906496148099e-12,0.152350405186021,3.00162240868139e-12,-1.16668643584248e-13,0.00794938286856937,-1.50837447529661e-12,-4.40695825194300e-05,-5.14128495481978e-05,-6.44848757329297e-05,-7.63684769347453e-05,-7.63684769277870e-05,-6.44848757369987e-05,-5.14128495495029e-05,-4.40695825203201e-05,-0.417115826690478,391.351472404328,0.218547435030862,4.92083101018540e-11,-1.76505682321618e-08,-5.48498318039577e-12,1.24527726902329e-12,0.311150938885990,-1.16503811926010e-11,-4.04746629647203,-1.19852960234560,3.69384981657055e-11;3.91657350997505e-05,-6.91592887136158e-16,-0.000181731885393452,0.982707925111177,-3.03820457866563e-15,0.000129458807219782,0.00973254476164960,-4.98846676142505e-15,0.000246502204109644,-0.00173936494062697,-2.57579684487487e-14,0.00340004027048994,-2.84688766665195e-15,-7.10840993966185e-05,5.79720737125220e-16,4.09148247464361e-16,-6.69111121781173e-06,6.13406591753000e-16,-4.29529546303889e-08,-3.66517253434699e-08,-5.06416178706522e-08,-4.60829579909771e-08,-4.60829579935131e-08,-5.06416178725115e-08,-3.66517253437229e-08,-4.29529546309824e-08,-0.000244886696906477,0.0580073607782544,-2.54906578919236e-05,1.90114917168621e-14,-7.26759379330794e-12,-1.57937717783404e-15,-2.55439230145209e-15,-5.46047839621651e-05,-1.79834113229638e-15,0.000373414633750093,9.52076005205257e-05,-4.15017552880761e-15;7.10374838713563e-16,-1.04272278015676e-05,-2.91537604736226e-15,1.00809300750474e-15,0.982729794823445,1.45544324796059e-15,2.49861343346204e-15,0.00977510843745187,2.36641898394335e-15,-4.71777207428384e-14,-0.000579479711765266,5.70249119722145e-14,-4.51164835020999e-05,-3.21144315005672e-15,-9.97961194655444e-06,9.53757076441079e-07,-4.63910514197657e-16,-1.54606298111831e-07,3.63311941282076e-09,-3.01060801662937e-10,-3.05602808138872e-10,4.31575128056191e-09,-4.31575128188372e-09,3.05602807110162e-10,3.01060798117394e-10,-3.63311941446141e-09,-1.96831196056868e-14,5.02150786410433e-12,-3.50132106332450e-15,6.57141792293203e-05,-0.0180982139273282,5.00291884854027e-06,7.79253559326635e-07,3.05367436058979e-15,5.21598625719779e-06,-4.63827541640493e-17,2.27435907878966e-15,1.11785601212742e-05;-0.000101149168318660,2.09035877783618e-15,0.000853080123419926,8.94474026715984e-05,7.53085762390648e-15,0.982093538114418,0.000202344470577763,1.27417708851737e-14,0.00855867933911817,0.00465200063727754,6.74069054876615e-14,-0.0167014850982662,6.32809797538289e-15,0.000605713989585104,-3.49096408809955e-15,-2.07043141843887e-15,2.61961073468932e-05,-2.68772028639613e-15,2.04673059978680e-07,2.00170165128524e-07,2.17484841037856e-07,2.22525852194538e-07,2.22525852204320e-07,2.17484841046578e-07,2.00170165129905e-07,2.04673059980567e-07,0.000753740872889316,-0.154117759808554,9.10939373210543e-05,-9.18174360920016e-14,3.50033909543553e-11,8.37785803554698e-15,1.10185051473582e-14,0.000303164850954460,1.02640417085527e-14,-0.00188403760762569,-0.000419755452772858,2.00836550305183e-14;7.32094217676296e-05,-1.59831143715239e-15,-0.000442325504260017,-0.00994211401880523,-6.25070591706363e-15,0.000321586317249404,0.982586376749943,-1.03509559584196e-14,0.000615833277036541,-0.00329402935619823,-6.08435633633093e-14,0.00847372014976050,-6.01760378735789e-15,-0.000245767987440805,1.40227127808519e-15,1.05642003217932e-15,-1.48915978763053e-05,1.43840161496657e-15,-1.05359417299904e-07,-9.67289858996164e-08,-1.17847123442957e-07,-1.13822799817346e-07,-1.13822799823136e-07,-1.17847123447468e-07,-9.67289859002711e-08,-1.05359417301264e-07,-0.000490187633465723,0.109580817823450,-5.44037843247611e-05,4.83743582630510e-14,-1.83155751713466e-11,-3.99436090251188e-15,-5.94047771447074e-15,-0.000145432269954129,-4.75227960931042e-15,0.000943950277165227,0.000224465021421954,-1.00219542223522e-14;1.98437156956996e-15,1.39566170715103e-06,-5.44572231635700e-15,-1.78098982412441e-15,-0.00989921494742744,3.73803275253862e-15,-3.14636845083608e-15,0.982694824376552,7.26933523408375e-15,-8.44456207456668e-14,7.75620960021716e-05,9.41886596891647e-14,6.03874295100043e-06,8.50072940378762e-16,1.33574930087737e-06,-1.27658305241305e-07,-2.04575393517366e-16,2.06937159200281e-08,-4.86285113602867e-10,4.02963310135684e-11,4.09042677478803e-11,-5.77653900852798e-10,5.77653898193303e-10,-4.09042712648112e-11,-4.02963323341711e-11,4.86285111124280e-10,-9.16216935375162e-15,2.36489948001212e-12,-6.33744343070443e-16,-8.79569961571511e-06,0.00242240647532435,-6.69629780203147e-07,-1.04301389938927e-07,-1.69683186175530e-15,-6.98148388309175e-07,1.06943568145264e-14,3.02288902510025e-15,-1.49622590004243e-06;-0.000208275438428607,4.20103565248071e-15,0.00172142232685483,0.000180357478107116,1.52849576213416e-14,-0.0111734713242040,0.000409489040448604,2.58502653130944e-14,0.980236457363207,0.00956424870569242,1.34529214442358e-13,-0.0336644224670989,1.27979806036362e-14,0.00120855624924867,-7.07770237553726e-15,-4.16346329625200e-15,5.31235759505886e-05,-5.43430326539232e-15,4.12854409633802e-07,4.02504424539822e-07,4.39880311903172e-07,4.48720150421198e-07,4.48720150440982e-07,4.39880311920776e-07,4.02504424542605e-07,4.12854409637602e-07,0.00154091166297598,-0.316948024084178,0.000185256967727263,-1.84673024225465e-13,7.04574990546538e-11,1.68931693315818e-14,2.22887609288136e-14,0.000609393090900737,2.06575680295113e-14,-0.00379517197037171,-0.000848389467248920,4.05770464500745e-14;-0.0334249903145902,5.83889682556625e-14,-0.000746147493790350,-0.0173593063388105,1.06475761113185e-12,0.0140976897156276,-0.0512755431641465,1.97864364823646e-12,0.0289885650856212,0.419467990725027,-1.06594835008141e-12,0.314936413642484,1.07905204205173e-12,-0.110767043480120,-7.58010752227504e-14,9.98671768888154e-14,0.00208961447615421,-8.13917204790330e-14,-1.41281311205092e-06,-1.15705912152681e-05,8.00988135897929e-06,-2.70898066932293e-06,-2.70898066880955e-06,8.00988135899811e-06,-1.15705912153245e-05,-1.41281311197272e-06,0.159731986044976,-47.7552378710913,0.0114952446895215,3.69649997816372e-12,-1.09912174002460e-09,-2.02085335395006e-13,4.14860371691939e-13,-0.0192466658171864,-5.43217463703465e-13,0.0548009995170183,-0.0106410895077043,1.76160277983500e-13;1.16398819057451e-12,0.0288314709090979,-8.82326121822473e-13,-5.11593857506425e-12,-0.0163235834434227,2.11974654722594e-12,-1.10990896519123e-11,-0.0479599677122433,5.22973852926786e-12,-6.15468936830531e-12,0.602271741839285,-1.11259924758427e-12,0.124747881830685,9.33122460926518e-12,0.0275938050838320,-0.00263715533285110,8.77747487959282e-13,0.000427489172693096,-1.00456399989510e-05,8.32438486589835e-07,8.44997214141603e-07,-1.19331292925647e-05,1.19331292918640e-05,-8.44997216869160e-07,-8.32438479577171e-07,1.00456399993482e-05,3.66261089244446e-11,-9.31403503622345e-09,8.01204769080434e-12,-0.181700878010986,50.0418844090036,-0.0138331598759722,-0.00215464999461883,-1.04505825776289e-11,-0.0144222950623602,1.66002999414450e-11,-1.20886859124631e-12,-0.0309089181780095;-0.00779493457920172,8.02669024626708e-14,-0.0180053513563963,-0.000199262894865098,2.34864001764842e-13,-0.0144805047628883,-0.00243149700574153,3.40965615715852e-13,-0.0323251392639128,0.323598152529095,4.20000547123030e-12,-0.558508373206832,6.48790108446113e-13,-0.0454469441700450,1.52288137480920e-13,6.30071237758004e-14,7.30253359828561e-05,3.34951804544216e-14,-4.68545323791986e-06,-7.60126355684420e-06,-2.16065979426708e-06,-5.44167864037451e-06,-5.44167864033813e-06,-2.16065979444957e-06,-7.60126355689588e-06,-4.68545323786179e-06,0.0316142010856064,-10.9366494098673,0.00150691808687644,2.33791621017370e-12,-8.70456632006322e-10,-2.85650312315308e-13,-1.14746173090490e-13,-0.0120227775784789,-4.28224140954927e-13,0.0555139533665905,0.00559781201285686,-4.80713104815257e-13;-6.52775022011189e-12,-0.00120757308323257,7.00597790515505e-12,3.25548006257237e-11,-0.00139547873338915,-1.87757505585290e-11,7.08558972342464e-11,-0.00257361837370151,-4.56749974779550e-11,9.25855367925485e-12,0.0473548618090504,6.75888562619249e-12,0.992170577217425,-5.40684954233364e-11,-0.00268013546914667,0.0168096446013524,-6.32653707934326e-12,-0.00363587876785944,-1.63565984921719e-06,-4.11913037279263e-06,-4.04928800306294e-06,-2.14331197305252e-06,2.14331197852928e-06,4.04928801788627e-06,4.11913034132961e-06,1.63565984737486e-06,-2.90258530095119e-10,8.33589120604418e-08,-3.83340759810574e-11,-0.154180963921439,53.5068622513154,0.00669581697463133,-0.00136544918031912,5.50879903065323e-11,0.0120020525857756,-1.31124368140105e-10,-6.08720170902918e-12,-0.000323243564101524;0.00136955631885244,2.09059222222528e-13,-0.000951577374769012,-5.66801706716888e-05,2.26380497227498e-12,0.000856615413413320,5.18514517349906e-05,4.56883367976799e-12,0.00144524941932624,0.0453395869304263,4.03049247645212e-12,0.0785669746482249,5.38129436641703e-12,0.984128552339508,-1.31104932304423e-12,5.54057691318426e-13,0.0168437953854680,-8.13940278357418e-13,3.28123736636546e-06,6.01570720579175e-07,-1.86686355063410e-06,-5.24953664375350e-06,-5.24953664017608e-06,-1.86686355034460e-06,6.01570720955058e-07,3.28123736674647e-06,-0.149158178239542,52.5743671327821,0.00781442008268872,2.31874272956324e-11,-6.12023925847985e-09,2.46246374388001e-12,3.96072294707201e-12,-0.0140173873211728,2.52529362155636e-12,0.00140087068031239,-0.00115608280998718,3.73562309813766e-12;-1.39512802856165e-12,4.65919600398070e-05,9.21451542235804e-13,7.32890948121085e-12,0.000112129393231744,-1.79810384933483e-12,1.55996562011588e-11,0.000114789610591265,-4.46343807069164e-12,1.57169209074644e-12,-0.00347523675840474,-1.50909047885009e-12,-6.38665212661735e-06,-1.27305032275459e-11,0.996893173727405,6.53702864005654e-05,-1.16542623593366e-12,0.0191768007940802,-3.97698301064480e-07,4.90191944517305e-07,-4.07660203058712e-07,5.29443185525391e-07,-5.29443185984630e-07,4.07660199574255e-07,-4.90191959385682e-07,3.97698300525274e-07,-4.95205809973793e-11,1.19534440026874e-08,-1.11712299816282e-11,0.00312808951084826,-1.04114145359063,-5.41811375478944e-05,0.00580768032453376,1.72849194039793e-11,-0.000143009897373433,-2.69752896766170e-11,3.43556404711464e-12,8.99059119607765e-05;-7.08340974389085e-10,-0.195232828415628,6.26507819048837e-10,3.38842731629587e-09,-0.0742529634371539,-1.85821541206616e-09,7.35184834885340e-09,-0.168122736696998,-4.55591507152307e-09,2.03102916889709e-09,0.480679270476361,2.82205804786810e-09,-1.10915827323882,-5.86305184296028e-09,-0.399532784628019,0.689107628873400,-6.57780247221651e-10,0.00186781358855754,-0.000144908626164213,-0.000404718812085056,-0.000414930250821357,-0.000173017171146842,0.000173017171632974,0.000414930252214387,0.000404718808566925,0.000144908625920798,-2.99741810758877e-08,8.55284331425982e-06,-4.06515337587187e-09,-14.8833311674574,5199.97367031518,0.704706207526015,-0.0198961950908486,5.81215992600409e-09,1.23517178031424,-1.34051669479704e-08,-5.05784262084663e-10,0.0501908780473144;0.237625619169872,1.80819769817898e-11,0.129786895802512,0.0758028916760551,2.25077658171951e-10,-0.0444069736438124,0.170423953295374,4.56544676246894e-10,-0.141453161609334,0.351591124189514,2.49796292319735e-10,2.31160057130748,5.22063410382903e-10,-1.00855700604138,-1.34737088985873e-10,5.47750342975904e-11,0.683260069376584,-8.18311977664599e-11,0.000386781069725784,0.000156238664135677,-0.000160496857507436,-0.000456723081315649,-0.000456723080959917,-0.000160496857476103,0.000156238664174021,0.000386781069761888,-15.3265081778329,5398.76652909379,0.761480760939720,2.30062048701618e-09,-6.04195079516519e-07,2.50705119873752e-10,3.97599868673201e-10,-1.24967437701959,2.58839700443522e-10,-0.558672444723528,-0.184463485968610,3.81495015098631e-10;-1.44945702664221e-10,0.00985030962689112,7.23640631349808e-11,7.12991192045911e-10,0.00412413062803459,-1.65079244105594e-10,1.51380714012887e-09,0.00916312020570114,-4.16000801079223e-10,4.94772460183930e-10,-0.0478512377978430,2.95205419257938e-10,0.0222057393201437,-1.29461433763415e-09,-0.299903128959791,0.00593380656959828,-1.14131217216172e-10,0.882396231274726,-4.08751282916953e-05,4.82676400539973e-05,-3.98596333925456e-05,4.97839005288824e-05,-4.97839005849888e-05,3.98596330459504e-05,-4.82676415275780e-05,4.08751282294811e-05,-4.82036979915814e-09,1.16005449550708e-06,-1.09339793992045e-09,0.273783997681100,-93.0351014560492,-0.00785008930738234,0.569664282485132,1.68247887505403e-09,-0.0166769942488688,-2.58876573429494e-09,3.42420139592002e-10,0.00316753944076953;14.0784993089939,40.1412287715236,-115.148803882347,-289.489015893950,459.192616426531,334.026842365134,-639.786087299357,929.659299415541,814.880786110448,-192.632991097388,586.055090922843,-1016.55178053500,1073.52367306372,-579.688125901160,701.813466641570,113.601356810481,-238.882015830073,196.634345681022,0.224729068315498,-0.0500403094971825,-0.0277013690057933,0.0372029746903216,0.0822951186950742,0.0483560750063933,0.00526130979872431,-0.0341525433972016,-11411.8833075312,4124094.61917691,975.848175629869,4774.61447679481,-1267638.14959312,496.878374353887,-947.742867041476,-1569.77871935310,504.437265923693,1362.27307540411,176.076438881050,772.167089050171;-107.240015397087,-44.2663109119118,84.0027107475432,515.165839039158,-188.253012784039,-231.582296691763,1110.93258791143,-374.577511441548,-569.300544247204,271.551075850681,-15.9335309505544,259.336799601727,475.935259185866,-935.356875468715,-445.879336431725,231.695436023546,-106.023227166435,-192.349783698030,-0.0319821899361302,0.236063356946784,-0.0204122586211460,0.0149387691863419,0.0630929953089571,0.0814491158891699,0.0421963942052088,-0.00906735235996326,-4777.91661722885,1352508.46071880,-642.387730167181,11455.6074198308,-4230924.91292601,-967.339543943250,924.618151739679,937.181337421983,-1469.55520534593,-1987.89672978322,18.9964909831898,-453.905305468488;125.425723628757,-42.4860045784766,-171.483824754670,-627.775447972762,-186.312191100221,522.420652516600,-1387.11009311939,-370.644427795688,1270.17062244464,-230.885408067089,-47.7130285994936,-632.673552351770,476.633840649849,890.846313316135,574.451549511786,231.928713799168,101.047284967130,188.446791988807,-0.00994374078269870,-0.0392130185255402,0.208038007443607,-0.0569368692344036,-0.0294194820991898,0.0199365670544775,0.0582527785490996,0.0290955731013172,4803.09660792434,-1407797.57489761,671.158880228991,11463.8958220259,-4232118.32199135,-964.553285132760,-918.160001806518,-902.899692981783,-1466.37269244206,2678.43015612847,413.070936535446,-450.876821064879;4.71514118136897,38.0962024574629,26.5615961310946,172.034857723745,453.319815265442,-39.7107444278463,353.118739558053,917.743835090675,-105.425498164339,227.771303554305,609.306651482126,628.249493717165,1059.42531839956,533.699479330546,-320.862101579201,111.193302588857,231.894378164989,-184.231720420661,0.0311206130254584,-0.0217490102801187,-0.0482112485941536,0.205835555713776,-0.0309129433382683,0.0109911566911769,0.0523844139200161,0.0708295716366163,11338.6847933065,-4142077.07917316,-934.254318234405,4667.62490256936,-1235116.03349079,493.120482659649,895.358696506247,1583.70566406971,502.369813064031,-651.631356471827,256.370165697015,763.118818661011;4.71514124825247,-38.0962024544710,26.5615960741881,172.034857417467,-453.319815224521,-39.7107442606243,353.118738886232,-917.743835002401,-105.425497752207,227.771303353367,-609.306651429408,628.249493459946,-1059.42531827082,533.699479873087,320.862101537874,-111.193302573758,231.894378222533,184.231720397667,0.0708295716429707,0.0523844140584633,0.0109911565495406,-0.0309129434218332,0.205835555725527,-0.0482112487272691,-0.0217490101311707,0.0311206130408543,11338.6847958073,-4142077.07986061,-934.254317800154,-4667.62490192119,1235116.03331846,-493.120482582569,-895.358696394361,1583.70566352960,-502.369812979322,-651.631355425212,256.370165705636,-763.118818550129;125.425723849437,42.4860045851006,-171.483824953856,-627.775449042150,186.312191176878,522.420653081394,-1387.11009543392,370.644427937141,1270.17062382591,-230.885408670879,47.7130286827019,-632.673553008033,-476.633840559063,890.846315153823,-574.451549521460,-231.928713791738,101.047285172227,-188.446791995259,0.0290955731347018,0.0582527791539277,0.0199365668985755,-0.0294194821855878,-0.0569368692759861,0.208038007291584,-0.0392130179155848,-0.00994374074237428,4803.09661728188,-1407797.57755280,671.158881515007,-11463.8958217885,4232118.32192312,964.553285136937,918.160001842291,-902.899694870193,1466.37269242464,2678.43016041470,413.070936649938,450.876821091174;-107.240015233292,44.2663109053311,84.0027105807912,515.165838255298,188.253012710028,-231.582296194065,1110.93258620365,374.577511304573,-569.300543033563,271.551075404845,15.9335308758465,259.336798973026,-475.935259276203,-935.356874140901,445.879336440713,-231.695436030471,-106.023227006743,192.349783706222,-0.00906735233126014,0.0421963945201591,0.0814491156665930,0.0630929952597454,0.0149387690865611,-0.0204122588471343,0.236063357257181,-0.0319821899140681,-4777.91660979080,1352508.45852439,-642.387729319344,-11455.6074200826,4230924.91299696,967.339543936904,-924.618151779825,937.181336210271,1469.55520536112,-1987.89672653995,18.9964912355625,453.905305418572;14.0784994345596,-40.1412287745170,-115.148803973436,-289.489016494596,-459.192616465310,334.026842604068,-639.786088597132,-929.659299500454,814.880786701500,-192.632991460808,-586.055090967755,-1016.55178082290,-1073.52367319371,-579.688124834372,-701.813466600849,-113.601356825238,-238.882015725702,-196.634345656139,-0.0341525433702104,0.00526131024731058,0.0483560749415132,0.0822951187202892,0.0372029746133538,-0.0277013690789717,-0.0500403090587898,0.224729068334387,-11411.8833030482,4124094.61801207,975.848176514564,-4774.61447746442,1267638.14976974,-496.878374434506,947.742866924519,-1569.77872058699,-504.437266012348,1362.27307752730,176.076438752007,-772.167089186505;0.000352233973751635,2.54151439102829e-14,0.000282034297198793,0.000124605074348977,3.15924031276237e-13,-0.000126020556241462,0.000292978749724284,6.41123546477766e-13,-0.000317294370483566,-0.000243039532765249,3.49611880291667e-13,0.00102684616379648,7.32557195332399e-13,-0.00124726143140081,-1.89840096766290e-13,7.66546725980359e-14,0.00236641976763817,-1.15227794921428e-13,5.68512652606729e-07,2.53177350903343e-07,-2.07493275165954e-07,-6.13513041796009e-07,-6.13513041295619e-07,-2.07493275120920e-07,2.53177350957444e-07,5.68512652657481e-07,-0.740559323835464,-92.2414723098949,0.00107023664133011,3.21992833052033e-12,-8.44262010460708e-10,3.53522069003910e-13,5.59612922827311e-13,-0.00170350722132546,3.65483951536066e-13,-0.00105294710105998,-0.000298337691584239,5.38432097608961e-13;3.17036817897123e-06,2.53074634524846e-16,6.48494964681928e-07,8.68059996552488e-07,3.16530090601812e-15,1.37586064337606e-07,1.81664721613556e-06,6.41816043836250e-15,-5.67117675267014e-07,1.16150069732210e-05,3.48124581624380e-15,5.82504688909724e-05,7.34281472654031e-15,-1.60195043391307e-05,-1.88793263731596e-15,7.73312960201168e-16,2.36359243437553e-05,-1.14691150094414e-15,5.14161327534506e-09,1.81492000054626e-09,-2.47949554584046e-09,-6.74738343772414e-09,-6.74738343273306e-09,-2.47949554541203e-09,1.81492000108257e-09,5.14161327585139e-09,0.00259540474542168,0.0771067006558280,1.06809563752739e-05,3.24789453945503e-14,-8.54569790070836e-12,3.50881682449293e-15,5.57556149421141e-15,-1.81521000169532e-05,3.61741708187420e-15,-4.76625962136144e-06,-2.11492485054672e-06,5.33333635837285e-15;-0.000101922516418239,-1.06098383621391e-14,1.78545771013614e-05,0.000119145621501602,-1.11167740735188e-13,0.000135111867270913,0.000180748995820485,-2.25223862582196e-13,0.000249693299170696,-0.00271510223196960,-2.19788776934413e-13,-0.00463756736647505,-2.71742348232645e-13,-0.00611830721843210,7.25806778447242e-14,-2.89833334065518e-14,-0.000908938902898888,4.39531721856012e-14,-1.84253582844181e-07,-3.85208940762970e-08,1.02584579112578e-07,2.86846511402604e-07,2.86846511206148e-07,1.02584579097916e-07,-3.85208940957204e-08,-1.84253582864236e-07,0.00789592616684950,-3.09901484683426,0.998002020414073,-1.23838241897756e-12,3.37586866334624e-10,-1.27243204901722e-13,-2.10949016739476e-13,-0.00806185359015777,-1.18892762742227e-13,-0.000275254781744517,-0.000875193656936198,-2.43531088371613e-13;-9.98418782390835e-13,-0.000290509090918770,8.42627395315295e-13,4.75897678367523e-12,-9.67673241669639e-05,-2.58771343515724e-12,1.03216960447060e-11,-0.000209101921611104,-6.35914966879154e-12,2.94464197197511e-12,-0.000223446660174907,4.77662706296325e-12,-0.00162853548714491,-8.29382433326294e-12,-0.000576890096079229,0.00237495596850465,-9.25517581422911e-13,2.38476492128251e-06,-1.97985022448712e-07,-5.69164092476935e-07,-5.83519903909378e-07,-2.36423303584550e-07,2.36423304247057e-07,5.83519905852263e-07,5.69164087506775e-07,1.97985022087602e-07,-4.21396991900547e-11,1.20203471896829e-08,-5.71838851843164e-12,-0.739778426589070,-92.5739580019889,0.000997989620074271,-2.67486885434417e-05,8.15538905953451e-12,0.00174371228398782,-1.87503593153191e-11,-6.93515301673963e-13,8.78665381315771e-05;-9.93405660108620e-15,-2.58110993039430e-06,9.23647001513149e-15,4.76391496846806e-14,-1.11713589138184e-06,-2.63859313094297e-14,1.03399993129746e-13,-2.63528208125262e-06,-6.45224222774802e-14,2.81452325147330e-14,1.57703576699912e-05,3.04943088456439e-14,-1.48835553938806e-05,-8.18093525289618e-14,-5.45882770757948e-06,2.37199258113999e-05,-9.22859389579778e-15,2.86513708484418e-08,-2.09273368728068e-09,-5.68228676301377e-09,-5.82570375422056e-09,-2.49832632973274e-09,2.49832633679795e-09,5.82570377397120e-09,5.68228671386715e-09,2.09273368407746e-09,-4.20903513941155e-13,1.20142547688371e-10,-5.70526592438698e-14,0.00260017395037614,0.0748227436556051,9.82445214829637e-06,-2.91698817472667e-07,8.18039656253174e-14,1.72750586397727e-05,-1.89341268415266e-13,-7.30815919388593e-15,5.31340002172647e-07;3.57470105807459e-13,3.49738501507839e-05,-3.86594567195935e-13,-1.81182782482492e-12,-3.60883220612536e-06,1.01885522334979e-12,-3.93775629476962e-12,-0.000256285833974978,2.47999028997416e-12,-3.19496257299692e-13,-0.00323992288820419,-2.01340105443835e-13,-0.00647364523070637,2.99137559731867e-12,-0.00113320325146207,-0.000901423948535561,3.48857519108135e-13,-3.50651139858739e-06,1.00808723774631e-07,2.18417689603637e-07,2.24149052587825e-07,1.19813429485134e-07,-1.19813429775089e-07,-2.24149053350719e-07,-2.18417687744189e-07,-1.00808723673467e-07,1.59690486697347e-11,-4.56006558406379e-09,2.17118511938861e-12,0.00823910256742190,-3.05650166453451,0.998495699516311,4.07334406166313e-05,-3.12688010011988e-12,0.00853092490829503,7.26523576961048e-12,2.54868545541671e-13,0.000908975280011867;1.39845144914400e-13,-4.12885784672801e-06,-9.35569749410421e-14,-7.40790904346757e-13,-1.27232010116836e-05,1.81305963394430e-13,-1.57424268874021e-12,1.01255864650777e-05,4.52130340921161e-13,-1.18488811260601e-13,0.000382481020382500,1.72365927894914e-13,3.12115267336163e-06,1.28099851663222e-12,-0.00518269561521840,-6.64851220741041e-06,1.17661028751169e-13,-0.00193501440327986,3.99300294930033e-08,-4.94459773513087e-08,4.11514721106165e-08,-5.36600704198458e-08,5.36600704657256e-08,-4.11514717588611e-08,4.94459788507758e-08,-3.99300294389314e-08,5.00195155757736e-12,-1.20769442792639e-09,1.12767780855941e-12,-0.000319245686575807,0.106049341596551,5.19249273802791e-06,0.999413936925878,-1.74510781332807e-12,1.41435604852119e-05,2.72510910106538e-12,-3.46746081977149e-13,-9.68565949173129e-06;0.00204940390480753,-4.59536955472496e-14,-0.00165950911508204,-0.00654443821016969,-4.58530311071510e-13,0.00423071091746361,-0.0179254654605681,-8.89546107997670e-13,0.0109713069196591,-0.00651817405265153,-4.00520175549358e-13,-0.00911033833922184,-7.46955872584987e-13,0.0160994362519504,-1.62349453925653e-13,-1.13134255740320e-14,0.000226093831261329,9.06866581068770e-15,3.90556599972730e-08,2.10734683299506e-07,-4.29085684369671e-07,-2.76526956096578e-07,-2.76526956014394e-07,-4.29085684387533e-07,2.10734683287070e-07,3.90556599786014e-08,0.00132541850101628,8.57221398453720,0.0613654226283191,-6.47051599571274e-14,-4.38553366094794e-10,-1.36813686970727e-13,-7.59475221903189e-14,0.893298942512361,-5.58276126902004e-13,0.0266228883709893,0.0582056649441539,1.32529282622544e-12;2.46555257256580e-14,-0.000789784282028545,1.87298072090772e-14,-7.41701185009339e-14,-0.00399078045110627,-2.58111213965742e-14,-1.89600425051711e-13,-0.0104818164595995,-1.23310745398172e-13,-1.05552525851244e-13,-0.00830259790854282,4.98649362769814e-14,-0.0145458560389385,2.56824699065918e-13,-0.00232292734297774,-0.000274640644340470,1.00408130332002e-14,-7.40583828334191e-05,2.58758555499016e-07,1.82234934015501e-07,1.95450031434662e-07,2.90386867136701e-07,-2.90386867133100e-07,-1.95450031851777e-07,-1.82234932303138e-07,-2.58758555480781e-07,5.56790498160004e-14,1.52603173263705e-10,7.42081126678338e-13,-9.09734451210466e-05,-8.39593458384506,-0.0482824659797456,0.00110511668662421,-6.90645215361402e-13,0.905583705706679,-2.04087554168970e-13,-1.48474801725643e-12,0.0369296820583351;-0.00148225429798810,5.63127195817411e-14,0.0199234995436570,0.0121712663301675,3.14986670652863e-13,-0.0683257011867272,0.0343355321886927,5.54248050746583e-13,-0.164068403144598,-0.0220891635852134,-5.11929696458706e-13,0.0809131992035732,2.23407156765447e-13,0.0339528781492707,1.74404703533962e-13,-7.08684046529178e-14,0.00148634920457406,-2.00061726427716e-13,2.82477348239940e-06,2.14136086277477e-06,1.47797693518822e-06,7.23307122405333e-07,7.23307123293080e-07,1.47797693513500e-06,2.14136086271620e-06,2.82477348239156e-06,-0.0103826591394415,28.2990705544160,0.0338051019064170,1.09900790817077e-12,-2.61367691485514e-10,-8.12949601462667e-14,4.36304399168233e-13,0.0274737095379513,-4.79095827201159e-13,0.569156855882655,-0.103719572811141,3.40389784968603e-12;0.00251694804441282,5.71016181105666e-14,-0.00334307746203560,-0.0141955411362156,6.26321184624741e-13,0.00967014242114527,-0.0309564559314008,1.10042713062092e-12,0.0236289541331126,-0.00719874552485611,6.25658400103724e-13,-0.0167400191419976,3.94281280404958e-13,0.0145248151485270,8.31764096805345e-14,1.06490718883895e-14,0.000128534050203273,2.81637259587236e-14,-1.79398233388164e-07,1.10906504640103e-07,-6.10391515211486e-07,-3.31894219701835e-07,-3.31894219630122e-07,-6.10391515210467e-07,1.10906504646855e-07,-1.79398233355256e-07,0.00168076605232811,7.05955285214912,0.0223950493955465,-2.97968954448865e-13,2.77754432252716e-11,-2.90645752940972e-13,-1.03124571370584e-13,-0.0334775039971912,-5.55095948165622e-13,0.0585279523648476,0.997575789179534,-1.38730507551991e-13;1.71450799429240e-13,-0.00119233863845863,2.02042170837577e-14,-7.17169256676405e-13,-0.0111503757423544,-1.80516211650244e-13,-1.56315241497355e-12,-0.0225175693736990,-3.82286559005378e-13,-6.73233353961790e-13,-0.0127638377309001,-6.67121693410423e-14,-0.0181550069921130,1.54123663539216e-12,-0.00335709672840165,-0.000341896644106501,5.53621489615818e-14,-0.000122060409164768,3.67880695880341e-07,2.11664065377510e-07,2.31215458384602e-07,4.11638304083903e-07,-4.11638304129486e-07,-2.31215458908006e-07,-2.11664063243905e-07,-3.67880695800739e-07,9.47323166087049e-14,8.25734685666982e-10,2.25566601075994e-12,0.000545377563665964,-10.4757883401499,-0.0186970679517441,0.000457166521607832,-2.53309094996975e-12,-0.0232163513632939,2.28009679661753e-13,-1.41876613401534e-12,0.973788227305817];
            obj.hor_B = [-0.00870905571472402,-3.82214082864584e-13,0.0145488879796723,-0.00628753367577229,-6.54642210689425e-13,0.0257963576018473,-1.78204818634048,2.91823038120153e-12,0.414155171094741;3.41669856636390e-13,0.00166360234481826,-5.93743634738710e-13,6.30952610162780e-13,-0.00419833226047639,-9.99625435808705e-13,1.27928172460717e-11,1.36951968852063,-2.54296650183215e-11;0.0366445418727635,6.76026385047570e-13,-0.171108552612757,0.0673947529686865,5.94461085186293e-13,-0.270574676687918,2.47244120638199,3.17592782093965e-11,-8.89154244862377;0.0172979585131224,1.25440315875808e-15,-7.81823143007286e-05,0.0101009547519573,1.71795744099178e-15,-0.000128849509950795,0.00184044520648929,2.38051779005132e-14,-0.00334028720459089;1.18790153007520e-15,0.0172760234052687,-1.45410949289301e-15,2.12209258642130e-15,0.0100638483288768,-2.48095297719163e-15,5.07001971929901e-14,0.000687123352450363,-5.74759938228899e-14;-7.83892554451585e-05,-1.43932202075641e-15,0.0176453901141394,-0.000149828279266177,-1.48918569686759e-15,0.0106676246882530,-0.00459397366327114,-6.06186464006242e-14,0.0164839948723878;0.00994609061243223,2.64441499497545e-15,-0.000191333992593214,0.0173079627360959,3.73127846538435e-15,-0.000316456762219989,0.0231093405711540,5.69287426844081e-14,-0.00831899074333421;2.06029778348528e-15,0.00989843619019941,-2.29398490161955e-15,3.52997935791349e-15,0.0172046229665912,-3.95925274141501e-15,8.47981906230840e-14,0.0197480620506695,-9.25521269233321e-14;-0.000159992351126561,-2.83587740599664e-15,0.0106476836096947,-0.000307638512894905,-2.85545706089702e-15,0.0184543472497356,-0.00945085514694095,-1.20643912041843e-13,0.0528569320376512;0.00167643106632413,8.24403276782622e-14,-0.00561420356576715,0.0179250233097024,1.12800886351294e-13,-0.00870329193088272,0.550133145419738,2.38782560750788e-12,-0.300539733408343;-1.04921225954669e-13,0.000236077244729662,1.04209769727731e-13,-1.97489337615473e-13,0.0155598918359513,1.71487390629794e-13,-2.37310041746694e-12,0.369895563786440,4.80090002656981e-12;-0.00467800040568651,-8.13980642938431e-14,0.0224367031617405,-0.00847676522216800,-7.42336064438668e-14,0.0505453140402760,-0.333645037495586,-4.05073570380813e-12,1.56868315835701;8.73323261349199e-13,0.000189516074104770,-1.20785550557494e-12,1.41342094093665e-12,4.66991993207054e-05,-2.12382997439331e-12,4.58288388304863e-11,-0.0491249267665814,-3.98686421725915e-11;0.000262727500525746,4.08899708504817e-14,-0.000695278435453626,0.000474799600405351,8.87785415761492e-14,-0.00109949528322495,-0.0446661347462149,-6.69324516669837e-13,-0.0784342186908293;1.70657087302208e-13,-3.86948195637868e-05,-1.19330676450321e-13,2.59746345492434e-13,3.55560519711825e-05,-2.26657277959145e-13,1.01264600455989e-11,0.00357919034179269,-1.55227085065389e-12;9.28670288415013e-11,-0.00288644624732395,-1.44726575987320e-10,1.65298514682185e-10,0.00320412088215644,-2.39951482904894e-10,3.68389367609219e-09,-0.583326562381606,-6.15140678978223e-09;0.00727138459921296,4.37441936537913e-12,-0.0396672601818139,0.0225589827584054,7.72518204155386e-12,-0.0566339499307155,-0.154453704536008,8.60887742524963e-11,-2.43369620847068;1.71110879225465e-11,0.000139413925067532,-1.46893185355907e-11,2.94562291701134e-11,-0.000374927025591809,-2.47133589931382e-11,6.41371670641498e-10,0.0530274217584798,-5.84171089422388e-10;-11.2121296635106,8.93771370160820,33.0437621427514,-22.0203536581027,16.2353579233650,52.9179003893915,-334.775599980519,96.8738424897060,1631.62853112377;13.3467723395842,-3.55043635339607,-17.3822522514166,23.2152914448241,-5.18755826684739,-29.2079747730143,582.568684652590,-244.661693288249,-670.506497501566;-18.9708713760146,-3.57918508940101,38.3033761453900,-33.8280487169073,-5.25658307304357,63.8310085010332,-878.219798852622,-210.328031337451,1570.81457344947;5.42355819725635,8.88601974015880,-11.7270248393345,11.0817733700064,16.1456599633902,-17.6719940102317,36.3383581983044,65.1375806746655,-709.876682957828;5.42355818882978,-8.88601973929810,-11.7270248263728,11.0817733548805,-16.1456599618015,-17.6719939887641,36.3383578732248,-65.1375806576365,-709.876682399141;-18.9708714046368,3.57918509047468,38.3033761865215,-33.8280487667859,5.25658307504467,63.8310085701792,-878.219800040857,210.328031346826,1570.81457510906;13.3467723177066,3.55043635222955,-17.3822522151553,23.2152914059597,5.18755826480183,-29.2079747126026,582.568683761323,244.661693273443,-670.506495984634;-11.2121296789162,-8.93771370256813,33.0437621608944,-22.0203536846410,-16.2353579250278,52.9179004201238,-334.775600612930,-96.8738425123958,1631.62853183640;7.51896552063692e-06,6.27126166930896e-15,-3.00437434370312e-05,1.42100446297017e-05,1.08790655633987e-14,-4.88088517819371e-05,0.000553968388652321,1.22133650872065e-13,-0.00124745491642610;1.39989837862037e-07,6.03461476890370e-17,-8.83438565936526e-07,5.17492855336155e-07,1.07968007641157e-16,-1.20917890083069e-06,-9.20205584792552e-06,1.24011891259166e-15,-5.94071678597851e-05;-1.91304514851467e-05,-2.17309730288283e-15,4.30409604701188e-05,8.66568029372907e-05,-4.30985171199694e-15,0.000153950203796443,0.00294400132472505,5.34828523906986e-14,0.00494913743684223;1.31997903269579e-13,-2.60119240596448e-06,-2.14590469151514e-13,2.38234669705442e-13,-4.47540147358531e-06,-3.51909912170897e-13,5.08170058178528e-12,9.48267602710434e-05,-9.44007610257879e-12;1.28536477335437e-15,-5.73255524995539e-08,-1.89617812682623e-15,2.25583106620634e-15,1.35766906631726e-07,-3.18309211611408e-15,5.21988424312948e-14,-1.73693676737504e-05,-7.74440794731501e-14;-4.70032514668287e-14,-2.48428916957776e-05,6.60749875933419e-14,-7.80517901343254e-14,0.000145577645706917,1.14507356131069e-13,-2.73820300583897e-12,0.00308743233928071,1.99519263621849e-12;-1.65355709000459e-14,4.99390533655947e-06,1.26384075045321e-14,-2.73176317096457e-14,-2.59502870175970e-05,2.22139431727110e-14,-1.06301866144639e-12,-0.000393296414861110,1.37248565517877e-13;-0.000221002238897473,-1.07121762350670e-14,0.000390425310000921,-0.000433385643985812,-1.69091315474309e-14,0.000644398355694421,-0.0107414977625455,-1.95321241788123e-13,0.0168866934765483;-1.47464981131314e-15,-9.16580618352659e-05,-3.46816718451630e-15,-1.95123436879351e-15,-0.000199552896858351,-5.69298214952844e-15,-3.10304052521462e-14,-0.00232248205867283,-1.77305155511355e-13;0.000998771838004876,2.00807735424487e-14,-0.00477053295315279,0.00192313888400829,2.18397239728921e-14,-0.00790465260157300,0.0589543329148047,7.83534426379932e-13,-0.204795212399882;-0.000417193161935967,7.90079575863772e-15,0.000789835461686644,-0.000752856788808018,1.43215921993496e-14,0.00130108429561536,-0.0171794088168470,4.10021066924592e-14,0.0341679316979910;-1.49869027669021e-14,-0.000217002747542256,-5.21192729530969e-15,-2.52591911156556e-14,-0.000383369056370404,-8.06854741622998e-15,-4.89070453166640e-13,-0.00365579949004745,-2.38272683422357e-13];
            obj.hor_C = [0.00420819239057138,-9.43592070089980e-14,-0.00340759262426987,-0.0134381782975998,-9.41627720529082e-13,0.00868723117381278,-0.0368076820638717,-1.82651714907364e-12,0.0225281947524485,-0.0133842481633036,-8.22368178414428e-13,-0.0187069305297561,-1.53377579667862e-12,0.0330581614336826,-3.33363323891460e-13,-2.32308220293612e-14,0.000464255161238415,1.86214520589504e-14,8.01958710158299e-08,4.32717088422719e-07,-8.81073324605151e-07,-5.67813221055981e-07,-5.67813220887227e-07,-8.81073324641828e-07,4.32717088397183e-07,8.01958709774906e-08,0.00272157969310745,17.6019600506553,0.126006154249492,-1.32892276649011e-13,-9.00511402752590e-10,-2.81004584266838e-13,-1.55948723712748e-13,0.762650542807776,-1.14634983941819e-12,0.0546667428489897,0.119517990441781,2.72132071727850e-12;5.06233046367661e-14,-0.00162172239353547,3.84798030053868e-14,-1.52329643220489e-13,-0.00819456422786691,-5.30196909496824e-14,-3.89261540958207e-13,-0.0215230878408982,-2.53205179062002e-13,-2.16573121356873e-13,-0.0170483374500965,1.02001678197937e-13,-0.0298680804471077,5.27367220722071e-13,-0.00476983826645299,-0.000563939917819044,2.06182418736614e-14,-0.000152069546840688,5.31328051874909e-07,3.74196448450869e-07,4.01331983944638e-07,5.96272799978541e-07,-5.96272799971137e-07,-4.01331984801119e-07,-3.74196444934739e-07,-5.31328051837455e-07,1.14352656955311e-13,3.13346479087756e-10,1.52376597024851e-12,-0.000186802493449116,-17.2399925386061,-0.0991419531588468,0.00226921770785940,-1.41814613008859e-12,0.787875754330518,-4.19111448114537e-13,-3.04875329520929e-12,0.0758304435058161;-0.00275258226685433,1.04572026545438e-13,0.0369984230182283,0.0226023374741122,5.84979696238825e-13,-0.126882488188598,0.0637619180149347,1.02919738557367e-12,-0.304679013351428,-0.0410201137934505,-9.50774147490970e-13,0.150257778023995,4.14862915129363e-13,0.0630513201609210,3.23869924031626e-13,-1.31604052643315e-13,0.00276018660793687,-3.71520405145254e-13,5.24567303065793e-06,3.97655918138352e-06,2.74463910014683e-06,1.34319891082437e-06,1.34319891247294e-06,2.74463910004799e-06,3.97655918127475e-06,5.24567303064337e-06,-0.0192808504376145,52.5520620060100,0.0627768960853155,2.04091292828197e-12,-4.85376667580379e-10,-1.51302638081945e-13,8.10231454956546e-13,0.0510193465328572,-8.89692975698306e-13,-0.211037570423569,-0.192609768265247,6.32112069492634e-12;0.0315606908737327,7.16000076642001e-13,-0.0419197506203840,-0.178001721799536,7.85318818100129e-12,0.121256525869184,-0.388171356324201,1.37985766729003e-11,0.296289833522866,-0.0902670131364012,7.84460927578776e-12,-0.209907618289436,4.94395002476640e-12,0.182130577513654,1.04296029259912e-12,1.33534081352275e-13,0.00161172314788910,3.53152739678068e-13,-2.24952286950306e-06,1.39068659625392e-06,-7.65386395889362e-06,-4.16171120180276e-06,-4.16171120090355e-06,-7.65386395888084e-06,1.39068659633859e-06,-2.24952286909042e-06,0.0210755791826311,88.5216386440883,0.280817568977552,-3.73638199648703e-12,3.48243877146517e-10,-3.64514956557513e-12,-1.29310834221911e-12,-0.419783458472617,-6.96050214667666e-12,0.733897791875334,0.0481419825476128,-1.73956780624433e-12;2.14985526232525e-12,-0.0149510560095725,2.53427736878909e-13,-8.99265868219749e-12,-0.139817571010891,-2.26393040930042e-12,-1.96006420374308e-11,-0.282353880052752,-4.79369988563432e-12,-8.44157379579402e-12,-0.160049206371840,-8.37944565211292e-13,-0.227650532858816,1.93260419358896e-11,-0.0420955419852623,-0.00428713430114891,6.94204088491628e-13,-0.00153054841561868,4.61295533964898e-06,2.65411284562913e-06,2.89927303962576e-06,5.16164379944939e-06,-5.16164380002093e-06,-2.89927304618883e-06,-2.65411281887525e-06,-4.61295533865079e-06,1.18799030358879e-12,1.03540803925336e-08,2.82842609705121e-11,0.00683863647266779,-131.358737497999,-0.234447581538317,0.00573253440741965,-3.17630768769170e-11,-0.291116096027250,2.85888620449139e-12,-1.77903456286679e-11,-0.250136676244215];
            obj.hor_D = [-0.000453800218612761,-2.19031751127721e-14,0.000801689122763085,-0.000889902749246098,-3.47764151910209e-14,0.00132319073393427,-0.0220563106382532,-4.01117398844686e-13,0.0346746948335677;-2.99772371638258e-15,-0.000188208267508801,-7.10760175590728e-15,-4.06580011334343e-15,-0.000409756700524476,-1.17017749506522e-14,-6.38816798938235e-14,-0.00476892393130437,-3.63691498792007e-13;0.00185474358459091,3.72479647939258e-14,-0.00885899567173868,0.00357131566155982,4.06098555873973e-14,-0.0146791320532967,0.109479629477736,1.45515170009089e-12,-0.380309688259111;-0.00523129766135792,9.94983287055377e-14,0.00990396003713235,-0.00944027447706185,1.79566298233688e-13,0.0163146471560015,-0.215417244017238,5.14837087892128e-13,0.428440917765020;-1.88041806268048e-13,-0.00272105602224713,-6.49917111179318e-14,-3.16866799403647e-13,-0.00480716807226933,-1.01136063651378e-13,-6.13285420711374e-12,-0.0458410565358541,-2.98638197504530e-12];
            obj.hor_op = [0.106829259352255;0;-0.548558287675047;0;0];
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
            obj.stick_input_xy_prev = zeros(2,1);

            % Matrices
            obj.R_setpoint = zeros(3,3);

            % Structures saved as arrays
            obj.ref_pos      = zeros(6,1);
            obj.att_sp       = zeros(14,1);
            obj.att_sp(5)    = -1; % Landing gear down
            obj.local_pos_sp = zeros(13,1);
            obj.vel_deriv    = [ inf; inf; inf; 0; 0; 0; obj.veld_lp; obj.dt ];
            obj.hysteresis   = [ 0; 1; 0; 0; 100000 ];

            % Char arrays
            obj.user_intention_xy = 1; % Brake
            obj.user_intention_z  = 1; % Brake
            
            % Horizontal thrust
            obj.hor_state = zeros(38, 1);
            obj.pos_err_int = zeros(3,1);
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
                'horThrustDes';
                };
        end

        function num = getNumOutputsImpl( obj )
        %GETNUMOUTPUTSIMPL Return number of outputs
        %   Written: 2021/03/05, J.X.J. Bannwarth
            num = 5;
        end

        function varargout = getOutputSizeImpl( obj )
        %GETOUTPUTSIZEIMPL Return output size(s)
        %   Written: 2021/03/05, J.X.J. Bannwarth
            varargout = { ...
                         [4 1];
                         [1 1];
                         [1 1];
                         [3 1];
                         [2 1];
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
                    sz = [2 1];
                    dt = 'double';
                    cp = 0;
                % Matrix
                case 'R_setpoint'
                    sz = [3 3];
                    dt = 'double';
                    cp = 0;
                % Structures
                case 'att_sp'
                    sz = [14 1];
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
                % Horizontal thrust
                case 'hor_state'
                    sz = [38 1];
                    dt = 'double';
                    cp = 0;
                case 'pos_err_int'
                    sz = [3 1];
                    dt = 'double';
                    cp = 0;
                otherwise
                    error(['Error: Incorrect State Name: ', name.']);
            end
        end
        
        function icon = getIconImpl( ~ )
        %GETICONIMPL Define icon for System block
        %   Written: 2021/03/05, J.X.J. Bannwarth
            icon = ["mc_pos_control","","v1.82","","HORIZONTAL"];
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
                eul = DcmToEuler( QuatToDcm( att.q ) );
                in.yaw = eul(3);

                if false % in.control_mode.flag_control_manual_enabled
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
                if false % in.control_mode.flag_control_manual_enabled
                    if obj.z_reset_counter ~= in.local_pos.z_reset_counter
                        obj.pos_sp(3) = in.local_pos.z;
                    end

                    if obj.xy_reset_counter ~= in.local_pos.xy_reset_counter
                        obj.pos_sp(1) = in.local_pos.x;
                        obj.pos_sp(2) = in.local_pos.y;
                    end
                end

                % (1.7) position_setpoint_triplet
                current_triplet.a_x                = 0;
                current_triplet.a_y                = 0;
                current_triplet.a_z                = 0;
                current_triplet.acceleration_valid = 1;
                current_triplet.acceptance_radius  = 0;
                current_triplet.alt                = obj.ref_alt-xiDes(3);
                current_triplet.alt_valid          = isfinite(xiDes(3));
                current_triplet.cruising_speed     = 0;
                current_triplet.lat                = obj.ref_lat_init;
                current_triplet.lon                = obj.ref_lon_init;
                current_triplet.position_valid     = sum(isfinite(xiDes)) == 3;
                current_triplet.type               = 'SETPOINT_TYPE_POSITION';
                current_triplet.valid              = 1;
                current_triplet.velocity_frame     = 'VELOCITY_FRAME_LOCAL_NED';
                current_triplet.velocity_valid     = 1;
                current_triplet.vx                 = 0;
                current_triplet.vy                 = 0;
                current_triplet.vz                 = 0;
                current_triplet.x                  = xiDes(1);
                current_triplet.y                  = xiDes(2);
                current_triplet.yaw                = 0;
                current_triplet.yaw_valid          = 1;
                current_triplet.yawspeed           = 0;
                current_triplet.yawspeed_valid     = 0;
                current_triplet.z                  = xiDes(3);

                % We are station keeping so all waypoints are the same
                in.pos_sp_triplet.current  = current_triplet;
                in.pos_sp_triplet.next     = current_triplet;
                in.pos_sp_triplet.previous = current_triplet;

                % To be a valid current triplet, altitude has to be finite
                if ~isfinite( current_triplet.alt )
                    in.pos_sp_triplet.current.valid = 0;
                end

                % To be a valid previous triplet, lat/lon/alt has to be finite
                if ( ~isfinite( current_triplet.lat ) || ...
                    ~isfinite( current_triplet.lon ) || ...
                    ~isfinite( current_triplet.alt ) )
                    in.pos_sp_triplet.previous.valid = 0;
                end
                
                % (1.8) home_position
                in.home_pos.z = 0;
                
                % Update the reset counters in any case
                obj.z_reset_counter = in.local_pos.z_reset_counter;
                obj.xy_reset_counter = in.local_pos.xy_reset_counter;
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
            varargout{5} = extra_states.att_sp.hor_thrust;
            
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
                obj.yaw_takeoff = in.yaw;
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
            is_aligned = dot(stick_xy_norm, stick_xy_prev_norm) > 0.5;
        
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
                    else
                        vel_xy_norm = vel_xy;
                    end
        
                    stick_vel_aligned = dot(vel_xy_norm, stick_xy_norm) > 0;
        
                    % Update manual direction change hysteresis
                    extra_states.hysteresis = hysteresis_set_state_and_update( extra_states.hysteresis, ...
                        double(~stick_vel_aligned), getCurrentTime(obj)*1E6 );
        
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
                    % warning( 'User intention not recognized' );
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
                man_vel_sp(3) = -expo_deadzone( (in.manual.z - 0.5) * 2, ...
                                                obj.z_vel_man_expo, obj.hold_dz );
        
                % Reset alt setpoint to current altitude if needed
                reset_altitude_sp( obj );
            end
        
            if in.control_mode.flag_control_position_enabled
                % Set horizontal velocity setpoint with roll/pitch stick
                man_vel_sp(1) = expo_deadzone( in.manual.x, ...
                    obj.xy_vel_man_expo, obj.hold_dz );
                man_vel_sp(2) = expo_deadzone( in.manual.y, ...
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
            alt_hold_desired = double( in.control_mode.flag_control_altitude_enabled && ...
                strcmp(extra_states.user_intention_z, 'brake') );
        
            % Want to get/stay in position hold if user has xy stick in the middle
            % (accounted for deadzone already)
            pos_hold_desired = double( in.control_mode.flag_control_position_enabled && ...
                strcmp(extra_states.user_intention_xy, 'brake') );
        
            % Check vertical hold engaged flag
            if (obj.alt_hold_engaged)
                obj.alt_hold_engaged = alt_hold_desired;
        
            else
                % Check if we switch to alt_hold_engaged
                smooth_alt_transition = double( alt_hold_desired && ...
                    ( (max_acc_z - obj.acc_state_dependent_z) < obj.flt_epsilon ) && ...
                    (obj.hold_max_z < obj.flt_epsilon || abs(obj.vel(3)) < obj.hold_max_z) );
        
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
                extra_states.att_sp.yaw_body = in.yaw;
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
                            obj.vel_sp(1) = cos(in.yaw) * in.pos_sp_triplet.current.vx ...
                                - sin(in.yaw) * in.pos_sp_triplet.current.vy;
                            obj.vel_sp(2) = sin(in.yaw) * in.pos_sp_triplet.current.vx ...
                            + cos(in.yaw) * in.pos_sp_triplet.current.vy;
                        else
                            % warning( 'Unknown velocity offboard coordinate frame' );
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
                    yaw_offs = WrapPi( yaw_target - in.yaw );
        
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
                    [ ~, curr_pos_sp(1), curr_pos_sp(2) ] = ...
                        map_projection_project( extra_states.ref_pos, ...
                                    in.pos_sp_triplet.current.lat, ...
                                    in.pos_sp_triplet.current.lon );
        
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
        
                yaw_diff = WrapPi(extra_states.att_sp.yaw_body - in.yaw);
        
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
                        vel_sp_along_track_prev = dot( [obj.vel_sp(1); obj.vel_sp(2)], ...
                            unit_prev_to_current );
        
                        % Distance to target when brake should occur
                        target_threshold_xy = 1.5 * get_cruising_speed_xy( obj, in );
        
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
                        vel_sp_along_track = get_cruising_speed_xy( obj, in );
        
                        % Compute velocity setpoint along track
                        % Only go directly to previous setpoint if more than 5m away and
                        % previous in front
                        if ( previous_in_front && (norm(vec_prev_to_pos) > 5) )
        
                            % Just use the default velocity along track
                            vel_sp_along_track = norm(vec_prev_to_pos) * obj.pos_p(1);
        
                            if (vel_sp_along_track > get_cruising_speed_xy(obj, in))
                                vel_sp_along_track = get_cruising_speed_xy(obj, in);
                            end
        
                        elseif current_behind
                            % Go directly to current setpoint
                            vel_sp_along_track = norm(vec_pos_to_current) * obj.pos_p(1);
                            if ~( vel_sp_along_track < get_cruising_speed_xy(obj, in) )
                                vel_sp_along_track = get_cruising_speed_xy( obj, in );
                            end
        
                        elseif (close_to_prev)
                            % Accelerate from previous setpoint towards current setpoint
                            % We are close to previous and current setpoint
                            % We first compute the start velocity when close to current
                            % septoint and use this velocity as final velocity when
                            % transition occurs from acceleration to deceleration.
                            % This ensures smooth transition
                            final_cruise_speed = get_cruising_speed_xy( obj, in );
        
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
                                    vel_close = get_vel_close( obj, in, ...
                                                                unit_prev_to_current, ...
                                                                unit_current_to_next );
                                    acceptance_radius = obj.nav_rad;
                                end
        
                                % Compute velocity at transition where vehicle switches
                                % from acceleration to deceleration
                                if (target_threshold_tmp - acceptance_radius) < obj.sigma_norm
                                    final_cruise_speed = vel_close;
        
                                else
                                    slope = (get_cruising_speed_xy(obj, in) - vel_close) / ...
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
                                vel_close = get_vel_close( obj, in, unit_prev_to_current, ...
                                                            unit_current_to_next );
        
                                % Compute velocity along line which depends on distance
                                % to current setpoint
                                if ( norm(vec_closest_to_current) < obj.nav_rad )
                                    vel_sp_along_track = vel_close;
                                else
                                    if (target_threshold_xy - obj.nav_rad) < obj.sigma_norm
                                        vel_sp_along_track = vel_close;
                                    else
                                        slope = ( get_cruising_speed_xy(obj, in) - vel_close ) / ...
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
                                slope = get_cruising_speed_xy(obj, in) / target_threshold_xy;
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
                        if ( vel_sp_orthogonal < get_cruising_speed_xy(obj, in) ) && ...
                            ~current_behind
        
                            % We need to limit vel_sp_along_track such that cruise speed
                            % is never exceeded but still can keep velocity orthogonal
                            % to track
                            if cruise_sp_mag > get_cruising_speed_xy(obj, in)
                                vel_sp_along_track = sqrt( ...
                                    get_cruising_speed_xy(obj, in) * get_cruising_speed_xy(obj, in) ...
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
        
                            if cruise_sp > get_cruising_speed_xy( obj, in )
                                cruise_sp = get_cruising_speed_xy( obj, in );
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
                        obj.vel_sp(1) = obj.vel(1) / vel_xy_mag * get_cruising_speed_xy(obj, in);
                        obj.vel_sp(2) = obj.vel(2) / vel_xy_mag * get_cruising_speed_xy(obj, in);
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
                    obj.vel_max_xy = get_cruising_speed_xy( obj, in );
                end
        
                % sanity check
                if ( ~(isfinite(obj.pos_sp(1)) && isfinite(obj.pos_sp(2)) && ...
                        isfinite(obj.pos_sp(3))) )
                    % warning( 'Auto: Position setpoint not finite' );
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
                    % warning( 'Caught invalid pos_sp in x and y' );
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
                    % warning( 'Caught invalid pos_sp in z' );
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
                ~manual_wants_landing( obj, in ) )                        % Operator is not trying to land
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
            hor_thrust_sp = zeros(2,1);
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
                % APPLY CONTROL LAW
                thrust_sp = nan(3,1);
                hor_thrust_sp = zeros(3,1);
                if (obj.use_hor_thrust)
                    % Use h_infinity controller
                    [thrust_sp, hor_thrust_sp] = hinf_controller( obj );
                end
                
                if ~(obj.use_hor_thrust) || isnan(thrust_sp(1)) ...
                        || isnan(thrust_sp(2)) || isnan(thrust_sp(3))
                    % Default way, using PID, no horizontal thrust
                    thrust_sp = vel_err .* obj.vel_p + obj.vel_err_d .* obj.vel_d ...
                            + obj.thrust_int - [ 0; 0; obj.thr_hover];
                    hor_thrust_sp = 0 .* hor_thrust_sp;
                end
                
                % Rotate horizontal thrust to body frame
                hor_thrust_sp = (in.R') * hor_thrust_sp;
                
                % The vertical component cannot be produced using
                % horizontal thrust, so rotate it back in the world frame
                % and add it to thrust_sp
                thrust_comp = in.R * [ 0; 0; hor_thrust_sp(3) ];
                thrust_sp = thrust_sp + thrust_comp;
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
                % warning( 'Thrust setpoint not finite' );
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
        
            % Add horizontal thrust
            extra_states.att_sp.hor_thrust = hor_thrust_sp(1:2);
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
                extra_states.att_sp.yaw_body = in.yaw;
            elseif ( ~in.vehicle_land_detected.landed && ...
                    ~(~in.control_mode.flag_control_altitude_enabled && in.manual.z < 0.1) )
                % Do not move yaw while sitting on the ground
                % We want to know the real constraint, and global overrides manual
                if obj.man_yaw_max < obj.global_yaw_max
                    yaw_rate_max = obj.man_yaw_max;
                else
                    yaw_rate_max = obj.global_yaw_max;
                end
                yaw_offset_max = yaw_rate_max / obj.mc_att_yaw_p;
        
                extra_states.att_sp.yaw_sp_move_rate = in.manual.r * yaw_rate_max;
        
                yaw_target = WrapPi( extra_states.att_sp.yaw_body ...
                    + extra_states.att_sp.yaw_sp_move_rate * obj.dt );
                yaw_offs = WrapPi( yaw_target - in.yaw );
        
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
                thr_val = MulticopterPositionControlHorizontal.throttle_curve( in.manual.z, obj.thr_hover );
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
                
                x = in.manual.x * obj.man_tilt_max;
                y = in.manual.y * obj.man_tilt_max;
        
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
                    yaw_error = WrapPi( extra_states.att_sp.yaw_body - in.yaw );
        
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
            if ( strcmp(in.manual.gear_switch, 'SWITCH_POS_ON') && ...
                    obj.gear_state_initialized && ~in.vehicle_land_detected.landed )
                extra_states.att_sp.landing_gear = vehicle_attitude_setpoint_s_LANDING_GEAR_UP;
        
            elseif strcmp(in.manual.gear_switch, 'SWITCH_POS_OFF')
                extra_states.att_sp.landing_gear = vehicle_attitude_setpoint_s_LANDING_GEAR_DOWN;
                % Switching the gear off does put it into a safe defined state
                obj.gear_state_initialized = 1;
            end
        
            extra_states.att_sp.timestamp = getCurrentTime( obj );
        end

        %% Horizontal thrust control
        function [thrust_sp, hor_thrust_sp] = hinf_controller( obj )
        %HINF_CONTROLLER Run h-infinity horizontal thrust controller
        %   Written: 2021/04/06, J.X.J. Bannwarth
        
            % If for any reason, we get a NaN position setpoint, we better just stay
            % where we are.
            if ( isfinite(obj.pos_sp(1)) && isfinite(obj.pos_sp(2)) && isfinite(obj.pos_sp(3)) )
                % Update integrals
                pos_err = obj.pos_sp - obj.pos;
                obj.pos_err_int = obj.pos_err_int + obj.dt * pos_err;

                % Controller implemented as
                % x(t+1) = A_K x(t) + B_K y(t)
                % u(t)   = C_K x(t) + D_K y(t)

                % y(t)
                regulated_output = [ -obj.pos_err_int; -pos_err; obj.vel ];

                % u(t)
                control_input = obj.hor_C * obj.hor_state + obj.hor_D * regulated_output;

                % Add operating point
                control_input = control_input + obj.hor_op;

                % x(t+1)
                obj.hor_state = obj.hor_A * obj.hor_state + obj.hor_B * regulated_output;

                % Extract control variables
                thrust_sp = control_input(1:3,1);
                hor_thrust_sp = [ control_input(4:5,1); 0 ];
            else
                % Invalid setpoint, return invalid output
                % This is caught in calculate_thrust_setpoint, and the
                % default controller is run
                % Without a setpoint it will not work either, but at
                % least the original logic handling such issues should
                % work unmodified
                obj.pos_err_int = zeros(3, 1);
                thrust_sp     = nan(3, 1);
                hor_thrust_sp = zeros(3, 1);
            end
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

        function vel_close = get_vel_close( obj, in, unit_prev_to_current, unit_current_to_next )
        %GET_VEL_CLOSE Get close velocity
        %    Written:  2021/03/06, J.X.J. Bannwarth
        
            % Minimum cruise speed when passing waypoint
            min_cruise_speed = 1;
        
            % Make sure that cruise speed is larger than minimum*/
            if ( (get_cruising_speed_xy(obj, in) - min_cruise_speed) < obj.sigma_norm )
                vel_close = get_cruising_speed_xy(obj, in);
                return
            end
        
            % Middle cruise speed is a number between maximum cruising speed and minimum
            % cruising speed and corresponds to speed at angle of 90degrees it needs to
            % be always larger than minimum cruise speed
            middle_cruise_speed = obj.cruise_speed_90;
        
            if ( (middle_cruise_speed - min_cruise_speed) < obj.sigma_norm )
                middle_cruise_speed = min_cruise_speed + obj.sigma_norm;
            end
        
            if ( (get_cruising_speed_xy(obj, in) - middle_cruise_speed) < obj.sigma_norm )
                middle_cruise_speed = (get_cruising_speed_xy(obj, in) + min_cruise_speed) * 0.5;
            end
        
            % If middle cruise speed is exactly in the middle, then compute vel_close
            % linearly
            use_linear_approach = 0;
        
            if ( ( (get_cruising_speed_xy(obj, in) + min_cruise_speed) * 0.5 - ...
                middle_cruise_speed ) < obj.sigma_norm )
                use_linear_approach = 1;
            end
        
            % angle = cos(x) + 1.0
            % angle goes from 0 to 2 with 0 = large angle, 2 = small angle:
            % 0 = PI, 2 = PI*0
            angle = 2.0;
            if ( norm(unit_current_to_next) > obj.sigma_norm )
                angle = dot( unit_current_to_next, ...
                    (unit_prev_to_current * -1) ) + 1;
            end
        
            % Compute velocity target close to waypoint
            if use_linear_approach
                % Velocity close to target adjusted to angle, vel_close =  m*x+q
                slope = -( get_cruising_speed_xy(obj, in) - min_cruise_speed ) / 2.0;
                vel_close = slope * angle + get_cruising_speed_xy(obj, in);
            else
                % Velocity close to target adjusted to angle
                % vel_close = a *b ^x + c; where at angle = 0 -> vel_close = vel_cruise;
                % angle = 1 -> vel_close = middle_cruise_speed (this means that at
                % 90degrees the velocity at target is middle_cruise_speed);
                % angle = 2 -> vel_close = min_cruising_speed
        
                % From maximum cruise speed, minimum cruise speed and middle cruise
                % speed compute constants a, b and c
                a = -( (middle_cruise_speed -  get_cruising_speed_xy(obj, in)) * ...
                        (middle_cruise_speed -  get_cruising_speed_xy(obj, in)) ) / ...
                        ( 2.0 * middle_cruise_speed - get_cruising_speed_xy(obj, in) - ...
                        min_cruise_speed );
                c =  get_cruising_speed_xy(obj, in) - a;
                b = (middle_cruise_speed - c) / a;
                vel_close = a * b^angle + c;
            end
        
            % vel_close needs to be in between max and min
            vel_close = constrain( vel_close, min_cruise_speed, get_cruising_speed_xy(obj, in) );
        end

        function out = get_cruising_speed_xy( obj, in )
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
            obj.in_smooth_takeoff = double( obj.takeoff_vel_limit < -vel_sp_z );
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
                'q_d'              , obj.att_sp(1:4),  ...
                'landing_gear'     , obj.att_sp(5)  ,  ...
                'pitch_body'       , obj.att_sp(6)  ,  ...
                'q_d_valid'        , obj.att_sp(7)  ,  ...
                'roll_body'        , obj.att_sp(8)  ,  ...
                'thrust'           , obj.att_sp(9)  ,  ...
                'timestamp'        , obj.att_sp(10) ,  ...
                'yaw_body'         , obj.att_sp(11) ,  ...
                'yaw_sp_move_rate' , obj.att_sp(12) ,  ...
                'hor_thrust'       , obj.att_sp(13:14) ...
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
                extra_states.att_sp.hor_thrust;
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
            obj.user_intention_xy = 1;
            obj.user_intention_z = 1;
            for ii = 1:length(intentions)
                if strcmp( extra_states.user_intention_xy, intentions{ii} )
                    obj.user_intention_xy = ii;
                end
                if strcmp( extra_states.user_intention_z, intentions{ii} )
                    obj.user_intention_z = ii;
                end
            end
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
        lat = 0;
        lon = 0;
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
        x = 0;
        y = 0;
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

function init = map_projection_initialized( ref )
%MAP_PROJECTION_INITIALIZED
%   Written: 2021/03/23, J.X.J. Bannwarth
    init = ref.init_done;
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