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
        controller                                                                                                                                                                 = -1; % Position controller [-1/SS]
        controller_op                                                                                                                                                              = -1; % Operating point [-1/vec]
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
            if isStateSpace( obj.controller ) && ( obj.controller.Ts == 0 )
                % Continuous controller, discretize it
                Kd = c2d( obj.controller, obj.dt, 'Tustin' );
                obj.hor_A = Kd.A;
                obj.hor_B = Kd.B;
                obj.hor_C = Kd.C;
                obj.hor_D = Kd.D;
            elseif isStateSpace( obj.controller ) && ( obj.controller.Ts > 0 )
                % Discrete controller
                obj.hor_A = obj.controller.A;
                obj.hor_B = obj.controller.B;
                obj.hor_C = obj.controller.C;
                obj.hor_D = obj.controller.D;
            else
                % Default controller, full order
                obj.hor_A = [0.595481129797594,-3.57698607475214e-13,0.00228362749419932,-0.100212089916595,-2.15557171325621e-12,0.0775510315257041,-0.272770523079340,-4.03000715163612e-12,0.222676465176164,1.70447511495906,-6.74823130867900e-12,-0.607642382895557,-3.96781111245287e-12,0.0625236641727226,-1.82122342516935e-12,5.24040311627574e-14,0.00487930989864242,5.78243852921058e-14,5.99400852152053e-06,-2.65736955504044e-06,1.25412634008350e-05,3.56246875419723e-06,3.56246875546744e-06,1.25412634012472e-05,-2.65736955498549e-06,5.99400852154327e-06,0.231892674737202,15.9283706121770,0.562539520036931,-1.14467964267015e-12,-1.89625267775153e-09,1.29335939501609e-12,-2.38893089042036e-13,-0.517797171844042,-9.02305075924227e-13,0.693529931442368,0.716898951450065,6.12409722644797e-12;-1.04586468369111e-12,0.591771231281133,5.97027889867596e-13,4.18437333479931e-12,0.0395262675598608,-2.53258047964028e-12,1.01042974455470e-11,0.0966978504470652,-6.50308037906104e-12,-3.55687060473849e-13,-1.28423648770337,1.11452796393452e-11,-0.0272863913000257,-8.99010844606427e-12,-0.0102107106719943,0.00363375054741697,-8.02749301527021e-13,6.54306176188556e-05,6.94088872496808e-06,-1.63228604454941e-06,-1.71544181125443e-06,8.33857754475315e-06,-8.33857754434729e-06,1.71544181547042e-06,1.63228602969184e-06,-6.94088872553809e-06,-3.17121712921023e-11,7.45632616820428e-09,-1.34684477672959e-11,0.150555787581950,2.43916008049737,0.220755135595049,-0.00282814863388896,1.53155874947561e-11,0.167289886872547,-2.27003340901787e-11,6.63065418320088e-12,-0.155618786875417;0.00803177232559623,-2.36448076304920e-13,0.548602075377217,0.146059787231506,-5.82662900791011e-13,-0.577596113437916,0.457205302959571,-2.41008492858435e-12,-1.56229561596009,-2.19196009138831,-2.52250632602794e-11,6.90669850908410,-4.80797383723600e-12,0.0449797162210732,2.99063386905304e-12,-2.72027795077359e-13,0.00487030488840469,-1.41266952253137e-12,-4.11645700572651e-05,-4.65122673509139e-05,-5.89032160882956e-05,-6.81618343526023e-05,-6.81618343473248e-05,-5.89032160922484e-05,-4.65122673523179e-05,-4.11645700582406e-05,-0.437283126818215,307.928878293309,-0.179654576140012,4.43453893967566e-11,-1.88008849586962e-08,-1.14171564987281e-11,2.32789316775070e-12,0.628194592702689,-1.95755336085381e-11,-4.37227728136771,-1.62379898304976,4.28763323946901e-11;4.72121628671092e-05,-4.90342773306037e-16,-0.000196167972823850,0.982718794035154,-1.82191963821850e-15,0.000126166671268461,0.00972786323103922,-3.20053997861754e-15,0.000251185677410358,-0.00201722031480434,-1.70508364833258e-14,0.00362873679127992,-1.58562907573512e-15,-5.06171582554510e-05,9.32258410481868e-16,4.25024215970441e-16,-7.14500951818067e-06,6.69154694324492e-16,-4.59682984200770e-08,-3.80546088589708e-08,-5.56331517832428e-08,-4.95596673150967e-08,-4.95596673175567e-08,-5.56331517852506e-08,-3.80546088592367e-08,-4.59682984205562e-08,-0.000278959013249078,0.0755704573053286,-2.98128790945923e-06,1.88927628199660e-14,-7.13497561516890e-12,-1.62628656198432e-15,-2.88960832665117e-15,-8.18988101574603e-05,-1.78564048102271e-15,0.000430525271778297,0.000145837387498684,-5.39302518840228e-15;3.51846088394388e-16,-9.80531208204877e-06,-7.67450105586757e-16,6.14319358249397e-16,0.982747825663707,-6.37096393997701e-17,1.82512626259185e-15,0.00979065294555755,-6.19923889948187e-16,-2.69458293897105e-14,-0.000538231633167256,1.60772993015480e-14,-4.36872656376796e-05,-1.49699129485093e-15,-9.64597286813805e-06,8.51601288284879e-07,-3.45406827309861e-16,-1.59369717064110e-07,3.40693196373570e-09,-2.61049280893644e-10,-2.62899763275599e-10,4.04394833466209e-09,-4.04394833491169e-09,2.62899763323560e-10,2.61049278634190e-10,-3.40693196431557e-09,-1.57795913608292e-14,4.39937891178143e-12,-3.14982344778198e-15,6.09770794474685e-05,-0.0177741494496106,4.40641699489018e-06,7.23485124816765e-07,3.14170376091914e-15,3.91725889618859e-06,-4.33543424838688e-15,1.30629677596057e-15,1.30433667124078e-05;-0.000112601943472935,1.28968376714004e-15,0.000726631756145013,9.48478527238452e-05,3.80280652139575e-15,0.982263700543781,0.000230079612637878,6.40898201272457e-15,0.00882432258657413,0.00485691770577461,3.88078516664764e-14,-0.0140007507482373,2.16274951220068e-15,0.000395395419039255,-3.65539118051671e-15,-1.71254754403293e-15,2.26702335327654e-05,-2.33228415045299e-15,1.72639752963477e-07,1.62077473631177e-07,1.90950282183254e-07,1.88219871654530e-07,1.88219871662343e-07,1.90950282190663e-07,1.62077473632283e-07,1.72639752964794e-07,0.000739021886602926,-0.189338327263593,-1.67951055800765e-05,-7.37567595735307e-14,2.74942053370947e-11,6.13742850815163e-15,9.96529304581962e-15,0.000339416249639862,7.06604899916399e-15,-0.00169780248675256,-0.000514987348500179,2.03658149278163e-14;9.88460275105828e-05,-1.23116634446292e-15,-0.000507134960379679,-0.00994579905470201,-4.05916905860484e-15,0.000332127140464124,0.982548073362349,-7.01087974012633e-15,0.000664947688657366,-0.00424043496891249,-4.37756712576980e-14,0.00958950610253370,-3.42827895110132e-15,-0.000208328534111944,2.32825597349024e-15,1.16415110095371e-15,-1.70568665897618e-05,1.67277071830761e-15,-1.19719703694050e-07,-1.06248543589159e-07,-1.38188148340314e-07,-1.29852718737120e-07,-1.29852718743127e-07,-1.38188148345485e-07,-1.06248543589883e-07,-1.19719703695229e-07,-0.000611507355511196,0.161610033871467,2.66633811476108e-06,5.12421499121526e-14,-1.91319800991830e-11,-4.17476851665201e-15,-7.18337028037923e-15,-0.000225159949809763,-4.72078500231316e-15,0.00115140913730186,0.000367624468505026,-1.38746527808329e-14;1.10508394643481e-15,2.56820363049216e-06,-9.78700386511789e-16,-1.17401567992645e-15,-0.00987979620551296,5.86928239055171e-16,-2.00939964438859e-15,0.982718660744414,1.19045103601478e-15,-4.34246869607712e-14,0.000140973425708348,9.40725932683949e-15,1.14425520858609e-05,3.13661364368546e-15,2.52646956387388e-06,-2.23051087207012e-07,-8.16575724211482e-18,4.17420559926773e-08,-8.92342331101798e-10,6.83739289592401e-11,6.88586059673706e-11,-1.05918941793502e-09,1.05918941754053e-09,-6.88586070926005e-11,-6.83739282057453e-11,8.92342330770222e-10,-1.86024838601769e-15,6.30614261404810e-13,3.98821111799563e-16,-1.59710935769702e-05,0.00465539849863323,-1.15412707204468e-06,-1.89494950146784e-07,-7.86681758770628e-16,-1.02600696791562e-06,2.21291907062221e-15,9.04643784646392e-16,-3.41631367409236e-06;-0.000240484862535470,2.66881443409582e-15,0.00151941120663821,0.000198865204029551,8.05165721713499e-15,-0.0108852773850889,0.000483776960007152,1.36118614157418e-14,0.980688412698996,0.0103672111510925,7.92458846324479e-14,-0.0292308482769359,4.50802107067809e-15,0.000809991848045126,-7.69656160787302e-15,-3.56370835735977e-15,4.77108164896837e-05,-4.88985102851233e-15,3.60804267580179e-07,3.37203591793379e-07,4.00504878565754e-07,3.93198874605754e-07,3.93198874622138e-07,4.00504878581253e-07,3.37203591795686e-07,3.60804267582920e-07,0.00156908944571966,-0.403229621097800,-3.28705239295829e-05,-1.53484494298595e-13,5.72774642018001e-11,1.28560830223855e-14,2.09025876062884e-14,0.000706818589911180,1.47670124767852e-14,-0.00354183531267279,-0.00107889228359056,4.26479212319749e-14;-0.0332202924960335,2.46684837121366e-14,-0.00217213908907727,-0.0131963969851934,6.57281756824029e-13,0.0136289408334455,-0.0443339949791903,1.25573400990423e-12,0.0292468875220364,0.394574756663568,-1.00512037200907e-12,0.343290387779781,8.55756852606628e-13,-0.113204951750032,-1.31209652759152e-13,1.01752739523355e-13,0.00197750553817860,-7.60343184100522e-14,-1.79156927479656e-06,-1.18635097706832e-05,7.57736953463269e-06,-3.06516333847256e-06,-3.06516333800654e-06,7.57736953463929e-06,-1.18635097707382e-05,-1.79156927473077e-06,0.156355144912560,-48.2430812197930,0.0150500792566827,3.84200770038673e-12,-1.11886139475278e-09,3.52194009130406e-14,3.86798257012949e-13,-0.0204404993447249,-1.62601520996712e-13,0.0606176230541091,-0.0120460879361878,6.88415449077666e-14;9.61644252075628e-13,0.0292833541837405,-8.21098316337079e-13,-3.58343796159883e-12,-0.0111369298331776,1.98065114801749e-12,-8.34308037740150e-12,-0.0383389877054169,5.22807908808064e-12,-4.58204043886217e-12,0.607417225993751,-1.77570063108692e-12,0.130471081622118,7.88439302784073e-12,0.0288074910396343,-0.00254328897841314,8.41171871987443e-13,0.000475954241119444,-1.01747292228781e-05,7.79618077031229e-07,7.85144501953274e-07,-1.20771649490122e-05,1.20771649483731e-05,-7.85144504475690e-07,-7.79618070634537e-07,1.01747292232469e-05,3.67295944468575e-11,-1.01020522164635e-08,8.79492269618225e-12,-0.182106739653840,53.0821160297970,-0.0131596698262011,-0.00216067280463860,-9.95106145549785e-12,-0.0116988096581349,1.76890639433298e-11,-1.07546673938908e-12,-0.0389537348726631;-0.00858087524840774,5.02654496098157e-14,-0.0220019847249485,0.000523898708328151,1.22170010833890e-13,-0.0104653582915743,-0.00136749489530806,2.05192019044708e-13,-0.0248429811838471,0.356426255509602,2.85050844352140e-12,-0.468357159691103,5.95134465625820e-13,-0.0519998815004262,1.37733978212120e-13,7.95115039701242e-14,4.43665959569348e-05,4.22468214548549e-14,-5.68318401480419e-06,-8.97350585041315e-06,-2.87039183747985e-06,-6.59337406045045e-06,-6.59337406044690e-06,-2.87039183769817e-06,-8.97350585046933e-06,-5.68318401476416e-06,0.0342803254573320,-11.7071809577400,0.00586823185382958,3.03528128548884e-12,-1.05949827908127e-09,-2.02255449941899e-13,-1.58728237043124e-13,-0.0172184896073348,-2.96118840492291e-13,0.0712546000734553,0.0107391883658697,-6.75567408171424e-13;-5.84963411962573e-12,-0.00117889230509445,7.15113895545733e-12,2.45422903005484e-11,-0.00105595742558202,-1.80552957695678e-11,5.78790701775261e-11,-0.00194042940205521,-4.70987636887502e-11,2.18202859255458e-12,0.0476277699707621,7.60160531413741e-12,0.992550366042633,-4.93361932385028e-11,-0.00260019028049652,0.0168162889268220,-6.25325921584708e-12,-0.00363266527808244,-1.64388442021062e-06,-4.12289159137162e-06,-4.05352408159068e-06,-2.15245756232103e-06,2.15245756774759e-06,4.05352409618208e-06,4.12289156019029e-06,1.64388441849568e-06,-2.91127566973714e-10,8.60026801804268e-08,-5.18552993187499e-11,-0.154199475954967,53.7137517281098,0.00668453364580562,-0.00136258063438813,6.20112743460581e-11,0.0121853173938030,-1.49123953294860e-10,-9.32665558328669e-12,-0.000863828823355688;0.00135535665571131,1.48636801593909e-13,-0.000484091971225346,-4.08802182632046e-05,1.54671831091108e-12,0.000390448163426973,0.000107211382712980,3.23775893420948e-12,0.000541415188529374,0.0461721242811999,3.41495128891922e-12,0.0695995261668500,4.59704379007459e-12,0.984632262906035,-1.48022392407580e-12,5.39648702115411e-13,0.0168575418021172,-8.22251913551291e-13,3.39313529750815e-06,7.21696570161753e-07,-1.76123770774508e-06,-5.13022436675658e-06,-5.13022436321228e-06,-1.76123770746780e-06,7.21696570523644e-07,3.39313529785504e-06,-0.148951482980049,52.5991300059498,0.00777843157698512,2.31865198761614e-11,-6.52079270368441e-09,2.44741290989480e-12,3.96519690150017e-12,-0.0138074687069201,2.18914787611682e-12,-0.000121945259053695,-0.00145260472711112,4.84070849785920e-12;-1.18870271601390e-12,4.47423660734024e-05,1.00634427465185e-12,5.38676394014676e-12,9.01104527535229e-05,-2.13812129559964e-12,1.23190743644216e-11,7.38635833308751e-05,-5.65108481336586e-12,-2.44430413508315e-13,-0.00349259832243745,-7.55090445166709e-13,-3.04680008209394e-05,-1.10058748004311e-11,0.996888048848994,6.49584293752042e-05,-1.13310879257534e-12,0.0191765934624780,-3.97174365397484e-07,4.90421659206005e-07,-4.07400172604124e-07,5.30025547128751e-07,-5.30025547615329e-07,4.07400169050293e-07,-4.90421673970309e-07,3.97174364915274e-07,-4.97406474884100e-11,1.29666917966972e-08,-1.30012339958125e-11,0.00312931575970456,-1.05404500900708,-5.39913761512241e-05,0.00580759011989080,1.79725281416691e-11,-0.000153545829046392,-3.13386635659736e-11,3.80942030372557e-12,0.000123495233228339;-6.36524814394118e-10,-0.193499200551658,6.55501533677084e-10,2.55035233115304e-09,-0.0538590279690024,-1.80739321842625e-09,5.99038231502137e-09,-0.130168002027112,-4.74547779886159e-09,1.31669106455253e-09,0.498696579833394,2.66018412473138e-09,-1.08623432633003,-5.33845080925834e-09,-0.394728029247636,0.689511685826995,-6.49339559421049e-10,0.00205984554082726,-0.000145412165696185,-0.000404950282222678,-0.000415190229347881,-0.000173577961647783,0.000173577962135048,0.000415190230721988,0.000404950278743116,0.000145412165473068,-3.00581432113173e-08,8.83906533815268e-06,-5.43776901349465e-09,-14.8846088056282,5212.61761807981,0.701784070408030,-0.0196013816914507,6.51032545485727e-09,1.24614479183288,-1.53040434247715e-08,-8.15376177215386e-10,0.0176945033201283;0.238305076924634,1.20108529787981e-11,0.145134187357462,0.0619481996766062,1.53679633516625e-10,-0.0545889227108924,0.152213732110094,3.23876027493189e-10,-0.172791656216932,0.336064902203613,1.86090825961309e-10,2.03817340318804,4.43130813847530e-10,-0.988258977206765,-1.51467074786755e-10,5.33942113945751e-11,0.683700800321111,-8.25681314933621e-11,0.000390343573473115,0.000160359070248584,-0.000157499635649457,-0.000452988040772705,-0.000452988040420674,-0.000157499635619704,0.000160359070285436,0.000390343573505736,-15.3256354242107,5402.58559905456,0.714025413486564,2.30385399782975e-09,-6.46244158284137e-07,2.47549790139517e-10,3.97753376709662e-10,-1.21348609753675,2.22588465301693e-10,-0.684159030165831,-0.228682155501741,4.93083973514834e-10;-1.24904421007986e-10,0.00974584033744750,8.32520219771833e-11,5.23882956921284e-10,0.00288441020439310,-2.01393732217380e-10,1.19426594718287e-09,0.00686759581954177,-5.37317311158716e-10,3.25255590730388e-10,-0.0489225772953374,3.18713001549491e-10,0.0208644497727154,-1.12322096107318e-09,-0.300189562703525,0.00591104957837740,-1.10890033494619e-10,0.882384659517402,-4.08453723074371e-05,4.82803703418658e-05,-3.98452156164086e-05,4.98170214059801e-05,-4.98170214634321e-05,3.98452152642736e-05,-4.82803718040750e-05,4.08453722520462e-05,-4.84030543202954e-09,1.25923156147416e-06,-1.26936485024450e-09,0.273865080878213,-93.7554120595589,-0.00770566453809476,0.569654741325759,1.74886620433414e-09,-0.0172104303516067,-3.01906495430771e-09,3.80252457103277e-10,0.00499957613447445;10.5474308547245,27.6364609550378,-113.336784277836,-220.381976094404,313.069654190565,284.764993248806,-532.190111622525,658.920965967434,744.664140731184,-147.371412111676,452.388926921263,-874.364837549071,914.590657341983,-582.114119621997,667.895647987449,110.965358362699,-238.383233437437,195.269168882582,0.228054350982145,-0.0499792157761230,-0.0260619138256815,0.0398111126482092,0.0769870077989163,0.0466560789100355,0.00237904389588065,-0.0379325148067176,-11409.3291301106,4131605.72234880,1167.10362580344,4784.98872562330,-1352096.68735007,493.494174908917,-947.995063089696,-1678.53384624211,436.470188981760,1433.74648430097,251.271611597557,991.556739096444;-95.1919737062551,-38.9539580658976,89.0507433028752,385.013588415143,-125.829668936671,-234.464990133587,896.745599067652,-259.242746951499,-616.709097493008,157.460137445523,40.5780689052043,260.045109589905,542.492987176273,-842.055165189096,-431.539391611342,232.778990359329,-104.404956122143,-191.769437994593,-0.0317626681593237,0.238739324770860,-0.0228567892968425,0.0130252087511624,0.0645121348359687,0.0803660862559745,0.0460668885303263,-0.00585576666990664,-4791.50371473963,1405128.48856557,-825.219723211579,11451.0903085247,-4196000.78942359,-966.479177229011,924.580369685464,1024.59248062092,-1444.14618679108,-2284.39917327703,-5.45782413086928,-543.841709896304;115.673965673058,-37.2234912403444,-171.895765722149,-481.186731674263,-124.478143775077,474.096586058218,-1157.60025456258,-256.401243779260,1236.47232272658,-111.518581583074,8.29202118669722,-568.386889475886,542.556047459043,840.923177405171,588.655364320819,233.001653489843,100.633178274574,189.021630817667,-0.0121084185548415,-0.0423932359960476,0.208839001050506,-0.0590908992368255,-0.0282721939225922,0.0220856766001318,0.0562553000783051,0.0298947621086883,4816.88747185196,-1430633.46423542,1010.03346408577,11459.4173687484,-4197531.75619884,-963.677015167946,-918.199496902926,-1090.79175404258,-1441.21902660456,2992.89677562160,546.303137075482,-539.944058574220;10.4477077400623,25.7476621094454,29.3436118336802,120.585251246372,309.027649420690,-41.9714746868279,262.828639436560,650.398580042358,-116.549180086848,188.676736577294,477.287261116963,552.846011039397,902.483739312891,578.917329079891,-354.354998584527,108.590488311189,232.593587063961,-185.579780950203,0.0358699526663853,-0.0182383360142974,-0.0468187403364153,0.210326314019273,-0.0342395644564416,0.00908624145559866,0.0529887846020713,0.0685624228710113,11336.4054920830,-4120043.51394578,-967.194422067528,4677.87247029740,-1318514.57481990,489.750883305014,895.110999201090,1590.78769556727,435.250614472249,-703.151223608002,290.496560964057,979.759692218626;10.4477077963860,-25.7476621073094,29.3436117782566,120.585251028777,-309.027649391074,-41.9714745348143,262.828638921563,-650.398579976047,-116.549179686321,188.676736445976,-477.287261074718,552.846010813932,-902.483739200938,578.917329547073,354.354998539995,-108.590488296491,232.593587119559,185.579780926969,0.0685624228763908,0.0529887847232133,0.00908624131911404,-0.0342395645390898,0.210326314031816,-0.0468187404649037,-0.0182383358831430,0.0358699526801492,11336.4054945860,-4120043.51467134,-967.194421561818,-4677.87246965021,1318514.57463561,-489.750883233578,-895.110999089145,1590.78769503716,-435.250614405498,-703.151222493624,290.496560999859,-979.759692075097;115.673965872181,37.2234912444128,-171.895765931387,-481.186732483052,124.478143823758,474.096586614937,-1157.60025645739,256.401243872712,1236.47232418531,-111.518581961022,-8.29202112711372,-568.386890118031,-542.556047381610,840.923179083439,-588.655364335315,-233.001653482281,100.633178477369,-189.021630824109,0.0298947621396144,0.0562553006831111,0.0220856764457993,-0.0282721940085445,-0.0590908992791383,0.208839000900494,-0.0423932353857091,-0.0121084185178203,4816.88748123868,-1430633.46698018,1010.03346581287,-11459.4173685051,4197531.75613362,963.677015193338,918.199496937477,-1090.79175617046,1441.21902661910,2992.89678054731,546.303137275827,539.944058590794;-95.1919735559661,38.9539580618655,89.0507431299295,385.013587816112,125.829668889997,-234.464989658094,896.745597651500,259.242746862273,-616.709096249790,157.460137156697,-40.5780689577335,260.045108995322,-542.492987250667,-842.055163952454,431.539391625140,-232.778990366331,-104.404955963464,191.769438002629,-0.00585576664330210,0.0460668888477796,0.0803660860341281,0.0645121347849095,0.0130252086521262,-0.0228567895224558,0.238739325083473,-0.0317626681384344,-4791.50370728062,1405128.48632141,-825.219721992859,-11451.0903087806,4196000.78949384,966.479177207035,-924.580369724902,1024.59247919589,1444.14618678249,-2284.39916956561,-5.45782377729615,543.841709851339;10.5474309614493,-27.6364609571738,-113.336784371321,-220.381976528266,-313.069654218536,284.764993486478,-532.190112631486,-658.920966030383,744.664141358792,-147.371412334714,-452.388926957046,-874.364837825156,-914.590657452337,-582.114118700481,-667.895647943492,-110.965358376992,-238.383233336335,-195.269168857602,-0.0379325147826748,0.00237904432424405,0.0466560788510382,0.0769870078246340,0.0398111125732920,-0.0260619138923558,-0.0499792153573356,0.228054350998834,-11409.3291256169,4131605.72110513,1167.10362683266,-4784.98872629000,1352096.68754117,-493.494174978270,947.995062972157,-1678.53384749321,-436.470189043867,1433.74648666723,251.271611480988,-991.556739270786;0.000353182745296549,1.68529222142224e-14,0.000290878424324814,0.000101932820032915,2.15611417193528e-13,-0.000124998076627298,0.000263527793913907,4.54692936934012e-13,-0.000336606529399811,-0.000281852867865929,2.58652685327606e-13,0.000903163981541738,6.21438661015741e-13,-0.00123521029510402,-2.13288131130983e-13,7.47444222590430e-14,0.00236665346786404,-1.16224790678066e-13,5.70362289923810e-07,2.55505851578250e-07,-2.06187020601875e-07,-6.11631747229116e-07,-6.11631746734109e-07,-2.06187020559200e-07,2.55505851630224e-07,5.70362289969635e-07,-0.740563698840106,-92.2375853885064,0.000987101345175814,3.22598288401122e-12,-9.04083369685275e-10,3.48597946087319e-13,5.59718215441858e-13,-0.00164613125577580,3.13813817610727e-13,-0.00121712732356313,-0.000370080957620936,6.95380313910887e-13;3.18218602559826e-06,1.68199436815878e-16,1.01314581256410e-06,7.04601504089955e-07,2.16255450231611e-15,-1.86579001181258e-07,1.59411261407143e-06,4.55470148918688e-15,-1.29934762642735e-06,1.15277590326266e-05,2.60479992555848e-15,5.13363910577981e-05,6.23636049024968e-15,-1.55314655596496e-05,-2.12362324364844e-15,7.53548665081315e-16,2.36466072545321e-05,-1.15771452471132e-15,5.22878450354984e-09,1.91405694800265e-09,-2.40379387307937e-09,-6.65532737944377e-09,-6.65532737450265e-09,-2.40379387267152e-09,1.91405694851838e-09,5.22878450400772e-09,0.00259547525661215,0.0771822595938074,1.02024261496331e-05,3.25062218097444e-14,-9.12792657751184e-12,3.46988911659257e-15,5.57898895015677e-15,-1.77192411515730e-05,3.11599766241425e-15,-6.67711074307474e-06,-2.61778986984692e-06,6.89985059101883e-15;-0.000100954350961048,-7.62396600555501e-15,1.98987291739261e-05,0.000122832385982516,-7.59085649425006e-14,5.34830615530796e-05,0.000202847045802978,-1.59958190754642e-13,5.00838601450709e-05,-0.00281198003378160,-1.91204328288373e-13,-0.00399294050167010,-2.35130476097323e-13,-0.00607772987930431,8.06724469281071e-14,-2.84385652636522e-14,-0.000907009401286425,4.40381610312670e-14,-1.86250773930097e-07,-4.19082151491034e-08,9.81410486745436e-08,2.80545036085285e-07,2.80545035891935e-07,9.81410486602561e-08,-4.19082151680191e-08,-1.86250773948550e-07,0.00786780420242807,-3.04302371342273,0.997985472769938,-1.23630313144358e-12,3.54972381344731e-10,-1.26936970175079e-13,-2.10712205034865e-13,-0.00804277891983776,-1.04599367211208e-13,-0.000739704630940062,-0.000976887040817391,-2.88520534377729e-13;-8.97222111448832e-13,-0.000288305918054426,8.87937939785355e-13,3.58107318537678e-12,-7.08996175614570e-05,-2.52185741500289e-12,8.40744486543698e-12,-0.000161008402315617,-6.63464368944555e-12,1.94170013071222e-12,-0.000199882672017240,4.45369451202311e-12,-0.00159944228631296,-7.54904891028387e-12,-0.000570799189256615,0.00237546928302213,-9.13494542066456e-13,2.62774815835426e-06,-1.98627333214144e-07,-5.69459164297885e-07,-5.83851085701690e-07,-2.37139017773396e-07,2.37139018439915e-07,5.83851087619916e-07,5.69459159384877e-07,1.98627332883882e-07,-4.22561513596093e-11,1.24239083462454e-08,-7.64178374160591e-12,-0.739780122587779,-92.5578593134517,0.000993496423653332,-2.63325713654569e-05,9.13363980937657e-12,0.00175759505166285,-2.14237373966109e-11,-1.12433910442442e-12,4.66894486837589e-05;-8.92876751336528e-15,-2.55442780501221e-06,9.59150909591556e-15,3.58665395953799e-14,-8.02696336554167e-07,-2.56068998378790e-14,8.42846695703857e-14,-2.04954067363306e-06,-6.70807460443335e-14,1.81217743930641e-14,1.60403545402011e-05,2.93206178572681e-14,-1.45302159026971e-05,-7.45272904208185e-14,-5.38470087383313e-06,2.37261508623133e-05,-9.11180390393942e-15,3.16188607154569e-08,-2.10045886000210e-09,-5.68584240730101e-09,-5.82969960129215e-09,-2.50692572120453e-09,2.50692572825781e-09,5.82969962075503e-09,5.68584235866251e-09,2.10045885705738e-09,-4.22098204208285e-13,1.24147849050735e-10,-7.64003699431346e-14,0.00260015508787061,0.0750171189788421,9.78728032121141e-06,-2.87573785491144e-07,9.16452869846892e-14,1.74446610514282e-05,-2.15963387186218e-13,-1.17082220629648e-14,2.97376163113251e-08;3.18932540900971e-13,3.43999918568058e-05,-3.93701659482394e-13,-1.36425918332200e-12,-1.25297664654172e-05,9.81142640321697e-13,-3.21060160233792e-12,-0.000270814748179796,2.56135613730403e-12,8.01345870205237e-14,-0.00324313664165478,-2.67042348778596e-13,-0.00647480442692341,2.72187977122267e-12,-0.00113433289564282,-0.000901309333032493,3.44665007646579e-13,-3.57296757575490e-06,1.00881762077114e-07,2.18302276184748e-07,2.24038663564731e-07,1.19891370727545e-07,-1.19891371014041e-07,-2.24038664313278e-07,-2.18302274343765e-07,-1.00881761982792e-07,1.60177786668329e-11,-4.71149454829956e-09,2.90639973978047e-12,0.00824015750982155,-3.05412120694779,0.998496657502223,4.17763164678388e-05,-3.49647956824844e-12,0.00854964780001260,8.25285477751707e-12,4.20893003466299e-13,0.000898419344781465;1.18987553141844e-13,-3.93460230854945e-06,-1.02045364746330e-13,-5.44733030433566e-13,-1.04127870766092e-05,2.15523952022752e-13,-1.24274766054635e-12,1.44223852514525e-05,5.71845634867349e-13,6.55039807288024e-14,0.000384260994058673,9.49423535193697e-14,5.65869328301555e-06,1.10692183834046e-12,-0.00518215572113625,-6.60497309695687e-06,1.14401282751098e-13,-0.00193499254223343,3.98750676745106e-08,-4.94702412609308e-08,4.11240093220280e-08,-5.37211367998475e-08,5.37211368485247e-08,-4.11240089632248e-08,4.94702427499107e-08,-3.98750676261564e-08,5.02425301152031e-12,-1.30999965571917e-09,1.31253722466136e-12,-0.000319369089779139,0.107409286970317,5.18732392246510e-06,0.999413946007820,-1.81453892780710e-12,1.52612967431703e-05,3.16543150976187e-12,-3.84490878830255e-13,-1.32329088397443e-05;0.00222017689538015,-3.13480695601871e-14,-0.00258919018517321,-0.00676788930970489,-2.97471432411942e-13,0.00639086258726846,-0.0195494456289820,-5.72088777722873e-13,0.0174343805968720,-0.00506742577779813,-2.19462335483765e-13,-0.0122081991932019,-4.57779620585763e-13,0.0157524557143302,-1.16268854921297e-13,-1.42566081323013e-15,0.000177302301173011,2.01123229958223e-14,-8.07427267614193e-08,1.24537167628627e-07,-5.13792550077670e-07,-3.26106358716818e-07,-3.26106358653167e-07,-5.13792550084297e-07,1.24537167627236e-07,-8.07427267679887e-08,0.00191904265304069,7.84072473522328,0.0680155297897229,-1.12442011804851e-13,-2.43147576783026e-10,-4.16228288584837e-14,-1.05761094720914e-13,0.888057474315240,-3.10291897711162e-13,0.0483951975729371,0.0678058757536845,6.97932285193662e-13;1.39220351475757e-14,-0.000744723208145352,4.72575833705726e-14,-3.48697892666480e-14,-0.00354652497713625,-1.09262985153550e-13,-7.64132437628029e-14,-0.00956441832397961,-3.34526608985591e-13,-1.13085922216629e-13,-0.00779594013139257,1.44031089242685e-13,-0.0136775890088278,2.22214982685269e-13,-0.00217716395816912,-0.000253785231292113,1.00817878435200e-14,-6.90711693508245e-05,2.42047106970014e-07,1.68330536065694e-07,1.80607965654321e-07,2.71676006605373e-07,-2.71676006597291e-07,-1.80607966032443e-07,-1.68330534478299e-07,-2.42047106946624e-07,3.26453042823874e-14,1.37100196145300e-10,5.59001408578333e-13,-8.95299357866600e-05,-7.79747764664264,-0.0482573730551804,0.00115510062270606,-4.15201676616901e-13,0.906811583380974,-8.01547033902391e-13,-1.59499209128007e-12,0.0351807428223788;-0.00243431299552198,3.10665384034099e-14,0.0183441674860574,0.0119204061642287,1.29288057523024e-13,-0.0543920719198099,0.0367778657986174,1.43038830971800e-13,-0.140107851833775,-0.0196443380850834,-5.22453561790093e-13,0.0641771321582431,-1.70257207875202e-13,0.0195291624210428,1.39666666839721e-13,-7.27338062770524e-14,0.00111004275022141,-1.75147432317113e-13,2.46862508125633e-06,1.83209346175418e-06,1.58495402447323e-06,9.07921516463860e-07,9.07921517128746e-07,1.58495402441888e-06,1.83209346168903e-06,2.46862508123184e-06,-0.00991821657587839,18.7022138990667,-0.000203027450237282,9.04440839795417e-13,-4.41033371273889e-10,-5.36326064084727e-13,4.87412961368464e-13,0.0501426225890249,-1.07068490144989e-12,0.577704274024062,-0.129782458593238,3.63728426294554e-12;0.00224415966197428,3.37095146137973e-14,-0.00340544707017282,-0.0107195505312167,3.80677383254752e-13,0.00897947574557099,-0.0253631586191607,6.87549708219598e-13,0.0235143995639134,-0.00434284875174237,4.24974479267531e-13,-0.0151045948876787,3.18548504038131e-13,0.0127908524406048,4.61015722562204e-14,1.36751631057415e-14,0.000106025211860828,2.97904419642549e-14,-2.11388533709222e-07,3.32468060404758e-08,-5.74932303413353e-07,-3.43134227222032e-07,-3.43134227166687e-07,-5.74932303405569e-07,3.32468060533392e-08,-2.11388533682643e-07,0.00199338575459236,6.19265797720319,0.0289343648246839,-2.36149699750531e-13,1.05691494208445e-10,-5.75821580085104e-14,-1.14813339606238e-13,-0.0369173962464197,-1.60504764501196e-13,0.0655710444516398,0.999434657591835,-3.72308194956790e-13;9.40365946004938e-14,-0.000887030665474114,5.06721609836484e-14,-3.49221014135921e-13,-0.00757982308741716,-2.01320005907348e-13,-7.79158553636452e-13,-0.0159048194876942,-4.84834048561337e-13,-4.07186919950270e-13,-0.00950234091039628,5.71746122871810e-14,-0.0142829926512781,9.33328914879126e-13,-0.00252959765127275,-0.000277854975689861,3.53527673135965e-14,-8.87267848553184e-05,2.81256226707767e-07,1.75975929943070e-07,1.90697738200665e-07,3.15130084868844e-07,-3.15130084889202e-07,-1.90697738618549e-07,-1.75975928211581e-07,-2.81256226649056e-07,5.37149175928555e-14,5.17680152783292e-10,1.62466393501997e-12,0.000291093738502898,-8.42203288551758,-0.0186190197759809,0.000462052851478610,-1.59016977634148e-12,-0.0215868151845720,-5.50459877104653e-13,-1.61606811470709e-12,0.968460077536758];
                obj.hor_B = [-0.0111234593484477,-2.64156176005413e-13,0.0219778177572083,-0.0120840843659835,-4.37611264949960e-13,0.0391568476895057,-1.96940440775607,4.16770267470606e-12,0.804000863876940;2.67996023531979e-13,0.00124809218297858,-3.50730567732278e-13,4.28406550151883e-13,-0.00477619997070969,-6.18908258584280e-13,9.01430228674929e-12,1.36560923120279,-1.61848711787863e-11;0.0394163105622034,6.37893013705803e-14,-0.145444738953257,0.0777023711167019,-8.54303393141434e-14,-0.235092432851980,2.75789266658598,2.15144728461040e-11,-8.27785202478113;0.0172821004362875,5.41477965982415e-16,-7.76284958996867e-05,0.0100927505039122,8.59935628274081e-16,-0.000131703239346280,0.00210648812815543,1.56725773564189e-14,-0.00356624608662123;7.58797913218468e-16,0.0172558082781974,-4.71440901175643e-16,1.30373979683763e-15,0.0100443848380295,-7.93851571808398e-16,2.95422201540488e-14,0.000643268565035052,-1.72174212764711e-14;-7.76411601460004e-05,-2.03802811196465e-16,0.0175430860443157,-0.000158279075703354,1.40377174909015e-16,0.0105263019747542,-0.00477795338861792,-3.50235192136373e-14,0.0138417842622731;0.00994046492737615,1.23443991986254e-15,-0.000201644436003399,0.0173181451298691,1.86883158860107e-15,-0.000343291467285455,0.0240276956259771,4.08133275759250e-14,-0.00941846788152439;1.12067620803836e-15,0.00987884440482443,-3.51695899918208e-16,1.82215553424738e-15,0.0171805613050927,-6.03690513381920e-16,4.33702734312390e-14,0.0196843455894339,-9.10487929711027e-15;-0.000164542913444833,-4.07495799987595e-16,0.0104823458273925,-0.000337390312099648,3.07318983029742e-16,0.0182184957653064,-0.0102058630076888,-7.11770552396052e-14,0.0485228312616380;0.00208833557847877,4.29196350293468e-14,-0.00581944405086010,0.0189347445126442,5.48708827928438e-14,-0.00932178447990853,0.579433189338463,1.87455344916010e-12,-0.328766656045960;-8.69436411349119e-14,0.000284240294247395,1.15602943653369e-13,-1.79854016735823e-13,0.0156512533423510,1.96326007647953e-13,-2.37496212639970e-12,0.370904590290668,5.65835156863259e-12;-0.00499560981443709,-6.43298689595685e-15,0.0188998235290590,-0.00972533653909620,1.07742331253762e-14,0.0455972648370522,-0.367808319561552,-2.70849373996067e-12,1.48041509231224;7.75274994089883e-13,0.000200140692864034,-1.20046928901269e-12,1.32182663022459e-12,6.30442808594425e-05,-2.17301190863405e-12,4.70529781106944e-11,-0.0489884034853879,-4.32095995509350e-11;0.000247407730525917,2.65208337691850e-14,-0.000448571595974749,0.000459615671809551,6.67521422399718e-14,-0.000718422310350703,-0.0454256508594213,-9.06242613794458e-13,-0.0697877514609953;1.45935964794245e-13,-3.93911137805886e-05,-1.52312691694785e-13,2.31195233498358e-13,3.44678907874264e-05,-2.89617393046337e-13,1.02644550425475e-11,0.00357012104563420,-3.39409724244760e-12;8.20203538290832e-11,-0.00241437708450547,-1.37524580804477e-10,1.54778044229133e-10,0.00395226221516279,-2.35076742296677e-10,3.77808491826944e-09,-0.577048056878673,-6.26638786004520e-09;0.00703118523425714,2.93288973796174e-12,-0.0312989097449079,0.0231287155444984,5.44057859612481e-12,-0.0442776249674946,-0.139410644876758,6.45801912885526e-11,-2.17849098331437;1.45960503094973e-11,0.000109713399759826,-1.66299005986929e-11,2.64660517837903e-11,-0.000421633349582233,-2.88304470291940e-11,6.47104215110065e-10,0.0526345751284237,-7.15539050147778e-10;-10.1031228601672,6.21628541550200,26.8774141471178,-21.2058904008357,11.7577157324883,44.0303789060097,-338.179230958074,57.3258442370814,1454.00319292261;11.6724143575961,-2.36364206646904,-17.4289621683468,21.4733174946285,-3.24615829857657,-30.1893215402121,595.052822592590,-227.593913759231,-725.101192276420;-17.1211774566395,-2.40681845817824,34.4916463403321,-32.4302019217075,-3.33790304371598,59.2427256174066,-905.920354825663,-193.452677768532,1515.59541378821;4.50896966130350,6.20179527969875,-9.48530900713983,9.94122778811205,11.7283289984256,-14.4471374463544,24.5345494128795,26.1148805094751,-643.843824041440;4.50896965433979,-6.20179527913163,-9.48530899569077,9.94122777486433,-11.7283289972786,-14.4471374268167,24.5345491048778,-26.1148804969150,-643.843823511526;-17.1211774820245,2.40681845899166,34.4916463806141,-32.4302019685386,3.33790304521505,59.2427256872225,-905.920356052457,193.452677774495,1515.59541553419;11.6724143379535,2.36364206567804,-17.4289621340247,21.4733174576277,3.24615829713932,-30.1893214812397,595.052821665892,227.593913750038,-725.101190736177;-10.1031228730849,-6.21628541606281,26.8774141648161,-21.2058904242783,-11.7577157335879,44.0303789369239,-338.179231576527,-57.3258442530089,1454.00319367005;7.53027923287787e-06,4.26829859163429e-15,-2.56590847721120e-05,1.52253372302208e-05,7.69008120157640e-15,-4.27847361017930e-05,0.000591845742421765,9.33105255056695e-14,-0.00114363471435229;1.33148260090904e-07,3.95630364495058e-17,-6.77277060287389e-07,5.25035199381885e-07,7.56641101094824e-17,-9.00179975341300e-07,-9.12370194576221e-06,9.20037753448285e-16,-5.28168181429444e-05;-1.70005991747319e-05,-1.43862101968279e-15,2.15123671724517e-05,9.00890024065735e-05,-3.20468859809300e-15,0.000119845028192513,0.00307551697103691,6.64557780091376e-14,0.00413697263975954;1.16646999583437e-13,-2.09310457417357e-06,-2.01702620013674e-13,2.23400655951439e-13,-3.64833140946001e-06,-3.40788427783040e-13,5.21187499208167e-12,0.000101936287116030,-9.50838224010522e-12;1.13414579624431e-15,-4.90247977884660e-08,-1.82811237184237e-15,2.10728432914942e-15,1.48673900292617e-07,-3.16448986305857e-15,5.35240331998990e-14,-1.72631930037996e-05,-8.01353398256374e-14;-4.16055461153468e-14,-2.53263290880202e-05,6.62176991208431e-14,-7.27903271251327e-14,0.000144860243888897,1.18167295758742e-13,-2.80649808104686e-12,0.00308240047654281,2.20072244777613e-12;-1.39713132931988e-14,5.07999888907847e-06,1.59878857553898e-14,-2.46419320523933e-14,-2.58139288325625e-05,2.85928263110481e-14,-1.07754616034212e-12,-0.000392286454548232,3.24408725239129e-13;-0.000275986564133611,-6.44938433588919e-15,0.000544768046238743,-0.000563068407587679,-1.00733327979976e-14,0.000926795926268475,-0.0148333994846653,-1.50477755170845e-13,0.0253502451147018;6.31938303279804e-16,-8.24514035148802e-05,-8.86456818851635e-15,2.39426321266803e-15,-0.000183601886054386,-1.49843568940950e-14,9.00731773559938e-14,-0.00218306381098496,-4.33179119936599e-13;0.000996754680308102,4.03773772806394e-15,-0.00375615237937763,0.00204505155931085,1.33908911027886e-15,-0.00640304811648029,0.0618504419719703,4.56817666845255e-13,-0.173653858246443;-0.000370777061220736,6.02575060080826e-15,0.000712718895716667,-0.000710831427534162,1.06799610730621e-14,0.00120932490091879,-0.0175699802571161,1.77944525832065e-14,0.0330874836270274;-6.26220409436905e-15,-0.000150342268200290,-9.42538422864695e-15,-9.93853092656353e-15,-0.000273774251697472,-1.55920893850033e-14,-1.66743799005054e-13,-0.00268839491829440,-4.45992375773046e-13];
                obj.hor_C = [0.00455885318406213,-6.43591372771601e-14,-0.00531657542440924,-0.0138970069426135,-6.10886262211861e-13,0.0131228301292108,-0.0401423204780598,-1.17481973023538e-12,0.0357993012455846,-0.0104053195897068,-4.50079844755609e-13,-0.0250679970048348,-9.39947698026254e-13,0.0323456806705377,-2.38733303370391e-13,-2.92834192823790e-15,0.000364067909149969,4.12983862372512e-14,-1.65795003881046e-07,2.55721363626133e-07,-1.05500818774529e-06,-6.69618270000295e-07,-6.69618269869586e-07,-1.05500818775890e-06,2.55721363623276e-07,-1.65795003894529e-07,0.00394051200486330,16.0999391530047,0.139661310408537,-2.30937077373553e-13,-4.99254099321995e-10,-8.55192464146989e-14,-2.17167697514799e-13,0.751887849065072,-6.37149349294414e-13,0.0993734332646152,0.139230812292945,1.43310222784366e-12;2.85855898851609e-14,-0.00152919516267513,9.70462261862146e-14,-7.16679371339560e-14,-0.00728234165395414,-2.24383400639968e-13,-1.56864764358262e-13,-0.0196393264972296,-6.86871630941906e-13,-2.32141038329225e-13,-0.0160079796185212,2.95583590100343e-13,-0.0280852036308175,4.56294688764852e-13,-0.00447053154348997,-0.000521115957990315,2.07019589003422e-14,-0.000141828933080571,4.97013200434347e-07,3.45645521271213e-07,3.70855674159786e-07,5.57852408212782e-07,-5.57852408196183e-07,-3.70855674936209e-07,-3.45645518011694e-07,-4.97013200386313e-07,6.70477099459184e-14,2.81515430195275e-10,1.14791611133782e-12,-0.000183838429126572,-16.0111367121328,-0.0990904280036712,0.00237185341523609,-8.52560310031814e-13,0.790397046240520,-1.64589564085867e-12,-3.27512128619242e-12,0.0722392174097511;-0.00452057841393466,5.76905818145615e-14,0.0340655649916915,0.0221364840472336,2.40122536578546e-13,-0.101007399895653,0.0682973909048067,2.65608314842992e-13,-0.260183687055701,-0.0364800134028866,-9.70254902038014e-13,0.119178494645705,-3.16175983434809e-13,0.0362661293946958,2.59363550104088e-13,-1.35068555305355e-13,0.00206137637371465,-3.25253145180541e-13,4.58429679131387e-06,3.40224209900373e-06,2.94329815569842e-06,1.68603232880218e-06,1.68603233003689e-06,2.94329815559749e-06,3.40224209888274e-06,4.58429679126838e-06,-0.0184183693058873,34.7304658852143,-0.000377026910946169,1.67951928471306e-12,-8.19005262799291e-10,-9.95512267753308e-13,9.05140023825669e-13,0.0931160691788583,-1.98828773088400e-12,-0.195164807136352,-0.241009180784560,6.75452639759406e-12;0.0281401237185228,4.22691951171694e-13,-0.0427018199708833,-0.134415337405192,4.77333996105349e-12,0.112596070007579,-0.318035491649381,8.62137581090959e-12,0.294853402860462,-0.0544561526684562,5.32882001124013e-12,-0.189400592150166,3.99436152583101e-12,0.160387951108316,5.78076806190500e-13,1.71476021898245e-13,0.00132947874859382,3.73549649251565e-13,-2.65065787967220e-06,4.16890674526098e-07,-7.20923133142584e-06,-4.30265268986834e-06,-4.30265268917437e-06,-7.20923133132825e-06,4.16890674687396e-07,-2.65065787933893e-06,0.0249956020079346,77.6514098251321,0.362815810158166,-2.96107602711825e-12,1.32529861927842e-09,-7.22062049922748e-13,-1.43967094250601e-12,-0.462917195840713,-2.01260480620948e-12,0.822213024539716,0.0714508352932281,-4.66846730701229e-12;1.17914923590707e-12,-0.0111227169312035,6.35391133016860e-13,-4.37919585410308e-12,-0.0950454475493010,-2.52453797137940e-12,-9.77013764202817e-12,-0.199434824396918,-6.07936458710941e-12,-5.10572693321105e-12,-0.119152417434904,7.16965956307196e-13,-0.179098299950782,1.17032545305428e-11,-0.0317193077083816,-0.00348409853550315,4.43298213933223e-13,-0.00111256910338496,3.52674774004212e-06,2.20660968289734e-06,2.39121041017565e-06,3.95150119035492e-06,-3.95150119061019e-06,-2.39121041541562e-06,-2.20660966118574e-06,-3.52674773930591e-06,6.73615128110580e-13,6.49132532404377e-09,2.03723881619642e-11,0.00365010295566628,-105.606143526988,-0.233468914396576,0.00579380541652976,-1.99395666868895e-11,-0.270682902057128,-6.90235915749711e-12,-2.02643139414672e-11,-0.316947784309591];
                obj.hor_D = [-0.000566703594329343,-1.31807111228820e-14,0.00111861246161901,-0.00115618994509731,-2.05855726532676e-14,0.00190305852125420,-0.0304585146399125,-3.09554248553723e-13,0.0520535304634478;1.36495113014267e-15,-0.000169303556048326,-1.81788708638292e-14,4.87618577932987e-15,-0.000377003312024743,-3.08106427492511e-14,1.84888345952650e-13,-0.00448264614699319,-8.89314988837542e-13;0.00185099767370835,7.46740188291604e-15,-0.00697526628535409,0.00379771046344910,2.50469359264994e-15,-0.0118906160185614,0.114857774404993,8.48368440385314e-13,-0.322479436510182;-0.00464927364640490,7.56361920799341e-14,0.00893697460204411,-0.00891330712906049,1.33916038084508e-13,0.0151640513392915,-0.220314724725194,2.23179764599624e-13,0.414892887781448;-7.83048243731214e-14,-0.00188518227957016,-1.18054755322637e-13,-1.24566767833516e-13,-0.00343292923594165,-1.95611591141666e-13,-2.09094764072604e-12,-0.0337105095002455,-5.59245098147310e-12];
            end
            
            if obj.controller_op == -1
                obj.hor_op = [0.106829259352255;0;-0.548558287675047;0;0];
            else
                obj.hor_op = obj.controller_op;
            end
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
            obj.hor_state = zeros( size(obj.hor_A, 1), 1 );
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

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl( obj, name )
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
                    sz = [size(obj.hor_A, 1) 1];
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