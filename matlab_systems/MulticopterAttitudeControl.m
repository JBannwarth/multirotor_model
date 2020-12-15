classdef MulticopterAttitudeControl < matlab.System & matlab.system.mixin.CustomIcon & ...
        matlab.system.mixin.SampleTime & matlab.system.mixin.Propagates
    %MULTICOPTERATTITUDECONTROL PX4 Firmware attitude controller (v1.82)
    %   Original description:
    %   This implements the multicopter attitude and rate controller. It
    %   takes attitude setpoints (vehicle_attitude_setpoint) or rate
    %   setpoints (in acro mode via manual_control_setpoint topic) as
    %   inputs and outputs actuator control messages.
    %   The controller has two loops: a P loop for angular error and a PID
    %   loop for angular rate error.
    %   Publication documenting the implemented Quaternion Attitude Control:
    %       Nonlinear Quadrocopter Attitude Control (2013)
    %       by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
    %       Institute for Dynamic Systems and Control (IDSC), ETH Zurich
    %       https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
    %   To reduce control latency, the module directly polls on the gyro
    %   topic published by the IMU driver.
    %
    %   Based on code from PX4 Firmware (retrieved 2019/01/15, v1.82):
    %       https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp
    %   Written:       J.X.J. Bannwarth, 2019/01/18
    %   Last modified: J.X.J. Bannwarth, 2019/02/05

    %% Public, tunable properties (equivalent to those available on PX4)
    properties (Nontunable)
        acro_expo_rp                  (1,1) {mustBeGreaterThanOrEqual(acro_expo_rp                 ,    0), mustBeLessThanOrEqual(acro_expo_rp                 ,    1)                                              } =  0.69; % MC_ACRO_EXPO [0-1]
        acro_expo_y                   (1,1) {mustBeGreaterThanOrEqual(acro_expo_y                  ,    0), mustBeLessThanOrEqual(acro_expo_y                  ,    1)                                              } =  0.69; % MC_ACRO_EXPO_Y [0-1]
        acro_pitch_max                (1,1) {mustBeGreaterThanOrEqual(acro_pitch_max               ,    0), mustBeLessThanOrEqual(acro_pitch_max               , 1800)                                              } =   720; % MC_ACRO_P_MAX [0-1800]
        acro_roll_max                 (1,1) {mustBeGreaterThanOrEqual(acro_roll_max                ,    0), mustBeLessThanOrEqual(acro_roll_max                , 1800)                                              } =   720; % MC_ACRO_R_MAX [0-1800]
        acro_superexpo_rp             (1,1) {mustBeGreaterThanOrEqual(acro_superexpo_rp            ,    0), mustBeLessThanOrEqual(acro_superexpo_rp            , 0.95)                                              } =   0.7; % MC_ACRO_SUPEXPO [0-0.95]
        acro_superexpo_y              (1,1) {mustBeGreaterThanOrEqual(acro_superexpo_y             ,    0), mustBeLessThanOrEqual(acro_superexpo_y             , 0.95)                                              } =   0.7; % MC_ACRO_SUPEXPOY [0-0.95]
        acro_yaw_max                  (1,1) {mustBeGreaterThanOrEqual(acro_yaw_max                 ,    0), mustBeLessThanOrEqual(acro_yaw_max                 , 1800)                                              } =   540; % MC_ACRO_Y_MAX [0-1800]
        airmode                       (1,1) {mustBeGreaterThanOrEqual(airmode                      ,    0), mustBeLessThanOrEqual(airmode                      ,    2), mustBeInteger(airmode                      )} =     0; % MC_AIRMODE [0-2]
        d_term_cutoff_freq            (1,1) {mustBeGreaterThanOrEqual(d_term_cutoff_freq           ,    0), mustBeLessThanOrEqual(d_term_cutoff_freq           , 1000)                                              } =    30; % MC_DTERM_CUTOFF [0-1000]
        pitch_rate_d                  (1,1) {mustBeGreaterThanOrEqual(pitch_rate_d                 ,    0)                                                                                                          } = 0.003; % MC_PITCHRATE_D [>0]
        pitch_rate_ff                 (1,1) {mustBeGreaterThanOrEqual(pitch_rate_ff                ,    0)                                                                                                          } =     0; % MC_PITCHRATE_FF [>0]
        pitch_rate_i                  (1,1) {mustBeGreaterThanOrEqual(pitch_rate_i                 ,    0)                                                                                                          } =  0.05; % MC_PITCHRATE_I [>0]
        pitch_rate_max                (1,1) {mustBeGreaterThanOrEqual(pitch_rate_max               ,    0), mustBeLessThanOrEqual(pitch_rate_max               , 1800)                                              } =   220; % MC_PITCHRATE_MAX [0-1800]
        pitch_rate_p                  (1,1) {mustBeGreaterThanOrEqual(pitch_rate_p                 ,    0), mustBeLessThanOrEqual(pitch_rate_p                 ,  0.6)                                              } =  0.15; % MC_PITCHRATE_P [0-0.6]
        pitch_p                       (1,1) {mustBeGreaterThanOrEqual(pitch_p                      ,    0), mustBeLessThanOrEqual(pitch_p                      ,   12)                                              } =   6.5; % MC_PITCH_P [0-12]
        pitch_rate_integ_lim          (1,1) {mustBeGreaterThanOrEqual(pitch_rate_integ_lim         ,    0), mustBeLessThanOrEqual(pitch_rate_integ_lim         ,    2)                                              } =   0.3; % MC_PR_INT_LIM [0-2]
        rattitude_thres               (1,1) {mustBeGreaterThanOrEqual(rattitude_thres              ,    0)                                                                                                          } =   0.8; % MC_RATT_TH [>0]
        roll_rate_d                   (1,1) {mustBeGreaterThanOrEqual(roll_rate_d                  ,    0), mustBeLessThanOrEqual(roll_rate_d                  , 0.01)                                              } = 0.003; % MC_ROLLRATE_D [0-0.01]
        roll_rate_ff                  (1,1) {mustBeGreaterThanOrEqual(roll_rate_ff                 ,    0)                                                                                                          } =     0; % MC_ROLLRATE_FF [>0]
        roll_rate_i                   (1,1) {mustBeGreaterThanOrEqual(roll_rate_i                  ,    0)                                                                                                          } =  0.05; % MC_ROLLRATE_I [>0]
        roll_rate_max                 (1,1) {mustBeGreaterThanOrEqual(roll_rate_max                ,    0)                                                                                                          } =   220; % MC_ROLLRATE_MAX [>0]
        roll_rate_p                   (1,1) {mustBeGreaterThanOrEqual(roll_rate_p                  ,    0), mustBeLessThanOrEqual(roll_rate_p                  ,  0.5)                                              } =  0.15; % MC_ROLLRATE_P [0-0.5]
        roll_p                        (1,1) {mustBeGreaterThanOrEqual(roll_p                       ,    0), mustBeLessThanOrEqual(roll_p                       ,   12)                                              } =   6.5; % MC_ROLL_P [0-12]
        roll_rate_integ_lim           (1,1) {mustBeGreaterThanOrEqual(roll_rate_integ_lim          ,    0)                                                                                                          } =   0.3; % MC_RR_INT_LIM [>0]
        tpa_breakpoint_d              (1,1) {mustBeGreaterThanOrEqual(tpa_breakpoint_d             ,    0), mustBeLessThanOrEqual(tpa_breakpoint_d             ,    1)                                              } =     1; % MC_TPA_BREAK_D [0-1]
        tpa_breakpoint_i              (1,1) {mustBeGreaterThanOrEqual(tpa_breakpoint_i             ,    0), mustBeLessThanOrEqual(tpa_breakpoint_i             ,    1)                                              } =     1; % MC_TPA_BREAK_I [0-1]
        tpa_breakpoint_p              (1,1) {mustBeGreaterThanOrEqual(tpa_breakpoint_p             ,    0), mustBeLessThanOrEqual(tpa_breakpoint_p             ,    1)                                              } =     1; % MC_TPA_BREAK_P [0-1]
        tpa_rate_d                    (1,1) {mustBeGreaterThanOrEqual(tpa_rate_d                   ,    0), mustBeLessThanOrEqual(tpa_rate_d                   ,    1)                                              } =     0; % MC_TPA_RATE_D [0-1]
        tpa_rate_i                    (1,1) {mustBeGreaterThanOrEqual(tpa_rate_i                   ,    0), mustBeLessThanOrEqual(tpa_rate_i                   ,    1)                                              } =     0; % MC_TPA_RATE_I [0-1]
        tpa_rate_p                    (1,1) {mustBeGreaterThanOrEqual(tpa_rate_p                   ,    0), mustBeLessThanOrEqual(tpa_rate_p                   ,    1)                                              } =     0; % MC_TPA_RATE_P [0-1]
        yaw_rate_d                    (1,1) {mustBeGreaterThanOrEqual(yaw_rate_d                   ,    0)                                                                                                          } =     0; % MC_YAWRATE_D [>0]
        yaw_rate_ff                   (1,1) {mustBeGreaterThanOrEqual(yaw_rate_ff                  ,    0)                                                                                                          } =     0; % MC_YAWRATE_FF [>0]
        yaw_rate_i                    (1,1) {mustBeGreaterThanOrEqual(yaw_rate_i                   ,    0)                                                                                                          } =   0.1; % MC_YAWRATE_I [>0]
        yaw_rate_max                  (1,1) {mustBeGreaterThanOrEqual(yaw_rate_max                 ,    0), mustBeLessThanOrEqual(yaw_rate_max                 , 1800)                                              } =   200; % MC_YAWRATE_MAX [0-1800]
        yaw_rate_p                    (1,1) {mustBeGreaterThanOrEqual(yaw_rate_p                   ,    0), mustBeLessThanOrEqual(yaw_rate_p                   ,  0.6)                                              } =   0.2; % MC_YAWRATE_P [0-0.6]
        yaw_auto_max                  (1,1) {mustBeGreaterThanOrEqual(yaw_auto_max                 ,    0), mustBeLessThanOrEqual(yaw_auto_max                 ,  360)                                              } =    45; % MC_YAWRAUTO_MAX [0-360]
        yaw_p                         (1,1) {mustBeGreaterThanOrEqual(yaw_p                        ,    0), mustBeLessThanOrEqual(yaw_p                        ,    5)                                              } =   2.8; % MC_YAW_P [0-5]
        yaw_rate_integ_lim            (1,1) {mustBeGreaterThanOrEqual(yaw_rate_integ_lim           ,    0)                                                                                                          } =   0.3; % MC_YR_INT_LIM [>0]
        man_throttle_min              (1,1) {mustBeGreaterThanOrEqual(man_throttle_min             ,    0), mustBeLessThanOrEqual(man_throttle_min             ,    1)                                              } =  0.08; % MPC_MANTHR_MIN [0-1]
        man_tilt_max_deg              (1,1) {mustBeGreaterThanOrEqual(man_tilt_max_deg             ,    0), mustBeLessThanOrEqual(man_tilt_max_deg             ,   90)                                              } =    35; % MPC_MAN_TILT_MAX [0-90]
        yaw_rate_scaling              (1,1) {mustBeGreaterThanOrEqual(yaw_rate_scaling             ,    0), mustBeLessThanOrEqual(yaw_rate_scaling             ,  400)                                              } =   200; % MPC_MAN_Y_MAX [0-400]
        throttle_hover                (1,1) {mustBeGreaterThanOrEqual(throttle_hover               ,  0.1), mustBeLessThanOrEqual(throttle_hover               ,  0.8)                                              } =   0.5; % MPC_THR_HOVER [0.1-0.8]
        throttle_max                  (1,1) {mustBeGreaterThanOrEqual(throttle_max                 ,    0), mustBeLessThanOrEqual(throttle_max                 ,    1)                                              } =     1; % MPC_THR_MAX [0-1]
        board_rotation_param          (1,1) {mustBeGreaterThanOrEqual(board_rotation_param         ,    0), mustBeLessThanOrEqual(board_rotation_param         ,   34), mustBeInteger(board_rotation_param         )} =     0; % SENS_BOARD_ROT [0-34]
        board_offset_x                (1,1) {                                                                                                                                                                       } =     0; % SENS_BOARD_X_OFF 
        board_offset_y                (1,1) {                                                                                                                                                                       } =     0; % SENS_BOARD_Y_OFF 
        board_offset_z                (1,1) {                                                                                                                                                                       } =     0; % SENS_BOARD_Z_OFF
        bat_scale_en                  (1,1) {mustBeGreaterThanOrEqual(bat_scale_en                 ,    0), mustBeLessThanOrEqual(bat_scale_en                 ,    1), mustBeInteger(bat_scale_en                 )} =     0; % MC_BAT_SCALE_EN [0-1]
        throttle_curve                (1,1) {mustBeGreaterThanOrEqual(throttle_curve               ,    0), mustBeLessThanOrEqual(throttle_curve               ,    1), mustBeInteger(throttle_curve               )} =     0; % MPC_THR_CURVE [0-1]
        vehicle_status_is_rotary_wing (1,1) {mustBeGreaterThanOrEqual(vehicle_status_is_rotary_wing,    0), mustBeLessThanOrEqual(vehicle_status_is_rotary_wing,    1), mustBeInteger(vehicle_status_is_rotary_wing)} =     1; % IS_ROTARY_WING [0-1]
        vehicle_status_is_vtol        (1,1) {mustBeGreaterThanOrEqual(vehicle_status_is_vtol       ,    0), mustBeLessThanOrEqual(vehicle_status_is_vtol       ,    1), mustBeInteger(vehicle_status_is_vtol       )} =     0; % IS_VTOL [0-1]
        loop_update_rate_hz           (1,1) {mustBeGreaterThanOrEqual(loop_update_rate_hz          ,   50), mustBeLessThanOrEqual(loop_update_rate_hz          , 5000)                                              } =   250; % Loop update rate Hz [50-5000]
        simple_mode                   (1,1) {mustBeGreaterThanOrEqual(simple_mode                  ,    0), mustBeLessThanOrEqual(simple_mode                  ,    1), mustBeInteger(simple_mode                  )} =     1; % Simple mode [0-1]
        output_rates                  (1,1) {mustBeGreaterThanOrEqual(output_rates                 ,    0), mustBeLessThanOrEqual(output_rates                 ,    1), mustBeInteger(output_rates                 )} =     1; % Output rate setpoints [0-1]
    end

    %% Pre-computed constants
    properties(Access = private)
        attitude_p                    (3,1) = zeros(3,1); % P gain for attitude control
        auto_rate_max                 (3,1) = zeros(3,1); % attitude rate limits in auto modes
        board_rotation                (3,3) = eye  (3,3); % Rotation matrix for the orientation that the board is mounted
        acro_rate_max                 (3,1) = zeros(3,1); % Max attitude rates in acro mode
        mc_rate_max                   (3,1) = zeros(3,1); % Attitude rate limits in stabilized modes
        rate_d                        (3,1) = zeros(3,1); % D gain for angular rate error
        rate_ff                       (3,1) = zeros(3,1); % Feedforward gain for desired rates
        rate_i                        (3,1) = zeros(3,1); % I gain for angular rate error
        rate_int_lim                  (3,1) = zeros(3,1); % Integrator state limit for rate loop
        rate_p                        (3,1) = zeros(3,1); % P gain for angular rate error
        man_tilt_max                  (1,1) = 0; % Maximum tilt allowed for manual flight [rad]
        lp_filters_d_a1               (1,1) = 0; % Filter constants
        lp_filters_d_a2               (1,1) = 0; % Filter constants
        lp_filters_d_b0               (1,1) = 0; % Filter constants
        lp_filters_d_b1               (1,1) = 0; % Filter constants
        lp_filters_d_b2               (1,1) = 0; % Filter constants
        lp_filters_d_cutoff_freq      (1,1) = 0; % Filter constants
        dt                            (1,1) = 0; % sampling time
    end
    
    %% Discrete states
    properties(DiscreteState)
        rates_int                    % Angular rates integral error
        rates_prev                   % Angular rates on previous step
        rates_prev_filtered          % Angular rates on previous step (low-pass filtered)
        rates_sp                     % Angular rates setpoint
        thrust_sp                    % Thrust setpoint
        man_yaw_sp                   % Current yaw setpoint in manual mode
        gear_state_initialized       % True if the gear state has been initialized
        lp_filters_d_delay_element_1 % Filter delay 1
        lp_filters_d_delay_element_2 % Filter delay 2
        prev_quat_reset_counter      % Quat reset
        reset_yaw_sp                 % Reset yaw sp
    end

    methods(Access = protected)
        function setupImpl( obj )
            %SETUPIMPL Perform one-time calculations, such as computing constants
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            obj.dt = 1 / obj.loop_update_rate_hz;
            LowPassFilter2pVectorSetCutOffFrequency( obj, obj.loop_update_rate_hz, obj.d_term_cutoff_freq );
            LowPassFilter2pVectorReset( obj, zeros(3,1) );
            
            % Gains
            obj.attitude_p     = [ obj.roll_p             ; obj.pitch_p             ; obj.yaw_p              ];
            obj.rate_p         = [ obj.roll_rate_p        ; obj.pitch_rate_p        ; obj.yaw_rate_p         ];
            obj.rate_i         = [ obj.roll_rate_i        ; obj.pitch_rate_i        ; obj.yaw_rate_i         ];
            obj.rate_int_lim   = [ obj.roll_rate_integ_lim; obj.pitch_rate_integ_lim; obj.yaw_rate_integ_lim ];
            obj.rate_d         = [ obj.roll_rate_d        ; obj.pitch_rate_d        ; obj.yaw_rate_d         ];
            obj.rate_ff        = [ obj.roll_rate_ff       ; obj.pitch_rate_ff       ; obj.yaw_rate_ff        ];
            
            % Angular rate limits
            obj.mc_rate_max   = deg2rad( [ obj.roll_rate_max; obj.pitch_rate_max; obj.yaw_rate_max ] );
            obj.auto_rate_max = deg2rad( [ obj.roll_rate_max; obj.pitch_rate_max; obj.yaw_auto_max ] );
            obj.acro_rate_max = deg2rad( [ obj.acro_roll_max; obj.acro_pitch_max; obj.acro_yaw_max ] );
            
            obj.man_tilt_max = deg2rad( obj.man_tilt_max_deg );
            % In the original code the board rotation is calculated here,
            % but not necessary in simulation
        end
        
        function resetImpl( obj )
            %RESETIMPL Initialize / reset discrete-state properties
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            obj.rates_int                    = zeros(3,1);
            obj.rates_prev                   = zeros(3,1);
            obj.rates_prev_filtered          = zeros(3,1);
            obj.rates_sp                     = zeros(3,1);
            obj.thrust_sp                    = 0;
            obj.man_yaw_sp                   = 0;
            obj.gear_state_initialized       = 0;
            obj.lp_filters_d_delay_element_1 = zeros(3,1);
            obj.lp_filters_d_delay_element_2 = zeros(3,1);
            obj.prev_quat_reset_counter      = 0;
            obj.reset_yaw_sp                 = 0;
        end

        function num = getNumOutputsImpl( obj )
            %GETNUMOUTPUTSIMPL Return number of outputs
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            if obj.output_rates
                num = 2;
            else
                num = 1;
            end
        end
        
        function num = getNumInputsImpl( obj )
            %GETNUMINPUTSIMPL Return number of inputs
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            if obj.simple_mode
                num = 5;
            else
                num = 8;
            end
        end
        
        function varargout = getOutputSizeImpl( obj )
            %GETOUTPUTSIZEIMPL Return output size(s)
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            varargout{1} = [8 1];
            if obj.output_rates
                varargout{2} = [3 1];
            end
        end
        
        function varargout = getOutputDataTypeImpl( obj )
            %GETOUTPUTDATATYPEIMPL Return output data type(s)
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            varargout{1} = 'double';
            if obj.output_rates
                varargout{2} = 'double';
            end
        end
        
        function varargout = isOutputComplexImpl( obj )
            %ISOUTPUTCOMPLEXIMPL Return whether output is complex
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            varargout{1} = false;
            if obj.output_rates
                varargout{2} = false;
            end
        end
        
        function varargout = isOutputFixedSizeImpl( obj )
            %ISOUTPUTFIXEDSIZEIMPL Return whether output is fixed size
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            varargout{1} = true;
            if obj.output_rates
                varargout{2} = true;
            end
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl( ~, name )
            %GETDISCRETESTATESPECIFICATIONIMPL Return specs of des. state
            % Return size, data type, and complexity of discrete-state
            % specified in name
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            switch name
                case 'rates_int'
                    sz = [3 1];
                    dt = "double";
                    cp = false;
                case 'rates_prev'
                    sz = [3 1];
                    dt = "double";
                    cp = false;
                case 'rates_prev_filtered'
                    sz = [3 1];
                    dt = "double";
                    cp = false;
                case 'rates_sp'
                    sz = [3 1];
                    dt = "double";
                    cp = false;
                case 'thrust_sp'
                    sz = [1 1];
                    dt = "double";
                    cp = false;
                case 'man_yaw_sp'
                    sz = [1 1];
                    dt = "double";
                    cp = false;
                case 'gear_state_initialized'
                    sz = [1 1];
                    dt = "double";
                    cp = false;
                case 'lp_filters_d_delay_element_1'
                    sz = [3 1];
                    dt = "double";
                    cp = false;
                case 'lp_filters_d_delay_element_2'
                    sz = [3 1];
                    dt = "double";
                    cp = false;
                case 'prev_quat_reset_counter'
                    sz = [1 1];
                    dt = "double";
                    cp = false;
                case 'reset_yaw_sp'
                    sz = [1 1];
                    dt = "double";
                    cp = false;
                otherwise
                    error(['Error: Incorrect State Name: ', name.']);
            end
        end
        
        function icon = getIconImpl( ~ )
            %GETICONIMPL Define icon for System block
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            icon = ["MulticopterAttitudeControl","","v1.82"];
        end

        function varargout = getInputNamesImpl( obj )
            %GETINPUTNAMESIMPL Return input port names for System block
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            if obj.simple_mode
                varargout = { ...
                    'qDes'      , ...
                    'thrustDes' , ...
                    'yawRateDes', ...
                    'qMeasured' , ...
                    'nuBody'    , ...
                    };
            else
                varargout = { 'v_control_mode_flags',  ...
                    'vehicle_land_detected', ...
                    'saturation_status',     ...
                    'manual_control_sp',     ...
                    'v_att_sp',              ...
                    'v_rates_sp',            ...
                    'v_att_in',              ...
                    'sensor_gyro' };
            end
        end

        function varargout = getOutputNamesImpl( obj )
            %GETOUTPUTNAMESIMPL Return input port names for System block
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            varargout{1} = 'actuatorsControl';
            if obj.output_rates
                varargout{2} = 'attRateSp';
            end
        end
        
        function sts = getSampleTimeImpl( obj )
            %GETSAMPLETIMEIMPL Return sample time of the controller
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            sts = createSampleTime(obj, 'Type','Discrete', ...
                'SampleTime', 1/obj.loop_update_rate_hz, ...
                'OffsetTime', 0);
        end
    
        %% Main function
        function varargout = stepImpl( obj, varargin )
            %STEPIMPL Main function executed at each time step
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            % Initialize output
            actuators_control = zeros(8,1);
            att_control       = zeros(3,1);
            
            % Assign inputs based on which mode we are operating in
            % This is not a PX4 feature and was added purely to reduce the
            % number of input signals in the simulation model
            if obj.simple_mode
                % In simple mode, we assume the attitude controller is
                % receiving attitude, thrust, and yaw rate commands from a
                % position controller. In addition, we assume no saturation
                % occurs
                v_control_mode_flags_in  = [ 1; 1; 1; 1; 0; 1; 1; 0; 0; 0 ];
                vehicle_land_detected_in = zeros(2,1);
                saturation_status_in     = zeros(12,1);
                manual_control_sp_in     = zeros(4,1);
                v_rates_sp_in            = zeros(6,1);
                v_att_in                 = [ varargin{4}; 0; 0 ];
                v_att_sp_in              = [ varargin{1}; 0; 0; varargin{2}; varargin{3} ];
                sensor_gyro_in           = varargin{5};
            else
                % In normal mode, all the features of the controller
                % are available by setting the appropriate control flags
                % (see below for a description of each element)
                v_control_mode_flags_in  = varargin{1};
                vehicle_land_detected_in = varargin{2};
                saturation_status_in     = varargin{3};
                manual_control_sp_in     = varargin{4};
                v_att_sp_in              = varargin{5};
                v_rates_sp_in            = varargin{6};
                v_att_in                 = varargin{7};
                sensor_gyro_in           = varargin{8};
            end
            
            % Assign input
            sensor_gyro = sensor_gyro_in(1:3,1);
            
            % Assign control modes
            v_control_mode.flag_armed                       = v_control_mode_flags_in(1);  % is set whenever the writing thread stores new data
            v_control_mode.flag_control_altitude_enabled    = v_control_mode_flags_in(2);  % true if rate/attitude stabilization is enabled
            v_control_mode.flag_control_attitude_enabled    = v_control_mode_flags_in(3);  % true if attitude stabilization is mixed in
            v_control_mode.flag_control_auto_enabled        = v_control_mode_flags_in(4);  % true if onboard autopilot should act
            v_control_mode.flag_control_manual_enabled      = v_control_mode_flags_in(5);  % true if manual input is mixed in
            v_control_mode.flag_control_position_enabled    = v_control_mode_flags_in(6);  % true if position is controlled
            v_control_mode.flag_control_rates_enabled       = v_control_mode_flags_in(7);  % true if rates are stabilized
            v_control_mode.flag_control_rattitude_enabled   = v_control_mode_flags_in(8);  % true if rate/attitude stabilization is enabled
            v_control_mode.flag_control_termination_enabled = v_control_mode_flags_in(9);  % true if flighttermination is enabled
            v_control_mode.flag_control_velocity_enabled    = v_control_mode_flags_in(10); % true if horizontal velocity (implies direction) is controlled
            
            % Assign control modes
            manual_control_sp.r = manual_control_sp_in(1);
            manual_control_sp.x = manual_control_sp_in(2);
            manual_control_sp.y = manual_control_sp_in(3);
            manual_control_sp.z = manual_control_sp_in(4);
            
            % Assign land detected
            vehicle_land_detected.landed       = vehicle_land_detected_in(1);
            vehicle_land_detected.maybe_landed = vehicle_land_detected_in(2);
            
            % Assign rate setpoints
            v_rates_sp.roll    = v_rates_sp_in(1);
            v_rates_sp.pitch   = v_rates_sp_in(2);
            v_rates_sp.yaw     = v_rates_sp_in(3);
            v_rates_sp.thrust_body  = v_rates_sp_in(4:6);
            
            % Assign saturation status
            saturation_status = struct( 'value', saturation_status_in( 1), 'flags', struct( ...
                'valid'     , saturation_status_in( 2), ... %  0 - true when the saturation status is used
                'motor_pos' , saturation_status_in( 3), ... %  1 - true when any motor has saturated in the positive direction
                'motor_neg' , saturation_status_in( 4), ... %  2 - true when any motor has saturated in the negative direction
                'roll_pos'  , saturation_status_in( 5), ... %  3 - true when a positive roll demand change will increase saturation
                'roll_neg'  , saturation_status_in( 6), ... %  4 - true when a negative roll demand change will increase saturation
                'pitch_pos' , saturation_status_in( 7), ... %  5 - true when a positive pitch demand change will increase saturation
                'pitch_neg' , saturation_status_in( 8), ... %  6 - true when a negative pitch demand change will increase saturation
                'yaw_pos'   , saturation_status_in( 9), ... %  7 - true when a positive yaw demand change will increase saturation
                'yaw_neg'   , saturation_status_in(10), ... %  8 - true when a negative yaw demand change will increase saturation
                'thrust_pos', saturation_status_in(11), ... %  9 - true when a positive thrust demand change will increase saturation
                'thrust_neg', saturation_status_in(12)  ... % 10 - true when a negative thrust demand change will increase saturation
                ) );
            
            % Assign v_att
            v_att = struct( 'q', zeros(4,1), 'quat_reset_counter', 0, 'delta_q_reset', 0 );
            v_att.q                   = v_att_in(1:4);
            v_att.quat_reset_counter  = v_att_in(5);
            v_att.delta_q_reset       = v_att_in(6);

            % Assign v_att_sp
            v_att_sp = struct( 'yaw_sp_move_rate', 0, ...
                'roll_body'       , 0         , ...
                'pitch_body'      , 0         , ...
                'yaw_body'        , 0         , ...
                'thrust_body'     , zeros(3,1), ...
                'q_d'             , zeros(4,1), ...
                'q_d_valid'       , 0 );
            v_att_sp.q_d              = v_att_sp_in(1:4,1);
            v_att_sp.thrust_body      = v_att_sp_in(5:7,1);
            v_att_sp.yaw_sp_move_rate = v_att_sp_in(8,1);
            
            % Assume the attitude is always updated
            attitude_updated = 1;
            if v_control_mode.flag_control_manual_enabled
                manual_control_updated = 1;
            else
                manual_control_updated = 0;
            end
            
            % Run the rate controller immediately after a gyro update (does not
            % matter for simulation)
            if (v_control_mode.flag_control_rates_enabled)
                att_control = control_attitude_rates( obj, sensor_gyro, ...
                    v_control_mode, vehicle_land_detected, saturation_status );
                % Publish actuator controls
                actuators_control = publish_actuator_controls( obj, att_control );
                % Publish rate controller status
            end
            
            % Vehicle attitude poll
            if obj.prev_quat_reset_counter ~= v_att.quat_reset_counter
                tmp = obj.man_yaw_sp + MulticopterAttitudeControl.QuatToEuler( v_att.delta_q_reset );
                obj.man_yaw_sp = obj.man_yaw_sp + tmp(3);
            end
            obj.prev_quat_reset_counter = v_att.quat_reset_counter;
            
            % Check if we are in rattitude mode and the pilot is above the
            % threshold on pitch or roll (yaw can rotate 360 in normal att
            % control). If both are true don't even bother running the attitude
            % controllers
            if (v_control_mode.flag_control_rattitude_enabled)
                v_control_mode.flag_control_attitude_enabled = double( ...
                    abs(manual_control_sp.y) <= obj.rattitude_thres && ...
                    abs(manual_control_sp.x) <= obj.rattitude_thres );
            end
            
            attitude_setpoint_generated = 0;
            
            if (v_control_mode.flag_control_attitude_enabled && obj.vehicle_status_is_rotary_wing)
                if ( attitude_updated )
                    % Generate the attitude setpoint from stick inputs if we are in
                    % Manual/Stabilized mode
                    if (v_control_mode.flag_control_manual_enabled && ...
                            ~v_control_mode.flag_control_altitude_enabled && ...
                            ~v_control_mode.flag_control_velocity_enabled && ...
                            ~v_control_mode.flag_control_position_enabled)
                        v_att_sp = generate_attitude_setpoint( obj, obj.reset_yaw_sp, manual_control_sp, v_att );
                        attitude_setpoint_generated = 1;
                    end
                    control_attitude( obj, v_control_mode, v_att, v_att_sp );
                    % Publish rates setpoints
                    v_rates_sp.roll  = obj.rates_sp(1);
                    v_rates_sp.pitch = obj.rates_sp(2);
                    v_rates_sp.yaw   = obj.rates_sp(3);
                    v_rates_sp.thrust_body = [ 0; 0; -obj.thrust_sp ];
                end
            else
                % Attitude controller disabled, poll rates setpoint topic
                if (v_control_mode.flag_control_manual_enabled && obj.vehicle_status_is_rotary_wing)
                    if (manual_control_updated)
                        % Manual rates control - ACRO mode
                        man_rate_sp = [ ...
                            MulticopterAttitudeControl.SuperExpo( manual_control_sp.y, obj.acro_expo_rp, obj.acro_superexpo_rp );
                            MulticopterAttitudeControl.SuperExpo( -manual_control_sp.x, obj.acro_expo_rp, obj.acro_superexpo_rp );
                            MulticopterAttitudeControl.SuperExpo( manual_control_sp.r, obj.acro_expo_y, obj.acro_superexpo_y ) ];
                        obj.rates_sp = man_rate_sp .* obj.acro_rate_max;
                        obj.thrust_sp = manual_control_sp.z;
                        % Publish rates setpoints
                        v_rates_sp.roll  = obj.rates_sp(1);
                        v_rates_sp.pitch = obj.rates_sp(2);
                        v_rates_sp.yaw   = obj.rates_sp(3);
                        v_rates_sp.thrust_body = [ 0; 0; -obj.thrust_sp ];
                    end
                else
                    % attitude controller disabled, poll rates setpoint topic
                    obj.rates_sp(1) = v_rates_sp.roll;
                    obj.rates_sp(2) = v_rates_sp.pitch;
                    obj.rates_sp(3) = v_rates_sp.yaw;
                    obj.thrust_sp = -v_rates_sp.thrust_body(3);
                end
            end
            
            if ( v_control_mode.flag_control_termination_enabled )
                if (~obj.vehicle_status_is_vtol)
                    obj.rates_sp = obj.rates_sp .* 0;
                    obj.rates_int = obj.rates_int .* 0;
                    obj.thrust_sp = 0.0;
                    att_control = att_control .* 0;
                    % Publish actuator controls
                    actuators_control = publish_actuator_controls( obj, att_control );
                end
            end
            
            if ( attitude_updated )
                obj.reset_yaw_sp = double( (~attitude_setpoint_generated && ~v_control_mode.flag_control_rattitude_enabled) || ...
                    vehicle_land_detected.landed || ...
                    (obj.vehicle_status_is_vtol && ~obj.vehicle_status_is_rotary_wing) ); % VTOL in FW mode
            end
            
            varargout{1} = actuators_control;
            if obj.output_rates
                varargout{2} = obj.rates_sp;
            end
        end

        %% mc_att_control_main.cpp - Main control functions
        function attitude_setpoint = generate_attitude_setpoint( obj, reset_yaw_sp, manual_control_sp, v_att )
            %GENERATE_ATTITUDE_SETPOINT Generate attitude setpoint (manual control)
            %   Based on code from PX4 Firmware (retrieved 2019/01/15, v1.82):
            %       https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            
            eul = MulticopterAttitudeControl.QuatToEuler( v_att.q ); % 
            yaw = eul(3);
            attitude_setpoint = struct( 'yaw_sp_move_rate', 0         , ...
                                        'roll_body'       , 0         , ...
                                        'pitch_body'      , 0         , ...
                                        'yaw_body'        , 0         , ...
                                        'thrust_body'     , zeros(3,1), ...
                                        'q_d'             , zeros(4,1), ...
                                        'q_d_valid'       , 0 );
            if reset_yaw_sp
                obj.man_yaw_sp = yaw;
            elseif ( manual_control_sp.z > 0.05 || strcmp( obj.airmode, 'roll_pitch_yaw' ) )
                yaw_rate = obj.yaw_rate_scaling;
                attitude_setpoint.yaw_sp_move_rate = manual_control_sp.r * yaw_rate;
                obj.man_yaw_sp = MulticopterAttitudeControl.WrapPi( obj.man_yaw_sp + attitude_setpoint.yaw_sp_move_rate*obj.dt );
            end
            
            % Input mapping for roll & pitch setpoints
            % ----------------------------------------
            % We control the following 2 angles:
            % - tilt angle, given by sqrt(x*x + y*y)
            % - the direction of the maximum tilt in the XY-plane, which also
            % defines the direction of the motion
            %
            % This allows a simple limitation of the tilt angle, the vehicle flies
            % towards the direction that the stick points to, and changes of the
            % stick input are linear.
            x = manual_control_sp.x * obj.man_tilt_max;
            y = manual_control_sp.y * obj.man_tilt_max;
            
            % We want to fly towards the direction of (x, y), so we use a
            % perpendicular axis angle vector in the XY-plane
            v = [y, -x];
            v_norm = norm( v );
            
            if (v_norm > obj.man_tilt_max) % limit to the configured maximum tilt angle
                v = v * obj.man_tilt_max / v_norm;
            end
            
            q_sp_rpy = MulticopterAttitudeControl.AxisAngleToQuat( [ v(1); v(2); 0 ] );
            euler_sp = MulticopterAttitudeControl.QuatToEuler( q_sp_rpy ); %
            attitude_setpoint.roll_body  = euler_sp(1);
            attitude_setpoint.pitch_body = euler_sp(2);
            % The axis angle can change the yaw as well (noticeable at higher tilt angles).
            % This is the formula by how much the yaw changes:
            %   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
            %   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
            attitude_setpoint.yaw_body = obj.man_yaw_sp + euler_sp(3);
            
            if obj.vehicle_status_is_vtol
                % Construct attitude setpoint rotation matrix. Modify the setpoints
                % for roll and pitch such that they reflect the user's intention
                % even if a large yaw error (yaw_sp - yaw) is present. In the
                % presence of a yaw error constructing a rotation matrix from the
                % pure euler angle setpoints will lead to unexpected attitude
                % behaviour from the user's view as the euler angle sequence uses
                % the  yaw setpoint and not the current heading of the vehicle.
                % However there's also a coupling effect that causes oscillations
                % for fast roll/pitch changes at higher tilt angles, so we want to
                % avoid using this on multicopters.
                % The effect of that can be seen with:
                % - roll/pitch into one direction, keep it fixed (at high angle)
                % - apply a fast yaw rotation
                % - look at the roll and pitch angles: they should stay pretty much
                % the same as when not yawing
                
                % Calculate our current yaw error
                yaw_error = MulticopterAttitudeControl.WrapPi(attitude_setpoint.yaw_body - yaw);
                
                % Compute the vector obtained by rotating a z unit vector by the rotation
                % given by the roll and pitch commands of the user
                zB = [0.0; 0.0; 1.0];
                R_sp_roll_pitch = MulticopterAttitudeControl.EulerToDcm( [attitude_setpoint.roll_body; attitude_setpoint.pitch_body; 0.0] );
                z_roll_pitch_sp = R_sp_roll_pitch * zB;
                
                % Transform the vector into a new frame which is rotated around the z axis
                % by the current yaw error. this vector defines the desired tilt when we look
                % into the direction of the desired heading
                R_yaw_correction = MulticopterAttitudeControl.EulerToDcm( [0.0; 0.0; -yaw_error] );
                z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;
                
                % Use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
                % R_tilt is computed from_euler; only true if cos(roll) not equal zero
                % -> valid if roll is not +-pi/2;
                attitude_setpoint.roll_body = -asin(z_roll_pitch_sp(1));
                attitude_setpoint.pitch_body = atan2(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
            end
            
            % Publish the setpoint
            q_sp = MulticopterAttitudeControl.EulerToQuat( [ attitude_setpoint.roll_body;
                attitude_setpoint.pitch_body;
                attitude_setpoint.yaw_body ] );
            attitude_setpoint.q_d = q_sp;
            attitude_setpoint.q_d_valid = 1;
            
            attitude_setpoint.thrust_body(3) = -apply_throttle_curve( obj, manual_control_sp.z );
            % attitude_setpoint.timestamp = time;
        end
        
        function control_attitude( obj, v_control_mode, v_att, v_att_sp )
            %CONTROL_ATTITUDE Attitude controller
            %   Based on code from PX4 Firmware (retrieved 2019/01/15, v1.82):
            %       https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp
            %   Input: 'vehicle_attitude_setpoint' (depending on mode)
            %   Output: obj.rates_sp, obj.thrust_sp
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/29
            
            % Physical thrust axis is the negative of body z axis
            obj.thrust_sp = -v_att_sp.thrust_body(3);
            
            % Prepare yaw weight from the ratio between roll/pitch and yaw gains
            attitude_gain = obj.attitude_p;
            roll_pitch_gain = ( attitude_gain(1) + attitude_gain(2) ) / 2;
            yaw_w = MulticopterAttitudeControl.constrain( attitude_gain(3) / roll_pitch_gain, 0, 1 );
            attitude_gain(3) = roll_pitch_gain;
            
            % Get estimated and desired vehicle attitude
            q  = v_att.q;
            qd = v_att_sp.q_d;
            
            % Ensure input quaternions are exactly normalized because
            % acosf(1.00001) == NaN
            q = MulticopterAttitudeControl.normalize( q );
            qd = MulticopterAttitudeControl.normalize( qd );
            
            % Calculate reduced desired attitude neglecting vehicle's yaw to
            % prioritize roll and pitch
            e_z   = MulticopterAttitudeControl.QuatToDcmZ( q );
            e_z_d = MulticopterAttitudeControl.QuatToDcmZ( qd );
            qd_red = MulticopterAttitudeControl.VecsToQuat( e_z, e_z_d );
            
            if ( abs(qd_red(2)) > (1 - 1e-5) ) || ( abs(qd_red(3)) > (1 - 1e-5) )
                % In the infinitesimal corner case where the vehicle and thrust have
                % the completely opposite direction, full attitude control anyways
                % generates no yaw input and directly takes the combination of roll
                % and pitch leading to the correct desired yaw. Ignoring this case
                % would still be totally safe and stable.
                qd_red = qd;
            else
                % Transform rotation from current to desired thrust vector into a
                % world frame reduced desired attitude
                qd_red = MulticopterAttitudeControl.QuatMult( qd_red, q );
            end
            
            % Mix full and reduced desired attitude
            q_mix = MulticopterAttitudeControl.QuatMult( MulticopterAttitudeControl.InvertQuat(qd_red), qd );
            q_mix = q_mix .* MulticopterAttitudeControl.SignNoZero( q_mix(1) );
            
            % Catch numerical problems with the domain of acosf and asinf
            q_mix(1) = MulticopterAttitudeControl.constrain( q_mix(1), -1, 1 );
            q_mix(4) = MulticopterAttitudeControl.constrain( q_mix(4), -1, 1 );
            qd = MulticopterAttitudeControl.QuatMult( qd_red, ...
                [ cos( yaw_w * acos(q_mix(1)) );
                  0;
                  0;
                  sin( yaw_w * asin(q_mix(4)) ) ] );
            
            % Quaternion attitude control law, qe is rotation from q to qd
            qe = MulticopterAttitudeControl.QuatMult( MulticopterAttitudeControl.InvertQuat(q), qd );
            
            % Using sin(alpha/2) scaled rotation axis as attitude error (see
            % quaternion definition by axis angle)
            % Also taking care of the antipodal unit quaternion ambiguity
            % qe(2:4) corresponds to the imaginary part of the quaternion
            eq = 2 .* MulticopterAttitudeControl.SignNoZero(qe(1)) .* qe(2:4);
            
            % calculate angular rates setpoint
            obj.rates_sp = eq .* attitude_gain;
            
            % Feed forward the yaw setpoint rate.
            % yaw_sp_move_rate is the feed forward commanded rotation around the
            % world z-axis, but we need to apply it in the body frame (because
            % obj.rates_sp is expressed in the body frame).
            % Therefore we infer the world z-axis (expressed in the body frame) by
            % taking the last column of R.transposed (== q.inversed) and multiply
            % it by the yaw setpoint rate (yaw_sp_move_rate).
            % This yields a vector representing the commanded rotation around the
            % world z-axis expressed in the body frame such that it can be added to
            % the rates setpoint.
            obj.rates_sp = obj.rates_sp + MulticopterAttitudeControl.QuatToDcmZ( MulticopterAttitudeControl.InvertQuat(q) ) .* v_att_sp.yaw_sp_move_rate;
            
            % limit rates
            for i = 1:3
                if (( v_control_mode.flag_control_velocity_enabled || ...
                        v_control_mode.flag_control_auto_enabled ) && ...
                        ~v_control_mode.flag_control_manual_enabled )
                    obj.rates_sp(i) = MulticopterAttitudeControl.constrain(obj.rates_sp(i), -obj.auto_rate_max(i), obj.auto_rate_max(i));
                else
                    obj.rates_sp(i) = MulticopterAttitudeControl.constrain(obj.rates_sp(i), -obj.mc_rate_max(i), obj.mc_rate_max(i));
                end
            end
        end
        
        function att_control = control_attitude_rates( obj, sensor_gyro, v_control_mode, vehicle_land_detected, saturation_status )
            %CONTROL_ATTITUDE_RATES Attitude rates controller
            %   Input: obj.rates_sp, obj.thrust_sp
            %   Output: att_control
            %   Based on code from PX4 Firmware (retrieved 2019/01/15, v1.82):
            %       https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/21
            
            % Reset integral if disarmed */
            if (~v_control_mode.flag_armed || ~obj.vehicle_status_is_rotary_wing)
                obj.rates_int = zeros(3,1);
            end
            
            % Get the raw gyro data and correct for thermal errors
            % Note: the original code applies different correction based on which
            % gyro is being used, but since this is a simulation we do not need to
            % worry about this.
            rates = sensor_gyro;
            
            % Rotate corrected measurements from sensor to body frame
            rates = obj.board_rotation * rates;
            
            % Correct for in-run bias errors
            % rates(1) = rates(1) - obj.sensor_bias.gyro_x_bias;
            % rates(2) = rates(2) - obj.sensor_bias.gyro_y_bias;
            % rates(3) = rates(3) - obj.sensor_bias.gyro_z_bias;
            
            rates_p_scaled = obj.rate_p .* MulticopterAttitudeControl.pid_attenuations( obj.tpa_breakpoint_p, obj.tpa_rate_p, obj.thrust_sp );
            rates_i_scaled = obj.rate_i .* MulticopterAttitudeControl.pid_attenuations( obj.tpa_breakpoint_i, obj.tpa_rate_i, obj.thrust_sp );
            rates_d_scaled = obj.rate_d .* MulticopterAttitudeControl.pid_attenuations( obj.tpa_breakpoint_d, obj.tpa_rate_d, obj.thrust_sp );
            
            % Angular rates error
            rates_err = obj.rates_sp - rates;
            
            % Apply low-pass filtering to the rates for D-term
            rates_filtered = LowPassFilter2pVector3Apply( obj, rates );
            
            att_control = rates_p_scaled .* rates_err + ...
                obj.rates_int - ...
                rates_d_scaled .* (rates_filtered - obj.rates_prev_filtered) / obj.dt + ...
                obj.rate_ff .* obj.rates_sp;
            
            obj.rates_prev = rates;
            obj.rates_prev_filtered = rates_filtered;
            
            % update integral only if we are not landed */
            if (~vehicle_land_detected.maybe_landed && ~vehicle_land_detected.landed)
                for i = 1:3
                    % Check for positive control saturation
                    positive_saturation = ...
                        ((i == 1) && saturation_status.flags.roll_pos) || ...
                        ((i == 2) && saturation_status.flags.pitch_pos) || ...
                        ((i == 3) && saturation_status.flags.yaw_pos);
                    
                    % Check for negative control saturation
                    negative_saturation = ...
                        ((i == 1) && saturation_status.flags.roll_neg) || ...
                        ((i == 2) && saturation_status.flags.pitch_neg) || ...
                        ((i == 3) && saturation_status.flags.yaw_neg);
                    
                    % prevent further positive control saturation
                    if (positive_saturation)
                        rates_err(i) = min( [rates_err(i), 0.0] );
                    end
                    
                    % prevent further negative control saturation
                    if (negative_saturation)
                        rates_err(i) = max( [rates_err(i), 0.0] );
                    end
                    
                    % Perform the integration using a first order method and do not propagate the result if out of range or invalid
                    rate_i_tmp = obj.rates_int(i) + rates_i_scaled(i) * rates_err(i) * obj.dt;
                    
                    if (isfinite(rate_i_tmp) && (rate_i_tmp > -obj.rate_int_lim(i)) && (rate_i_tmp < obj.rate_int_lim(i)))
                        obj.rates_int(i) = rate_i_tmp;
                    end
                end
            end
            
            % explicitly limit the integrator state */
            for i = 1:3
                obj.rates_int(i) = MulticopterAttitudeControl.constrain( obj.rates_int(i), -obj.rate_int_lim(i), obj.rate_int_lim(i) );
            end
        end
        
        %% mc_att_control_main.cpp - Secondary functions
        function throttle = apply_throttle_curve( obj, throttle_stick_input )
            %APPLY_THROTTLE_CURVE Apply throttle curve
            %   Based on code from PX4 Firmware:
            %       https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp
            %   Written:       J.X.J. Bannwarth, 2019/01/16
            %   Last modified: J.X.J. Bannwarth, 2019/01/21
            switch obj.throttle_curve
                case 1 % no rescaling to hover throttle
                    throttle = obj.man_throttle_min + ...
                        throttle_stick_input * (obj.throttle_max - obj.man_throttle_min);
                otherwise % 0 or other: rescale to hover throttle at 0.5 stick
                    if throttle_stick_input < 0.5
                        throttle = (obj.throttle_hover - obj.man_throttle_min) / ...
                            0.5 * throttle_stick_input + obj.man_throttle_min;
                    else
                        throttle = (obj.throttle_max - obj.throttle_hover) / ...
                            0.5 * (throttle_stick_input - 1.0) + obj.throttle_max;
                    end
            end
        end

        function actuators_control = publish_actuator_controls( obj, att_control )
            %PUBLISH_ACTUATOR_CONTROLS Perform checks before outputing commands
            %   Based on code from PX4 Firmware:
            %       https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp
            %   Written:       J.X.J. Bannwarth, 2019/01/21
            %   Last modified: J.X.J. Bannwarth, 2019/01/30
            actuators_control = zeros(8,1);
            if isfinite( att_control(1) )
                actuators_control(1) = att_control(1);
            end
            if isfinite( att_control(2) )
                actuators_control(2) = att_control(2);
            end
            if isfinite( att_control(3) )
                actuators_control(3) = att_control(3);
            end
            if isfinite( obj.thrust_sp )
                actuators_control(4) = -obj.thrust_sp;
            end
%             if isfinite( landing_gear_landing_gear )
%                 actuators_control(8) = landing_gear_landing_gear;
%             end
            actuators_control(8) = 0; % Bypass landing gear
            % actuators_timestamp = time;
            % actuators_timestamp_sample = sensor_gyro_timestamp;
            
            % Scale effort by battery status
            if ( obj.bat_scale_en && battery_status_scale > 0.0 )
                for i = 1:4
                    actuators_control(i) = actuators.control(i) * battery_status_scale;
                end
            end
            
            actuators_0_circuit_breaker_enabled = 0; % Assume we don't want to suppress the output
            if (actuators_0_circuit_breaker_enabled)
                actuators_control = 0 * actuators_control;
            end
        end
        
        %% Low-pass filter functions
        function output = LowPassFilter2pVector3Apply( obj, sample )
            %LOWPASSFILTER2PVECTOR3APPLY Apply low pass filter
            %   Based on code from PX4 Firmware:
            %       https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/filter/LowPassFilter2pVector3f.hpp
            %   Written:       J.X.J. Bannwarth, 2019/01/16
            %   Last modified: J.X.J. Bannwarth, 2019/01/21
            delay_element_0 = sample - obj.lp_filters_d_delay_element_1.*obj.lp_filters_d_a1 - obj.lp_filters_d_delay_element_2.*obj.lp_filters_d_a2;
            output = delay_element_0.*obj.lp_filters_d_b0 + obj.lp_filters_d_delay_element_1.*obj.lp_filters_d_b1 + obj.lp_filters_d_delay_element_2*obj.lp_filters_d_b2;
            obj.lp_filters_d_delay_element_2 = obj.lp_filters_d_delay_element_1;
            obj.lp_filters_d_delay_element_1 = delay_element_0;
        end
        
        function LowPassFilter2pVectorSetCutOffFrequency( obj, sample_freq, cutoff_freq )
            %LOWPASSFILTER2PVECTORSETCUTOFFFREQUENCY Set cutoff frequency
            %   Based on code from PX4 Firmware:
            %       https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/filter/LowPassFilter2pVector3f.cpp
            %   Written:       J.X.J. Bannwarth, 2019/01/16
            %   Last modified: J.X.J. Bannwarth, 2019/01/21
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
            %   Written:       J.X.J. Bannwarth, 2019/01/16
            %   Last modified: J.X.J. Bannwarth, 2019/01/21
            dval = sample ./ (obj.lp_filters_d_b0 + obj.lp_filters_d_b1 + obj.lp_filters_d_b2);
            
            if (isfinite( dval(1) ) && isfinite( dval(2) ) && isfinite( dval(3) ))
                obj.lp_filters_d_delay_element_1 = dval;
                obj.lp_filters_d_delay_element_2 = dval;
            else
                obj.lp_filters_d_delay_element_1 = sample;
                obj.lp_filters_d_delay_element_2 = sample;
            end
            
            output = LowPassFilter2pVector3Apply( obj, sample );
        end
    end
    
    %% Static functions
    methods(Static)
        % Static secondary function
        function pidAttenuationPerAxis = pid_attenuations( tpa_breakpoint, tpa_rate, thrust_sp )
            %PID_ATTENUATIONS Throttle PID attenuation
            %   Function visualization available here https://www.desmos.com/calculator/gn4mfoddje
            %   Input: tpa_breakpoint, tpa_rate, obj.thrust_sp
            %   Based on code from PX4 Firmware:
            %       https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp
            %   Written:       J.X.J. Bannwarth, 2019/01/16
            %   Last modified: J.X.J. Bannwarth, 2019/01/16
            TPA_RATE_LOWER_LIMIT = 0.05;
            tpa = 1.0 - tpa_rate * (abs(thrust_sp) - tpa_breakpoint) / (1.0 - tpa_breakpoint);
            tpa = max([TPA_RATE_LOWER_LIMIT, min([1.0, tpa])]);
            pidAttenuationPerAxis = [ tpa; tpa; 1.0 ];
        end

        % Angle/orientation functions
        function euler = QuatToEuler( quat )
            %QUATTOEULER Convert quaternion to Euler angles
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            quat_tmp = ones(4,1).*quat;
            euler = MulticopterAttitudeControl.DcmToEuler( MulticopterAttitudeControl.QuatToDcm( quat_tmp ) );
        end
        
        function euler = DcmToEuler( dcm )
            %DCMTOEULER Convert DCM to Euler angles
            %   Based on code from PX4 Firmware:
            %       https://github.com/PX4/Matrix/blob/master/matrix/Euler.hpp
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
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
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
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
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            
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
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
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
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            
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
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
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
            quat = MulticopterAttitudeControl.normalize( quat );
        end
        
        function quatInv = InvertQuat( quat )
            %INVERTQUAT Invert quaternion
            %   Based on code from PX4 Firmware:
            %       https://github.com/PX4/Matrix/blob/master/matrix/Quaternion.hpp
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
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
        
        % Math functions
        function out = normalize( in )
            %NORMALIZE Normalise vector
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            out = in ./ norm(in);
        end
        
        function y = constrain( x, x_min, x_max )
            %CONSTRAIN Constrain value between minimum and maximum
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/15
            y = max( min(x, x_max), x_min );
        end
        
        function out = SignNoZero( in )
            %SIGNNOZERO Signum function that returns 1 at in=0
            %   Based on code from PX4 Firmware:
            %       https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/Functions.hpp
            %   Written:       J.X.J. Bannwarth, 2019/01/15
            %   Last modified: J.X.J. Bannwarth, 2019/01/17
            out = ( 0 <= in ) - ( in < 0 );
        end
        
        function out = SuperExpo( value, e, g )
            %SUPEREXPO So called SuperExpo function implementation
            %   It is a 1/(1-x) function to further shape the rc input curve intuitively.
            %   It is enhanced compared to other implementations to keep the scale between
            %   [-1,1].
            %   Input:
            %       - value [-1,1] input value to function
            %       - e [0,1] function parameter to set ratio between linear and cubic shape (see expo)
            %       - g [0,1) function parameter to set SuperExpo shape
            %           0 - pure expo function
            %           0.99 - very strong bent curve, stays zero until maximum stick input
            %   Based on code from PX4 Firmware:
            %       https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/Functions.hpp
            %   Written:       J.X.J. Bannwarth, 2019/01/17
            %   Last modified: J.X.J. Bannwarth, 2019/01/17
            x = MulticopterAttitudeControl.constrain(value, - 1, 1);
            gc = MulticopterAttitudeControl.constrain(g, 0, 0.99);
            out = MulticopterAttitudeControl.Expo(x, e) * (1 - gc) / (1 - abs(x) * gc);
        end
        
        function out = Expo( value, e )
            %EXPO So called exponential curve function implementation
            %   It is essentially a linear combination between a linear and a cubic function.
            %   Input:
            %       - value [-1,1] input value to function
            %       - e [0,1] function parameter to set ratio between linear and cubic shape
            %           0 - pure linear function
            %           1 - pure cubic function
            %   Based on code from PX4 Firmware:
            %       https://github.com/PX4/Firmware/blob/master/src/lib/mathlib/math/Functions.hpp
            %   Written:       J.X.J. Bannwarth, 2019/01/17
            %   Last modified: J.X.J. Bannwarth, 2019/01/17
            x = MulticopterAttitudeControl.constrain( value, -1, 1 );
            ec = MulticopterAttitudeControl.constrain( e, 0, 1 );
            out = (1 - ec) * x + ec * x * x * x;
        end
    end
end