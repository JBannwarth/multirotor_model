classdef mixer_multirotor < matlab.System & matlab.system.mixin.CustomIcon & ...
        matlab.system.mixin.SampleTime & matlab.system.mixin.Propagates
    %MIXER_MULTIROTOR Apply mixer to input
    %   Based on code from PX4 Firmware (retrieved 2019/01/31, v1.82):
    %       https://github.com/PX4/Firmware/blob/master/src/lib/mixer/mixer_multirotor.cpp
    %   Written:       J.X.J. Bannwarth, 2019/01/31
    %   Last modified: J.X.J. Bannwarth, 2019/01/31

    properties (Nontunable)
        airframeConfig (1,1) {mustBeGreaterThanOrEqual(airframeConfig, 1), mustBeInteger(airframeConfig)} = 1; % Airframe configuration
        useHorThrust   (1,1) {mustBeGreaterThanOrEqual(useHorThrust, 0), mustBeLessThanOrEqual(useHorThrust, 1),    mustBeInteger(useHorThrust)}   = 0; % Use horizontal thrust [0-1]
    end

    properties(Access = private)
        rotor_count  (1,1) = -1;
        thrust_factor(1,1) = 0;
        idle_speed   (1,1) = 0;
        airmode            = 'roll_pitch_yaw';
    end

    properties(Constant,Access = private)
        airframeTypes = struct( ...
            'quad_x', [ -0.707107,  0.707107,  1.000000,  1.000000 ;
                         0.707107, -0.707107,  1.000000,  1.000000 ;
                         0.707107,  0.707107, -1.000000,  1.000000 ;
                        -0.707107, -0.707107, -1.000000,  1.000000 ], ...
            'octa_x', [ -0.382683,  0.923880, -1.000000,  1.000000 ;
                         0.382683, -0.923880, -1.000000,  1.000000 ;
                        -0.923880,  0.382683,  1.000000,  1.000000 ;
                        -0.382683, -0.923880,  1.000000,  1.000000 ;
                         0.382683,  0.923880,  1.000000,  1.000000 ;
                         0.923880, -0.382683,  1.000000,  1.000000 ;
                         0.923880,  0.382683, -1.000000,  1.000000 ;
                        -0.923880, -0.382683, -1.000000,  1.000000 ] ...
            );
            % 'HexarotorP', [], ...
            % 'HexarotorX', [], ...
            % 'TricopterYP', [], ...
            % 'TricopterYM', [] ...
            % 'OctorotorP', [], ...
            % 'OctorotorX', [], ...
            % 'QuadrotorP', [], ...
            % 'QuadrotorH', [], ...
            % 'QuadrotorWide', [], ...
    end
    
    methods
        function set.airframeConfig( obj, val )
            % Support name-value pair arguments when constructing object
            if val > numel(fieldnames(obj.airframeTypes))
                error( 'Value must be less than or equal to %d.', ...
                    numel(fieldnames(obj.airframeTypes)) );
            end
            obj.airframeConfig = val;
        end
    end

    methods(Access = protected)
%         function validateInputsImpl( ~, varargin )
%             % Validate inputs to the step method at initialization
%             if length( varargin{1} ) ~= 8
%                 error( 'Block input should have a length of 8.' )
%             end
%         end

        function flag = isInputSizeMutableImpl( ~, ~ )
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            if obj.useHorThrust
                num = 2;
            else
                num = 1;
            end
        end

        function px4PWM = stepImpl( obj, varargin )
            % TO IMPLEMENT
            actuatorsControl = varargin{1};
            if obj.useHorThrust
                horThrust = varargin{2};
            else
                horThrust = nan;
            end
        end

        function icon = getIconImpl( obj )
            % Define icon for System block
            airframes = fieldnames(mixer_multirotor.airframeTypes);
            lastLine = airframes{obj.airframeConfig};
            icon = [ "MOTOR", "MAP", lastLine ];
        end

        function varargout = getInputNamesImpl(obj)
            % Return input port names for System block
            varargout{1} = 'actuatorsControl';
            if obj.useHorThrust
                varargout{2} = 'Thor';
            end
        end

        function out = getOutputSizeImpl( obj )
            % Inherit size from first input port
            airframes = fieldnames(mixer_multirotor.airframeTypes);
            mixingMatrix = obj.airframeTypes.(airframes{obj.airframeConfig});
            obj.rotor_count = size( mixingMatrix, 2 );
            out = [ obj.rotor_count, 1 ];
        end
        
        %% Main function
        function setupImpl( obj )
            % Perform one-time calculations, such as computing constants
            airframes = fieldnames(mixer_multirotor.airframeTypes);
            mixingMatrix = obj.airframeTypes.(airframes{obj.airframeConfig});
            obj.rotor_count = size( mixingMatrix, 2 );
        end
        
        %% MIXER_MULTIROTOR functions
        function gain = compute_desaturation_gain( obj, desaturation_vector, outputs, min_output, max_output )
            % Computes the gain k by which desaturation_vector has to be multiplied
            % in order to unsaturate the output that has the greatest saturation.
            % See also minimize_saturation().
            k_min = 0;
            k_max = 0;
            
            for i = 1:obj.rotor_count
                % Avoid division by zero. If desaturation_vector(i) is zero,
                % there's nothing we can do to unsaturate anyway
                if ( abs(desaturation_vector(i)) < eps )
                    continue;
                end
                
                if ( outputs(i) < min_output )
                    k = ( min_output - outputs(i) ) / desaturation_vector(i);
                    
                    if (k < k_min)
                        k_min = k;
                    end
                    
                    if (k > k_max)
                        k_max = k;
                    end
                    
                    sat_status.flags.motor_neg = true;
                end
                
                if ( outputs(i) > max_output )
                    k = ( max_output - outputs(i) ) / desaturation_vector(i);
                    
                    if (k < k_min)
                        k_min = k;
                    end
                    
                    if (k > k_max)
                        k_max = k;
                    end
                    
                    sat_status.flags.motor_pos = true;
                end
            end
            % Reduce the saturation as much as possible
            gain = k_min + k_max;
        end
        
        function outputs = minimize_saturation( obj, desaturation_vector, outputs, sat_status, min_output, max_output, reduce_only)
            % Minimize the saturation of the actuators by adding or
            % substracting a fraction of desaturation_vector.
            % desaturation_vector is the vector that added to the output
            % outputs, modifies the thrust or angular acceleration on a
            % specific axis.
            % For example, if desaturation_vector is given to slide along
            % the vertical thrust axis (thrust_scale), the saturation will
            % be minimized by shifting the vertical thrust setpoint, without
            % changing the roll/pitch/yaw accelerations.
            %
            % Note that as we only slide along the given axis, in extreme
            % cases outputs can still contain values outside of
            % [min_output, max_output].
            k1 = compute_desaturation_gain(obj, desaturation_vector, outputs, ...
                sat_status, min_output, max_output);
            
            if ( reduce_only && k1 > 0 )
                return;
            end
            
            for i = 1:obj.rotor_count
                outputs(i) = outputs(1) + k1 * desaturation_vector(i);
            end
            
            % Compute the desaturation gain again based on the updated outputs.
            % In most cases it will be zero. It won't be if
            % max(outputs) - min(outputs) > max_output - min_output.
            % In that case adding 0.5 of the gain will equilibrate saturations.
            k2 = 0.5 * compute_desaturation_gain(obj, desaturation_vector, ...
                outputs, sat_status, min_output, max_output);
            
            for i = 1:obj.rotor_count
                outputs(i) = outputs(i) + k2 * desaturation_vector(i);
            end
        end

        function outputs = mix_airmode_rp( obj, roll, pitch, yaw, thrust )
            % Mix roll, pitch, yaw, thrust and set the outputs vector.
            % Desaturation behavior: airmode for roll/pitch:
            % thrust is increased/decreased as much as required to meet the
            % demanded roll/pitch. Yaw is not allowed to increase the thrust,
            % @see mix_yaw() for the exact behavior.
            
            % Airmode for roll and pitch, but not yaw
            outputs = zeros( obj.rotor_count, 1 );
            
            % Mix without yaw
            for i = 1:obj.rotor_count
                outputs(i) = roll * obj.rotors(i).roll_scale + ...
                    pitch * obj.rotors(i).pitch_scale + ...
                    thrust * obj.rotors(i).thrust_scale;
                
                % Thrust will be used to unsaturate if needed
                obj.tmp_array(i) = obj.rotors(i).thrust_scale;
            end
            
            minimize_saturation(obj.tmp_array, outputs, obj.saturation_status);
            
            % Mix yaw independently
            outputs = mix_yaw(obj, yaw);
        end
        
        function outputs = mix_airmode_rpy( obj, roll, pitch, yaw, thrust )
            % Mix roll, pitch, yaw, thrust and set the outputs vector.
            % Desaturation behavior: full airmode for roll/pitch/yaw:
            % thrust is increased/decreased as much as required to meet
            % demanded the roll/pitch/yaw.
            
            % Airmode for roll, pitch and yaw
            outputs = zeros( obj.rotor_count, 1 );
            
            % Do full mixing
            for i = 1:obj.rotor_count
                outputs(i) = roll * obj.rotors(i).roll_scale + ...
                    pitch * obj.rotors(i).pitch_scale + ...
                    yaw * obj.rotors(i).yaw_scale + ...
                    thrust * obj.rotors(i).thrust_scale;
                
                % Thrust will be used to unsaturate if needed
                obj.tmp_array(i) = obj.rotors(i).thrust_scale;
            end
            
            minimize_saturation(obj.tmp_array, outputs, obj.saturation_status);
        end
        
        function outputs = mix_airmode_disabled( obj, roll, pitch, yaw, thrust )
            % Mix roll, pitch, yaw, thrust and set the outputs vector.
            % Desaturation behavior: no airmode, thrust is NEVER increased
            % to meet the demanded roll/pitch/yaw. Instead roll/pitch/yaw
            % is reduced as much as needed.
            % Thrust can be reduced to unsaturate the upper side.
            % @see mix_yaw() for the exact yaw behavior.
            
            % Airmode disabled: never allow to increase the thrust to
            % unsaturate a motor Mix without yaw
            outputs = zeros( obj.rotor_count, 1 );
            for i = 1:obj.rotor_count
                outputs(i) = roll * obj.rotors(i).roll_scale + ...
                    pitch * obj.rotors(i).pitch_scale + ...
                    thrust * obj.rotors(i).thrust_scale;
                
                % Thrust will be used to unsaturate if needed
                obj.tmp_array(i) = obj.rotors(i).thrust_scale;
            end
            
            % only reduce thrust
            minimize_saturation(obj.tmp_array, outputs, obj.saturation_status, 0, 1, true);
            
            % Reduce roll/pitch acceleration if needed to unsaturate
            for i = 1:obj.rotor_count
                obj.tmp_array(i) = obj.rotors(i).roll_scale;
            end
            
            minimize_saturation(obj.tmp_array, outputs, obj.saturation_status);
            
            for i = 1:obj.rotor_count
                obj.tmp_array(i) = obj.rotors(i).pitch_scale;
            end
            
            minimize_saturation(obj.tmp_array, outputs, obj.saturation_status);
            
            % Mix yaw independently
            outputs = mix_yaw( obj, yaw );
        end
        
        function outputs = mix_yaw( obj, outputs, yaw )
            % Mix yaw by updating an existing output vector (that already
            % contains roll/pitch/thrust).
            % Desaturation behavior: thrust is allowed to be decreased up
            % to 15% in order to allow some yaw control on the upper end.
            % On the lower end thrust will never be increased, but yaw is
            % decreased as much as required.
            % Add yaw to outputs
            for i = 1:obj.rotor_count
                outputs(i) = outputs(i) + yaw * obj.rotors(i).yaw_scale;
                
                % Yaw will be used to unsaturate if needed
                obj.tmp_array(i) = obj.rotors(i).yaw_scale;
            end
            
            % Change yaw acceleration to unsaturate the outputs if needed (do not change roll/pitch),
            % and allow some yaw response at maximum thrust
            outputs = minimize_saturation(obj, obj.tmp_array, outputs, obj.saturation_status, 0, 1.15);
            
            for i = 1:obj.rotor_count
                obj.tmp_array(i) = obj.rotors(i).thrust_scale;
            end
            
            % reduce thrust only
            minimize_saturation(obj.tmp_array, outputs, obj.saturation_status, 0, 1, true);
        end
                
        function [ outputs, saturation_status ] = mix( obj, actuator_control )
            % Perform the mixing function
            roll    = mixer_multirotor.constrain( actuator_control(1) * obj.roll_scale , -1.0, 1.0);
            pitch   = mixer_multirotor.constrain( actuator_control(2) * obj.pitch_scale, -1.0, 1.0);
            yaw     = mixer_multirotor.constrain( actuator_control(3) * obj.yaw_scale  , -1.0, 1.0);
            thrust  = mixer_multirotor.constrain( actuator_control(4), 0.0, 1.0);
            
            saturation_status = struct( 'value', 0, 'flags', struct( ...
                'valid'      , 0, ... % 0 - true when the saturation status is used
                'motor_pos'  , 0, ... % 1 - true when any motor has saturated in the positive direction
                'motor_neg'  , 0, ... % 2 - true when any motor has saturated in the negative direction
                'roll_pos'   , 0, ... % 3 - true when a positive roll demand change will increase saturation
                'roll_neg'   , 0, ... % 4 - true when a negative roll demand change will increase saturation
                'pitch_pos'  , 0, ... % 5 - true when a positive pitch demand change will increase saturation
                'pitch_neg'  , 0, ... % 6 - true when a negative pitch demand change will increase saturation
                'yaw_pos'    , 0, ... % 7 - true when a positive yaw demand change will increase saturation
                'yaw_neg'    , 0, ... % 8 - true when a negative yaw demand change will increase saturation
                'thrust_pos' , 0, ... % 9 - true when a positive thrust demand change will increase saturation
                'thrust_neg' , 0  ... % 0 - true when a negative thrust demand change will increase saturation
                ) );
            
            switch (obj.airmode)
                case 'roll_pitch'
                    outputs = mix_airmode_rp( obj, roll, pitch, yaw, thrust );
                case 'roll_pitch_yaw'
                    outputs = mix_airmode_rpy( obj, roll, pitch, yaw, thrust );
                case 'disabled'
                    outputs = mix_airmode_disabled( obj, roll, pitch, yaw, thrust );
                otherwise % just in case: default to disabled
                    outputs = mix_airmode_disabled( obj, roll, pitch, yaw, thrust );
            end
            
            % Apply thrust model and scale outputs to range [idle_speed, 1].
            % At this point the outputs are expected to be in [0, 1], but
            % they can be outside, for example if a roll command exceeds
            % the motor band limit.
            for i = 1:obj.rotor_count
                % Implement simple model for static relationship between
                % applied motor pwm and motor thrust model: 
                % thrust = (1 - obj.thrust_factor) * PWM + obj.thrust_factor * PWM^2
                if (obj.thrust_factor > 0.0)
                    outputs(i) = -(1.0 - obj.thrust_factor) / (2.0 * obj.thrust_factor) + ...
                        sqrt((1.0 - obj.thrust_factor) * (1.0 - obj.thrust_factor) / (4.0 * obj.thrust_factor * obj.thrust_factor) + ...
                        ( max( [ outputs(i), 0 ] ) / obj.thrust_factor) );
                end
                
                outputs(i) = mixer_multirotor.constrain( obj.idle_speed + ...
                    (outputs(i) * (1.0 - obj.idle_speed)), obj.idle_speed, 1.0 );
            end
            
            % Slew rate limiting and saturation checking
            for i = 1:obj.rotor_count
                clipping_high = false;
                clipping_low = false;
                
                % check for saturation against static limits
                if ( outputs(i) > 0.99 )
                    clipping_high = true;
                elseif ( outputs(i) < obj.idle_speed + 0.01 )
                    clipping_low = true;
                    
                end
                
                % check for saturation against slew rate limits
                if (obj.delta_out_max > 0.0)
                    delta_out = outputs(i) - obj.outputs_prev(i);
                    
                    if (delta_out > obj.delta_out_max)
                        outputs(i) = obj.outputs_prev(i) + obj.delta_out_max;
                        clipping_high = true;
                        
                    elseif (delta_out < -obj.delta_out_max)
                        outputs(i) = obj.outputs_prev(i) - obj.delta_out_max;
                        clipping_low = true;
                    end
                end
                
                obj.outputs_prev(i) = outputs(i);
                
                % update the saturation status report
                saturation_status = update_saturation_status( obj, i, clipping_high, clipping_low, saturation_status );
            end
            
            % This will force the caller of the mixer to always supply new
            % slew rate values, otherwise no slew rate limiting will happen
            obj.delta_out_max = 0.0; % ???
        end
        
        function saturation_status = updatesaturation_status( obj, index, clipping_high, clipping_low, saturation_status )
            % The motor is saturated at the upper limit
            % check which control axes and which directions are contributing
            if (clipping_high)
                if (obj.rotors(index).roll_scale > 0.0)
                    % A positive change in roll will increase saturation
                    saturation_status.flags.roll_pos = 1;
                    
                elseif (obj.rotors(index).roll_scale < 0.0)
                    % A negative change in roll will increase saturation
                    saturation_status.flags.roll_neg = 1;
                end
                
                % check if the pitch input is saturating
                if (obj.rotors(index).pitch_scale > 0.0)
                    % A positive change in pitch will increase saturation
                    saturation_status.flags.pitch_pos = 1;
                    
                elseif (obj.rotors(index).pitch_scale < 0.0)
                    % A negative change in pitch will increase saturation
                    saturation_status.flags.pitch_neg = 1;
                end
                
                % check if the yaw input is saturating
                if (obj.rotors(index).yaw_scale > 0.0)
                    % A positive change in yaw will increase saturation
                    saturation_status.flags.yaw_pos = 1;
                    
                elseif (obj.rotors(index).yaw_scale < 0.0)
                    % A negative change in yaw will increase saturation
                    saturation_status.flags.yaw_neg = 1;
                end
                
                % A positive change in thrust will increase saturation
                saturation_status.flags.thrust_pos = 1;
                
            end
            
            % The motor is saturated at the lower limit
            % check which control axes and which directions are contributing
            if (clipping_low)
                % check if the roll input is saturating
                if (obj.rotors(index).roll_scale > 0.0)
                    % A negative change in roll will increase saturation
                    saturation_status.flags.roll_neg = 1;
                    
                elseif (obj.rotors(index).roll_scale < 0.0)
                    % A positive change in roll will increase saturation
                    saturation_status.flags.roll_pos = 1;
                end
                
                % check if the pitch input is saturating
                if (obj.rotors(index).pitch_scale > 0.0)
                    % A negative change in pitch will increase saturation
                    saturation_status.flags.pitch_neg = 1;
                    
                elseif (obj.rotors(index).pitch_scale < 0.0)
                    % A positive change in pitch will increase saturation
                    saturation_status.flags.pitch_pos = 1;
                end
                
                % check if the yaw input is saturating
                if (obj.rotors(index).yaw_scale > 0.0)
                    % A negative change in yaw will increase saturation
                    saturation_status.flags.yaw_neg = 1;
                    
                elseif (obj.rotors(index).yaw_scale < 0.0)
                    % A positive change in yaw will increase saturation
                    saturation_status.flags.yaw_pos = 1;
                end
                
                % A negative change in thrust will increase saturation
                saturation_status.flags.thrust_neg = 1;
            end
            
            saturation_status.flags.valid = 1;
        end

    end

    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            definition = [ ...
                'Apply mixer to input\n' ...
                'Available frames:\n' ] ;
            airframes = fieldnames(mixer_multirotor.airframeTypes);
            for i = 1:numel( airframes )
                definition = [ definition, sprintf( '%02d %s ', i, airframes{i} ), '\n' ];
            end
            definition = sprintf( definition(1:end-2) );
            header = matlab.system.display.Header( 'MotorMap', ...
                'Title', 'multirotor_mixer', ...
                'Text', definition );
        end
        
        function y = constrain( x, x_min, x_max )
            y = max( min(x, x_max), x_min );
        end
    end
end