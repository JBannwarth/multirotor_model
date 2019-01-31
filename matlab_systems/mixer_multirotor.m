classdef mixer_multirotor < matlab.System & matlab.system.mixin.CustomIcon & ...
        matlab.system.mixin.SampleTime & matlab.system.mixin.Propagates
    %MIXER_MULTIROTOR Apply mixer to input
    %   Based on code from PX4 Firmware (retrieved 2019/01/31, v1.82):
    %       https://github.com/PX4/Firmware/blob/master/src/lib/mixer/mixer_multirotor.cpp
    %   Written:       J.X.J. Bannwarth, 2019/01/31
    %   Last modified: J.X.J. Bannwarth, 2019/01/31

    properties (Nontunable)
        airframeConfig(1,1) {mustBeGreaterThanOrEqual(airframeConfig, 1), mustBeInteger(airframeConfig)} = 1; % Airframe configuration
    end

    properties(Access = private)
        rotor_count(1,1) = -1;
    end

    properties(Constant,Access = private)
        airframeTypes = struct( ...
            'QuadrotorX', [ -0.707107,  0.707107,  1.000000,  1.000000 ;
                             0.707107, -0.707107,  1.000000,  1.000000 ;
                             0.707107,  0.707107, -1.000000,  1.000000 ;
                            -0.707107, -0.707107, -1.000000,  1.000000 ] ...
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
        function validateInputsImpl( obj, px4PWM )
            % Validate inputs to the step method at initialization
            airframes = fieldnames(mixer_multirotor.airframeTypes);
            rotors = obj.airframeTypes.(airframes{obj.airframeConfig});
            if length(px4PWM) ~= length(rotors)
                error( 'Block input should have a length of %d.', ...
                    length(rotors) )
            end
        end

        function flag = isInputSizeMutableImpl( ~, ~ )
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function simPWM = stepImpl( obj, px4PWM )
            simPWM = px4PWM( obj.rotorOrder );
        end

        function icon = getIconImpl( obj )
            % Define icon for System block
            airframes = fieldnames(mixer_multirotor.airframeTypes);
            lastLine = replace( airframes{obj.airframeConfig}, 'P', '+' );
            lastLine = replace( lastLine, 'M', '-' );
            icon = [ "MOTOR", "MAP", lastLine ];
        end

        function out = getOutputSizeImpl( obj )
            % Inherit size from first input port
            out = propagatedInputSize(obj,1);
        end
        
        %% Main function
        function setupImpl( obj )
            % Perform one-time calculations, such as computing constants
            airframes = fieldnames(mixer_multirotor.airframeTypes);
            mixingMatrix = obj.airframeTypes.(airframes{obj.airframeConfig});
            obj.rotor_count = size( mixingMatrix, 2 );
            
            %obj.rotorOrder = obj.airframeTypes.(airframes{obj.airframeConfig});
        end
        
        %% MIXER_MULTIROTOR functions
        function outputs = mix( obj )
            outputs = zeros( obj.rotor_count, 0 );
            roll    = mixer_multirotor.constrain( get_control(1) * obj.roll_scale , -1.0, 1.0);
            pitch   = mixer_multirotor.constrain( get_control(2) * obj.pitch_scale, -1.0, 1.0);
            yaw     = mixer_multirotor.constrain( get_control(3) * obj.yaw_scale  , -1.0, 1.0);
            thrust  = mixer_multirotor.constrain( get_control(4), 0.0, 1.0);
            
            saturation_status_value = 0;
            
            switch (obj.airmode)
                case 'roll_pitch'
                    outputs = mix_airmode_rp( roll, pitch, yaw, thrust );
                case 'roll_pitch_yaw'
                    outputs = mix_airmode_rpy( roll, pitch, yaw, thrust );
                case 'disabled'
                    outputs = mix_airmode_disabled( roll, pitch, yaw, thrust );
                otherwise % just in case: default to disabled
                    outputs = mix_airmode_disabled( roll, pitch, yaw, thrust );
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
                
                outputs(i) = mixer_multirotor.constrain(obj.idle_speed + ...
                    (outputs(i) * (1.0 - obj.idle_speed)), obj.idle_speed, 1.0);
            end
            
            % Slew rate limiting and saturation checking
            for i = 1:obj.rotor_count
                clipping_high = false;
                clipping_low = false;
                
                % check for saturation against static limits
                if (outputs(i) > 0.99)
                    clipping_high = true;
                elseif (outputs(i) < obj.idle_speed + 0.01)
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
                update_saturation_status(i, clipping_high, clipping_low);
            end
            
            % this will force the caller of the mixer to always supply new
            % slew rate values, otherwise no slew rate limiting will happen
            obj.delta_out_max = 0.0;
        end
        
        function outputs = mix_airmode_rp( obj, roll, pitch, yaw, thrust )
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
            % Airmode disabled: never allow to increase the thrust to unsaturate a motor
            % Mix without yaw
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
        
        function outputs = mix_yaw(yaw)
            
            % Add yaw to outputs
            for i = 1:obj.rotor_count
                outputs(i) = outputs(i) + yaw * obj.rotors(i).yaw_scale;
                
                % Yaw will be used to unsaturate if needed
                obj.tmp_array(i) = obj.rotors(i).yaw_scale;
            end
            
            % Change yaw acceleration to unsaturate the outputs if needed (do not change roll/pitch),
            % and allow some yaw response at maximum thrust
            minimize_saturation(obj.tmp_array, outputs, obj.saturation_status, 0, 1.15);
            
            for i = 1:obj.rotor_count
                obj.tmp_array(i) = obj.rotors(i).thrust_scale;
            end
            
            % reduce thrust only
            minimize_saturation(obj.tmp_array, outputs, obj.saturation_status, 0, 1, true);
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
                airframes{i} = replace( airframes{i}, 'P', '+' );
                airframes{i} = replace( airframes{i}, 'M', '-' );
                definition = [ definition, sprintf( '%02d %s ', i, airframes{i} ), '\n' ];
            end
            definition = sprintf( definition(1:end-2) );
            header = matlab.system.display.Header( 'MotorMap', ...
                'Title', 'multirotor_mixer', ...
                'Text', definition );
        end
    end
end