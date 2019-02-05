classdef MotorMap < matlab.System & matlab.system.mixin.CustomIcon & ...
        matlab.system.mixin.SampleTime & matlab.system.mixin.Propagates
    %MOTORMAP Convert from PX4 motor ordering to NED CCW ordering
    %   See PX4 airframe reference (retrieved 2019/01/31, v1.82):
    %       https://dev.px4.io/en/airframes/airframe_reference.html
    %   Written:       J.X.J. Bannwarth, 2019/01/31
    %   Last modified: J.X.J. Bannwarth, 2019/02/05

    properties (Nontunable)
        airframeConfig(1,1) {mustBeGreaterThanOrEqual(airframeConfig, 1), mustBeInteger(airframeConfig)} = 5; % Airframe configuration
    end

    properties (Access = private)
        rotorOrder = -1;
    end

    properties (Constant, Access = private)
        airframeTypes = struct( ...
            'hex_plus' , [1 4 6 2 3 5], ...
            'hex_x'    , [5 1 4 6 2 3], ...
            'octa_plus', [1 3 8 4 2 6 7 5], ...
            'octa_x'   , [1 3 8 4 2 6 7 5], ...
            'quad_plus', [3 1 4 2], ...
            'quad_h'   , [1 4 2 3], ...
            'quad_wide', [1 4 2 3], ...
            'quad_x'   , [1 4 2 3], ...
            'tri_y'    , [1 3 2] ...
            );
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
        function setupImpl( obj )
            % Perform one-time calculations, such as computing constants
            airframes = fieldnames(MotorMap.airframeTypes);
            obj.rotorOrder = obj.airframeTypes.(airframes{obj.airframeConfig});
        end

        function validateInputsImpl( obj, px4PWM )
            % Validate inputs to the step method at initialization
            airframes = fieldnames(MotorMap.airframeTypes);
            rotors = obj.airframeTypes.(airframes{obj.airframeConfig});
            if length(px4PWM) ~= length(rotors)
                error( 'Block input should have a length of %d.', ...
                    length(rotors) )
            end
        end

        function simPWM = stepImpl( obj, px4PWM )
            simPWM = px4PWM( obj.rotorOrder );
        end

        function icon = getIconImpl( obj )
            % Define icon for System block
            airframes = fieldnames(MotorMap.airframeTypes);
            lastLine = airframes{obj.airframeConfig};
            icon = [ "MOTOR", "MAP", lastLine ];
        end

        function out = getOutputSizeImpl( obj )
            % Inherit size from first input port
            out = propagatedInputSize(obj,1);
        end
        
    end

    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            definition = [ ...
                'Convert from PX4 motor ordering to NED CCW ordering\n' ...
                'Available frames:\n' ] ;
            airframes = fieldnames(MotorMap.airframeTypes);
            for i = 1:numel( airframes )
                definition = [ definition, sprintf( '%02d %s ', i, airframes{i} ), '\n' ];
            end
            definition = sprintf( definition(1:end-2) );
            header = matlab.system.display.Header( 'MotorMap', ...
                'Title', 'Motor Map', ...
                'Text', definition );
        end
    end
end