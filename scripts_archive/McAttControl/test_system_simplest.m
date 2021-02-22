classdef test_system < matlab.System & matlab.system.mixin.CustomIcon & matlab.system.mixin.SampleTime
    %TEST_SYSTEM System to play around with MATLAB System features
    %   Written:       J.X.J. Bannwarth, 2019/01/30
    %   Last modified: J.X.J. Bannwarth, 2019/01/30

    %% Public, tunable properties
    properties (Nontunable)
        simple_mode (1,1) {mustBeGreaterThanOrEqual(simple_mode, 0), mustBeLessThanOrEqual(simple_mode, 1), mustBeInteger(simple_mode)} = 1; % Simple mode [0-1]
    end

    %% Pre-computed constants
    properties(Access = private)
        some_var = 1;
    end
    
    %% Discrete states
    properties(DiscreteState)
        rates_int                    % Angular rates integral error
    end

    methods(Access = protected)
        function setupImpl( obj )
            % Perform one-time calculations, such as computing constants
            obj.some_var = obj.some_var + 1;
        end

%         
%         function num = getOutputSizeImpl( ~ )
%             num = [2];
%         end

        %% Main function
        function output = stepImpl( obj, varargin)
            input1 = varargin{1};
            if obj.simple_mode
                input2 = 0.5;
            else
                input2 = varargin{2};
            end
            output = [input1; input1+input2];
        end

        function resetImpl( obj )
            % Initialize / reset discrete-state properties
            obj.rates_int = 0;
        end

        function icon = getIconImpl( ~ )
            % Define icon for System block
            icon = ["Test","System"];
        end

        function varargout = getInputNamesImpl(obj)
            % Return output port names for System block
            if ~obj.simple_mode
                varargout = { 'x1', 'x2' };
            else
                varargout{1} = 'x';
            end
        end
    end
end