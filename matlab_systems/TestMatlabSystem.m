classdef TestMatlabSystem < matlab.System & matlab.system.mixin.Propagates & matlab.system.mixin.SampleTime
    %TESTMATLABSYSTEM System for testing purposes, currently a PI controller
    %   Written:       J.X.J. Bannwarth, 2019/01/31
    %   Last modified: J.X.J. Bannwarth, 2019/02/12

    properties (Nontunable)
        Ts(1,1) {mustBeGreaterThanOrEqual(Ts, 0)} = 1/200; % Ts (s)
        Kp(1,1) = 0.2; % Kp
        Ki(1,1) = 0.02; % tauI
    end

    properties(DiscreteState)
        uInt
    end

    properties (Access = private)
        %rotor_count = 2;
    end

    methods(Access = protected)
%         function setupImpl( obj )
            % Perform one-time calculations, such as computing constants
%             obj.rotor_count = obj.test;
%         end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.uInt = 0;
        end


        function y = stepImpl( obj, u )
            obj.uInt = obj.uInt + u*obj.Ts*obj.Ki;
            
            pwm = obj.Kp*u + obj.uInt;
            
            % Apply the same scaling as in the PX4 firmware to keep similar
            % gains
            pwm = (1000 * pwm + 1000);
            y = 0.84 * ( 7.969173e-1.*pwm - 6.528579e2 );
        end
        
%         function y = testFoo( obj, x )
%             y = zeros(obj.rotor_count, 1);
%             for i = 1:obj.rotor_count
%                 y(i,1) = x;
%             end
%         end

        function out = getOutputSizeImpl( ~ )
            % Inherit size from first input port
            out = [ 1, 1 ];
        end

        function out = isOutputFixedSizeImpl( ~ )
            % Return true for each output port with fixed size
            out = true;

            % Example: inherit fixed-size status from first input port
            % out = propagatedInputFixedSize(obj,1);
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl( ~, name )
            % Return size, data type, and complexity of discrete-state
            % specified in name
            switch name
                case 'uInt'
                    sz = [1 1];
                    dt = "double";
                    cp = false;
            end
        end

        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            % sts = obj.createSampleTime("Type", "Inherited");

            % Example: specify discrete sample time
            sts = obj.createSampleTime("Type", "Discrete", ...
                "SampleTime", obj.Ts);
        end
    end

end