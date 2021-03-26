function Motor = MotorTMotorMn1806( nRotors )
%MOTORTMOTORMN1806 Load motor parameters
%   MOTOR = MOTORTMOTORMN1806( ) loads motor parameters for 8 motors.
%   MOTOR = MOTORTMOTORMN1806( NROTORS ) defines the number of rotors.
%
%   Model: T-Motor MN1806-14, 2300KV.
%
%   Parameters computed by Z.J. Chen for static drag test.
%   Needs to be used with 'MotorModelZJChen.slx'.
%
%   See also MOTORRCTIMERMT2610.
%
%   Written: 2019/05/01, J.X.J. Bannwarth
    arguments
        nRotors (1,1) double = 8
    end

    %% MOTOR ELECTRICAL PROPERTIES
    % T-Motor MN1806-14, 2300KV
    % Datasheet properties
    Motor.R   = 0.117;        % Resistance [ohm]
    Motor.V_0 = 11.1;         % Nominal voltage [V]
    Motor.I_0 = 0.6;          % Idle current [A]
    Motor.K_V = 2*pi*2300/60; % Speed constant [rad/sV]
    
    % Measured values
    Motor.C_TAU = 1.0410e-08; % Aerodynamic torque coefficient [Nms^2/rad^2]
    Motor.K     = 1.5280e-06; % Thrust coefficient [Ns/rad]
    
    % Calculated values
    Motor.K_E = (Motor.V_0-Motor.I_0*Motor.R) / (Motor.K_V*Motor.V_0); % Back EMF constant [Nms/rad]
    Motor.K_T = Motor.K_E; % Motor torque constant [Nm/A]

    %% MOTOR MECHANICAL PROPERTIES
    for i = 1:nRotors
        Motor.I_R(:,:,i) = diag( [0, 0, 3.2903e-06] );
    end
    Motor.I_R_ZZ = reshape( Motor.I_R(3,3,:), [1, size(Motor.I_R,3)] );
end