function Motor = MotorRcTimerMt2610( nRotors )
%MOTORRCTIMERMT2610 Load motor parameters
%   MOTOR = MOTORRCTIMERMT2610( ) loads motor parameters for 4 motors.
%   MOTOR = MOTORRCTIMERMT2610( NROTORS ) defines the number of rotors.
%
%   Model: RCTimer MT2610, 920KV.
%
%   See also MOTORTMOTORMN1806.
%
%   Written: 2016, J.X.J. Bannwarth
    arguments
        nRotors (1,1) double = 4
    end

    %% MOTOR ELECTRICAL PROPERTIES
    % RCTimer MT2610, 920KV - modelled as brushed DC motor
    % Datasheet properties
    Motor.R   = 0.11;         % Resistance [ohm]
    Motor.V_0 = 11.1;         % Nominal voltage [V]
    Motor.I_0 = 0.7;          % Idle current [A]
    Motor.K_V = 2*pi*920/60;  % Speed constant [rad/sV]
    
    % Measured values
    Motor.C_TAU = 4.4335282346e-7; % Aerodynamic torque coefficient [Nms^2/rad^2]
    
    % Calculated values
    Motor.K = 7.59588e-6; % Thrust coefficient [Ns^2/rad^2], 960g thrust @ 11.1V, 15.2A
    Motor.K_E = (Motor.V_0-Motor.I_0*Motor.R) / (Motor.K_V*Motor.V_0); % Back EMF constant [Nms/rad]
    Motor.K_T = Motor.K_E; % Motor torque constant [Nm/A]

    %% MOTOR MECHANICAL PROPERTIES
    mProp = 0.011; % [kg]
    dProp = 10 * 0.0254; % [m]
    for i = 1:nRotors
        Motor.I_R(:,:,i) = mProp * diag( [0, 0, (1/12) * dProp^2 ] );
    end
    Motor.I_R_ZZ = reshape( Motor.I_R(3,3,:), [1, size(Motor.I_R,3)]);
end