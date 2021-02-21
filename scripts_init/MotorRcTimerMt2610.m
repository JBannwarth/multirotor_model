function Motor = MotorRcTimerMt2610( )
%MOTORRCTIMERMT2610 Load motor parameters
%   MOTOR = MOTORRCTIMERMT2610( ) loads motor parameters.
%
%   Model: RCTimer MT2610, 920KV.
%
%   See also MOTORTMOTORMN1806.
%
%   Written: 2016, J.X.J. Bannwarth

    %% MOTOR ELECTRICAL PROPERTIES
    % RCTimer MT2610, 920KV - modelled as brushed DC motor
    Motor.R   = 0.11;              % motor resistance [Ohm]
    Motor.K_E = 1 / (920*pi/30);   % motor back emf Constant [Vs/rad]
    Motor.K_T = Motor.K_E * (3/2); % effective motor torque constant [Nm/A] (3/2 factor for BLDC c.f. brushed DC)

    % Rotor thrust coefficient
    % 960g thrust @ 11.1V, 15.2A
    Motor.K = 7.59588e-6;

    % Rotor drag coefficient
    Motor.B = 4.4335282346e-7 .*ones(1,Uav.N_ROTORS);

    % Masses
    Motor.M_M = 0.054; % Motor
    Motor.M_P = 0.011; % Propeller

    % Motor inertia [kgm^2]
    Motor.R_M = 0.016;
    Motor.H_M = 0.0193;
    Motor.I_M = Motor.M_M * diag( [(1/12) * (3*Motor.R_M^2 + Motor.H_M^2), ...
                                   (1/12) * (3*Motor.R_M^2 + Motor.H_M^2), ...
                                   (1/2)  * Motor.R_M^2] );

    % Propeller inertia (only model inertia in z-axis) [kgm^s]
    Motor.R_P = 10 * 0.0254 / 2; % 13*0.0254/2;
    Motor.I_P = Motor.M_P*diag([0, 0, (1/12) * (2*Motor.R_P)^2]);

    % Rotor inertia (spinning part of motor + propeller) [kgm^s]
    Motor.M_R = 0;
    Motor.R_R = 0;
    Motor.H_R = 0;
    for i = 1:Uav.N_ROTORS
        Motor.I_R(:,:,i) = (Motor.I_M + Motor.I_P);
    end
    Motor.I_R_ZZ = reshape( Motor.I_R(3,3,:), [1, size(Motor.I_R,3)]);
end