function Motor = MotorTMotorMn1806( )
%MOTORTMOTORMN1806 Load motor parameters
%   MOTOR = MOTORTMOTORMN1806( ) loads motor parameters.
%
%   Model: T-Motor MN1806-14, 2300KV.
%
%   Parameters computed by Z.J. Chen for static drag test.
%   Needs to be used with 'MotorModelJeremy.slx'.
%
%   See also MOTORRCTIMERMT2610.
%
%   Written: 2019/05/01, J.X.J. Bannwarth

    %% MOTOR ELECTRICAL PROPERTIES
    % T-Motor MN1806-14, 2300KV
    Motor.R   = 0.117;      % Motor resistance [Ohm]
    Motor.K_T = 1.928e-08;  % Effective motor torque constant [Nm/A]
    Motor.K   = 1.5280e-06; % Rotor thrust coefficient

    Motor.V_0 = 12.4;
    Motor.I_0 = 0.6;
    Motor.K_V = 240.8554;

    %% MOTOR MECHANICAL PROPERTIES
    % Rotor drag coefficient (no value, but necessary)
    Motor.B = 4.4335282346e-7 .* ones(1,Uav.N_ROTORS);

    for i = 1:Uav.N_ROTORS
        Motor.I_R(:,:,i) = diag( [0, 0, 3.2903e-06] );
    end
    Motor.I_R_ZZ = reshape( Motor.I_R(3,3,:), [1, size(Motor.I_R,3)] );
end