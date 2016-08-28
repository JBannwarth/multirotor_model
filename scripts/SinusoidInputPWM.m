function [ pwm ] = SinusoidInputPWM( lags, amplitudes, Simulation, Uav )
%SINUSOIDINPUTPWM Summary of this function goes here
%   Detailed explanation goes here

if ((length(lags) ~= length(amplitudes)) || (length(amplitudes)~= Uav.N_ROTORS))
    error(['lags and amplitudes must have length = number of rotors (' ...
        num2str(Uav.N_ROTORS), ')']);
elseif ( (min(size(lags) > 1) || (min(size(amplitudes)) > 1)) )
    error(['lags and amplitudes must be [number of rotors (', ...
        num2str(Uav.N_ROTORS),')] * 1 arrays'])
end

lags = setCorrectOrientation(lags);
amplitudes = setCorrectOrientation(amplitudes);

time = 0:Simulation.T_S:Simulation.T_END;
lags = lags * ones( 1, length(time) );
amplitudes = amplitudes * ones( 1, length(time) );
times = ones(Uav.N_ROTORS, 1) * time;
base_pwm = Uav.THROTTLE_HOVER * ones(Uav.N_ROTORS, 1) * ones( 1, length(time) );

pwm.time = time';
pwm.signals.values = ( base_pwm + amplitudes .* sin(times + lags) )';
pwm.signals.dimensions = Uav.N_ROTORS;
end

function [rotatedArray] = setCorrectOrientation(array)
    if (size(array,2) ~= 1)
        rotatedArray = array';
    else
        rotatedArray = array;
    end
end