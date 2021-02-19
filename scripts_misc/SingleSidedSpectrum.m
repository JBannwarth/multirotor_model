function [ f, P1, P2 ] = SingleSidedSpectrum( t, x )
%SINGLESIDEDSPECTRUM Calculate single sided spectrum from time-domain data
%   Written: 2021/02/19, J.X.J. Bannwarth

    % Sampling constants
    T  = median( diff(t) ); % Sampling period
    Fs = 1 / T;             % Sampling frequency

    % Remove DC component
    x = x - mean(x,1);
    
    % Calculate FFT
    xFFT = fft( x );
    Y = xFFT(:,1);
    if rem( length(Y), 2) ~= 0
        Y = Y(1:end-1);
    end
    L = size( Y, 1 ); % Length of signal
    P2 = abs( Y / L );
    P1 = P2( 1:(L/2)+1 );
    P1(2:end-1) = 2 * P1(2:end-1);
    f = Fs * ( 0:(L/2) ) / L;
end