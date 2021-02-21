%PLOTSTEADYSTATEHOVERANGLES Compare steady state hover angles
%   Written by:    J.X.J. Bannwarth, 2017
%   Last modified: J.X.J. Bannwarth, 2018/12/10
close all;
outFolder = '../journal_paper_1/fig';
fontSize  = 12;
outSize   = [9 5];
printResults = false;

load( 'ExpFlightData' )

figure; grid on; box on; hold on;
plot( avgWindSpeed, rad2deg(avgPitch), 'x', windSpeedX, pitchSS, 'o', ...
    'linewidth', 1)
xlabel('Mean wind speed (m/s)' );
ylim([-15 0])
ylabel( 'Mean hover pitch angle ($^\circ$)' );
legend( {'exp', 'sim'}, 'location', 'northeast' )
FormatFigure( outSize , fontSize );

if ( printResults )
    fileName = [ outFolder '/' 'PitchMean-' 'simvexp'];
    PrintFigure( fileName );
end
