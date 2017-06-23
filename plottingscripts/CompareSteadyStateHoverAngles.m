close all;
outFolder = '../journal_paper_1/fig';
fontSize  = 9;
outSize   = [8.85684 5];
printResults = true;

load( 'ExpFlightData' )

tCuttOff = windInput.Time(end);

figure; grid on; box on; hold on;
plot( avgWindSpeed, -rad2deg(avgPitch), 'x', windSpeedX, pitchSS, 'o', ...
    'linewidth', 1)
xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');
ylim([0 15])
set( gca, 'TickLabelInterpreter', 'latex' )
ylabel('Mean hover pitch angle (deg)', 'Interpreter', 'latex');
legend( {'exp', 'sim'}, 'location', 'southeast', 'Interpreter', 'latex' )

if ( printResults )
    fileName = [ outFolder '/' 'PitchMean-' 'simvexp'];
    SetFigProp( outSize , fontSize );
    MatlabToLatexEps( fileName, [], false );
end
