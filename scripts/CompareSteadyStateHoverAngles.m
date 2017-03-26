close all;

windSpeedXExp = [0 1.31 2.68 4.25 5.78 7.28];
pitchSSExp = [0 2.623 5.218 7.896 10.558 13.641];

figure; grid on; box on; hold on;
plot( windSpeedXExp, pitchSSExp, 'o', windSpeedX, pitchSS, 'o', ...
    'linewidth', 1)
xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');
ylim([0 20])
ylabel('Mean hover pitch angle (deg)', 'Interpreter', 'latex');
legend( {'exp', 'sim'}, 'location', 'southeast', 'Interpreter', 'latex' )

SetFigProp([12 8], 12)