close all;

windSpeedXExp = [0;2.55999837265402;3.45522298273824;4.40686338715537;5.17829923510479;6.09013240228385;6.79493302374613];
pitchSSExp = -1*[-0.305087653418061;-4.07998532241812;-6.19058698321617;-8.00281208413321;-9.67179114530467;-11.6857426135563;-13.1550507234807];

figure; grid on; box on; hold on;
plot( windSpeedXExp, pitchSSExp, 'x', windSpeedX, pitchSS, 'o', ...
    'linewidth', 1)
xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');
%ylim([0 20])
ylabel('Mean hover pitch angle (deg)', 'Interpreter', 'latex');
legend( {'exp', 'sim'}, 'location', 'southeast', 'Interpreter', 'latex' )

SetFigProp([12 8], 12)