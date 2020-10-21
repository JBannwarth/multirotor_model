pwm = [flogOriginal.actuator_outputs{:,3:6}];
time = flogOriginal.actuator_outputs.time;
close all;

subplot(2,1,1);  hold on; grid on; box on;
plot(time,pwm);
legend({'1FR','2BL','3FL','4BR'}, ...
    'location','south', 'orientation', 'horizontal' );
xlim([0,inf]);
xlabel('Time (s)'); ylabel('PWM');

subplot(2,1,2); hold on; grid on; box on;
plot(tExp, rad2deg(eulExp.Pitch) );
xlim([0,inf]);
xlabel('Time (s)'); ylabel('Pitch (deg)');

SetFigProp([12 15])

MatlabToLatexEps( 'PwmPitch' )