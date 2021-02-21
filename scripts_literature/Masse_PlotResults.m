%MASSE_PLOTRESULTS Plot results from Masse et al.'s simulation
%   Written by:    J.X.J. Bannwarth, 2018/11/30
%	Last modified: J.X.J. Bannwarth, 2019/01/14
rVec = out.get('r');
zVec = out.get('z');
uVec = out.get('u_m');

figure
subplot(4,2,1); hold on; grid on; box on;
plot( rVec.Time, rVec.Data(:,1), zVec.Time, zVec.Data(:,1) )
ylabel('$x$ (m)')

subplot(4,2,3); hold on; grid on; box on;
plot( rVec.Time, rVec.Data(:,2), zVec.Time, zVec.Data(:,2) )
ylabel('$y$ (m)')

subplot(4,2,5); hold on; grid on; box on;
plot( rVec.Time, rVec.Data(:,3), zVec.Time, zVec.Data(:,3) )
ylabel('$z$ (m)')

subplot(4,2,7); hold on; grid on; box on;
plot( rVec.Time, rad2deg(rVec.Data(:,4)), zVec.Time, rad2deg(zVec.Data(:,4)) )
ylabel('$\psi$ ($^\circ$)'); xlabel('Time (s)')
legend( {'ref', 'act' }, 'Orientation', 'horizontal', 'Location', 'best' )

subplot(1,2,2); hold on; grid on; box on
plot( uVec.Time, uVec.Data )
ylabel('Normalised motor input (-)'); xlabel('Time(s)')
if size(uVec.Data,2) == 8
    legend( {'FRO', 'FRI', 'BRI', 'BRO', 'BLO', 'BLI', 'FLI', 'FLO' })
else
    legend( {'FR', 'BR', 'BL', 'FL' })
end

FormatFigure([17 13])
PrintFigure( fullfile( 'work', 'MasseResults' ) );