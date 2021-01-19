%FLIGHTCONDITIONSSCATTERPLOT
%   Written: 2017, J.X.J. Bannwarth
close all;

plotHeight = 6;
plotWidth = 10;
fontSize = 12;

set(0,'defaultlinelinewidth',1)

outputFolder = 'work/';
saveFigures = false;

%% Analyse data
% Get timeseries from logs
wind  = logsout.get('windInput').Values;
omega = logsout.get('omega').Values;
q     = logsout.get('q').Values;
VBody = logsout.get('VBody').Values;
xiDot = logsout.get('xiDot').Values;

% Convert wind data to world frame
windWorld = wind.Data .* [ ones(length(wind.Data), 2), -1.* ones( length(wind.Data), 1) ];

% Find wind in body frame
qInv = QuatInv( q.Data );
windQuat = [ zeros(length(windWorld), 1), windWorld ];
windQuatBody = QuatMult( qInv, QuatMult( windQuat, q.Data ) );
windBody = windQuatBody(:, 2:end);
windAppBody = windBody + VBody.Data;

figure('Name', 'Wind body frame vs world frame')
ylabelStr = {'x (m/s)', 'y (m/s)' 'z (m/s)'};
% legendStr = { {'x_o', 'x_b'}, {'y_o', 'y_b'}, {'z_o', 'z_b'} };
for i = 1:3
    subplot(3, 1, i)
    plot( wind.Time, windWorld(:,i), wind.Time, windBody(:,i) )
    grid on
    ylabel( [ ylabelStr{i} ] )
    legend( {'world', 'body'} )
    SetFigProp([plotWidth plotHeight], fontSize)
end
xlabel('time(s)')

% Find incident angle - assume lateral wind is negligible
incAngle = rad2deg( atan2( windAppBody(:, 3), windAppBody(:, 1) ) );
incAngleWorld = rad2deg( atan2( windWorld(:, 3), windWorld(:, 1) ) );

figure('Name', 'Incident angle'); grid on; box on; hold on;
plot( wind.Time, incAngle, wind.Time, incAngleWorld )
ylabel( 'Incident angle (deg)' )
xlabel( 'Time (s)' )
legend( {'Body', 'World (incorrect)'} )
xlim([-inf,inf])
ylim([-inf,inf])
SetFigProp([plotWidth plotHeight], fontSize)

if saveFigures
    matlabToLatexEps( [outputFolder 'incidentAngle'], [] )
end

figure('Name', 'Wind speed'); grid on; box on; hold on;
plot( wind.Time, windWorld(:,1) + VBody.Data(:,1), wind.Time, windWorld(:,1) )
ylabel( 'x wind speed (m/s)' )
xlabel( 'Time (s)' )
legend( {'Applied', 'Absolute'} )
xlim([-inf,inf])
ylim([-inf,inf])
SetFigProp([plotWidth plotHeight], fontSize)

if saveFigures
    matlabToLatexEps( [outputFolder 'windSpeed'], [] )
end

% Find average rotor speed
omegaAvg = mean( omega.Data, 2 );
figure('Name', 'Average rotor speed')
plot( omega.Time, 60 .* omegaAvg ./ 360 )
ylabel( 'Average rotor speed (RPM)' )
xlabel( 'Time (s)' )
xlim([-inf,inf])
ylim([-inf,inf])
SetFigProp([plotWidth plotHeight], fontSize)

if saveFigures
    matlabToLatexEps( [outputFolder 'rotorSpeed'], [] )
end

% Statistics
% close all;
figure;
histfit(incAngle, 50, 'Normal')
xlabel('Incident angle (deg)')
ylabel('Occurences')
hold on
[muhat, sigmahat] = normfit(incAngle);
ylimits = ylim;
plot( [muhat - 2*sigmahat, muhat - 2*sigmahat], [0 ylimits(2)], 'r--', ...
      [muhat - 3*sigmahat, muhat - 3*sigmahat], [0 ylimits(2)], 'g--')
plot( [muhat + 2*sigmahat, muhat + 2*sigmahat], [0 ylimits(2)], 'r--', ...
      [muhat + 3*sigmahat, muhat + 3*sigmahat], [0 ylimits(2)], 'g--')
legend( {'Data', 'fit', '$\pm 2 \sigma$', '$\pm 3 \sigma$'}, ...
    'interpreter', 'latex')
SetFigProp([plotWidth plotHeight], fontSize)
xlim([-inf,inf])
ylim([-inf,inf])

if saveFigures
    matlabToLatexEps( [outputFolder 'incidentAngleHist'], [] )
end

figure;
histfit(60 * omegaAvg ./ 360, 50, 'Normal')
xlabel('Mean rotor speed (RPM)')
ylabel('Occurences')
hold on
[muhat, sigmahat] = normfit( 60 * omegaAvg ./ 360 );
ylimits = ylim;
plot( [muhat - 2*sigmahat, muhat - 2*sigmahat], [0 ylimits(2)], 'r--', ...
      [muhat - 3*sigmahat, muhat - 3*sigmahat], [0 ylimits(2)], 'g--')
plot( [muhat + 2*sigmahat, muhat + 2*sigmahat], [0 ylimits(2)], 'r--', ...
      [muhat + 3*sigmahat, muhat + 3*sigmahat], [0 ylimits(2)], 'g--')
legend( {'Data', 'fit', '$\pm 2 \sigma$', '$\pm 3 \sigma$'}, ...
    'interpreter', 'latex')
xlim([-inf,inf])
ylim([-inf,inf])
SetFigProp([plotWidth plotHeight], fontSize)

if saveFigures
    matlabToLatexEps( [outputFolder 'omegaHist'], [] )
end

figure;
histfit(xiDot.Data(:,1) + windWorld(:,1), 50, 'Normal')
xlabel('x wind speed (m/s)')
ylabel('Occurences')
hold on
[muhat, sigmahat] = normfit( xiDot.Data(:,1) + windWorld(:,1) );
ylimits = ylim;
plot( [muhat - 2*sigmahat, muhat - 2*sigmahat], [0 ylimits(2)], 'r--', ...
      [muhat - 3*sigmahat, muhat - 3*sigmahat], [0 ylimits(2)], 'g--')
plot( [muhat + 2*sigmahat, muhat + 2*sigmahat], [0 ylimits(2)], 'r--', ...
      [muhat + 3*sigmahat, muhat + 3*sigmahat], [0 ylimits(2)], 'g--')
legend( {'Data', 'fit', '$\pm 2 \sigma$', '$\pm 3 \sigma$'}, ...
    'interpreter', 'latex')
xlim([-inf,inf])
ylim([-inf,inf])
SetFigProp([plotWidth plotHeight], fontSize)

if saveFigures
    matlabToLatexEps( [outputFolder 'windSpeedHist'], [] )
end

%% Plot data
figure('Name', 'Misc')
toPlot = {'eta', 'xi', 'windInput'};

for i = 1:length(toPlot)
    subplot(2, 2, i);
    plot( logsout.get( toPlot{i} ).Values );
    xlim([-inf,inf])
    ylim([-inf,inf])
    SetFigProp([plotWidth plotHeight], fontSize)
end