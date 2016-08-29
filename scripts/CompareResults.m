% Determine which signals need to be plotted, what their legend should be
% and whether to display the legend or not
xyzLegend = {'x', 'y', 'z'};
rpyLegend = {'roll', 'pitch', 'yaw'};
motorLegend = arrayfun(@num2str, 1:Uav.N_ROTORS, 'unif', 0);
signalsToPlot = { ...
%  signal name,   legend,      plotlegend?, cols to plot (0 = all)
%   {'xi'},        xyz,         true,       0;
    {'xi'},        'x',         true,       1;
    {'xi'},        'y',         true,       2;
    {'xi'},        'z',         true,       3;
    {'xiDot'},     xyzLegend,   false,      0;
    {'xiDDot'},    xyzLegend,   false,      0;
    {'omega'},     motorLegend, false,      0;
    {'eta'},       rpyLegend,   true,       0;
    {'etaDot'},    rpyLegend,   true,       0;
    {'etaDDot'},   rpyLegend,   true,       0;
    {'nuBody'},    xyzLegend,   false,      0;
    {'nuBodyDot'}, xyzLegend,   false,      0;
    {'VBody'},     xyzLegend,   false,      0;
    {'VBodyDot'},  xyzLegend,   false,      0;
    {'pwm'},       motorLegend, false,      0;
    {'TBody'},     motorLegend, false,      0;
    {'tauBody'},   motorLegend, false,      0
};
columnHeadings = {'signalName', 'legend', 'isToBeLabelled', 'columnsToPlot'};
signalsToPlot = cell2struct(signalsToPlot, columnHeadings, 2);

% Compute the dimensions of the grid to fit all signals on screen
nGridCellX = ceil( sqrt( length(signalsToPlot) ) );
nGridCellY = ceil( length(signalsToPlot) / nGridCellX );

figure;
for iSubplot = 1:length(signalsToPlot)
    subplot(nGridCellX, nGridCellY, iSubplot)
    PlotTimeseries(logsoutOld, logsoutNew, signalsToPlot(iSubplot));
end