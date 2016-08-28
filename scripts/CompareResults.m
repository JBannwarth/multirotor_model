% Determine which signals need to be plotted, what their legend should be
% and whether to display the legend or not
xyz = {'x', 'y', 'z'};
rpy = {'roll', 'pitch', 'yaw'};
motor = arrayfun(@num2str, 1:Uav.N_ROTORS, 'unif', 0);
signalsToPlot = { ...
%  signal name,   legend, plotlegend?, cols to plot
%   {'xi'},        xyz,   true,       0;
    {'xi'},        'x',   true,       1;
    {'xi'},        'y',   true,       2;
    {'xi'},        'z',   true,       3;
    {'xiDot'},     xyz,   false,      0;
    {'xiDDot'},    xyz,   false,      0;
    {'omega'},     motor, false,      0;
    {'eta'},       rpy,   true,       0;
    {'etaDot'},    rpy,   true,       0;
    {'etaDDot'},   rpy,   true,       0;
    {'nuBody'},    xyz,   false,      0;
    {'nuBodyDot'}, xyz,   false,      0;
    {'VBody'},     xyz,   false,      0;
    {'VBodyDot'},  xyz,   false,      0;
    {'pwm'},       motor, false,      0;
    {'TBody'},     motor, false,      0;
    {'tauBody'},   motor, false,      0 };
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