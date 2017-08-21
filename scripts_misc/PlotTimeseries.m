function [ ] = PlotTimeseries( dataset1, dataset2, signalToPlot )
%PLOTTIMESERIES Summary of this function goes here
%   Detailed explanation goes here
    signalToPlotName = char(signalToPlot.signalName);
    
    hold on;
    if ( (length(signalToPlot.columnsToPlot) == 1) && ...
            (signalToPlot.columnsToPlot == 0) )
        % Plot all
%         plot( dataset1.get(signalToPlotName).Values );
%         plot( dataset2.get(signalToPlotName).Values.time, ...
%         dataset2.get(signalToPlotName).Values.data, '--', 'linewidth', 1 ) 
        plot( dataset1.get(signalToPlotName).Values.data );
        plot( dataset2.get(signalToPlotName).Values.data, ...
            '--', 'linewidth', 1 );
    else
        plot( dataset1.get(signalToPlotName).Values.data(:, ...
            signalToPlot.columnsToPlot) );
        plot( dataset2.get(signalToPlotName).Values.data(:, ...
            signalToPlot.columnsToPlot), '--', 'linewidth', 1 );
    end
    
    %title(['Time Series Plot:' dataset1.get(signalToPlotName).Values.Name])
    xlabel( ['Time (' ...
        dataset1.get(signalToPlotName).Values.TimeInfo.Units ')'] )
    ylabel( [dataset1.get(signalToPlotName).Values.Name ' (' ...
        dataset1.get(signalToPlotName).Values.DataInfo.Units ')' ] )
    xlim([0 inf])
    
    if (~isempty(signalToPlot.legend) && signalToPlot.isToBeLabelled)
        if (length(signalToPlot.legend) == 1)
            % If the legend contains a single string, it is not considered
            % a cell array and therefore if we didn't use curly brackets,
            % MATLAB would simply concatenate the two strings together
            % 'X_1X_2' instead of giving us a cell array {'X_1' 'X_2'}
            legendLabels = { strcat(signalToPlot.legend,'_1') ...
                strcat(signalToPlot.legend,'_2') };
        else
            legendLabels = [ strcat(signalToPlot.legend,'_1') ...
                strcat(signalToPlot.legend,'_2') ];
        end
        
        legend(legendLabels, 'Location', 'northwest', ...
            'Orientation', 'horizontal')
    end
    hold off;
end

