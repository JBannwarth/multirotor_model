function datasetOut = SetDatasetUnits(datasetIn, unitsLookUpTable, ...
    convertAnglesToDegrees)
%SETDATASETUNITS Add units to all the timeseries in a dataset
    
    % Argument management
    if nargin == 2
        convertAnglesToDegrees = false;
    end

    datasetOut = datasetIn;
    signalNames = datasetIn.getElementNames;
    
    for iSignal = 1:length(signalNames)
        % Find matching unit
        unit = char( unitsLookUpTable{ ...
            strcmp(signalNames{iSignal}, unitsLookUpTable.Signal), ...
            2 } );

        element = datasetIn.getElement(iSignal);
        
        if (convertAnglesToDegrees && strncmp(unit, 'rad', 3) )
            element.Values.Data = rad2deg(element.Values.Data);
            unit(1:3) = 'deg';
        end
        
        element.Values.DataInfo.Units = unit;
        datasetOut = datasetOut.setElement(iSignal, element);
    end
end