close all;

outFolder = '../journal_paper_1/fig';
fontSize  = 9;
outSize   = [8.85684 5];
printResults = false;

windXLimits = [0 7];

if ( useClosedContraptionData )
    suffix = '-narrow';
else
    suffix = '-wide';
end

% Compute statistics
dt = str2double( get_param( [model '/To Workspace errX'], 'SampleTime') );

if ( useClosedContraptionData )
    load( 'ExpFlightData' )
else
    load( 'ExpFlightDataWide' )
end

tCuttOff = Simulation.T_END / 2;

for i = 1:length( output )
    % Get error
    error(i).x = output(i).get('posTrackingX');
    error(i).y = output(i).get('posTrackingY');
    error(i).z = output(i).get('posTrackingZ');
    error(i).xEst = outputEst(i).get('posTrackingX');
    error(i).yEst = outputEst(i).get('posTrackingY');
    error(i).zEst = outputEst(i).get('posTrackingZ');
    error(i).t = (0:dt:Simulation.T_END)';
    
    % UAV takes about 20 s to recover from initial wind
    error(i).x = error(i).x( error(i).t >= tCuttOff );
    error(i).y = error(i).y( error(i).t >= tCuttOff );
    error(i).z = error(i).z( error(i).t >= tCuttOff );
    error(i).xEst = error(i).xEst( error(i).t >= tCuttOff );
    error(i).yEst = error(i).yEst( error(i).t >= tCuttOff );
    error(i).zEst = error(i).zEst( error(i).t >= tCuttOff );
        
    meanE.x(i) = mean( error(i).x );
    meanE.y(i) = mean( error(i).y );
    meanE.z(i) = mean( error(i).z );
    stdE.x(i)  =  std( error(i).x );
    stdE.y(i)  =  std( error(i).y );
    stdE.z(i)  =  std( error(i).z );

    meanE.xEst(i) = mean( error(i).xEst );
    meanE.yEst(i) = mean( error(i).yEst );
    meanE.zEst(i) = mean( error(i).zEst );
    stdE.xEst(i)  =  std( error(i).xEst );
    stdE.yEst(i)  =  std( error(i).yEst );
    stdE.zEst(i)  =  std( error(i).zEst );
    
    % Pitch
    pitchTmp = output(i).get('pitch');
    angle(i).pitch = pitchTmp(:,1);
    angle(i).pitch = angle(i).pitch( error(i).t >= tCuttOff );
    meanA.pitch(i) = mean( angle(i).pitch );
    stdA.pitch(i)  =  std( angle(i).pitch - meanA.pitch(i) );
    
    pitchTmp = outputEst(i).get('pitch');
    angle(i).pitchEst = pitchTmp(:,1);
    angle(i).pitchEst = angle(i).pitchEst( error(i).t >= tCuttOff );
    
    meanA.pitchEst(i) = mean( angle(i).pitchEst );
    stdA.pitchEst(i)  =  std( angle(i).pitchEst - meanA.pitchEst(i) );
    
    error(i).t = error(i).t( error(i).t >= tCuttOff ) - tCuttOff;
end


% subplot(2,1,1); hold on; grid on; box on
% plot( windSpeedX, meanX, 'o' )
% ylabel('Mean $x$-axis error (m)', 'Interpreter', 'LaTeX')
% xlabel('Wind speed (m/s)', 'Interpreter', 'LaTeX')
% set( gca, 'TickLabelInterpreter', 'latex' )
% 
% subplot(2,1,2);

for ax = ['x', 'y', 'z']
    figure( 'Name', [ 'Simulation error statistics - ' ax ] )
    hold on; grid on; box on
    plot( avgWindSpeed, stdE.(ax), 'o', avgWindSpeed, stdE.([ax 'Est']), 'x', ...
        avgWindSpeed, avgStd.(ax), '+' )
    ylabel(['Stdev $' ax '$-axis error (m)'], 'Interpreter', 'LaTeX')
    xlabel('$U_\mathrm{mean}$ (m/s)', 'Interpreter', 'LaTeX')
    set( gca, 'TickLabelInterpreter', 'latex' )
%     if (ax == 'x')
%         legend( { 'sim', 'exp' }, 'Interpreter', 'LaTeX', 'Orientation', ...
%             'Vertical', 'Location', 'SouthEast' )
%     else
    legend( { 'sim', 'sim+est', 'exp' }, 'Interpreter', 'LaTeX', 'Orientation', ...
        'Vertical', 'Location', 'NorthWest' )
%     end
    
    if ( useClosedContraptionData )
        ylim([0, 0.15])
    else
        ylim([0, 0.04])
    end
    
    xlim( windXLimits )
    
    if ( printResults )
        fileName = [ outFolder '/' 'UavErrorMean-' ax '-simvexp' suffix];
        FormatFigure( outSize , fontSize );
        PrintFigure( fileName );
    end
    
end

% Angles
figure; grid on; box on; hold on;
plot( avgWindSpeed, meanA.pitch, 'o', avgWindSpeed, meanA.pitchEst, 'x', ...
    avgWindSpeed, -rad2deg(avgPitch), '+', 'linewidth', 1)
xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');
ylim([0 15])
xlim( windXLimits )
set( gca, 'TickLabelInterpreter', 'latex' )
ylabel('Mean hover pitch angle (deg)', 'Interpreter', 'latex');
legend( {'sim', 'sim+est', 'exp' }, 'location', 'northwest', 'Interpreter', 'latex' )

if ( printResults )
    fileName = [ outFolder '/' 'PitchMean-' 'simvexp' suffix];
    FormatFigure( outSize , fontSize );
    PrintFigure( fileName );
end

% Angles
figure; grid on; box on; hold on;
plot( avgWindSpeed, stdA.pitch, 'o', avgWindSpeed, stdA.pitchEst, 'x', ...
    avgWindSpeed, rad2deg(avgStd.pitch), '+', 'linewidth', 1)
xlabel('Mean wind speed (m/s)', 'Interpreter', 'latex');

if ( useClosedContraptionData )
else
    ylim( [0 1] )
end

xlim( windXLimits )

set( gca, 'TickLabelInterpreter', 'latex' )
ylabel('Stdev pitch angle (deg)', 'Interpreter', 'latex');
legend( {'sim', 'sim+est', 'exp'}, 'location', 'northwest', 'Interpreter', 'latex' )

if ( printResults )
    fileName = [ outFolder '/' 'PitchStd-' 'simvexp' suffix];
    FormatFigure( outSize , fontSize );
    PrintFigure( fileName );
end
