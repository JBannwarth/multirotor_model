%CHECKATTITUDECONTROLPLOT Plot step responses
%   Written by: J.X.J. Bannwarth, 2017/08/21

% close all;

outFolder = '../multirotor_model_verification_report/fig';
fontSize  = 9;
outSize   = [8.85684 8.85684];
printResults = true;

% Assign values
q = output.get('logsout').get('q').Values;
eta = output.get('logsout').get('eta').Values;

% Get data
tSim = q.Time - tDesOffset;
qSim = q.Data;
qSimN = length( qSim );
qSim = [ ones(qSimN,1), ones(qSimN,1), -ones(qSimN,1), -ones(qSimN,1)] .* qSim;
%[ eulSim.Roll, eulSim.Pitch, eulSim.Yaw ] = QuatToEuler( qSim );
etaSim = [ ones(qSimN,1), -ones(qSimN,1), -ones(qSimN,1)] .* eta.Data;
eulSim.Roll = etaSim(:,1); eulSim.Pitch = etaSim(:,2); eulSim.Yaw = etaSim(:,3);
eulSim.Yaw = unwrap( eulSim.Yaw );

% Unwrap (need to write function to do this without DSP toolbox)
eulExp.Yaw = unwrap( eulExp.Yaw );
eulDes.Yaw = unwrap( eulDes.Yaw );

if ( tDes(end) < tSim(end) )
    eulDes.Roll(end+1) =  eulDes.Roll(end);
    eulDes.Roll(end+1) =  eulDes.Pitch(end);
    eulDes.Pitch(end+1) =  eulDes.Yaw(end);
    tDes(end+1) = tSim(end);
end

% Get axis to look at
fileNameCurrent = inputFiles{n};
% dashLoc = strfind( fileNameCurrent, '-' );
% underscoreLoc = strfind( fileNameCurrent, '_' );
% if isempty( dashLoc )
%     dashLoc = strfind( fileNameCurrent, '+' );
% end

if ~isempty( strfind( fileNameCurrent, '+' ) )
    legendLoc = 'southeast';
else
    legendLoc = 'northeast';
end
if ~isempty( strfind( fileNameCurrent, 'roll' ) )
    ax = 'Roll';
elseif ~isempty( strfind( fileNameCurrent, 'pitch' ) )
    ax = 'Pitch';
elseif ~isempty( strfind( fileNameCurrent, 'yaw' ) )
    ax = 'Yaw';
else
    error('No axis')
end
% ax = fileNameCurrent( underscoreLoc+1:dashLoc-1 );
% ax(1) = upper(ax(1));
% Plot results
% ind = 1;
% for ax = { 'Roll', 'Pitch', 'Yaw' }
%     subplot( 3, 1, ind )
figure('name', inputFiles{n} )
hold on; grid on; box on;
stairs( tDes, rad2deg( eulDes.(char(ax)) ) )
stairs( tExp, rad2deg( eulExp.(char(ax)) ) )
stairs( tSim, rad2deg( eulSim.(char(ax)) ) )
xlim( [0 inf] )
%ylim( [-inf inf] )
xlabel( 'Time (s)', 'interpreter', 'latex' )
ylabel( [ ax ' (deg) ' ], 'interpreter', 'latex' )
legend( { 'Des', 'Exp', 'Sim' }, 'location', legendLoc, 'interpreter', 'latex')
%     ind = ind + 1;
% end

if ( printResults )
    fileName = [ outFolder '/' fileNameCurrent(1:end-4)];
    SetFigProp( outSize , fontSize );
    MatlabToLatexEps( fileName, [], false );
end
