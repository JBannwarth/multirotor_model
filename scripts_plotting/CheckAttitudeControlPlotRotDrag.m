%CHECKATTITUDECONTROLPLOTROTDRAG Plot step responses
%   Written by: J.X.J. Bannwarth, 2017/09/13

% close all;

outFolder = '../multirotor_model_verification_report/fig';
fontSize  = 9;
outSize   = [8.85684 8.85684];
printResults = true;

% Assign values
% q = out.get('q');
eta = out.get('eta');
etaRD = outRD.get('eta');

% Get data
tSim = eta.time - tDesOffset;
tSimRD = etaRD.time - tDesOffset;
% qSim = q.signals.values;
% qSimN = length( qSim );
% qSim = [ ones(qSimN,1), ones(qSimN,1), -ones(qSimN,1), -ones(qSimN,1)] .* qSim;
%[ eulSim.Roll, eulSim.Pitch, eulSim.Yaw ] = QuatToEuler( qSim );
etaSim = [ ones(length(eta),1), -ones(length(eta),1), -ones(length(eta),1)] .* eta.signals.values;
eulSim.Roll = etaSim(:,1); eulSim.Pitch = etaSim(:,2); eulSim.Yaw = etaSim(:,3);
eulSim.Yaw = unwrap( eulSim.Yaw );
etaSimRD = [ ones(length(etaRD),1), -ones(length(etaRD),1), -ones(length(etaRD),1)] .* etaRD.signals.values;
eulSimRD.Roll = etaSimRD(:,1); eulSimRD.Pitch = etaSimRD(:,2); eulSimRD.Yaw = etaSimRD(:,3);
eulSimRD.Yaw = unwrap( eulSimRD.Yaw );

% Unwrap (need to write function to do this without DSP toolbox)
eulExp.Yaw = unwrap( eulExp.Yaw );
eulDes.Yaw = unwrap( eulDes.Yaw );

% Get axis to look at
fileNameCurrent = inputFiles{i};
% dashLoc = strfind( fileNameCurrent, '-' );
% underscoreLoc = strfind( fileNameCurrent, '_' );
% if isempty( dashLoc )
%     dashLoc = strfind( fileNameCurrent, '+' );
% end
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
figure('name', inputFiles{i} )
hold on; grid on; box on;
stairs( tDes, rad2deg( eulDes.(char(ax)) ) )
stairs( tExp, rad2deg( eulExp.(char(ax)) ) )
plot( tSim, rad2deg( eulSim.(char(ax)) ) )
plot( tSimRD, rad2deg( eulSimRD.(char(ax)) ) )
xlim( [0 inf] )
%ylim( [-inf inf] )
xlabel( 'Time (s)', 'interpreter', 'latex' )
ylabel( [ ax ' (deg) ' ], 'interpreter', 'latex' )
legend( { 'Des', 'Exp', 'Sim', 'SimRD' }, 'location', 'southeast', 'interpreter', 'latex')
%     ind = ind + 1;
% end

if ( printResults )
    fileName = [ outFolder '/' fileNameCurrent(1:end-4)];
    SetFigProp( outSize , fontSize );
    MatlabToLatexEps( fileName, [], false );
end
