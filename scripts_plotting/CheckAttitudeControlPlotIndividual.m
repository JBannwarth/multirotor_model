%CHECKATTITUDECONTROLPLOTINDIVIDUAL Plot step responses
%   Written by: J.X.J. Bannwarth, 2017/08/21

% close all;

outFolder = '../multirotor_model_verification_report/fig';
fontSize  = 9;
outSize   = [8.85684 8.85684];
printResults = true;

logsout = output.get('logsout');
q = output.get('logsout').get('q').Values;
eta = output.get('logsout').get('eta').Values;
pwm = output.get('logsout').get('pwm').Values;
pitch = output.get('logsout').get('pitch').Values;

if (exist('flogOriginal'))
    pwmExp = [ flogOriginal.actuator_outputs.output_0_, ...
            flogOriginal.actuator_outputs.output_1_,    ...
            flogOriginal.actuator_outputs.output_2_,    ...
            flogOriginal.actuator_outputs.output_3_     ...
        ];
    tExpPwm = flogOriginal.actuator_outputs.time;
end

% Get data
tSim = q.Time - tDesOffset;
etaSim = [ ones(qSimN,1), -ones(qSimN,1), -ones(qSimN,1)] .* eta.Data;
eulSim.Roll = etaSim(:,1); eulSim.Pitch = etaSim(:,2); eulSim.Yaw = etaSim(:,3);
eulSim.Yaw = unwrap( eulSim.Yaw );

errorSim.Pitch = pitch.Data(:,2) - pitch.Data(:,1);
avgErrSim = abs(mean(errorSim.Pitch));
maxErrSim = max(abs(errorSim.Pitch));
rmsErrSim = rms(errorSim.Pitch);

tSimPwm = pwm.Time - tDesOffset;
pwmSim = pwm.Data;
% Reorder PWM to match experimental order
% [ px4Pwm(3); px4Pwm(2); px4Pwm(4); px4Pwm(1) ];
pwmSim = [ pwmSim(:,4) pwmSim(:,2) pwmSim(:,1) pwmSim(:,3) ];

% Unwrap (need to write function to do this without DSP toolbox)
eulExp.Yaw = unwrap( eulExp.Yaw );
eulDes.Yaw = unwrap( eulDes.Yaw );

if ( tDes(end) < tSim(end) )
    eulDes.Roll(end+1) =  eulDes.Roll(end);
    eulDes.Roll(end+1) =  eulDes.Pitch(end);
    eulDes.Pitch(end+1) =  eulDes.Yaw(end);
    tDes(end+1) = tSim(end);
end

eulDes.Roll = eulDes.Roll( tDes >= 0 );
eulDes.Pitch = eulDes.Pitch( tDes >= 0 );
eulDes.Yaw = eulDes.Yaw( tDes >= 0 );
tDes = tDes( tDes >= 0 );

% if ( (tDes(1) >= tExp(1)) && (tDes(end) <= tExp(end)) )
%     tResample = tDes;
% elseif ( (tDes(1) <= tExp(1)) && (tDes(end) >= tExp(end)) )
%     tResample = tExp;
% else
    tResample = max([min(tDes), min(tExp) ]):median(diff(tExp)):min([max(tDes), max(tExp) ]);
% end

tResample = tResample( tResample >= 0 );

errorExp.Pitch = resample(timeseries( eulDes.Pitch, tDes), tResample) - ...
    resample(timeseries( eulExp.Pitch, tExp ), tResample) ;
errorExp.Roll = resample(timeseries( eulDes.Roll, tDes), tResample) - ...
    resample(timeseries( eulExp.Roll, tExp ), tResample);
errorExp.Yaw = resample(timeseries( eulDes.Yaw, tDes), tResample) - ...
    resample(timeseries( eulExp.Yaw, tExp ), tResample);
avgErrExp = abs(rad2deg(mean(errorExp.Pitch.Data)));
maxErrExp = rad2deg(max(abs(errorExp.Pitch.Data)));
rmsErrExp = rad2deg(rms(errorExp.Pitch.Data));

tableStr = sprintf('%f & %f & %f & %f & %f & %f \\\\', avgErrSim, avgErrExp, maxErrSim, maxErrExp, rmsErrSim, rmsErrExp);

% Get axis to look at
fileNameCurrent = inputFiles{n};

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
%for i = 1:2
subplot(2,1,1)
hold on; grid on; box on;
stairs( tDes, rad2deg( eulDes.(char(ax)) ) )
stairs( tExp, rad2deg( eulExp.(char(ax)) ) )
stairs( tSim, rad2deg( eulSim.(char(ax)) ) )
xlim( [0 inf] )
%ylim( [-inf inf] )
xlabel( 'Time (s)' )
ylabel( [ ax ' (deg) ' ] )
legend( { 'Des', 'Exp', 'Sim' }, 'location', legendLoc)
%     ind = ind + 1;
% end
%end

subplot(2,1,2)
hold on; grid on; box on;
col = get(gca,'colororder');
for i = 1:4
    stairs( tExpPwm, pwmExp(:,i), 'color', col(i,:) )
    stairs( tSimPwm, 1000+1000.*pwmSim(:,i), '--', 'color', col(i,:) )
end
xlim( [0 inf] )
xlabel( 'Time (s)' )
ylabel( 'PWM magnitude (-)' )
legend( {'Exp1', 'Sim1', 'Exp2', 'Sim2', 'Exp3', 'Sim3', 'Exp4', 'Sim4'} )

SetFigProp( outSize , fontSize );
if ( printResults )
    fileName = [ outFolder '/' fileNameCurrent(1:end-4)];
    MatlabToLatexEps( fileName );
end