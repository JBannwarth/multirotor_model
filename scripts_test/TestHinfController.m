%TESTHINFCONTROLLER Create simple H-infinity controller
%   Written by: J.X.J. Bannwarth, 2020/10/26

%% Initialisation
clearvars; clc;
project = simulinkproject; projectRoot = project.RootFolder;
addpath( fullfile( projectRoot, 'libraries', 'hifoo' ) )
addpath( fullfile( projectRoot, 'libraries', 'hanso' ) )

% Load data
load( fullfile( projectRoot, 'data_wind', 'blwt', 'TurbSim_50_01.mat' ) )
load( fullfile( projectRoot, 'work', 'pitch_tf' ) )

% Plant
M = 1; % kg
Cd = 2; % Ns/m

% Actuators
a =  1*2*pi; % rad/s
b = 10*2*pi; % rad/s

r = 0.0001;

%% Weighting functions
% Actuator weighting functions
order = 1;
wCorner = 0.1*2*pi; % rad/s
if order == 1
    % First order lead-lag compensators
    k1 = wCorner;     % rad/s
    k2 = wCorner*10; % rad/s
    k3 = wCorner;     % rad/s
    k4 = wCorner/10; % rad/s
    tfAct1 = db2mag(0)*tf( 10*[1 k1], [1,k2] );
    tfAct2 = db2mag(0)*tf(     [1 k3], [1,k4] );
elseif order == 2
    % Second order butterworth filters
    [num1,den1] = butter( 2, wCorner*10, 'high', 's');
    [num2,den2] = butter( 2, wCorner/10, 's');
    [num3,den3] = butter( 2, wCorner   , 's');
    [num4,den4] = butter( 2, wCorner   , 'high', 's');
    tf1 = tf( num1*100, den1 );
    tf2 = tf( num2*100, den2 );
    tf3 = tf( num3    , den3 );
    tf4 = tf( num4    , den4 );
    tfAct1 = tf1 / tf4;
    tfAct2 = tf2 / tf3;
end

% Plot actuator weighting functions
figure( 1 )
set( gcf, 'name', 'Weighting functions' );
bode(tfAct1, 'k', tfAct2, '--k')
hold on; grid on; box on
ylim([-100, 100])
title('')
legend( {'$W_{\mathrm{act},11}(s)$', '$W_{\mathrm{act},22}(s)$'}, 'location', 'best' )
FormatFigure([14,10], 12)
legend( {'$W_{\mathrm{act},11}(s)$', '$W_{\mathrm{act},22}(s)$'}, 'location', 'best' )
PrintFigure( 'weighting_functions' )
hold off

% Wind up to 5 Hz
k5 = 5*2*pi; % rad/s

% Weighting functions
k6 = 50*2*pi; % rad/s
k7 = 10*2*pi; % rad/s

%% Get model
% Model without weighting functions
if exist( 'pitchTF', 'var' )
    act1 = pitchTF;
else
    error( 'Make sure to run AnalysePitchTimeConstant to generate pitchTF' )
end

% act1 = tf(a, [1 a]);
act2 = tf(b, [1 b]);
C = [act1, act2];
G = [ tf(1, [M Cd 0]); tf(1, [M Cd])];
OL = ss( series( [act1, act2], G) );

% Model with weighting functions
[sysA,sysB,sysC,sysD] = linmod( 'TestHinf2States' );
n = size(sysA,1);
m = size(sysB,2);
q = size(sysC,1);
nmeas = 2;
ncont = 2;
P = ss(sysA,sysB,sysC,sysD);

% Add information about index partitioning to SS object
inputGroup.U1 = 1:(m-ncont);
inputGroup.U2 = (m-ncont+1):m;
outputGroup.Y1 = 1:(q-nmeas);
outputGroup.Y2 = (q-nmeas+1):q;
set( P, 'InputGroup' , inputGroup );
set( P, 'OutputGroup', outputGroup );

%% Create H-infinity controller
% Dynamic output feedback controller
opts = hinfsynOptions('Display', 'on');
[K, CLweights, gamma] = hinfsyn(P, nmeas, ncont, opts);

CL = minreal(feedback(series( K, OL ), eye(nmeas), +1));

% Static output feedback controller
% OPTIONS.augmentHinf = true;
OPTIONS.nrand = 10;
[KStatic, F, VIOL, LOC] = hifoo( P, 0, [], [], [], OPTIONS );

CLStatic = minreal(feedback(series( KStatic, OL ), eye(nmeas), +1));

% Reduced order output feedback controller
% OPTIONS.augmentHinf = true;
OPTIONS.nrand = 10;
[KRed, FRed, VIOLRed, LOCRed] = hifoo( P, 1, [], [], [], OPTIONS );

CLRed = minreal(feedback(series( KRed, OL ), eye(nmeas), +1));

%% Plot results
% Examine bode plot - X/W dynamic
figure( 2 )
XWTF = minreal(tf(feedback(G, series(K, C), +1 )));
XWTFStatic = minreal(tf(feedback(G, series(KStatic, C), +1 )));
XWTFRed = minreal(tf(feedback(G, series(KRed, C), +1 )));
set( gcf, 'name', 'Dynamic output feedback X/W')
bodeplot(XWTFStatic(1), 'k', XWTFRed(1), '--k', XWTF(1), 'ok')
hold on; grid on; box on
title('')
legend( {'Static', 'Dynamic 1 state', 'Dynamic full state'}, 'location', 'northeast' )
FormatFigure([14,10], 12)
hold off
PrintFigure( 'bode_dist_to_output_2states' )

% Examine bode plot - U/W dynamic
distTF = minreal(tf(feedback( series( G, K ), C, +1 )));
figure( 4 )
set( gcf, 'name', 'Dynamic output feedback')
bodeplot(distTF(1), 'k', distTF(2), '--k')
hold on; grid on; box on
title('')
legend( {'$U_1(s)/W_d(s)$', '$U_2(s)/W_d(s)$'}, 'location', 'northeast' )
FormatFigure([14,10], 12)
legend( {'$U_1(s)/W_d(s)$', '$U_2(s)/W_d(s)$'}, 'location', 'northeast' )
hold off
PrintFigure( 'bode_dist_to_input_2states' )

% Examine bode plot - U/W static
distTFStatic = minreal(tf(feedback( series( G, KStatic ), C, +1 )));
figure( 5 )
set( gcf, 'name', 'Static output feedback' )
bodeplot(distTFStatic(1), 'k', distTFStatic(2), '--k')
hold on; grid on; box on
title('')
legend( {'$U_1(s)/W_d(s)$', '$U_2(s)/W_d(s)$'}, 'location', 'northeast' )
FormatFigure([14,10], 12)
legend( {'$U_1(s)/W_d(s)$', '$U_2(s)/W_d(s)$'}, 'location', 'northeast' )
hold off
PrintFigure( 'bode_dist_to_input_static_2states' )

% Plot poles
% figure( 4 )
% set( gcf, 'name', 'Closed-loop poles with dynamic controller' )
% pzmap( CL )
% grid on; box on
% 
% % Plot poles
% figure( 5 )
% set( gcf, 'name', 'Closed-loop poles with static controller' )
% pzmap( CLStatic )
% grid on; box on


%% Check transfer functions
% Equation derived by hand

num = @(k) ( [k(1,2), (k(1,1) + a*k(1,2) + b*k(1,2)), (a*k(1,1) + b*k(1,1) + a*b*k(1,2)), a*b*k(1,1); % Actuator 1
              k(2,2), (k(2,1) + a*k(2,2) + b*k(2,2)), (a*k(2,1) + b*k(2,1) + a*b*k(2,2)), a*b*k(2,1)]); % Actuator 2
den = @(k) ( [ M, ... % s^4
              (Cd + M*a + M*b), ... % s^3
              (Cd*a + Cd*b - a*k(1,2) - b*k(2,2) + M*a*b), ... % s^2
              (Cd*a*b - b*k(2,1) - a*k(1,1) - a*b*k(1,2) - a*b*k(2,2)), ... % s^1
              (-a*b*k(1,1) - a*b*k(2,1)) ] ); % s^0

numStatic = num(KStatic.D);
denStatic = den(KStatic.D);

ShowPoly(distTFStatic(1).Numerator{1}, 'Ex U1/Wd num')
ShowPoly(numStatic(1,:), 'An U1/Wd num')

ShowPoly(distTFStatic(2).Numerator{1}, 'Ex U2/Wd num')
ShowPoly(numStatic(2,:), 'An U2/Wd num')

ShowPoly(distTFStatic(1).Denominator{1}, 'Ex U1/Wd den')
ShowPoly(distTFStatic(2).Denominator{1}, 'Ex U2/Wd den')
ShowPoly(denStatic(1,:), 'An  U/Wd den')

function ShowPoly( pol, name )
    order = length( pol ) - 1;
    fprintf('%s = ', name)
    start = 1;
    if pol(1) < 0
        fprintf( '- ' )
    elseif pol(1) == 0
        start = 2;
        if pol(2) < 0
        fprintf( '- ' )
        else
            fprintf( '+ ' )
        end
    else
        fprintf( '+ ' )
    end
    for i = start:length(pol)
        fprintf('%.3f s^%d', abs(pol(i)), order-i+1)
        if i ~= length(pol)
            if pol(i+1) >= 0
                fprintf(' + ')
            else
                fprintf(' - ')
            end
        end
    end
    fprintf('\n')
end