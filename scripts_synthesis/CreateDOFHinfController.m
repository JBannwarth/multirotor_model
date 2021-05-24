%CREATEDOFHINFCONTROLLER Create dynamic output feedback Hinf controller
%
%   Reads Octocopter_LinMod_Att from the work folder. This file is created
%   by running LinearizeUavAtt.
%   Also loads simulation parameters for manual testing on linearised
%   model.
%
%   See also LINEARIZEUAVATT, WEIGHTEDHINFMODEL, HINFMODELLINEAR, 
%   MULTIROTORSIMPX4V1_8FPHTFULLGAINMATRIX.
%
%   Written: 2020/11/05, J.X.J. Bannwarth

%% Initialization
clearvars -except ctrlOrder
if ~exist( 'ctrlOrder', 'var' )
    ctrlOrder = -1;
end

project = simulinkproject; projectRoot = project.RootFolder;
addpath( fullfile( projectRoot, 'libraries', 'hifoo' ) )
addpath( fullfile( projectRoot, 'libraries', 'hanso' ) )

weightedModel = 'WeightedHinfModel';

%% Load linearised model
load( fullfile( projectRoot, 'work', 'Octocopter_LinMod_Att' ), ...
    'linsys', 'op', 'ULin' )

%% Get state and input indexes
% Use variables instead of hardcoding states
% Select output states
xiIdx      = find( startsWith( linsys.StateName, 'xi_' ) );
xiDotIdx   = find( startsWith( linsys.StateName, 'xiDot_' ) );
outputIdx = [xiIdx; xiDotIdx];

% Get input indexes - note that there is no function for rearranging the
% inputs in SS object
virThrustIdx = find( contains( linsys.InputName, 'T_a' ) );
horThrustIdx = find( contains( linsys.InputName, 'T_h' ) );
yawIdx       = find( contains( linsys.InputName, 'yaw' ) ); % Not used
windIdx      = find( startsWith( linsys.InputName, 'U' ) );

% Separate into control inputs and disturbance inputs
controlIdx = [virThrustIdx; horThrustIdx];
distIdx    = windIdx;

%% Augment state space model with integrators
% Extract state space matrices
A  = linsys.A;
B1 = linsys.B(:, distIdx);
B2 = linsys.B(:, controlIdx);
C  = linsys.C(outputIdx, :);
D21 = linsys.D(outputIdx, distIdx);
D22  = linsys.D(outputIdx, controlIdx);

% Zero extremely small elements to get accurate answers when doing tests
% such as ctrb and obsv
tol = 1e-10;
A(abs(A) < tol)   = 0;
B1(abs(B1) < tol) = 0;
B2(abs(B2) < tol) = 0;
C(abs(C) < tol)   = 0;
D22(abs(D22) < tol)   = 0;

% Sizes
n = size( linsys.A, 1 );
m1 = length( distIdx );
m2 = length( controlIdx );
p = length( outputIdx );

% Add integrator for zero steady state error
A = [ zeros(n+3, 3), [ eye(3, n); A ] ];
B1 = [ zeros(3, m1)  ;
       B1           ];
B2 = [ zeros(3, m2)  ;
       B2           ];
C = [ eye(3, n+3)     ;
      zeros(p, 3), C ];
D21 = [ zeros(3, m1)  ;
        D21          ];
D22 = [ zeros(3, m2)  ;
        D22          ];
D = [D21 D22];

% Create new state space object
G = ss( A, [B1 B2], C, [D21 D22] );
G.InputName = linsys.InputName( [distIdx; controlIdx] ) ;
G.OutputName = [ 'xiInt_x'; 'xiInt_y'; 'xiInt_z';
                 linsys.OutputName( outputIdx ) ];
G.StateName = [ 'xiInt_x'; 'xiInt_y'; 'xiInt_z'; linsys.StateName ];

% Input groups
G.InputGroup.dist = distIdx';
G.InputGroup.controls = controlIdx';

%% Define weighting functions
% Weighting functions
% Actuators
% First order lead-lag compensators
wCorner = 0.5*2*pi; % rad/s
k1 = wCorner;    % rad/s
k2 = wCorner*10; % rad/s
k3 = wCorner;    % rad/s
k4 = wCorner/10; % rad/s
kZ = 3*2*pi;     % rad/s
WActAtt  = db2mag(14) * tf( 10*[1 k1], [1,k2] );
WActAttZ  = db2mag(14) * tf( 10*[1 k1], [1,k2] );
% WActAttZ = db2mag(13) * tf( 10*[1 kZ/10], [1, kZ] );
WActHor  = db2mag(19) * tf(    [1 k3], [1,k4] );
WAct = [ WActAtt 0       0        0       0       ;
         0       WActAtt 0        0       0       ;
         0       0       WActAttZ 0       0       ;
         0       0       0        WActHor 0       ;
         0       0       0        0       WActHor ];

WReg = 0.5*diag( [1 1 1 1 1 1 1 1 1] );
WSensor = 0.05*eye( size(C, 1) );
wDistU = db2mag(15)*db2mag(0) * tf(  24.1184, [1 24.1184] );
wDistV = wDistU;
wDistW = wDistU;
% wDistV = db2mag(10)*db2mag(-17.7392) * tf( 128.8783, [1 128.8783] );
% wDistW = db2mag(10)*db2mag(-20.1404) * tf( 163.7495, [1 163.7495] );
WDist = [ wDistU 0      0;
          0      wDistV 0;
          0      0      wDistW ];

% Get model with weighting functions
% The system is already linear, so the linearisation does not induce any
% additional simplification
P = linearize( weightedModel );
nHat = size(P.A,1);
mHat = size(P.B,2);
pHat = size(P.C,1);
nMeas = size(C, 1);
nCont = size(B2, 2);

% Add information about index partitioning to SS object
inputGroup.U1 = 1:(mHat-nCont);       % Disturbances
inputGroup.U2 = (mHat-nCont+1):mHat;  % Controls
outputGroup.Y1 = 1:(pHat-nMeas);      % Regulated output
outputGroup.Y2 = (pHat-nMeas+1):pHat; % Output
set( P, 'InputGroup' , inputGroup );
set( P, 'OutputGroup', outputGroup );

%% Export weighting functions
% Print matrices
disp( '[WAct]' )
PrintTFMatrix( WAct );
disp( '[WDist]' )
PrintTFMatrix( WDist );

% Get magnitude response
[magAtt, phaseAtt, w] = bode( WActAtt, {wCorner/100, wCorner*100} );
[magHor, phaseHor] = bode( WActHor, w );
magAtt = squeeze( 20*log10(magAtt) );
phaseAtt = squeeze( phaseAtt );
magHor = squeeze( 20*log10(magHor) );
phaseHor = squeeze( phaseHor );

% Plot to verify
semilogx( w, magAtt, w, magHor )
axis tight

% Export
fileOut = fullfile( projectRoot, 'work', 'output', 'bode_weight_act.csv' );
fid = fopen( fileOut, 'w' );
fprintf( fid, 'w magA phaseA magH phaseH' );
fclose( fid );
writematrix( [w magAtt phaseAtt magHor phaseHor], fileOut, 'WriteMode', 'append', 'Delimiter', ' ' )

%% Create controller
opts = hinfsynOptions('Display', 'on');

disp( 'Creating controller...' )
tic;
if ctrlOrder == -1
    % Full order controller
    [ K, CLweights, gamma ] = hinfsyn(P, nMeas, nCont, opts);
else
    % Use HIFOO to create reduced order controller
    OPTIONS.nrand = 3;
    [ K, F, VIOL, LOC ] = hifoo( P, ctrlOrder, [], [], [], OPTIONS );
end

% Display time taken to create controller
tElapsed = seconds( toc );
tElapsed.Format = 'hh:mm:ss.SSS';
fprintf( 'Controller created. Time taken: %s s\n', tElapsed )

% Add details to controller
K.InputName = G.OutputName;
K.OutputName = G(:,'controls').InputName;

%% Check controller
CLTF = feedback( G(:,'controls'), K, 'name', +1 );
if ~isstable( CLTF )
    [p, z] = pzmap( CLTF );
    pRHP = p( p > 0 );
    pStr = sprintf( '\tp = %.3f\n', pRHP );
    warning( 'Close loop system not stable. RHP poles at:\n%s', pStr )
end

%% Save data
% Operating point
for ii = 1:length( op.Inputs )
    inputNames{ii} = op.Inputs(ii).Block;
end
virThrustOp = op.Input( find(contains( inputNames, 'virtualThrust' )) ).u;
horThrustOp = op.Input( find(contains( inputNames, 'horThrust' )) ).u;
thrustOp = [ virThrustOp; horThrustOp ];

save( fullfile( projectRoot, 'work', 'HinfGain.mat' ), ...
    'K', 'ULin', 'thrustOp' )

%% Helper function
PrintMatrix(K.A)
PrintMatrix(K.B)
PrintMatrix(K.C)
PrintMatrix(K.D)
function PrintTFMatrix( W )
    strOut = '';
    for ii = 1:size( W, 1 )
        for jj = 1:size( W, 2 )
            if length(W.Numerator{ii,jj}) == 1
                strOut = [strOut sprintf( '%.2f & ', W.Numerator{ii,jj} ) ];
            else
                strOut = [strOut sprintf( '\\frac{%.3fs + %.3f}{%.3fs + %.3f} & ', ...
                    W.Numerator{ii,jj}(1), W.Numerator{ii,jj}(2), ...
                    W.Denominator{ii,jj}(1), W.Denominator{ii,jj}(2) ) ];
            end
        end
        strOut = [strOut sprintf('\\\\\n') ];
    end
    strOut = replace( strOut, '1.000', '1' );
    strOut = replace( strOut, '1.00', '1' );
    strOut = replace( strOut, '0.00', '0' );
    strOut = replace( strOut, '{00s + ', '{' );
    strOut = replace( strOut, '1s', 's' );
    strOut = replace( strOut, '&\\', '\\' );
    disp( strOut )
end

function PrintMatrix( M )
    strOut = sprintf('\\begin{bmatrix}\n');
    for ii = 1:size( M, 1 )
        strOut = [strOut sprintf('    ')];
        for jj =1:size( M, 2 )
            strOut = [strOut sprintf( '%s & ', num2str(round( M(ii,jj), 3, 'significant' ) )) ];
        end
        strOut = [strOut sprintf('\\\\\n')];
    end
    strOut = [strOut sprintf('\\end{bmatrix}\n')];
    strOut = replace( strOut, ' & \\', '\\' );
    disp(strOut)
end