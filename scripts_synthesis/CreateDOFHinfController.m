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
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;
addpath( fullfile( projectRoot, 'libraries', 'hifoo' ) )
addpath( fullfile( projectRoot, 'libraries', 'hanso' ) )

weightedModel = 'WeightedHinfModel';

%% Load and prepare linearised model
load( fullfile( projectRoot, 'work', 'Octocopter_LinMod_Att' ), ...
    'linsys', 'op', 'ULin' )

% Rearrange states in order: [xi; xiDot; eta; nuBody; omega; pidStates]
% Find indexes of main states
xiIdx      = find( strcmp( linsys.StateName, 'xi' ) );
xiDotIdx   = find( strcmp( linsys.StateName, 'xiDot' ) );
etaIdx     = find( strcmp( linsys.StateName, 'eta' ) );
nuBodyIdx  = find( strcmp( linsys.StateName, 'nuBody' ) );
omegaIdx   = find( strcmp( linsys.StateName, 'omega' ) );

% Find remaining indexes (PID integrators, etc.)
remIdx = ones( size( linsys.StateName ) );
remIdx( [xiIdx; xiDotIdx; etaIdx; nuBodyIdx; omegaIdx] ) = 0;
remIdx = find( remIdx );

linsys = xperm( linsys, [ xiIdx; xiDotIdx; etaIdx; nuBodyIdx; omegaIdx; remIdx ] );

%% Get state and input indexes
% Use variables instead of hardcoding states
% Select output states - indexes will have changed because of xperm call
xiIdx      = find( strcmp( linsys.StateName, 'xi' ) );
xiDotIdx   = find( strcmp( linsys.StateName, 'xiDot' ) );
outputIdx = [xiIdx; xiDotIdx];

% Get input indexes - note that there is no function for rearranging the
% inputs in SS object
virThrustIdx = find( contains( linsys.InputName, 'virtualThrustDes' ) );
horThrustIdx = find( contains( linsys.InputName, 'horThrustDes' ) );
yawIdx       = find( contains( linsys.InputName, 'yaw' ) );
windIdx      = find( contains( linsys.InputName, '/U' ) );

% Separate into control inputs and disturbance inputs
controlIdx = [virThrustIdx; horThrustIdx];
distIdx    = windIdx;

%% Design controller
% Extract state space matrices
A  = linsys.A;
B1 = linsys.B(:, distIdx);
B2 = linsys.B(:, controlIdx);
C  = linsys.C(outputIdx, :);
D  = linsys.D(outputIdx, controlIdx);

% Zero extremely small elements to get accurate answers when doing tests
% such as ctrb and obsv
tol = 1e-10;
A(abs(A) < tol)   = 0;
B1(abs(B1) < tol) = 0;
B2(abs(B2) < tol) = 0;
C(abs(C) < tol)   = 0;
D(abs(D) < tol)   = 0;

% Sizes
n = size( linsys.A, 1 );
m1 = length( distIdx );
m2 = length( controlIdx );
p = length( outputIdx );

% Add integrator for zero steady state error
A = [ zeros(n+3, 3), [ eye(3, n); A ] ];
B1 = [ zeros(3, m1) ;
       B1           ];
B2 = [ zeros(3, m2) ;
       B2           ];
C = [ eye(3, n+3)    ;
      zeros(p, 3), C ];
D = [ zeros(3, m2) ;
      D            ];

% Weighting functions
% Actuators
% First order lead-lag compensators
wCorner = 0.1*2*pi; % rad/s
k1 = wCorner;    % rad/s
k2 = wCorner*10; % rad/s
k3 = wCorner;    % rad/s
k4 = wCorner/10; % rad/s
WActAtt = db2mag(-20) * tf( 10*[1 k1], [1,k2] );
WActAttZ = db2mag(-20) * tf( 10*[1 k2], [1,10*k2] );
WActHor = db2mag(-20) * tf(    [1 k3], [1,k4] );
WAct = [ WActAtt 0       0        0       0       ;
         0       WActAtt 0        0       0       ;
         0       0       WActAttZ 0       0       ;
         0       0       0        WActHor 0       ;
         0       0       0        0       WActHor ];

WReg = diag( [1 1 2 1 1 2 1 1 2] );
WSensor = 0.1*eye( size(C, 1) );
wDistU = db2mag(10)*db2mag(0)        * tf(  24.1184, [1 24.1184] );
wDistV = wDistU;
wDistW = wDistU;
% wDistV = db2mag(10)*db2mag(-17.7392) * tf( 128.8783, [1 128.8783] );
% wDistW = db2mag(10)*db2mag(-20.1404) * tf( 163.7495, [1 163.7495] );
WDist = [ wDistU 0      0;
          0      wDistV 0;
          0      0      wDistW ];

% Model with weighting functions
[AHat, BHat, CHat, DHat] = linmod( weightedModel );
nHat = size(AHat,1);
mHat = size(BHat,2);
pHat = size(CHat,1);
nMeas = size(C, 1);
nCont = size(B2, 2);
P = ss(AHat, BHat, CHat, DHat);

% Add information about index partitioning to SS object
inputGroup.U1 = 1:(mHat-nCont);
inputGroup.U2 = (mHat-nCont+1):mHat;
outputGroup.Y1 = 1:(pHat-nMeas);
outputGroup.Y2 = (pHat-nMeas+1):pHat;
set( P, 'InputGroup' , inputGroup );
set( P, 'OutputGroup', outputGroup );

opts = hinfsynOptions('Display', 'on');
[K, CLweights, gamma] = hinfsyn(P, nMeas, nCont, opts);

% OPTIONS.nrand = 10;
% [K, F, VIOL, LOC] = hifoo( P, 0, [], [], [], OPTIONS );

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