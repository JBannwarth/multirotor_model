%CREATEDOFHINFCONTROLLER Create dynamic output feedback Hinf controller
%   Written by:    J.X.J. Bannwarth, 2020/11/05

%% Initialization
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;
addpath( fullfile( projectRoot, 'libraries', 'hifoo' ) )
addpath( fullfile( projectRoot, 'libraries', 'hanso' ) )

%% Load wind data
ULin = [4 0 0];

%% Set up simulation
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 30; % windInputs{1}.Time(end);

%% Simulation parameters - simple
model = 'MultirotorSimPlantSimpleForceControl';
modelWeight = 'WeightedHinfModel';

%% Load parameters
load_system( model )
load_system( modelWeight )

% Get wind data and aero parameters
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );
load( fullfile( projectRoot, 'data_wind', 'TurbSimOC', 'TurbSim_40_01' ) );

% Coment out what might cause issues
set_param( [model '/Sinusoidal input']  , 'Commented', 'on' )
set_param( [model '/Varying wind input'], 'Commented', 'on' )
set_param( [model '/Step wind input']   , 'Commented', 'on' )
set_param( [model '/Manual Switch attThrustDes'], 'sw', '0' )
set_param( [model '/Manual Switch horThrustDes'], 'sw', '0' )

% Load UAV parameters
loadBuses = false;
InitializeParametersOctocopterCanted
InitializeModel
Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;

% Set wind speed
set_param( [model '/Input choice'], 'Value', '2' )
set_param( [model '/Fixed wind input'], 'Value', sprintf( '[%.15e;%.15e;%.15e]', ULin ) )

%% Find operating point
toLoad = { 'posOnly', 'fullInputThrustControl' };
FindOpPx4v1_8Cont

%% Finish setting up simulation
set_param( [model '/Manual Switch attThrustDes'], 'sw', '1' )
set_param( [model '/Manual Switch horThrustDes'], 'sw', '1' )
set_param( [model '/Fixed attThrustDes'], 'Value', sprintf( '[%.15e;%.15e;%.15e]', op.Input(2).u ) )
set_param( [model '/Fixed horThrustDes'], 'Value', sprintf( '[%.15e;%.15e]', op.Input(3).u ) )

io(1) = linio( [model '/Input switch'], 1, 'input' );
io(2) = linio( [model '/Manual Switch horThrustDes'], 1, 'input' );
io(3) = linio( [model '/Manual Switch attThrustDes'], 1, 'input' );
io(4) = linio( [model '/integrator_xiDot'], 1, 'output' );
io(5) = linio( [model '/integrator_xi'], 1, 'output' );
sys = linearize( model, io );
sys = xperm( sys, [ 1:6 7 9 8 10 11 12 ] );

%% Design controller
A = sys.A; B2 = sys.B(:,4:end); C = sys.C(1:6,:); D = sys.D(1:6,4:end);
B1 = sys.B(:,1:3);
% Zero extremely small elements to get accurate answers when doing tests
% such as ctrb and obsv
A(abs(A)<1e-10) = 0; B2(abs(B2)<1e-10) = 0; C(abs(C)<1e-10) = 0; D(abs(D)<1e-10) = 0;

% Add integrator for zero steady state error
A = [ zeros(3, 6)        , eye(3, size(A,2)-3) ;
      zeros(size(A,2), 3), A                  ];
B1 = [ zeros(3, size(B1,2)) ;
       B1                   ];
B2 = [ zeros(3, size(B2,2)) ;
       B2                   ];
C = [eye(3, size(C,2)+3);
     zeros(size(C,1), 3), C];
D = [zeros(3, size(D,2)+3);
     zeros(size(D,1), 3), D];

% Weighting functions
% Actuators
% First order lead-lag compensators
wCorner = 0.1*2*pi; % rad/s
k1 = wCorner;    % rad/s
k2 = wCorner*10; % rad/s
k3 = wCorner;    % rad/s
k4 = wCorner/10; % rad/s
WActHor = db2mag(0)*tf( 10*[1 k1], [1,k2] );
WActAtt = db2mag(0)*tf(    [1 k3], [1,k4] );
WAct = [ WActHor 0       0       0       0;
         0       WActHor 0       0       0;
         0       0       WActAtt 0       0;
         0       0       0       WActAtt 0;
         0       0       0       0       WActAtt];

WReg = diag( [1 1 1 1 1 1 1 1 1] );
WSensor = eye( size(C, 1) );
WDist = eye( size(B1, 2) );

% Model with weighting functions
[sysA, sysB, sysC, sysD] = linmod( modelWeight );
n = size(sysA,1);
m = size(sysB,2);
q = size(sysC,1);
nMeas = size(C, 1);
nCont = size(B2, 2);
P = ss(sysA, sysB, sysC, sysD);

% Add information about index partitioning to SS object
inputGroup.U1 = 1:(m-nCont);
inputGroup.U2 = (m-nCont+1):m;
outputGroup.Y1 = 1:(q-nMeas);
outputGroup.Y2 = (q-nMeas+1):q;
set( P, 'InputGroup' , inputGroup );
set( P, 'OutputGroup', outputGroup );

opts = hinfsynOptions('Display', 'on');
[K, CLweights, gamma] = hinfsyn(P, nMeas, nCont, opts);

% OPTIONS.nrand = 10;
% [K, F, VIOL, LOC] = hifoo( P, 1, [], [], [], OPTIONS );

%% Prepare for manual testing
Wind.StepTime = 2;
Wind.StepInit = ULin;
Wind.StepFinal = Wind.StepInit + [3 0 0];

Ctrl.BYPASS_ROTATION = false;