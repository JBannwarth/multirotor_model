%CREATESOFHINFCONTROLLER Create static-output feedback Hinf controller
%   Written by:    J.X.J. Bannwarth, 2020/08/05

%% Initialization
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;
addpath( fullfile( 'libraries', 'hifoo' ) )
addpath( fullfile( 'libraries', 'hanso' ) )

%% Load wind data
ULin = [4 0 0];

%% Set up simulation
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 20; % windInputs{1}.Time(end);

%% Simulation parameters - simple
model = 'MultirotorSimPlantSimpleForceControl';
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 30;

%% Load parameters
load_system( model )

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
% io(6) = linio( [model '/filter_xiDDotF'], 1, 'output' );
sys = linearize( model, io );
% sys = xperm( sys, [ 7 8 9 1:6 10 12 11 13 14 15 ] );
sys = xperm( sys, [ 1:6 7 9 8 10 11 12 ] );
% sys = modred( sys, 1:3 );

%% Design controller
A = sys.A; B2 = sys.B(:,4:end); C = sys.C(1:6,:); D = sys.D(1:6,4:end);
B1 = sys.B(:,1:3);
% Zero extremely small elements to get accurate answers when doing tests
% such as ctrb and obsv
A(abs(A)<1e-10) = 0; B2(abs(B2)<1e-10) = 0; C(abs(C)<1e-10) = 0; D(abs(D)<1e-10) = 0;

% Filtered acc states
% Cn = [ zeros(3,3) zeros(3,3) eye(3) ];
% Abar = [ zeros(12,3), [ Cn; A ] ];
% Bbar = [ zeros(3,5); B ];
% Q = diag( 1./([ 0.5 0.5 0.5 10 10 10 1 1 1 0.1 0.1 0.1 ].^2) );
% R = diag( 1./([ 0.5 0.5 0.5 0.5 0.5 ].^2) );
% [K,S,e] = lqr( Abar, Bbar, Q, R );

% State matrices
n = size(A,1);
m1 = size(B1,2);
m2 = size(B2,2);

Cn = [ zeros(3,3) eye(3,3) zeros(3,n-6) ];
A = [ zeros(n+3,3), [ Cn; A ] ];
B2 = [ zeros(3,m2); B2 ];
nNew = size(A,1);

B1 = [ [ zeros(3,m1); B1 ] zeros(nNew,9) ];
C2 = eye( 9, nNew );
C1 = [ eye(3+n-6,nNew); zeros(nNew-n-3+6, nNew); zeros( m2, nNew ) ];
D12 = [ zeros( nNew, m2 ); eye( m2 ) ];
D11 = zeros( size(C1,1), size(B1,2) );
D22 = zeros( size(C2,1), size(B2,2) );
D21 = [ zeros( size(C2,1), 3), eye(size(C2,1)) ];

P = struct( 'A', A, 'B1', B1, 'B2', B2, 'C1', C1, 'C2', C2, 'D11', D11, ...
    'D12', D12, 'D21', D21, 'D22', D22 );

[Ksys, f, viol, loc] = hifoo(P)
K = Ksys.d;
Ctrl.K

%% Prepare for manual testing
Wind.StepTime = 2;
Wind.StepInit = ULin;
Wind.StepFinal = Wind.StepInit + [3 0 0];

Ctrl.BYPASS_ROTATION = false;