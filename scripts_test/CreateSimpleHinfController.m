%CREATESIMPLEHINFCONTROLLER
%   Written by:    J.X.J. Bannwarth, 2019/07/29
%	Last modified: J.X.J. Bannwarth, 2020/02/13

%% First initialization
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;

%% Load wind data
ULin = [4 0 0];

%% Set up simulation
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 20; % windInputs{1}.Time(end);

project = simulinkproject; projectRoot = project.RootFolder;

%% Simulation parameters - simple
model = 'MultirotorSimPlantTranslationOnlyForceControl';
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
FindOpPx4

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
io(6) = linio( [model '/filter_xiDDotF'], 1, 'output' );
sys = linearize( model, io );
sys = xperm( sys, [ 7 8 9 1 2 3 4 5 6 ] );
sys = modred( sys, 1:3 );

%% Design controller
A = sys.A; B2 = sys.B(:,4:end); C = sys.C(1:end-3,:); D = sys.D(1:end-3,4:end);
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
Cn = [ zeros(3,3) eye(3,3) ];
A = [ zeros(9,3), [ Cn; A ] ];
B2 = [ zeros(3,5); B2 ];
B1 = [ [ zeros(3,3); B1 ] zeros(9,9) ];
C2 = eye( size(A,1) );
C1 = [ eye( size(A,1) ); zeros( size(B2,2), size(A,2) ) ];
D12 = [ zeros( size(A,1), size(B2,2) ); eye( size(B2,2) ) ];
D11 = zeros( size(C1,1), size(B1,2) );
D22 = zeros( size(C2,1), size(B2,2) );
D21 = [ zeros( size(C2,1), 3), eye(size(C2,1)) ];

% LQR
Q = diag( 1./([ 0.5 0.5 0.5 1 1 1 0.1 0.1 0.1 ].^2) );
R = diag( 1./([ 0.5 0.5 0.5 0.5 0.5 ].^2) );
[K,S,e] = lqr( A, B2, Q, R );

% Hinf
g = 5; gSuccessful = 10; n = 0;
while n < 30
    suitable = true;
    
    P = are(A, -(B1*B1')/g^2 + B2*B2', C1'*C1);
    
    eigHinf = eig(A + ((B1*B1')/(g^2) - (B2*B2'))*P);

    % Check stability
    if max( real(eigHinf) > 0 )
        suitable = false;
    end

    % Check spectral radius
    if (max(abs(eig(P))) > g^2)
        suitable = false;
    end 
    
    if (suitable == false)
        gTemp = g;
        g = g + (gSuccessful(end) - g)/2;
    else
        gTemp = g;
        g = g - (gSuccessful(end) - g)/2;
        gSuccessful(end+1) = gTemp;
    end
    n = n + 1;
end

% Display results
P
eigHinf

K = B2'*P

%% Prepare for manual testing
Wind.StepTime = 2;
Wind.StepInit = ULin;
Wind.StepFinal = Wind.StepInit + [3 0 0];

Ctrl.BYPASS_ROTATION = false;