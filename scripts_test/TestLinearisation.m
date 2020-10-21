%TESTLINEARISATION Test linearisation of MATLAB System object in Simulink
%   Written:       J.X.J. Bannwarth, 2019/02/11
%   Last modified: J.X.J. Bannwarth, 2019/02/12

clc

%% Paramaters
m = 0.25;
DProp  = 10 * 0.0254;
AProp  = (pi*DProp^2)/4;
rhoAir = 1.225;
tauM   = 7.006512e-2;
kM     = 5.128263e-3 * 0.5*rhoAir*DProp^2*AProp;

mdl = 'TestLinearisationSystem';
load_system( mdl )

%% Get operating point
opspec = operspec( mdl );
opspec = addoutputspec(opspec, [mdl '/int(vel)'], 1);
opspec.States(2).Known  = 1;
opspec.States(2).x      = 0;
opspec.States(3).x      = 500;
opspec.States(4).Known  = 1;
opspec.States(4).x      = 0;
opspec.Outputs(1).Known = 1;
opspec.Outputs(1).y     = 0;
op  = findop( mdl, opspec );

set_param( mdl, 'LoadInitialState', 'on' );
set_param( mdl, 'InitialState', 'getstatestruct(op)');

% To update opspec after changes
% opspec = update(opspec);

%% Linearize
io(1) = linio( [mdl, '/Pos control'], 1, 'input' );
io(2) = linio( [mdl, '/int(vel)'], 1, 'openoutput' );

linsys1 = linearize( mdl, io, op );

linsys1C = d2c( linsys1 );