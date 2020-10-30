%ANALYSEPITCHTIMECONSTANT Analyse time constant of the pitch control
%   Written by:    J.X.J. Bannwarth, 2020/08/04

%% First initialization
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;

%% Simulation parameters
model = 'MultirotorPx4PitchOnly';
load_system( model )

load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );

% Load UAV parameters
loadBuses = false;
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 2;
InitializeParametersOctocopterCanted
InitializeModel
Ctrl.THROTTLE_HOVER = Uav.THROTTLE_HOVER;
pwmIn = Ctrl.THROTTLE_HOVER;

%% Get linear model and reduce it
op = findop( model );
io(1) = linio( [model, '/Tdes_x'], 1 );
io(2) = linio( [model, '/Multiply'], 1, 'openoutput' );

linsys = linearize( model, io, op );

% Validate using nonlinear frequency response
in = frest.Sinestream(linsys);
in.Amplitude = 0.1;
in = frest.Sinestream('Amplitude', in.Amplitude, ...
    'Frequency', [0.01; in.Frequency], 'NumPeriods', [4; in.NumPeriods], ...
    'SettlingPeriods', [1; in.SettlingPeriods]);

% Estimate frequency response
frd = frestimate( model, op, io, in );

%% Fit model
linsys_frd_3 = tfest( frd, 3 );

% Model reduction
for i = 1:6
    linsys_simple{i} = balred( linsys, i );
end

% Try different method
linsys_simple_2 = reduce( linsys, 2 )

%% Plot results - Bode Plots
figure('Name', 'Bode plot')
hold on; grid on; box on
opts = bodeoptions;
opts.PhaseWrapping = 'on';
opts.FreqUnits = 'Hz';
for i = 1:length(linsys_simple)
    bode( linsys_simple{i}, opts )
    legendStr{i} = sprintf('Order = %d', i);
end
legendStr{end+1} = 'Original';
bode(linsys, 'k--', opts)
legendStr{end+1} = 'FRD';
bode(frd, 'r*', opts)
legendStr{end+1} = 'Test 2';
bode(linsys_simple_2, '-.', opts)
legend(legendStr)

%% Plot results - Steps
figure('Name', 'Steps')
hold on; grid on; box on
for i = 1:length(linsys_simple)
    step( linsys_simple{i} )
    legendStr{i} = sprintf('Order = %d', i);
end
legendStr{end+1} = 'Original';
step(linsys, 'k--')
legendStr{end+1} = 'Test 2';
step(linsys_simple_2, '-.')
legend(legendStr)

%% Plot results - Sinusoidal
t = 0:0.01:4;
u = sin(10*t);
figure('Name', 'Sinusoid')
hold on; grid on; box on
for i = 1:length(linsys_simple)
    lsim( linsys_simple{i}, u, t )
    legendStr{i} = sprintf('Order = %d', i);
end
legendStr{end+1} = 'Original';
lsim(linsys, u, t, 'k--')
legendStr{end+1} = 'Test 2';
lsim(linsys_simple_2, u, t, '-.')
legend(legendStr)

%% Save results
pitchTF = tf( linsys_simple_2 );
save( fullfile( projectRoot, 'work', 'pitch_tf' ), 'pitchTF' )