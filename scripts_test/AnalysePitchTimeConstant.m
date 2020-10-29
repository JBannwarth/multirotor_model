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
    'SettlingPeriods', [1; in.SettlingPeriods])

% Switch to accelerator mode to reduce simulation time
set_param(model, 'AccelVerboseBuild', 'on')
set_param(model, 'SimulationMode', 'normal');

% Estimate frequency response
frd = frestimate( model, op, io, in );

%% Fit model
linsys_frd_3 = tfest( frd, 3 );

% Model reduction
for i = 1:6
    linsys_simple{i} = balred(linsys,i);
end

%% Plot results
figure('Name', 'Bode plot')
hold on; grid on; box on
opts = bodeoptions;
%opts.PhaseWrapping = 'on';
opts.FreqUnits = 'Hz';
for i = 1:length(linsys_simple)
    bode( linsys_simple{i}, opts )
    legend_str{i} = sprintf('Order = %d', i);
end
legend_str{end+1} = 'Original';
bode(linsys, 'k--', opts)
legend_str{end+1} = 'FRD';
bode(frd, 'r*', opts)
legend_str{end+1} = 'Test 2';
bode(linsys_frd_3, '-.', opts)
legend(legend_str)