clear all; %#ok<CLALL>

%% 1) Load model and initialize
TestControllerInitOnly;
Simulation.T_END = 60;

model = 'MultirotorSimulationController';
load_system(model);

%% 2) Set up the iterations that we want to compute
windFiles = { 'p5_z150_tfwt15', 'p5_z150_tfwt20', 'p5_z150_tfwt25', ...
              'p5_z150_tfwt30', 'p5_z150_tfwt35', 'p5_z150_tfwt40' };
inFolder  = 'inputdata';

%% 3) Loop over the number of iterations

% No wind
set_param( [model '/Varying wind input'], 'commented', 'on' );
set_param( [model '/Wind switch'], 'sw', '0' );
set_param( [model '/Fixed wind input'], 'Value', '[0;0;0]' );
output(1) = sim( model, 'SimulationMode', 'normal' );

for i = 1:length( windFiles )
    load( [ inFolder '/' windFiles{i} 'NoDrops' '.mat' ] )
    set_param( [model '/Varying wind input'], 'commented', 'off' );
    set_param( [model '/Wind switch'], 'sw', '1' );
    set_param( [model '/Varying wind input'], 'VariableName', 'windInput' );
    output(i+1) = sim( model, 'SimulationMode', 'normal' );
end

%% 4) Plot data
PlotPositionHoldError;

%% 5) Clean up
% close_system(model, 0);