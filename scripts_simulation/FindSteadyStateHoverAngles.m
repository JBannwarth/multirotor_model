clear all; %#ok<CLALL>

%% 1) Load model and initialize
TestControllerInitOnly;
Simulation.T_END = 30;

model = 'MultirotorSimPx4';
load_system(model);

%% 2) Set up the iterations that we want to compute
windSpeedX = [0; 
              2.55999837265402;
              3.45522298273824;
              4.40686338715537;
              5.17829923510479;
              6.09013240228385;
              6.79493302374613];
windVelString = cell( size(windSpeedX) );

for i = 1:length(windSpeedX)
    windVelString{i} = [ '[' num2str(windSpeedX(i)) '; 0; 0]' ];
end

%% 3) Loop over the number of iterations
pitchSS = zeros( length(windSpeedX), 1 );
for i = 1:length(windSpeedX)
    UseWindProfile( model, false );
    set_param( [model '/Fixed wind input'], 'value', windVelString{i} );
    output = sim(model, 'SimulationMode', 'normal');
    pitch = output.get('pitch');
    pitchSS(i) = pitch(end,1);
    
end

%% 4) Plot data
CompareSteadyStateHoverAngles;

%% 5) Clean up
set_param( [model '/Varying wind input'], 'commented', 'off' );
% close_system(model, 0);