clear all; %#ok<CLALL>

%% 1) Load model and initialize
TestControllerInitOnly;

model = 'MultirotorSimulationController';
load_system(model);

%% 2) Set up the iterations that we want to compute
windSpeedX = [ 0 1.31 2.68 4.25 5.78 7.28 ];
windVelString = cell( size(windSpeedX) );

for i = 1:length(windSpeedX)
    windVelString{i} = [ '[' num2str(windSpeedX(i)) '; 0; 0]' ];
end

%% 3) Loop over the number of iterations
pitchSS = zeros( length(windSpeedX), 1 );
for i = 1:length(windSpeedX)
    
    set_param( [model '/Constant wind'], 'value', windVelString{i} );
    output = sim(model, 'SimulationMode', 'normal');
    pitch = output.get('pitch');
    pitchSS(i) = pitch(end);
    
end

%% 4) Plot data
CompareSteadyStateHoverAngles;

%% 5) Clean up
close_system(model, 0);