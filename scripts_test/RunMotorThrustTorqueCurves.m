%RUNMOTORTHRUSTTORQUECURVES Create thrust/torque curves for a motor model
%   Written by:    J.X.J. Bannwarth, 2020/08/03

%% First initialization
close all; clc; clearvars;
project = simulinkproject; projectRoot = project.RootFolder;

%% Simulation parameters
model = 'TestMotors';
load_system( model )

load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3' ) );

% Load UAV parameters
loadBuses = false;
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 2;
InitializeParametersOctocopterCanted
InitializeModel

%% Set-up Simulation
pwmNormIn = 0:0.05:1;
pwmIn = pwmNormIn*1000 + 1000;

for i = 1:length(pwmNormIn)
    simIn(i) = Simulink.SimulationInput( model );
    simIn(i) = setVariable( simIn(i), 'pwmIn', pwmNormIn(i) );
end

%% Run simulation
simOut = sim( simIn );

%% Process data
TBody = zeros( length(simOut), 3);
tauBody = zeros( length(simOut), 3);
TMotor = zeros( length(simOut), 3);
tauMotor = zeros( length(simOut), 3);
for i = 1:length( simOut )
    % Total
    TBody(i,:) = simOut(i).logsout.get('TBody').Values.Data(end,:);
    tauBody(i,:) = simOut(i).logsout.get('tauBody').Values.Data(end,:);
    
    % All motors should be the same
    TMotor(i,:) = simOut(i).logsout.get('TMotorIndividual').Values.Data(:,1,end);
    tauMotor(i,:) = simOut(i).logsout.get('tauMotorIndividual').Values.Data(:,1,end);
end

%% Plot data
figure( 'Name', 'Thrust curve' )
hold on; grid on; box on
plot( pwmIn, TMotor(:,3)*cos(Uav.ZETA) )
xlabel( 'PWM input (us)' )
ylabel( 'Motor thrust in body-z (N)' )

figure( 'Name', 'Thrust curve' )
hold on; grid on; box on
plot( pwmIn, tauMotor(:,3) )
xlabel( 'PWM input (us)' )
ylabel( 'Motor moment in body-z (Nm)' )