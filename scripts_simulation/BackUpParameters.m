%LOADPX4PARAMETERS Load PX4 parameters from structure to Simulink model
%   Written by: J.X.J. Bannwarth, 2017/09/13

%% Set-up
clearvars params flog
model = 'MultirotorSimPx4SeparateRotors';
px4Blocks = { 'mc_pos_control', 'mc_att_control', ...
    'Sensor Model/local_position_estimator', 'Sensor Model/attitude_estimator_q' };

%% Write parameters to Simulink model
for i = 1:length( px4Blocks )
    p = Simulink.Mask.get([model '/' px4Blocks{i}]);
    blockParams = { p.Parameters.Name };
    for j = 1:length( blockParams )
        flog.params.( blockParams{j} ) = p.Parameters(j).Value;
    end
end