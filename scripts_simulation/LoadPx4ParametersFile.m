function LoadPx4ParametersFile( model, paramsFile )
%LOADPX4PARAMETERSFILE Load PX4 parameters from file to Simulink model
%   Written by: J.X.J. Bannwarth, 2017/09/13
    %% Set-up
    load( paramsFile, 'flog' )
    try
        params = flog.params;
    catch
        error('Input file does not contain parameters')
    end
    
    paramNames = fieldnames( params );
    px4Blocks = { 'mc_pos_control', 'mc_att_control', ...
        'Sensor Model/local_position_estimator', 'Sensor Model/attitude_estimator_q' };

    %% Write parameters to Simulink model
    for i = 1:length( px4Blocks )
        p = Simulink.Mask.get([model '/' px4Blocks{i}]);
        blockParams = { p.Parameters.Name };
        for j = 1:length( blockParams )
            index = find(strcmp( blockParams{j}, paramNames ));
            if ( length( index ) == 1 )
                set_param( [ model '/' px4Blocks{i} ], blockParams{j}, num2str(params.(paramNames{index})) )
            elseif isempty( index ) % do nothing            
            else
                error( 'Two+ identical parameters in source' )
            end
        end
    end

end