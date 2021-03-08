function SwitchAirframeConfiguration( modelName, configuration )
%SWITCHAIRFRAMECONFIGURATION Switch between airframe types.
%   SWITCHAIRFRAMECONFIGURATION( MODELNAME, CONFIGURATION ) switches MODELNAME to use CONFIGURATION airframe.
%
%   Inputs:
%       - modelName:     name of the Simulink model file.
%       - configuration: name of the airframe type. Most common:
%                           'quad_x' and 'octa_x'.
%
%   See also USEESTIMATORS, USEPOSITIONCONTROLLER, USEWINDPROFILE,
%   SET_PARAM.
%
%   Written: 08/01/2019, Jeremie Bannwarth

    arguments
        modelName     (1,:) char {mustBeNonempty}
        configuration (1,:) char {mustBeNonempty}
    end

    %% Check input
    frames = { 'quad_x';
               'quad_plus';
               'hex_x';
               'hex_plus';
               'octa_x';
               'octa_plus' };

    frameIdx = find( strcmp( frames, configuration ), 1 );
    
    if isempty( frameIdx )
        framesStr = sprintf( strjoin( strcat( '\t-', {' '}, frames ), '\\n' ) );
        error( 'Airframe configuration %d not recognized. Available:\n\t%s', ...
            framesStr )
    end
    
    %% Switch blocks
    blocksToChange = { 'Mixer', 'PX4 rotor map to sim rotor map' };
    for ii = 1:length( blocksToChange )
        set_param( [ modelName '/' blocksToChange{1} ], ...
            'airframeConfig', num2str( frameIdx ) );
    end
    
end