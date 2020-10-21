function SwitchRotorNumber( modelName, rotorNumber )
%SWITCHROTORNUMBER Switch exclusive blocks like the motor map
%   Written by:    Jeremie Bannwarth, 08/01/2019
%   Last modified: Jeremie Bannwarth, 08/01/2019
    blocksToSwitch = {'Mixer', 'PX4 rotor map to sim rotor map'};
    switch rotorNumber
        case 4
            uavName = 'quadcopter';
        case 6
            uavName = 'hexacopter';
        case 8
            uavName = 'octocopter';
        otherwise
            error( 'Unrecognised number of rotors' );
    end
    
    for i = 1:length(blocksToSwitch)
        newBlock = sprintf( 'Px4Library/%s (%s)', blocksToSwitch{i}, uavName );
        replace_block( modelName, 'Name', blocksToSwitch{i}, newBlock, 'noprompt');
    end
    
    % Reconnect lines
    port = get_param(sprintf('%s/mc_att_control', modelName),'PortHandles');
    line = get_param(port.Outport(1), 'Line' );
    delete_line(line);
    add_line(modelName, 'mc_att_control/1', 'actuatorsControl selector/1', 'autorouting', 'on' );
    add_line(modelName, 'mc_att_control/1', 'Mixer/1', 'autorouting', 'on' );
end