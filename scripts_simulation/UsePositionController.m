function UsePositionController( modelName, toUse )
%USEPOSITIONCONTROLLER Switches the position controller on/off
%   Written by: Jeremie Bannwarth, 21/08/2017
%     toUncomment = { 'pos_from_q', 'pos_from_xi', 'pos_from_xiDot', ...

    if toUse
        posComment = 'off';
        attComment = 'on';
        sw = 1;
    else
        posComment = 'on';
        attComment = 'off';
        sw = 0;
    end

    posBlocks = find_system( modelName, 'SearchDepth', '1', 'regexp', 'on', ...
        'IncludeCommented', 'on', 'Name', 'pos_');
    posBlocks{end+1} = [modelName '/Default control stick positions'];
    attInputBlocks = find_system( modelName, 'SearchDepth', '1', 'regexp', 'on', ...
        'IncludeCommented', 'on', 'Name', 'att_\w*Input');
    attSwitches = find_system( modelName, 'SearchDepth', '1', 'regexp', 'on', ...
        'IncludeCommented', 'on', 'Name', 'att_\w*Switch');
    
%     for i = 1:length(posBlocks)
%         set_param(posBlocks{i}, 'Commented', posComment)
%     end
    
    for i = 1:length(attInputBlocks)
        set_param(attInputBlocks{i}, 'Commented', attComment)
    end
    
    for i = 1:length(attSwitches)
        set_param(attSwitches{i}, 'sw', num2str(sw))
    end
end