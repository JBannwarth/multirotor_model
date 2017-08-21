function UseEstimators( filename, toUse )
%USEESTIMATORS Switches the from blocks to use Q estimator and LPE
%   Written by: Jeremie Bannwarth, 18/08/2017
    fromBlocks = { 'pos_from_q', 'pos_from_xi', 'pos_from_xiDot', ...
        'att_from_q', 'att_from_nuBody' };

    for i = 1:length(fromBlocks)
        loc = strfind( fromBlocks{i}, '_' );
        variable = fromBlocks{i}(loc(end)+1:end);
        if toUse
            variable = [ variable 'Measured' ];
        end

        set_param( [filename '/' fromBlocks{i}], 'GotoTag', variable )
    end
end