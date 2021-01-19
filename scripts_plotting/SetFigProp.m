function SetFigProp( figSize, fontSize )
%SETFIGPROP Set figure proportions
%   SetFigProp()
%   SetFigProp(figSize)
%   SetFigProp(figSize, fontSize)
%   Figsize = [width, height] (cm)
%   Written:                   Z.J. Chen
%   Last modified: 2021/01/19, J.X.J. Bannwarth
    arguments
        figSize  (1, 2) double {mustBeNumeric,mustBeReal} = [8 8]
        fontSize (1, 1) double {mustBeNumeric,mustBeReal} = 12
    end
    
    set( findall(gcf, '-property', 'LineWidth'), 'LineWidth', 0.5)
    set( findall(gcf, '-property', 'TitleFontWeight'), 'TitleFontWeight', 'normal')
    set( findall(gcf, '-property', 'TitleFontSizeMultiplier'), 'TitleFontSizeMultiplier', 1)
    set( findall(gcf, '-property', 'LabelFontSizeMultiplier'), 'LabelFontSizeMultiplier', 1)
    

    set( gcf, 'PaperUnits',        'centimeters',   ...
              'PaperPositionMode', 'manual',        ...
              'PaperPosition',     [0, 0, figSize], ...
              'PaperSize',         figSize,         ...
              'Color',             [1, 1, 1]        )

    set( findall(gcf, '-property', 'FontSize'), 'FontSize', fontSize )
    set( findall(gcf, '-property', 'Interpreter'), 'Interpreter', 'latex' )
    set( findall(gcf, '-property', 'TickLabelInterpreter'), 'TickLabelInterpreter', 'latex' )
    
    % Only adjust axes colours if they are not part of the default set, in
    % order not to break multi-axes plots
    axs = 'XYZ';
    defaultColours = lines();
    for ii = 1:3
        axHandles = findall(gcf, '-property', [ axs(ii) 'Color' ]);
        for jj = 1:length( axHandles )
            if ~nnz( ismember( defaultColours, ...
                    get( axHandles(jj), [ axs(ii) 'Color' ] ), 'rows' ) )
                set( axHandles(jj), [ axs(ii) 'Color' ], 'k' );
            end
        end
    end
end