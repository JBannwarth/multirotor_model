function SetFigProp(varargin)
%SETFIGPROP Set figure proportions
%   SetFigProp()
%   SetFigProp(figSize)
%   SetFigProp(figSize, fontSize)
%   Figsize = [width, height] (cm)
%   Written by: Z.J. Chen
%   Last modified by: J.X.J. Bannwarth, 11/10/2017

    fontSize = 12;
    figSize  = [8, 8];
    if ~isempty(varargin)
       figSize = varargin{1}; 
    end
    if (length(varargin) == 2)
        fontSize = varargin{2};
    elseif (length(varargin) > 2)
        error('Too many input')
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

    set( findall(gcf, '-property', 'FontSize'), 'FontSize', fontSize)
    set( findall(gcf, '-property', 'Interpreter'), 'Interpreter', 'latex' )
    set( findall(gcf, '-property', 'TickLabelInterpreter'), 'TickLabelInterpreter', 'latex' )
end