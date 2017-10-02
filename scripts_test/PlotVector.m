function [] = PlotVector( in, scale, label, color )
% PLOTVECTOR.M Plot a vector
%   Written by:    J.X.J. Bannwarth, 28/09/2017
%   Last modified: J.x.J. Bannwarth, 28/09/2017

    sz = size( in );
    
    if ( ( sz(1) == 3 ) && ( sz(2) == 1 ) )
        v = in;
    elseif ( ( sz(1) == 1 ) && ( sz(2) == 3 ) )
        v = in';
    else
        error( 'Input needs to be a 3x1 or 1x3 vector' )
    end
    
    vRot = scale .* v .* [1;  -1;  -1]; % in NED
    
    hold on;    
    % Plot arrow
    quiver3(0, 0, 0, vRot(1), vRot(2), vRot(3), 'linewidth', 1.5, 'color', color )
        
    % Add arrow labels
    vRot = 1.05*vRot;
    text(vRot(1), vRot(2), vRot(3), label);
    
    %text(1.3,    0,    0, 'N' );
    %text(0,   -1.3,    0, 'E' );
    %text(0,      0, -1.3, 'D' );
end