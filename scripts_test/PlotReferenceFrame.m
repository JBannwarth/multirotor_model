function [h] = PlotReferenceFrame( in, scale, label, color )
% PLOTREFERENCEFRAME.M Plot a 3d reference frame
%   Written by:    J.X.J. Bannwarth, 27/03/2017
%   Last modified: J.x.J. Bannwarth, 02/10/2017

    sz = size( in );
    
    if ( ( sz(1) == 3 ) && ( sz(2) == 3 ) )
        R = in;
    elseif ( ( sz(1) == 4 ) && ( sz(2) == 1 ) )
        R = QuatToRot( in );
    elseif ( ( sz(1) == 1 ) && ( sz(2) == 4 ) )
        R = QuatToRot( in' );
    else
        error( 'Input needs to be a quaternion or a rotation matrix' )
    end
    
    xRot = scale .* R * [1;  0;  0]; % N
    yRot = scale .* R * [0; -1;  0]; % E
    zRot = scale .* R * [0;  0; -1]; % D
    
    hold on;    
    % Plot arrows
    h(1) = quiver3(0, 0, 0, xRot(1), xRot(2), xRot(3), 'linewidth', 1.5, 'color', color );
    h(2) = quiver3(0, 0, 0, yRot(1), yRot(2), yRot(3), 'linewidth', 1.5, 'color', color );
    h(3) = quiver3(0, 0, 0, zRot(1), zRot(2), zRot(3), 'linewidth', 1.5, 'color', color );
        
    % Add arrow labels
    if ( ~strcmp(label, '') )
        xstr = ['${' label '}_x$'];
        ystr = ['${' label '}_y$'];
        zstr = ['${' label '}_z$'];
    else
        xstr = '$x$';
        ystr = '$y$';
        zstr = '$z$';
    end
    
    xRot = 1.05 * xRot;
    yRot = 1.05 * yRot;
    zRot = 1.05 * zRot;
    text(xRot(1), xRot(2), xRot(3), xstr, 'interpreter', 'latex');
    text(yRot(1), yRot(2), yRot(3), ystr, 'interpreter', 'latex');
    text(zRot(1), zRot(2), zRot(3), zstr, 'interpreter', 'latex');
    
    %text(1.3,    0,    0, 'N' );
    %text(0,   -1.3,    0, 'E' );
    %text(0,      0, -1.3, 'D' );
end