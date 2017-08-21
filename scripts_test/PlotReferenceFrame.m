function [] = PlotReferenceFrame( q, scale, label )
% PLOTREFERENCEFRAME.M Plot a 3d reference frame
% Written by: J.X.J. Bannwarth, 27/03/2017

    R = QuaternionToRotationMatrixMat( q );
    
    xRot = scale .* R * [1; 0; 0];
    yRot = scale .* R * [0; 1; 0];
    zRot = scale .* R * [0; 0; 1];

    quiver3(0, 0, 0, xRot(1), xRot(2), xRot(3), 'linewidth', 1.5 )
    hold on;
    view([1, 1, 1])
    quiver3(0, 0, 0, yRot(1), yRot(2), yRot(3), 'linewidth', 1.5 )
    quiver3(0, 0, 0, zRot(1), zRot(2), zRot(3), 'linewidth', 1.5 )

    if ( ~strcmp(label, '') )
        xstr = ['x_{' label '}'];
        ystr = ['y_{' label '}'];
        zstr = ['z_{' label '}'];
    else
        xstr = 'x';
        ystr = 'y';
        zstr = 'z';
    end
    
    text(xRot(1), xRot(2), xRot(3), xstr);
    text(yRot(1), yRot(2), yRot(3), ystr);
    text(zRot(1), zRot(2), zRot(3), zstr)
end