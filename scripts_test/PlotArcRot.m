function [] = PlotArcRot( qO, q, scale, titleStr, labelO, labelF, labelRot, colorIdx )
% PLOTARCROT Plot showing rotation between two reference frames
%   Inputs:
%       - qO: origin orientation quaternion
%       - qD: destination orientation quaternion
%       - scale: radius of the arc
%       - label: label to put on graph
%       - color: color of arc
%   Written by:    J.X.J. Bannwarth, 2020/08/20
%   Last modified: J.x.J. Bannwarth, 2020/08/20
    
    %% Set-up
    
    % Useful functions
    vecRot = @(v,q) ( [zeros(3,1) eye(3)] * QuatMult( q, QuatMult([0;v],QuatInv(q)) ) );
    
    % Set-up figure
    axes( 'dataaspectratio', [1 1 1], 'projection', 'orthographic' )
    title( titleStr, 'interpreter', 'latex' )
    cmap = get(gca,'colororder');
    
    %% Plot drawing
    % Plot origin
    RO = QuatToRot( qO );
    ROScaled = scale .* RO;
    
    % Plot baseline
    hold on;
    if ~strcmp(labelO, '')
        axLabels = {['${' labelO '}_x$'], ['${' labelO '}_y$'], ['${' labelO '}_z$'] };
    else
        axLabels = {'x', 'y', 'z'};
    end
    for ax = 1:3
        h(ax) = quiver3(0, 0, 0, ROScaled(1,ax), -ROScaled(2,ax), -ROScaled(3,ax), 'linewidth', 1.5, 'color', cmap(colorIdx,:) );
        
        % Add labels
        text(1.05*ROScaled(1,ax), -1.05*ROScaled(2,ax), -1.05*ROScaled(3,ax), axLabels{ax}, 'interpreter', 'latex');
    end

    % Get rotation
    sinAngle = norm( q(2:end) );
    cosAngle = q(1);
    angle = atan2( sinAngle, cosAngle );
    n = q(2:end) / sinAngle;
    
    % Plot rotated coordinate frame
    if ~strcmp(labelF, '')
        axLabelsF = {['${' labelF '}_x$'], ['${' labelF '}_y$'], ['${' labelF '}_z$'] };
    else
        axLabelsF = {'x''', 'y''', 'z'''};
    end
    Rrot = scale.*QuatToRot( QuatMult( qO, q ) );
    for ax = 1:3
        axVector = Rrot(:,ax);%vecRot( ROScaled(:,ax), q );
        
        h2(ax) = quiver3(0, 0, 0, axVector(1), -axVector(2), -axVector(3), 'linewidth', 1.5, 'color', cmap(colorIdx+1,:) );        
        % Add labels
        if sqrt( sum((axVector - ROScaled(:,ax)).^2) ) < 0.1
            labelScale = 1.2;
        else
            labelScale = 1.05;
        end
        text(labelScale*axVector(1), -labelScale*axVector(2), -labelScale*axVector(3), axLabelsF{ax}, 'interpreter', 'latex');
    end
    
    % Plot arcs
    alpha = linspace(0,angle,100);
    labelAlpha = angle/2;
    labelQuat = [cos(labelAlpha); sin(labelAlpha)*n];
    labelR = 1.3*0.5*scale.*QuatToRot( QuatMult( qO, labelQuat ) );
    P = zeros(3,length(alpha),3);
    colorArc = mean(cmap(colorIdx:colorIdx+1,:),1);
    for ax = 1:3
        for ii = 1:length(alpha)
            Rrot = 0.5*scale.*QuatToRot( QuatMult( qO, [cos(alpha(ii)); sin(alpha(ii))*n] ) );
            P(:,ii,ax) = Rrot(:,ax);
        end
        % Don't show if the arc is too small
        if max(abs(diff( P(:,:,ax), [], 2 )) > 1e-6 )
            labelP = labelR(:,ax);
            text( labelP(1), -labelP(2), -labelP(3), labelRot, 'interpreter', 'latex' )
            plot3(P(1,:,ax),-P(2,:,ax),-P(3,:,ax), 'linewidth', 1.5, 'color', colorArc )
        end
    end
    
    %% Clean-up
    xlim auto; ylim auto; zlim auto
    view([1 1 1])
    set(findobj(gcf, 'type', 'axes'), 'Visible', 'off') % Clean up graph
    set(findall(gca, 'type', 'text'), 'Visible', 'on');
end