function [] = PlotArc( R, v2, scale, label, color )
% PLOTARC Plot an arc in 3D space
%   Written by:    J.X.J. Bannwarth, 28/09/2017
%   Last modified: J.x.J. Bannwarth, 28/09/2017

    v1 = R(:,3); % Use z axis as ref    
    e_R = R' * cross(v1, v2);
    e_R_z_sin = norm(e_R);
    e_R_z_cos = dot(v1, v2);
    e_R_z_angle = atan2(e_R_z_sin, e_R_z_cos);
    n = cross(v1, v2)./norm(cross(v1, v2));
    theta = linspace(0,e_R_z_angle,100);
    for i = 1:length(theta)
        P(:,i) = scale*cos(theta(i))*v1 + scale*sin(theta(i)) * cross( n, v1 );
    end
    plot3(P(1,:),-P(2,:),-P(3,:), 'linewidth', 1.5, 'color', color)
    
    labelTheta = e_R_z_angle/2;
    labelP = scale*1.1*cos(labelTheta)*v1 + scale*1.1*sin(labelTheta) * cross( n, v1 );
    
    % Add arrow labels
    text(labelP(1), -labelP(2), -labelP(3), label);
end