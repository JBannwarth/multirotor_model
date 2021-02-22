%% Cleanup
clear; clc; close all;

%% Load data
load turb_5ms.mat
load orientationTest.mat
tW = windData(:,1);
xW = windData(:,2);
yW = windData(:,3);
zW = windData(:,4);

%% Find instantaneous wind direction
for i = 1:length(tW)
    wMagnitude = sqrt( xW.^2 + yW.^2 + zW.^2 );
    xWNormd = xW ./ wMagnitude;
    yWNormd = yW ./ wMagnitude;
    zWNormd = zW ./ wMagnitude;
end

%% Plot results
figure('units','normalized','position',[.1 .1 .6 .4]); grid on;

% Setup
xlabel('x'); ylabel('y'); zlabel('z');

uavQuat = [1; 0; 0; 0];
uavRPY  = [0; 0; 0];

for i = 1:length(tW)
    % Comparison of reference frames
    subplot(1,2,1);
    PlotReferenceFrame( [1;0;0;0], 1, 'O' );
    ResetView; hold on;
    
    PlotReferenceFrame( q(i,:), 0.5, 'B' );
    
    quiver3(0, 0, 0, xWNormd(i), yWNormd(i), zWNormd(i), 'linewidth', 1.5 )
    text(xWNormd(i), yWNormd(i), zWNormd(i), 'w') % _{' num2str(tW(i)) '}
    pause(0.01)
    hold off;
    
    % Wind vector in body frame
    subplot(1,2,2);
    PlotReferenceFrame( [1;0;0;0], 1, 'B' );
    ResetView; hold on;
    
    qi = q(i,:)'; qInvi = QuatInv( qi );
    wQuatInO = [0; xWNormd(i); yWNormd(i); zWNormd(i)];
    
    % To go from B->O w_b = q * w_o * q^-1
    % To go from O->B w_o = q^-1 * w_o * q
    wQuatInB = QuatMult( qInvi, QuatMult( wQuatInO, qi ) );
    wInB = wQuatInB(2:end);
    
    quiver3(0, 0, 0, wInB(1), 0,       0,       'linewidth', 2)
    quiver3(0, 0, 0, 0,       wInB(2), 0,       'linewidth', 2)
    quiver3(0, 0, 0, 0,       0,       wInB(3), 'linewidth', 2)
    
    quiver3(0, 0, 0, wInB(1), wInB(2), wInB(3), 'linewidth', 1.5)
    text(wInB(1), wInB(2), wInB(3), 'w')
    hold off;
end

% Get example of orientation
ind = 1800;
qEx = q(1800, :);
etaEx = eta(1800, :);

% figure; grid on; box on; hold on;
% plot(tW, wMagnitude)