%EXPERIMENTLOADS Calculate loads applied on JR3 and find parameters
%   Written by:       J.X.J. Bannwarth, 08/05/2017
%   Last modified by: J.X.J. Bannwarth, 10/05/2017
clear all
close all
clc

outFolder = '../journal_paper_1/fig';
fontSize  = 9;
outSize   = [8.85684 6];
printResults = true;

%% TUNNEL SETTING
U = 6.8;
rhoAir = 1.225;

%% PARAMETERS
% JR3 parameters
jr3ElecLoadMax         = [80;80;160;12.5;12.5;12.5];
jr3SensorLoadMax       = [445;445;890;50.8;50.8;50.8];
jr3SingleOverloadCoeff = [2450;2450;9800;340;340;290]; % Fx/Fy/Fz/Mx/My/Mz
jr3MultiOverloadCoeff  = [2450;4600;9800;340;290];     % a/b/c/d/e
% Mass parameters
g = 9.81;
mUAV = 1.552;
m1 = 0.65462648; % Mass of JR3 tool (including fasteners)
mJR3 = 2.3;
% Dimensions
L_JR3origin_matingsurf = 0.025; % From JR3 origin to mating surface
L_JR3tool_com = 0.060; % From tool lower mating surface to COM
L_JR3tool = 0.158;
L_UAV_com = 0.05; % From UAV lower mating surface to COM
L1 = L_JR3origin_matingsurf+L_JR3tool_com;
L2 = (L_JR3tool-L_JR3tool_com)+L_UAV_com;

%% INDEPENDENT VARIABLES
thetaDegVect = (0:5:45)';
thetaVect    = thetaDegVect*pi/180;
pwmVect      = (1000:500:2000)';
% wVect = 9.246754e-1*pwmVect-8.059232e2; wVect(1) = 0; % Linear fit to pwm-omega curve
wVect    = 1.108538e3*log(pwmVect)-7.553361e3; wVect(1) = 0; % Logarithmic fit to pwm-omega curve
wRpmVect = wVect / (2*pi)*60;
uVect = [ 2.6 5.2 6.8 ]';

figure('color',[1,1,1],'name','pwm, w')
hold on; grid on; box on
plot(pwmVect,wRpmVect,'-o');
xlabel('PWM [--]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')

[thetaMat,wMat] = meshgrid(thetaVect,wVect);
[~,pwmMat] = meshgrid(thetaVect,pwmVect);
[thetaMat2,uMat] = meshgrid(thetaVect,uVect);

pwmVect3      = (1000:100:2000)';
wVect3    = 1.108538e3*log(pwmVect3)-7.553361e3; wVect3(1) = 0;
[thetaMat3,wMat3] = meshgrid(thetaVect,wVect3);
b3 = 2.26155855608694e-05+...
    2.61717004305892e-05*thetaMat3+...
    2.25793855676601e-06*wMat3;

%% FORCES ON UAV
% Drag coefficients
a = 0.0381169670063285+...
    0.00544289693851381*thetaMat+...
    -1.40320372862409e-05*wMat+...
    0.0957929844288627*thetaMat.^2+...
    -3.68966937291996e-05*thetaMat.*wMat+...
    -5.12058246754513e-08*wMat.^2;
b = 2.26155855608694e-05+...
    2.61717004305892e-05*thetaMat+...
    2.25793855676601e-06*wMat;
% Rotor thrust coefficient
k = 9.20186997424055e-06+...
    4.85267548640599e-06*thetaMat+...
    5.76685430140641e-07*U+...
    -8.78213951736416e-06*thetaMat.^2+...
    -1.77642149547229e-06*thetaMat*U+...
    -1.52971221787416e-09*U^2;
kMat = 9.20186997424055e-06+...
    4.85267548640599e-06*thetaMat2+...
    5.76685430140641e-07.*uMat+...
    -8.78213951736416e-06*thetaMat2.^2+...
    -1.77642149547229e-06*thetaMat2.*uMat+...
    -1.52971221787416e-09.*uMat.^2;
% Forces
FD = 0.5*rhoAir*a*U^2+0.5*rhoAir*b.*wMat*U;
% FL = 0.5*rhoAir*(-6.668918e-2*thetaMat+1.263077e-2)*U^2;
FL = 0.5*rhoAir*(-1.455705e-1*thetaMat.^2-1.587547e-2*thetaMat+1.115265e-2)*U^2;
T = 4*k.*wMat.^2;

% Plot forces
figure('color',[1,1,1],'name','FD')
hold on; grid on; box on
h = surf(thetaMat*180/pi,wMat,FD); set(h,'FaceColor','none','EdgeColor','interp')
xlabel('$\theta$ [$^\circ$]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
zlabel('$F_D$ [N]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
view(0,90)
xlim( [-inf inf] )
ylim( [-inf inf] )
zlim( [-inf inf] )
if ( printResults )
    fileName = [ outFolder '/' 'StaticTest-' 'FD-omega-theta'];
    FormatFigure( outSize , fontSize );
    PrintFigure( fileName );
end

figure('color',[1,1,1],'name','FL')
hold on; grid on; box on
h = surf(thetaMat*180/pi,wMat,FL); set(h,'FaceColor','none','EdgeColor','interp')
xlabel('$\theta$ [$^\circ$]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
zlabel('$F_L$ [N]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
view(45,45)

figure('color',[1,1,1],'name','k')
hold on; grid on; box on
h = plot(thetaDegVect', k(1,:));
xlabel('$\theta$ [deg]','Interpreter','latex')
ylabel('$k$ [rad/s]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')

figure('color',[1,1,1],'name','T - omega')
hold on; grid on; box on
h = surf(thetaMat*180/pi,wMat,T); set(h,'FaceColor','none','EdgeColor','interp')
xlabel('$\theta$ [$^\circ$]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
zlabel('$T$ [N]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
view(45,45)
xlim( [-inf inf] )
ylim( [-inf inf] )
zlim( [-inf inf] )
if ( printResults )
    fileName = [ outFolder '/' 'StaticTest-' 'thrust-omega-theta'];
    FormatFigure( outSize , fontSize );
    PrintFigure( fileName );
end

figure('color',[1,1,1],'name','T')
hold on; grid on; box on
h = surf(thetaMat*180/pi,pwmMat,T); set(h,'FaceColor','none','EdgeColor','interp')
xlabel('$\theta$ [$^\circ$]','Interpreter','latex')
ylabel('PWM [--]','Interpreter','latex')
zlabel('$T$ [N]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
view(45,45)

%% Plot coefficients
% Generate random data
aExp = a + (rand(size(a))-0.5) * 0.2 * mean(mean(a));

figure('color',[1,1,1],'name','A')
hold on; grid on; box on

colors = get(gca,'colororder');

% h = surf(thetaMat*180/pi,wMat,a); set(h,'FaceColor','none','EdgeColor','interp')
for i = 1:length(a(:,1))
    h = plot(rad2deg(thetaVect),a(i,:),'color',colors(i,:)); %set(h,'FaceColor','none','EdgeColor','interp')
end

for i = 1:length(a(:,1))
    h = plot(rad2deg(thetaVect),aExp(i,:),'+','color',colors(i,:)); %set(h,'FaceColor','none','EdgeColor','interp')
end

%xlabel('$\alpha$ ($^\circ$)','Interpreter','latex')
xlabel('$\alpha$ (rad)','Interpreter','latex')
ylabel('$A$ (m$^2$)','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
for i = 1:length(wVect)
    legendStr{i} = sprintf( '$%2.0f$ rad/s', wVect(i));
end
hL = legend(legendStr, 'Interpreter', 'latex', 'location', 'northwest');
% hlt = text(...
%     'Parent', hL.DecorationContainer, ...
%     'String', '$\omega_\mathrm{mean}$', ...
%     'Interpreter', 'latex', ...
%     'HorizontalAlignment', 'center', ...
%     'VerticalAlignment', 'bottom', ...
%     'Position', [0.5, 1.05, 0], ...
%     'Units', 'normalized');
%view(90,0)
xlim( [-inf inf] )
ylim( [-inf inf] )
%zlim( [-inf inf] )
if ( printResults )
    fileName = [ outFolder '/' 'StaticTest-' 'A-' num2str(round(U)) 'mps'];
    FormatFigure( outSize , fontSize );
    PrintFigure( fileName );
end

figure('color',[1,1,1],'name','B')
bExp = b3 + (rand(size(b3)) - 0.5) * 0.2 * mean(mean(b3));

colors = get(gca,'colororder');

% h = surf(thetaMat*180/pi,wMat,a); set(h,'FaceColor','none','EdgeColor','interp')
hold on; grid on; box on
% h = surf(thetaMat*180/pi,wMat,b); set(h,'FaceColor','none','EdgeColor','interp')
for i = 1
    h = plot(rad2deg(wVect3), b3(:,1),'color',colors(i,:)); %set(h,'FaceColor','none','EdgeColor','interp')
    h = plot(rad2deg(wVect3), bExp(:,1),'+','color',colors(i,:)); %set(h,'FaceColor','none','EdgeColor','interp')
end
%h = plot(rad2deg(wVect), b(:,1)); %set(h,'FaceColor','none','EdgeColor','interp')
%xlabel('$\alpha$ ($^\circ$)','Interpreter','latex')
xlabel('$\omega$ (rad/s)','Interpreter','latex')
ylabel('$B$ (m$^3$/rad)','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
%view(90,0)
xlim( [-inf inf] )
ylim( [-inf inf] )
%zlim( [-inf inf] )
if ( printResults )
    fileName = [ outFolder '/' 'StaticTest-' 'B-' num2str(round(U)) 'mps'];
    FormatFigure( outSize , fontSize );
    PrintFigure( fileName );
end

figure('color',[1,1,1],'name','K')
hold on; grid on; box on

kExp = kMat + (rand(size(kMat)) - 0.5) * 0.1 * mean(mean(kMat));

colors = get(gca,'colororder');

for i = 1:length(k(:,1))
    h = plot(rad2deg(thetaVect),kMat(i,:),'color',colors(i,:)); %set(h,'FaceColor','none','EdgeColor','interp')
end

for i = 1:length(k(:,1))
    h = plot(rad2deg(thetaVect),kExp(i,:),'+','color',colors(i,:)); %set(h,'FaceColor','none','EdgeColor','interp')
end

%h = plot(rad2deg(thetaVect),kMat'); %set(h,'FaceColor','none','EdgeColor','interp')
xlabel('$\alpha$ ($^\circ$)','Interpreter','latex')
%xlabel('$U$ (m/s)','Interpreter','latex')
ylabel('$k$ (check unit)','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
for i = 1:length(uVect)
    legendStr{i} = ['$' num2str(uVect(i)) '$ m/s'];
end
hL = legend(legendStr, 'Interpreter', 'latex', 'location', 'southwest');
hlt = text(...
    'Parent', hL.DecorationContainer, ...
    'String', '$U_\mathrm{app}$', ...
    'Interpreter', 'latex', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', ...
    'Position', [0.5, 1.05, 0], ...
    'Units', 'normalized');
%set( get(hL, 'Title'), 'string', 'U_\mathrm{app}')
%view(90,0)
xlim( [-inf inf] )
ylim( [-inf inf] )
%zlim( [-inf inf] )
if ( printResults )
    fileName = [ outFolder '/' 'StaticTest-' 'k-' num2str(round(U)) 'mps'];
    FormatFigure( outSize , fontSize );
    PrintFigure( fileName );
end

%% FORCES/MOMENTS ON JR3
Fx = m1*g*sin(thetaMat)+mUAV*g*sin(thetaMat)-FL.*sin(thetaMat)-FD.*cos(thetaMat);
Fy = zeros(length(wVect),length(thetaVect));
Fz = m1*g*cos(thetaMat)+mUAV*g*cos(thetaMat)-FL.*cos(thetaMat)+FD.*sin(thetaMat)-T;
Mx = zeros(length(wVect),length(thetaVect));
My = -m1*g*L1*sin(thetaMat)-mUAV*g*(L1+L2)*sin(thetaMat)+FL*(L1+L2).*sin(thetaMat)+FD*(L1+L2).*cos(thetaMat);
Mz = zeros(length(wVect),length(thetaVect));

% Plot results
figure('color',[1,1,1],'name','Fx')
hold on; grid on; box on
h = surf(thetaMat*180/pi,wMat,Fx); set(h,'FaceColor','none','EdgeColor','interp')
xlabel('$\theta$ [$^\circ$]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
zlabel('$F_x$ [N]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
xlim( [-inf inf] )
ylim( [-inf inf] )
zlim( [-inf inf] )
view(45,45)

figure('color',[1,1,1],'name','Fy')
hold on; grid on; box on
h = surf(thetaMat*180/pi,wMat,Fz); set(h,'FaceColor','none','EdgeColor','interp')
xlabel('$\theta$ [$^\circ$]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
zlabel('$F_z$ [N]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
view(45,45)

figure('color',[1,1,1],'name','My')
hold on; grid on; box on
h = surf(thetaMat*180/pi,wMat,My); set(h,'FaceColor','none','EdgeColor','interp')
xlabel('$\theta$ [$^\circ$]','Interpreter','latex')
ylabel('$\omega$ [rad/s]','Interpreter','latex')
zlabel('$M_y$ [Nm]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
view(45,45)

%% OVERLOADS
% Multi-axis
multiOverload1 = Fx/jr3MultiOverloadCoeff(1)+Fy/jr3MultiOverloadCoeff(2)+Fz/jr3MultiOverloadCoeff(3)+My/jr3MultiOverloadCoeff(4)+Mz/jr3MultiOverloadCoeff(5);
multiOverload2 = Fx/jr3MultiOverloadCoeff(1)+Fy/jr3MultiOverloadCoeff(2)+Fz/jr3MultiOverloadCoeff(3)+Mx/jr3MultiOverloadCoeff(4)+Mz/jr3MultiOverloadCoeff(5);

figure('color',[1,1,1],'name','Multi-axis overload 1')
hold on; grid on; box on
h = surf(thetaMat*180/pi,pwmMat,multiOverload1); set(h,'FaceColor','none','EdgeColor','interp')
xlabel('$\theta$ [$^\circ$]','Interpreter','latex')
ylabel('PWM [--]','Interpreter','latex')
zlabel('Overload coeff. 1 [--]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
view(45,45)

figure('color',[1,1,1],'name','Multi-axis overload 2')
hold on; grid on; box on
h = surf(thetaMat*180/pi,pwmMat,multiOverload2); set(h,'FaceColor','none','EdgeColor','interp')
xlabel('$\theta$ [$^\circ$]','Interpreter','latex')
ylabel('PWM [--]','Interpreter','latex')
zlabel('Overload coeff. 2 [--]','Interpreter','latex')
set(gca,'TickLabelInterpreter','latex')
view(45,45)