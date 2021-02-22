K_T = 0.09;
I = 0.0175;
K_R = 0.1043;
DthD = 0.003;
PthD = 0.15;
IthD = 0.05;
Pth = 6.5;
s = tf('s');

% syms K_T I K_R DthD PthD IthD Pth s

plant = 1 / (s*I + K_R);

pidD = (DthD*s^2 + PthD*s + IthD) / s;

velOLTF = pidD*plant;
velCLTF = velOLTF / (1 + velOLTF);
velCLTF = minreal( velCLTF )

posOLTF = Pth * velCLTF * 1/s;
posCLTF = posOLTF / (1 + posOLTF);

posCLTF = minreal( posCLTF )

distCLTF = K_T * feedback( plant, minreal(pidD*-(Pth/s+1)), +1) * 1/s