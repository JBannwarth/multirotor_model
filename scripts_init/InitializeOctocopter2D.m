Uav.M = 2.19;
Uav.J = 0.0175;
Uav.RHO_AIR = 1.225;
Uav.D_UAV = 0.5;
Uav.D_PROP = 10 * 0.0254;
Uav.A_UAV = (pi*Uav.D_UAV^2)/4;
Uav.A_PROP = (pi*Uav.D_PROP^2)/4;
Uav.ZETA = deg2rad(31);
Uav.N_ROTORS = 8;
Uav.C_T = 0.5*0.005052618460001*Uav.RHO_AIR*Uav.D_PROP^2*Uav.A_PROP;
Initial.OMEGA = ones(8,1)*500;

