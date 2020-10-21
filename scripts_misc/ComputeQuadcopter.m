inFile = 'data_misc/MeasurementsInertia_Quadcopter.xlsx';
m_object       = xlsread( inFile, 'chars', 'A2');
m_tri_plate    = xlsread( inFile, 'chars', 'C2');
l_tri_wire     = xlsread( inFile, 'chars', 'D2');
r_tri_wire     = xlsread( inFile, 'chars', 'E2');
r_cpd_pendulum = xlsread( inFile, 'chars', 'F2');

T_cpd_pendulum = xlsread( inFile, 'mmoi', 'D4:D7') ./ xlsread( inFile, 'mmoi', 'D2');
T_tri          = xlsread( inFile, 'mmoi', 'C4:C6') ./ xlsread( inFile, 'mmoi', 'C2');
T_tri_plate    = xlsread( inFile, 'mmoi', 'B4:B6') ./ xlsread( inFile, 'mmoi', 'B2');

rotation = eye(3);

I = CalculateInertiaTensor(m_object, m_tri_plate, ... 
    l_tri_wire, r_tri_wire, r_cpd_pendulum, T_cpd_pendulum, ...
    T_tri, T_tri_plate, rotation);