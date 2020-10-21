function I = CalculateInertiaTensor(m_object, m_tri_plate, ...
    l_tri_wire, r_tri_wire, r_cpd_pendulum, T_cpd_pendulum, ...
    T_tri, T_tri_plate, rotation)
% CALCULATEINERTIATENSOR 
% Calculate inertia tensor from data recorded in compound pendulum (for xx
% and yy MMOI), and trifilar pendulum tests (zz MMOI) and rotate it.
% Assume Ixx = Iyy before rotation.
% Inputs:
%   m_object:    [kg] Mass of the object                                       
%   m_tri_plate: [kg] Mass of support plate                  
%   l_tri_wire:  [m]  Length of wires holding the support plate                
%   r_tri_wire:  [m]  Radius between centre of support plate and wires         
%   r_cpd_pend:  [m]  Radius from COM to pivot in compound pendulum test 
%   T_cpd_pend:  [s]  <Vector> Periods in compound pendulum test 
%   T_tri:       [s]  <Vector> Periods for object and support plate
%   T_tri_plate: [s]  <Vector> Periods for support plate alone
%   rotation:    [-]  <3x3 Matrix> Rotation matrix
% Output:
%   I:           [kgm^2] <3x3 Matrix> Inertia Tensor 
% Written by: J.X.J Bannwarth, 01/2016

% General constants
g = 9.80665; % ms^-1

%% Ixx and Iyy
% Use data from compound pendulum test
I_xx_pivot = m_object * g * r_cpd_pendulum * mean(T_cpd_pendulum)^2 / (4 * pi^2);
I_xx = I_xx_pivot - m_object * r_cpd_pendulum^2;
I_yy = I_xx;

%% Izz
% Use data from trifilar pendulum test
I_zz_plat = r_tri_wire^2 * g * mean(T_tri_plate)^2 * m_tri_plate / (4 * l_tri_wire * pi^2);
I_zz_cb = r_tri_wire^2 * g * mean(T_tri)^2 * (m_tri_plate + m_object) / (4 * l_tri_wire * pi^2);
I_zz = I_zz_cb - I_zz_plat;

%% Inertia Tensor
I = diag([I_xx, I_yy, I_zz]);
I = rotation*I*rotation';