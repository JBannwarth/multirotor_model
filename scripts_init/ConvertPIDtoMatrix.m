%CONVERTPIDTOMATRIX Convert PID gains to single output feedback gain matrix
%   Written by:    J.X.J. Bannwarth, 2019/06/18
%   Last modified: J.X.J. Bannwarth, 2019/06/19

K.Pp = diag([0.95,0.95,1]);
K.Pv = diag([0.09,0.09,0.2]);
K.Iv = diag([0.02,0.02,0.1]);
K.Dv = diag([0.01,0.01,0]);
K.t = 1/(5*2*pi);

Kpid = [ K.Pp*K.Pv+K.Iv, (1/K.t)*K.Dv+K.Pv, K.Pp*K.Iv, -(1/K.t)*K.Dv ];
Kpid = [ K.Pp*K.Pv+K.Iv, K.Pv, K.Pp*K.Iv, K.Dv ]

PpPv = K.Pp*K.Pv;

loc = 'MultirotorSimPx4v1_8FPHT/Pos Control/Feedback gain/K_';
blocks = { 'xy', 'z', 'xyDot', 'zDot', 'xyInt', 'zInt', 'xyDDotF', 'zDDotF', ...
    'xyP', 'zP' };
gains = {K.Iv(1,1), K.Iv(3,3), Kpid(1,4), Kpid(3,6), Kpid(1,7),...
    Kpid(3,9), Kpid(1,10), Kpid(3,12), PpPv(1,1), PpPv(3,3) };

for i = 1:length( blocks )
    set_param( [ loc blocks{i} ], 'gain', num2str(gains{i}) );
end

