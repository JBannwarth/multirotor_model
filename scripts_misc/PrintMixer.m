%PRINTMIXER Print PX4 mixer to tex
%   Written: 2021/01/20, J.X.J. Bannwarth

clearvars;
M1 = [-0.382683,  0.923880,  1.000000,  1.000000 ;
  0.382683, -0.923880,  1.000000,  1.000000 ;
 -0.923880,  0.382683, -1.000000,  1.000000 ;
 -0.382683, -0.923880, -1.000000,  1.000000 ;
  0.382683,  0.923880, -1.000000,  1.000000 ;
  0.923880, -0.382683, -1.000000,  1.000000 ;
  0.923880,  0.382683,  1.000000,  1.000000 ;
 -0.923880, -0.382683,  1.000000,  1.000000 ];

for ii = 1:4
    M1(:,ii) = RotorMapPx4ToSim( M1(:,ii)' )';
end

Mout = cell(8, 2);
for ii = 1:size(M1,1)
    for jj = 1:2
        if M1(ii,jj) < 0
            mSign = '-';
        else
            mSign = ' ';
        end
        if abs(M1(ii,jj)) < 0.5
            Mout{ii,jj} = sprintf( '%s\\sg', mSign );
        else
            Mout{ii,jj} = sprintf( '%s\\cg', mSign );
        end
    end
end

disp( '\begin{bmatrix}' )
for ii = 1:size(M1,1)
    fprintf( '    %s & %s & % 2.0f & % 2.f \\\\\n', Mout{ii,1}, Mout{ii,2}, ...
        M1(ii,3), M1(ii,4) )
end
disp( '\end{bmatrix}' )