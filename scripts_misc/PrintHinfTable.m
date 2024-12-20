%PRINTHINFTABLE Print LaTeX table showing matrices used in H-inf design.
%   Written: 2018, J.X.J. Bannwarth

clearvars; clc
x = { '\int x', '\int y', '\int z', ...
    '\dot{x}', '\dot{y}', '\dot{z}', 'x', 'y', 'z' };
omega = [ 'U_x', 'U_y', 'U_z', strcat( x, '_n' ) ];
u = { 'T_{h,x}', 'T_{h,x}', 'T_{v,x}', 'T_{v,y}', 'T_{v_z}' };
xDot = { 'x', 'y', 'z', ...
    '\ddot{x}', '\ddot{y}', '\ddot{z}', '\dot{x}', '\dot{y}', '\dot{z}' };
z = [x, u];
y = { '\int x_m', '\int y_m', '\int z_m', ...
    '\dot{x}_m', '\dot{y}_m', '\dot{z}_m', 'x_m', 'y_m', 'z_m' };

x = strcat( '\(', x, '\)' );
omega = strcat( '\(', omega, '\)' );
u = strcat( '\(', u, '\)' );
xDot = strcat( '\(', xDot, '\)' );
z = strcat( '\(', z, '\)' );
y = strcat( '\(', y, '\)' );


nCol =  + length(omega) + length(u) + 1;
col = 'p{1.2ex}';
colDef = [ 'p{4ex} :' repmat( [' ' col], 1, length(x) ) ' :' ...
    repmat( [' ' col], 1, length(omega) ) ' :' ...
    repmat( [' ' col], 1, length(u) ) ];

topLine = [ ' & ' strjoin( [ x, omega, u ], ' & ' ) ' \\' ];
xLen = length(x);
xDotLen = length(xDot);
wLen = length(omega);
uLen = length(u);
zLen = length(z);
yLen = length(y);

matNames = {'\mathbf{A}', '\mathbf{B}_1', '\mathbf{B}_2';
    '\mathbf{C}_1', '\mathbf{D}_{11}', '\mathbf{D}_{12}';
    '\mathbf{C}_2', '\mathbf{D}_{21}', '\mathbf{D}_{22}' };
matNames = strcat( '{\Huge \(', matNames, '\) }' );

xDotLine1 = sprintf( [ '%s & \\multicolumn{%d}{c}{\\multirow{%d}{*}{ %s \\(\\in \\mathbb{R}^{%d\\times %d} \\)}}' ...
    ' & \\multicolumn{%d}{c}{\\multirow{%d}{*}{ %s \\(\\in \\mathbb{R}^{%d\\times %d}\\)}} ' ...
    ' & \\multicolumn{%d}{c}{\\multirow{%d}{*}{ %s \\(\\in \\mathbb{R}^{%d\\times %d}\\)}} \\\\' ], ...
    xDot{1}, ...
    xLen, xDotLen, matNames{1,1}, xDotLen, xLen, ...
    wLen, xDotLen, matNames{1,2}, xDotLen, wLen, ...
    uLen, xDotLen, matNames{1,3}, xDotLen, uLen );

xDotLines = strcat( xDot(2:end), repmat( ' &', 1, xLen+wLen+uLen ), ' \\' );

zLine1 = sprintf( [ '%s & \\multicolumn{%d}{c}{\\multirow{%d}{*}{ %s \\(\\in \\mathbb{R}^{%d\\times %d}\\)}} ' ...
    ' & \\multicolumn{%d}{c}{\\multirow{%d}{*}{ %s \\(\\in \\mathbb{R}^{%d\\times %d}\\)}} ' ...
    ' & \\multicolumn{%d}{c}{\\multirow{%d}{*}{ %s \\(\\in \\mathbb{R}^{%d\\times %d}\\)}} \\\\' ], ...
    z{1}, ...
    xLen, zLen, matNames{2,1}, zLen, xLen, ...
    wLen, zLen, matNames{2,2}, zLen, wLen, ...
    uLen, zLen, matNames{2,3}, zLen, uLen );

zLines = strcat( z(2:end), repmat( ' &', 1, xLen+wLen+uLen ), ' \\' );

yLine1 = sprintf( [ '%s & \\multicolumn{%d}{c}{\\multirow{%d}{*}{ %s \\(\\in \\mathbb{R}^{%d\\times %d}\\)}}' ...
    ' & \\multicolumn{%d}{c}{\\multirow{%d}{*}{ %s \\(\\in \\mathbb{R}^{%d\\times %d}\\)}} ' ...
    ' & \\multicolumn{%d}{c}{\\multirow{%d}{*}{ %s \\(\\in \\mathbb{R}^{%d\\times %d}\\)}} \\\\' ], ...
    y{1}, ...
    xLen, yLen, matNames{3,1}, yLen, xLen, ...
    wLen, yLen, matNames{3,2}, yLen, wLen, ...
    uLen, yLen, matNames{3,3}, yLen, uLen );

yLines = strcat( y(2:end), repmat( ' &', 1, xLen+wLen+uLen ), ' \\' );

lines = [topLine; '\hdashline'; xDotLine1; xDotLines'; '\hdashline'; zLine1; zLines'; '\hdashline'; yLine1; yLines'];

fprintf( '\\begin{tabular}{%s}\n', colDef )
for i = 1:length(lines)
    disp(lines{i})
end
fprintf( '\\end{tabular}\n' )