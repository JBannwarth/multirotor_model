%ANALYSELINEARMODEL Analyse linearised model
%
% Written: 2021/05/21, J.X.J. Bannwarth

clearvars;
project = simulinkproject; projectRoot = project.RootFolder;

%% Configuration
load( fullfile( projectRoot, 'work', 'Octocopter_LinMod_Att'), ...
    'linsys', 'op', 'ULin' )
load( fullfile( projectRoot, 'work', 'HinfGain'), ...
    'K' )

% Change C/D Matrix to output all states
sys = ss( linsys.A, linsys.B, eye(size(linsys.A)), ...
    zeros( size(linsys.A,1), size(linsys.B,2)), ...
    'StateName', linsys.StateName, 'InputName', linsys.InputName, ...
    'OutputName', linsys.StateName );

%% Create pole-zero map
[p, z] = pzmap( linsys );
p = sort(p);

% Export
fidPZ = fopen( fullfile( projectRoot, 'work', 'output', 'pzmap.csv' ), 'w' );
fprintf( fidPZ, 're im\n' );
for ii = 1:length(p)
    fprintf( fidPZ, '%f %f\n', real(p(ii)), imag(p(ii)) );
end
fclose( fidPZ );

%% Look at a couple of transfer functions
% x acceleration to x velocity
[magXA, phaseXA, w] = bode( tf('s')*sys( 'xiDot_x', 'T_ax' ) );
[magXH, phaseXH] = bode( tf('s')*sys( 'xiDot_x', 'T_hx' ), w );

% y acceleration to y velocity
[magYA, phaseYA] = bode( tf('s')*sys( 'xiDot_y', 'T_ay' ), w );
[magYH, phaseYH] = bode( tf('s')*sys( 'xiDot_y', 'T_hy' ), w );

% z acceleration to z velocity
[magZA, phaseZA] = bode( tf('s')*sys( 'xiDot_z', 'T_az' ), w );

% x acceleration to rotor 1
[magXW1A, phaseXW1A] = bode( sys( 'omega_1', 'T_ax' ), w );
[magXW1H, phaseXW1H] = bode( sys( 'omega_1', 'T_hx' ), w );
% x acceleration to rotor 3
[magXW3A, phaseXW3A] = bode( sys( 'omega_3', 'T_ax' ), w );
[magXW3H, phaseXW3H] = bode( sys( 'omega_3', 'T_hx' ), w );

% Save data
fileBode = fullfile( projectRoot, 'work', 'output', 'bode_linear.csv' );
fidBode = fopen( fileBode, 'w' );
fprintf( fidBode, 'w magXA phaseXA magXH phaseXH magYA phaseYA magYH phaseYH magZA phaseZA magXW1A phaseXW1A magXW1H phaseXW1H magXW3A phaseXW3A magXW3H phaseXW3H\n' );
fclose( fidBode );
data = [ w magXA(:) phaseXA(:) magXH(:) phaseXH(:) magYA(:) phaseYA(:) magYH(:) phaseYH(:) magZA(:) phaseZA(:) magXW1A(:) phaseXW1A(:) magXW1H(:) phaseXW1H(:) magXW3A(:) phaseXW3A(:) magXW3H(:) phaseXW3H(:) ];
% Convert mag to dB
data(:,2:2:end) = 20*log10( data(:,2:2:end) );
writematrix( data, fileBode, 'WriteMode', 'append', 'Delimiter', ' ' )

% Display results
fprintf( '\n\n[RESULTS]\n' )
results = [ CutoffFrequency( tf('s')*tf(sys( 'xiDot_x', 'T_ax' ) ));
    CutoffFrequency( tf('s')*sys( 'xiDot_x', 'T_hx' ) );
    CutoffFrequency( tf('s')*sys( 'xiDot_y', 'T_ay' ) );
    CutoffFrequency( tf('s')*sys( 'xiDot_y', 'T_hy' ) );
    CutoffFrequency( tf('s')*sys( 'xiDot_z', 'T_az' ) )
    ];
fprintf( 'fC aAx->xDDot = %.2f rad/s = %.2f Hz\n', results(1), results(1)/(2*pi) )
fprintf( 'fC aHx->xDDot = %.2f rad/s = %.2f Hz\n', results(2), results(2)/(2*pi) )
fprintf( 'fC aAy->yDDot = %.2f rad/s = %.2f Hz\n', results(3), results(3)/(2*pi) )
fprintf( 'fC aHy->yDDot = %.2f rad/s = %.2f Hz\n', results(4), results(4)/(2*pi) )
fprintf( 'fC aAz->zDDot = %.2f rad/s = %.2f Hz\n', results(5), results(5)/(2*pi) )

%% Look at closed-loop system
A2 = [ zeros(3,3) eye(3,length(linsys.A)); zeros(length(linsys.A),3), linsys.A ];
B2 = [zeros(3,5);linsys.B(:,4:8)];
sys2 = ss( A2, B2, eye(9,length(A2)), zeros(9,5), ...
    'StateName', [ 'xiInt_x'; 'xiInt_y'; 'xiInt_z'; linsys.StateName], 'InputName', linsys.InputName(4:8), ...
    'OutputName', [ 'xiInt_x'; 'xiInt_y'; 'xiInt_z'; linsys.StateName(1:6)] );
cltf = feedback( sys2, K, 'name', +1 );

[p, z] = pzmap( cltf );
p = sort(p);

% Export
fidPZCL = fopen( fullfile( projectRoot, 'work', 'output', 'pzmap_cl.csv' ), 'w' );
fprintf( fidPZCL, 're im\n' );
for ii = 1:length(p)
    fprintf( fidPZCL, '%f %f\n', real(p(ii)), imag(p(ii)) );
end
fclose( fidPZCL );

%% Helper function
function [wC] = CutoffFrequency( sys )

    [mag,~,w] = bode( sys );
    mag = 20*log10(squeeze(mag));
    
    [magP, idxP] = max( mag );
    
    idxC = find( (mag < magP-3) & ((1:length(mag))' >= idxP ), 1 );
    magC = mag( idxC );
    wC = w( idxC );
end