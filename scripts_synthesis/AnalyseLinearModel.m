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

%% Augment state space model with integrators
% Use variables instead of hardcoding states
% Select output states
xiIdx      = find( startsWith( linsys.StateName, 'xi_' ) );
xiDotIdx   = find( startsWith( linsys.StateName, 'xiDot_' ) );
outputIdx = [xiIdx; xiDotIdx];

% Get input indexes - note that there is no function for rearranging the
% inputs in SS object
virThrustIdx = find( contains( linsys.InputName, 'T_a' ) );
horThrustIdx = find( contains( linsys.InputName, 'T_h' ) );
yawIdx       = find( contains( linsys.InputName, 'yaw' ) ); % Not used
windIdx      = find( startsWith( linsys.InputName, 'U' ) );

% Separate into control inputs and disturbance inputs
controlIdx = [virThrustIdx; horThrustIdx];
distIdx    = windIdx;

% Extract state space matrices
A  = linsys.A;
B1 = linsys.B(:, distIdx);
B2 = linsys.B(:, controlIdx);
C  = linsys.C(outputIdx, :);
D21 = linsys.D(outputIdx, distIdx);
D22  = linsys.D(outputIdx, controlIdx);

% Zero extremely small elements to get accurate answers when doing tests
% such as ctrb and obsv
tol = 1e-10;
A(abs(A) < tol)   = 0;
B1(abs(B1) < tol) = 0;
B2(abs(B2) < tol) = 0;
C(abs(C) < tol)   = 0;
D22(abs(D22) < tol)   = 0;

% Sizes
n = size( linsys.A, 1 );
m1 = length( distIdx );
m2 = length( controlIdx );
p = length( outputIdx );

% Add integrator for zero steady state error
A = [ zeros(n+3, 3), [ eye(3, n); A ] ];
B1 = [ zeros(3, m1)  ;
       B1           ];
B2 = [ zeros(3, m2)  ;
       B2           ];
C = [ eye(3, n+3)     ;
      zeros(p, 3), C ];
D21 = [ zeros(3, m1)  ;
        D21          ];
D22 = [ zeros(3, m2)  ;
        D22          ];
D = [D21 D22];

% Create new state space object
G = ss( A, [B1 B2], C, [D21 D22] );
G.InputName = linsys.InputName( [distIdx; controlIdx] ) ;
G.OutputName = [ 'xiInt_x'; 'xiInt_y'; 'xiInt_z';
                 linsys.OutputName( outputIdx ) ];
G.StateName = [ 'xiInt_x'; 'xiInt_y'; 'xiInt_z'; linsys.StateName ];

% Input groups
G.InputGroup.dist = distIdx';
G.InputGroup.controls = controlIdx';

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

%% CL frequency response
% Get transfer function from wind input to desired acceleration
FB = ss( [], [], [], eye(5,5), 'InputName', K.OutputName, 'OutputName', K.OutputName );
T = feedback( series( G, K, 'name' ), FB, 'name', +1 );

% Get magnitude response
[magAx, phaseAx, w] = bode( T('T_ax', 'U_u') );
[magHx, phaseHx] = bode( T('T_hx', 'U_u'), w );
[magAy, phaseAy] = bode( T('T_ay', 'U_v'), w );
[magHy, phaseHy] = bode( T('T_hy', 'U_v'), w );
[magAz, phaseAz] = bode( T('T_az', 'U_w'), w );

mags = [squeeze(magAx) squeeze(magHx) squeeze(magAy) squeeze(magHy) squeeze(magAz) ];
mags = 20*log10(mags);
phases = [squeeze(phaseAx) squeeze(phaseHx) squeeze(phaseAy) squeeze(phaseHy) squeeze(phaseAz) ];

data = [w, mags, phases ];

% Plot to verify
figure( 'name', 'Actuator TF' )
subplot( 2, 1, 1 )
semilogx( w, mags(:,1), w, mags(:,2) )
subplot( 2, 1, 2 )
semilogx( w, phases(:,1), w, phases(:,2) )
axis tight

% Export
fileOut = fullfile( projectRoot, 'work', 'output', 'bode_wind_to_acceleration.csv' );
fid = fopen( fileOut, 'w' );
fprintf( fid, 'w magAx magHx magAy magHy magAz phaseAx phaseHx phaseAy phaseHy phaseAz' );
fclose( fid );
writematrix( data, fileOut, 'WriteMode', 'append', 'Delimiter', ' ' )

%% Helper function
function [wC] = CutoffFrequency( sys )

    [mag,~,w] = bode( sys );
    mag = 20*log10(squeeze(mag));
    
    [magP, idxP] = max( mag );
    
    idxC = find( (mag < magP-3) & ((1:length(mag))' >= idxP ), 1 );
    magC = mag( idxC );
    wC = w( idxC );
end