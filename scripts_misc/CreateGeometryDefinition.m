%CREATEGEOMETRYDEFINITION Create geometry definition used to compute mixer
%   Output: .toml file in PX4 format
%   See PX4 firmware for an example (retrieved 2019/02/05):
%       https://github.com/PX4/Firmware/blob/master/src/lib/mixer/geometries/quad_x.toml
%   Written:       J.X.J. Bannwarth, 2019/02/05
%   Last modified: J.X.J. Bannwarth, 2019/02/05

% Default properties
CtDef = 1.0;
CmDef = 0.05;
axDef = [0, 0, -1];
key = '8t';

% Canted rotor properties
cantAngle = 10;
names = { 'front_right', 'mid_front_right', 'mid_rear_right', 'rear_right', ...
    'rear_left', 'mid_rear_left', 'mid_front_left', 'front_left' };

intro = sprintf( [ ...
    '[info]\n'     ...
    'key = "%s"\n' ...
    'description = "Octocopter in X configuration with %d degrees canted rotors"\n' ...
    '\n' ...
    '[rotor_default]\n'  ...
    'direction = "CW"\n' ...
    'axis      = [%.1f, %.1f, %.1f]\n' ...
    'Ct        = %.1f\n' ...
    'Cm        = %.2f\n' ...
    ], key, cantAngle, axDef(1), axDef(2), axDef(3), CtDef, CmDef ...
    );

rotorFormat = [  ...
    '[[rotors]]\n' ...
    'name      = "%s"\n' ...
    'position  = [%.6f, %.6f, %.6f]\n' ...
    'axis      = [%.6f, %.6f, %.6f]\n' ...
    'direction = "%s"\n' ...
    ];

for i = 1:8
    beta = 22.5 + (i-1)*45;
    pos = [ cosd(beta), sind(beta), 0 ];
    if rem(i, 2) == 0
        direction = 'CW';
    else
        direction = 'CCW';
    end
    
    switch i
        case 1
            ax = [ 0, -sind(cantAngle), -cosd(cantAngle) ];
        case 2
            ax = [ -sind(cantAngle), 0, -cosd(cantAngle) ];
        case 3
            ax = [ sind(cantAngle), 0,-cosd(cantAngle) ];
        case 4
            ax = [ 0, -sind(cantAngle), -cosd(cantAngle) ];
        case 5
            ax = [ 0, sind(cantAngle), -cosd(cantAngle) ];
        case 6
            ax = [ sind(cantAngle), 0, -cosd(cantAngle) ];
        case 7
            ax = [ -sind(cantAngle), 0, -cosd(cantAngle) ];
        case 8
            ax = [ 0, sind(cantAngle), -cosd(cantAngle) ];
    end
    
    rotorFormats{i} = sprintf( rotorFormat, names{i}, pos(1), pos(2), pos(3), ...
        ax(1), ax(2), ax(3), direction );
end

% Rearrange to fit PX4 order
rotorFormats = rotorFormats( [ 1 5 2 4 8 6 7 3 ] );
rotorFormats{end} = rotorFormats{end}(1:end-1);

text = [ intro newline char(join(rotorFormats, newline)) ];

project = simulinkproject;
projectRoot = project.RootFolder;
fid = fopen( fullfile( projectRoot, 'scripts_test', 'MixerMultirotor', ...
    'geometries', 'octa_cant.toml' ), 'w' );
fprintf( fid, text );
fclose( fid );