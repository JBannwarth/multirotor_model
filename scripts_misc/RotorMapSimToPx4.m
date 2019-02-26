function px4 = RotorMapSimToPx4( sim )
%ROTORMAPSIMTOPX4 Map sim rotor order to PX4 rotor order
%   By default handle 4x and 8x frames
%   Simulation ordering is incremental in CCW order (NED frame)
%   See link below for PX4 ordering:
%   Written:       J.X.J. Bannwarth, 2019/02/20
%   Last modified: J.X.J. Bannwarth, 2019/02/26
    switch length(sim)
        case 4
            px4 = sim( [1 3 4 2]  );
        case 8
            px4 = sim( [ 1 5 2 4 8 6 7 3 ] );
        otherwise
            error( 'Unrecognized number of rotors' );
    end
end