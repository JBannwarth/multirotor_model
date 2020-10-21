function px4 = RotorMapPx4ToSim( sim )
%ROTORMAPPX4TOSIM Map px4 rotor order to simulation rotor order
%   By default handle 4x and 8x frames
%   Simulation ordering is incremental in CCW order (NED frame)
%   See link below for PX4 ordering:
%       https://dev.px4.io/en/airframes/airframe_reference.html
%   Written:       J.X.J. Bannwarth, 2019/02/20
%   Last modified: J.X.J. Bannwarth, 2019/02/26
    switch length(sim)
        case 4
            px4 = sim( [1 4 2 3]  );
        case 8
            px4 = sim( [ 1 3 8 4 2 6 7 5 ] );
        otherwise
            error( 'Unrecognized number of rotors' );
    end
end