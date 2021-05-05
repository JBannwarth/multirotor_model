function out = ExportController( filename, dt )
%EXPORTCONTROLLER Export state space controller to C++ Px4-Matrix format
%   OUT = EXPORTCONTROLLER( ) exports 'work/HinfGain.mat'
%   OUT = EXPORTCONTROLLER( FILENAME ) exports filename
%   OUT = EXPORTCONTROLLER( FILENAME, DT ) specifies the sampling rate
%
%   See also CREATEDOFHINFCONTROLLER.
%
%   Written: 2021/04/29, J.X.J. Bannwarth
    arguments
        filename (1,:) char   = fullfile('work', 'HinfGain.mat')
        dt       (1,1) double = 1/248 % 248 Hz
    end
    
    %% Load file
    % Add root folder so that this can be called from anywhere as long as
    % the project is open
    project = simulinkproject;
    filename = fullfile( project.RootFolder, filename );
    load( filename, 'K', 'thrustOp' )
    
    %% Prepare controller
    % Discretise the controller
    Kd = c2d( K, dt, 'Tustin' );
    % Need to transpose the matrices to get A(:) to work as expected.
    % A(:) reads down the columns one after the other, but Px4-Matrix
    % is initialised by reading across rows
    A = Kd.A'; B = Kd.B'; C = Kd.C'; D = Kd.D';
    m = size(Kd.A,1);
    n = size(Kd.B,2);
    p = size(Kd.C,1);
    
    
    %% Export controller
    out = sprintf( ['static constexpr uint8_t _ctrl_num_inputs{%d};\n' ...
                    'static constexpr uint8_t _ctrl_num_outputs{%d};\n' ...
                    'static constexpr uint8_t _ctrl_num_states{%d};\n' ], ...
                    n, p, m );
    out = [ out ...
            sprintf('float _data_A[%d] {', m*m) sprintf('%.8ff,', A(:)' ) sprintf('};\n') ...
            sprintf('float _data_B[%d] {', m*n) sprintf('%.8ff,', B(:)' ) sprintf('};\n') ...
            sprintf('float _data_C[%d] {', p*m) sprintf('%.8ff,', C(:)' ) sprintf('};\n') ...
            sprintf('float _data_D[%d] {', p*n) sprintf('%.8ff,', D(:)' ) sprintf('};\n') ...
            sprintf('float _data_op[%d] {', p)  sprintf('%.8ff,', thrustOp(:)' ) sprintf('};\n')];
    out = replace( out, ',}', '}' );
    out = replace( out, {'-0.00000000','0.00000000'}, '0.0' );
end