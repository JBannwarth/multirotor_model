%CREATELPESIMULINKMODEL Convert .m file to simulink model
%   The function automatically adds the necessary code to handle the LPE's
%   'self' structure. This structure contains data that would be normally
%   stored as class variables of the LPE object in the original C++
%   implementation. For example, any variable starting with '_' (e.g. '_x')
%   is a class variable in the original implementation.
%
%   In this Simulink implementation, the bulk of the code is contained
%   within a MATLAB function block. Within that block, the aforementioned
%   class variables are instead part of a structure called self. E.g. '_x'
%   becomes 'self.x'.
%
%   The issue with stuctures in Simulink is that they can't be the input or
%   the output of a MATLAB function block without being fully defined,
%   which takes time and populates the MATLAB workspace with a ton of extra
%   variables. However, since MATLAB function blocks do not have memory, it
%   is necessary to somehow output the 'self' structure, make it go through
%   a memory block, and have it loop back as an input.
%
%   This issue is solved by essentially converting the 'self' structure to
%   a very long vector containing all the data at the output, and then
%   convert the vector back to the structure at the input. Due to the large
%   number of variables in the 'self' structure, this would be painstaking
%   to perform manually (even more so since changing a single variable
%   would involve a lot of necessary work).
%
%   As a consequence, the following code automatically generate the code to
%   'encode' and 'decode' the 'self' structure, and automatically updates
%   the MATLAB function blocks in the Px4Library
%
%   Written by:    J.X.J. Bannwarth, 2017/07/05
%   Last modified: J.X.J. Bannwarth, 2017/07/05

%% Cleanup & initial setup
clear; clc;

paramFileName = 'Params.txt';
varFileName   = 'LpeVariables.txt';
implFileName  = 'BlockLocalPositionEstimator.m';
libName       = 'Px4Library';
subFolderName = 'local_position_estimator';

inputFcnName  = 'Input assignment';
outputFcnName = 'Output selection';
memoryName    = 'Self memory';
mainName      = 'local_position_estimator';

fullPath = [ libName '/' subFolderName ];


%% Clean-up blocks and lines

allblocks = find_system( fullPath, 'SearchDepth', 1, 'LookUnderMasks', 'on' );

ToKeep = { [fullPath '/' inputFcnName], [fullPath '/' outputFcnName], ...
    [fullPath '/' memoryName], [fullPath '/' mainName] };

ToDelete = setdiff(allblocks, ToKeep);
ToDelete = setxor(ToDelete, fullPath);

for i = 1:numel( ToDelete )
    delete_block( ToDelete{i} )
end

allLines = find_system( fullPath, 'SearchDepth', 1, 'LookUnderMasks', 'on', ...
    'FindAll', 'on', 'type', 'line' );

delete_line( allLines )

%% Load and process variables
fileId = fopen ( varFileName );
tmp = textscan( fileId, '%s %d %d %s %s %s', 'Delimiter', ' ' );
fclose( fileId );

varNames = tmp{1};
varM = tmp{2};
varN = tmp{3};
varType = tmp{4};
varInit = tmp{5};

mainDecodeStr  = sprintf('function self = DecodeSelf( selfVecIn )\n\t%% Variable assignment\n');
mainEncodeStr = sprintf(['function selfVecOut = EncodeSelf( self )\n' ...
                         '\t%% Variable assignment\n' ...
                         '\tselfVecOut = zeros(%d, 1);\n'], sum(varM.*varN) );
inputFcnHead = sprintf('function selfVec = InputAssignment( ...\n');
inputFcnBody = sprintf('\tselfVec = selfVecIn;\n\n\t%% Variable assignment\n');

outputFcnHead = sprintf('function [ ...\n');
outputFcnBody = sprintf('\t%% Variable assignment\n');

currIndex = 1;

for i = 1:length( varM )
  
    startIndex = currIndex;
    endIndex   = currIndex + varM(i) * varN(i) - 1;
    
    % Check if variable is an input
    if strcmp( 'in', varType{i} )
        inputName = strrep( varNames{i}(6:end), '.', '_' );
        inputFcnHead = [ inputFcnHead sprintf('\t% 20s, ...\n', inputName) ];
        inputFcnBody = [ inputFcnBody sprintf('\tselfVec(% 2d:% 2d) = % 20s;\n', ...
                startIndex, endIndex, inputName ) ];
    end
    
    if strcmp( 'out', varType{i} )
        outputName = strrep( varNames{i}(6:end), '.', '_' );        
        outputFcnHead = [ outputFcnHead sprintf('\t% 20s, ...\n', outputName) ];
        outputFcnBody = [ outputFcnBody sprintf('\t% 20s = selfVec(% 2d:% 2d);\n', ...
                outputName, startIndex, endIndex ) ];
    end
    
    
    % Main function
    
    % Preallocate vectors/matrices
    if ( ( varM(i) > 1 ) || ( varN(i) > 1 ) )
        mainDecodeStr = [ mainDecodeStr sprintf('\t% 20s = zeros(% 2d,% 2d);\n', ...
                varNames{i}, varM(i), varN(i) ) ];
    end
    
    for m = 1:varM(i)
        for n = 1:varN(i)
            % Main input: self.VAR_NAME = inputVec(currIndex)
            mainDecodeStr = [ mainDecodeStr sprintf('\t% 20s(% 2d,% 2d) = selfVecIn(% 3d);\n', ...
                varNames{i}, m, n, currIndex ) ];
            
            % Main output: outputVec(currIndex) = self.VAR_NAME
            mainEncodeStr = [mainEncodeStr sprintf('\tselfVecOut(% 3d) = % 20s(% 2d,% 2d);\n', ...
                currIndex, varNames{i}, m, n ) ];
            currIndex = currIndex + 1;
        end
    end
    
end

inputFcnHead = [ inputFcnHead, sprintf( '\t% 20s ...\n\t)\n%%#codegen\n\n', 'selfVecIn' ) ];

outputFcnHead = [ outputFcnHead, sprintf( '\t] = OutputSelection( selfVec )\n%%#codegen\n\n' ) ];

inputFcnCode = [ inputFcnHead, inputFcnBody, 'end' ];
outputFcnCode = [ outputFcnHead, outputFcnBody, 'end' ];

mainDecodeStr = [ mainDecodeStr, sprintf('end\n\n') ];
mainEncodeStr = [ mainEncodeStr, sprintf('end\n\n') ];

%% Format main function
implementationCode = fileread( implFileName );

mainFcnCode = sprintf( [ ...
        'function selfVecOut  = lpeMain( selfVecIn )\n' ...
        '%%#codegen\n' ...
        '\tself = DecodeSelf( selfVecIn );\n' ...
        '\tself = LocalPositionEstimator( self );\n' ...
        '\tselfVecOut = EncodeSelf( self );\n' ...
        'end\n\n' ...
    ] );

mainFcnCode = [ mainFcnCode mainDecodeStr mainEncodeStr implementationCode ];

%% Write functions to Simulink model
S = sfroot;
inFcn   = S.find('Name', inputFcnName, '-isa', 'Stateflow.EMChart' );
outFcn  = S.find('Name', outputFcnName, '-isa', 'Stateflow.EMChart' );
mainFcn = S.find('Name', mainName, '-isa', 'Stateflow.EMChart' );

inFcn.Script   = inputFcnCode;
outFcn.Script  = outputFcnCode;
mainFcn.Script = mainFcnCode;

%% Create I/O and link-up blocks

% Set length of output function based on number of inputs
varNamesIn = varNames( strcmp( varType, 'in' ) );
varNamesOut = varNames( strcmp( varType, 'out' ) );

fcnW = 150;
spacing = 50;

set_param( [fullPath '/' mainName], 'Position', [-fcnW/2, -25, fcnW/2, 25] );
set_param( [fullPath '/' inputFcnName], 'Position', ...
    [-fcnW/2-spacing-fcnW, ...
     -25*max(length(varNamesIn)+1,1), ...
     -fcnW/2-spacing, ...
     25*max(length(varNamesIn)+1,1)] );
set_param( [fullPath '/' outputFcnName], 'Position', ...
    [fcnW/2+spacing, ...
     -25*max(length(varNamesOut),1), ...
     fcnW/2+spacing+fcnW, ...
     25*max(length(varNamesOut),1)] );
 
memYOffset = 25*max(length(varNamesIn)+1,1) + spacing;
 
set_param( [fullPath '/' memoryName], 'Position', ...
    [-15, memYOffset, 15, memYOffset+30] );

% Re-add necessary lines
add_line( fullPath, [ inputFcnName '/1' ], [ mainName '/1' ] )
add_line( fullPath, [ mainName '/1' ],     [ outputFcnName '/1' ] )
add_line( fullPath, [ mainName '/1' ],     [ memoryName '/1' ], 'autorouting', 'on' )
add_line( fullPath, [ memoryName '/1' ], ...
    [ inputFcnName '/' num2str( length(varNamesIn)+1 ) ], 'autorouting', 'on' )

for i = 1:length( varNamesIn )
    inputName = strrep( varNamesIn{i}(6:end), '.', '_' );
    add_block('simulink/Sources/In1', [fullPath '/' inputName] )
    
    blockLoc = get_param( [fullPath '/' inputName] , 'Position');
    w = blockLoc(3) - blockLoc(1);
    h = blockLoc(4) - blockLoc(2);
    
    % Get offsets
    ports = get_param( [fullPath '/' inputFcnName], 'PortConnectivity');
    portLocs = [ports.Position];
    xOffset = portLocs( ((i-1)*2)+1 ) - spacing - w;
    yOffset  = portLocs( ((i-1)*2)+2 ) - h/2;
    
    blockLoc2(1) = xOffset;
    blockLoc2(2) = yOffset;
    blockLoc2(3) = xOffset + w;
    blockLoc2(4) = yOffset + h;
    
    set_param( [fullPath '/' inputName] , 'Position', blockLoc2);

    add_line( fullPath, [ inputName  '/1' ], ...
        [ inputFcnName '/' num2str(i) ] )
end


% set_param( libName, 'showgrid','on')

for i = 1:length( varNamesOut )
    outputName = strrep( varNamesOut{i}(6:end), '.', '_' );
    add_block('simulink/Sinks/Out1', [fullPath '/' outputName] )
    blockLoc = get_param( [fullPath '/' outputName] , 'Position');
    w = blockLoc(3) - blockLoc(1);
    h = blockLoc(4) - blockLoc(2);
    
    % Get offsets
    ports = get_param( [fullPath '/' outputFcnName], 'PortConnectivity');
    portLocs = [ports.Position];
    portLocs = portLocs(3:end); % First two elements are input
    xOffset = portLocs( ((i-1)*2)+1 ) + spacing;
    yOffset  = portLocs( ((i-1)*2)+2 ) - h/2;
    
    blockLoc2(1) = xOffset;
    blockLoc2(2) = yOffset;
    blockLoc2(3) = xOffset + w;
    blockLoc2(4) = yOffset + h;
    
    set_param( [fullPath '/' outputName] , 'Position', blockLoc2);
    add_line( fullPath, [ outputFcnName '/' num2str(i) ], ...
        [ outputName  '/1' ] )
end

%% Generate memory block initial conditions

initVector = '[';

currIndex = 1;
for i = 1:length( varM )
    
    for m = 1:varM(i)
        for n = 1:varN(i)
            if strncmp( varInit{i}, '[', 1 )
                initMat = str2num( varInit{i} );
                initVal = num2str( initMat(m, n) );
            else
                initVal = varInit{i};
            end
            initVector = [ initVector sprintf('%s, ', initVal ) ];
        end
    end
    
end

initVector = [initVector(1:end-2), ']']; % remove the last ', '

set_param('Px4Library/local_position_estimator/Self memory', 'X0', initVector)

%% Load-up parameters from file
fileId = fopen ( 'params.txt' );
tmp = textscan( fileId, '%s %f %f %f %s %s', 'Delimiter', ';' );
fclose( fileId );

pName = tmp{1};
pVal = tmp{2};
pMin = tmp{3};
pMax = tmp{4};
pUnit = tmp{5};
pInfo = tmp{6};

maskObj = Simulink.Mask.get('Px4Library/local_position_estimator');
maskObj.removeAllParameters();
maskObj.addParameter('Type', 'edit',   'Prompt', 'Sampling period (s)', ...
    'Name', 'LPE_SAMPLING_PERIOD', 'Value', '0.02');

for i = 1:length( pVal ) 
    maskObj.addParameter('Type', 'slider', 'Prompt', ...
        [pName{i} ' - ' pInfo{i} ' (' pUnit{i} ')'], 'Name', pName{i}, 'Range', ...
        [pMin(i) pMax(i)], 'Value', num2str( pVal(i) ));
end

% Add checkboxes for sensor fusion mask
sensors = { 'Barometer', 'GPS', 'LIDAR', 'Flow', 'Sonar', 'Vision', ...
    'Mocap', 'Land' };

for i = 1:numel( sensors )
    callbackStr = sprintf( ['status = get_param( gcb, ''%s'' );\n' ...
    'if strcmp( status, ''on'' )\n' ...
        '\tlpe = bitor( uint8(str2num(get_param( gcb, ''LPE_FUSION'' ))), uint8(2^%d) );\n' ...
        '\tset_param( gcb, ''LPE_FUSION'', num2str(lpe) );\n' ...
    'else\n' ...
        '\tlpe = bitand( uint8(str2num(get_param( gcb, ''LPE_FUSION'' ))), bitcmp(uint8(2^%d), ''uint8'' ));\n' ...
        '\tset_param( gcb, ''LPE_FUSION'', num2str(lpe) );\n' ...    
    'end'], sensors{i}, i-1, i-1 );

    maskObj.addParameter('Type', 'checkbox', 'Prompt', ...
        [ 'Use ' sensors{i} ], 'Name', sensors{i}, 'Value', 'off', ...
        'Callback', callbackStr );
end