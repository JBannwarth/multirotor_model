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
varFileName   = 'QVariables.txt';
implFileName  = 'attitude_estimator_q_main.m';
libName       = 'Px4Library';
subFolderName = 'attitude_estimator_q';

inputFcnName  = 'Input assignment q';
outputFcnName = 'Output selection q';
memoryName    = 'Self memory q';
mainName      = 'attitude_estimator_q';

fullPath = [ libName '/' subFolderName ];


%% Clean-up blocks and lines

% Load library
if ~bdIsLoaded( libName )
    load_system( libName )
end

% Unlock library
if strcmp( get_param( libName, 'Lock' ), 'on' )
    set_param( libName, 'Lock', 'off' );
end

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
tmp = textscan( fileId, '%s %d %d %s %s %s %s', 'Delimiter', ' ' );
fclose( fileId );

varNames = tmp{1};
varM = tmp{2};
varN = tmp{3};
varIO = tmp{4};
varParam = tmp{5};
varInit = tmp{6};
varDataType = tmp{7};

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
    if strcmp( 'in', varIO{i} )
        inputName = strrep( varNames{i}(1:end), '.', '_' );
        inputFcnHead = [ inputFcnHead sprintf('\t% 40s, ...\n', inputName) ];
        inputFcnBody = [ inputFcnBody sprintf('\tselfVec(%2d:%2d) = % 40s;\n', ...
                startIndex, endIndex, inputName ) ];
    end
    
    if strcmp( 'out', varIO{i} )
        outputName = strrep( varNames{i}(1:end), '.', '_' );        
        outputFcnHead = [ outputFcnHead sprintf('\t% 40s, ...\n', outputName) ];
        outputFcnBody = [ outputFcnBody sprintf('\t% 40s = selfVec(%2d:%2d);\n', ...
                outputName, startIndex, endIndex ) ];
    end
    
    
    % Main function
    
    % Preallocate vectors/matrices
    if ( ( varM(i) > 1 ) || ( varN(i) > 1 ) )
        mainDecodeStr = [ mainDecodeStr sprintf('\t% 40s        = zeros(%2d,%2d);\n', ...
                ['self.' varNames{i}], varM(i), varN(i) ) ];
    end
    
    for m = 1:varM(i)
        for n = 1:varN(i)
            if ~strcmp( varDataType{i}, 'double' )
                rhIn = [varDataType{i} '(selfVecIn(%3d));\n'];
                rhOut = 'double(% 33s(%2d,%2d));\n';
            else
                rhIn = 'selfVecIn(%3d);\n';
                rhOut = '% 40s(%2d,%2d);\n';
            end
            
            if ( max([varM(i),varN(i)]) == 1 )
                lhIn = '% 40s       ';
            else
                lhIn = ['% 40s' sprintf('(%2d,%2d)', m, n )];
            end
            
            lhOut = 'selfVecOut(%3d)';

            strRawIn = [ '\t' lhIn ' = ' rhIn ];
            strRawOut = [ '\t' lhOut ' = ' rhOut ];
            
            % Main input: self.VAR_NAME = inputVec(currIndex)
            mainDecodeStr = [ mainDecodeStr sprintf( strRawIn, ...
                ['self.' varNames{i}], currIndex ) ];

            % Main output: outputVec(currIndex) = self.VAR_NAME
            mainEncodeStr = [mainEncodeStr sprintf( strRawOut, ...
                currIndex, ['self.' varNames{i}], m, n ) ];
            currIndex = currIndex + 1;
        end
    end
    
end

inputFcnHead = [ inputFcnHead, sprintf( '\t% 40s ...\n\t)\n%%#codegen\n\n', 'selfVecIn' ) ];

outputFcnHead = [ outputFcnHead, sprintf( '\t] = OutputSelection( selfVec )\n%%#codegen\n\n' ) ];

inputFcnCode = [ inputFcnHead, inputFcnBody, 'end' ];
outputFcnCode = [ outputFcnHead, outputFcnBody, 'end' ];

mainDecodeStr = [ mainDecodeStr, sprintf('end\n\n') ];
mainEncodeStr = [ mainEncodeStr, sprintf('end\n\n') ];

%% Format main function
implementationCode = fileread( implFileName );

mainFcnCode = sprintf( [ ...
        'function selfVecOut  = qMain( time, selfVecIn, Q_SAMPLING_PERIOD )\n' ...
        '%%#codegen\n' ...
        '\tself = DecodeSelf( selfVecIn );\n' ...
        '\tself = AttitudeEstimatorQ( self, time*1e6 );\n' ...
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
if ( length(mainFcn.Inputs) == 3 )
    set(mainFcn.Inputs(3), 'Scope', 'Parameter')
end

%% Create I/O and link-up blocks

% Set length of output function based on number of inputs
varNamesIn = varNames( strcmp( varIO, 'in' ) );
varNamesOut = varNames( strcmp( varIO, 'out' ) );

fcnW = 200;
spacing = 70;

set_param( [fullPath '/' mainName], 'Position', [-fcnW/2, -25, fcnW/2, 25] );

portsMain = get_param( [fullPath '/' mainName], 'PortConnectivity');
portMainLocs = [portsMain.Position];
yOffIn = portMainLocs(4);
yOffOut = portMainLocs(6);

set_param( [fullPath '/' inputFcnName], 'Position', ...
    [-fcnW/2-spacing-fcnW, ...
     -25*max(length(varNamesIn)+1,1) + yOffIn, ...
     -fcnW/2-spacing, ...
     25*max(length(varNamesIn)+1,1) + yOffIn] );
set_param( [fullPath '/' outputFcnName], 'Position', ...
    [fcnW/2+spacing, ...
     -25*max(length(varNamesOut),1) + yOffOut, ...
     fcnW/2+spacing+fcnW, ...
     25*max(length(varNamesOut),1) + yOffOut] );
 
memYOffset = 25*max(length(varNamesIn)+1,1) + spacing;
 
set_param( [fullPath '/' memoryName], 'Position', ...
    [-15, memYOffset, 15, memYOffset+30] );

% Re-add necessary lines
add_line( fullPath, [ inputFcnName '/1' ], [ mainName '/2' ] )
add_line( fullPath, [ mainName '/1' ],     [ outputFcnName '/1' ] )
add_line( fullPath, [ mainName '/1' ],     [ memoryName '/1' ], 'autorouting', 'on' )
add_line( fullPath, [ memoryName '/1' ], ...
    [ inputFcnName '/' num2str( length(varNamesIn)+1 ) ], 'autorouting', 'on' )

% Add timestamp
add_block('simulink/Sources/Clock', [fullPath '/Time'] )
yOffClock = portMainLocs(2);

r = 10;
set_param( [fullPath '/Time'], 'Position', ...
    [-fcnW/2-spacing/2 - r, ...
     -r + yOffClock, ...
     -fcnW/2-spacing/2 + r, ...
     r + yOffClock] );

add_line( fullPath, ['Time/1'], [ mainName '/1' ] )

for i = 1:length( varNamesIn )
    inputName = strrep( varNamesIn{i}(1:end), '.', '_' );
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
    outputName = strrep( varNamesOut{i}(1:end), '.', '_' );
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

%% Load-up parameters from file
fileId = fopen ( paramFileName );
tmp = textscan( fileId, '%s %s %f %f %s %s %s %s', 'Delimiter', ';' );
fclose( fileId );

pName = tmp{1};
pVal = tmp{2};
pMin = tmp{3};
pMax = tmp{4};
pUnit = tmp{5};
pInfo = tmp{6};
pDataType = tmp{7};
pDropDown = tmp{8};

maskObj = Simulink.Mask.get( fullPath );
maskObj.removeAllParameters();
maskObj.addParameter('Type', 'edit', 'Prompt', 'Sampling period (s)', ...
    'Name', 'Q_SAMPLING_PERIOD', 'Value', '0.002');

for i = 1:length( pVal )
    if strcmp( pDropDown{i}, 'free' )
        maskObj.addParameter('Type', 'edit', 'Prompt', ...
            [pName{i} ' - ' pInfo{i} ' (' pUnit{i} ')'], 'Name', pName{i}, ...
            'Value', pVal{i}, 'Evaluate', 'on');
    elseif strcmp( pDataType, 'boolean' ) % Maybe implement this later
        maskObj.addParameter('Type', 'slider', 'Prompt', ...
            [pName{i} ' - ' pInfo{i} ' (' pUnit{i} ')'], 'Name', pName{i}, 'Range', ...
            [pMin(i) pMax(i)], 'Value', pVal{i}, 'Evaluate', 'on');
    elseif ( pDropDown{i} == '-' )
        maskObj.addParameter('Type', 'slider', 'Prompt', ...
            [pName{i} ' - ' pInfo{i} ' (' pUnit{i} ')'], 'Name', pName{i}, 'Range', ...
            [pMin(i) pMax(i)], 'Value', pVal{i}, 'Evaluate', 'on');
    else
        options = strsplit( pDropDown{i}, ',' );
        %options = options(2:2:end);
        optionStr = strjoin( options, ' ' );
        nMax = length(options)/2;
        maskObj.addParameter('Type', 'popup', 'Prompt', ...
            [pName{i} ' - ' pInfo{i} ' (' pUnit{i} '), ' optionStr], 'Name', pName{i}, ...
            'TypeOptions', strread(num2str((1:nMax)-1),'%s') , ...
            'Value', pVal{i}, 'Evaluate', 'on');
    end
end

% % Add checkboxes for sensor fusion mask
% sensors = { 'Barometer', 'GPS', 'LIDAR', 'Flow', 'Sonar', 'Vision', ...
%     'Mocap', 'Land' };
% 
% for i = 1:numel( sensors )
%     callbackStr = sprintf( ['status = get_param( gcb, ''%s'' );\n' ...
%     'if strcmp( status, ''on'' )\n' ...
%         '\tlpe = bitor( uint8(str2num(get_param( gcb, ''LPE_FUSION'' ))), uint8(2^%d) );\n' ...
%         '\tset_param( gcb, ''LPE_FUSION'', num2str(lpe) );\n' ...
%     'else\n' ...
%         '\tlpe = bitand( uint8(str2num(get_param( gcb, ''LPE_FUSION'' ))), bitcmp(uint8(2^%d), ''uint8'' ));\n' ...
%         '\tset_param( gcb, ''LPE_FUSION'', num2str(lpe) );\n' ...    
%     'end'], sensors{i}, i-1, i-1 );
% 
%     maskObj.addParameter('Type', 'checkbox', 'Prompt', ...
%         [ 'Use ' sensors{i} ], 'Name', sensors{i}, 'Value', 'off', ...
%         'Callback', callbackStr );
% end

%% Generate memory block initial conditions
initVector = '[';

currIndex = 1;
for i = 1:length( varM )
    if strncmp( varInit{i}, '[', 1 )
        clear initMat
        tmp = strsplit(varInit{i}(2:end-1), ';' )';
        for j = 1:length( tmp )
            initMat(j,:) = strsplit(tmp{j}, ',');
        end
    end
    
    for m = 1:varM(i)
        for n = 1:varN(i)
            if strncmp( varInit{i}, '[', 1 )
                % initMat = str2num( varInit{i} );
                initVal = initMat{m, n};
            else
                initVal = varInit{i};
            end
            initVector = [ initVector sprintf('%s, ', initVal ) ];
        end
    end
    
end

initVector = [initVector(1:end-2), ']']; % remove the last ', '

set_param( [ fullPath '/' memoryName ], 'X0', initVector)


%% Clean-up
close_system( libName, 1 )
clear
%TestControllerInitOnly