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

varFileName   = 'LpeVariables.txt';
implFileName  = 'BlockLocalPositionEstimator.m';
libName       = 'Px4Library';
subFolderName = 'local_position_estimator';

inputFcnName  = 'Input assignment';
outputFcnName = 'Output selection';
memoryName    = 'Self memory';
mainName      = 'local_position_estimator';

fullPath = [ libName '/' subFolderName ];

%% Load and process variables
fileId = fopen ( varFileName );
tmp = textscan( fileId, '%s %d %d %s', 'Delimiter', ' ' );
fclose( fileId );

varNames = tmp{1};
varM = tmp{2};
varN = tmp{3};
varType = tmp{4};

mainDecodeStr  = sprintf('function self = DecodeSelf( selfVecIn )\n\t%% Variable assignment\n');
mainEncodeStr = sprintf(['function selfVecOut = EncodeSelf( self )\n' ...
                         '\t%% Variable assignment\n' ...
                         '\tselfVecOut = zeros(%d, 1);\n'], sum(varM.*varN) );
inputFcnHead = sprintf('function selfVec = InputAssignment( selfVecIn ...\n');
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

inputFcnHead = [ inputFcnHead, sprintf( '\t)\n%%#codegen\n\n' ) ];

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