%COMPILEMEX Compile all mex files in the C code folder [outdated]
%   Written: 2016, J.X.J. Bannwarth
function CompileMex()
    currentFolder = pwd;
    project = simulinkproject;
    sourceFolder = fullfile(project.RootFolder,'/ccode');
    cd(sourceFolder);

    sourceFiles = dir('*.cpp');
    
    try
        % Compile mex files
        clear functions
        for fileToCompile = sourceFiles'
            mex(fileToCompile.name);
        end
    catch ME
        % Clean up folder if there's a compiler error
        mexFiles = dir('*.mexw64');
        for filetoMove = mexFiles'
            movefile(filetoMove.name, '../work');
        end
        
        cd(currentFolder);
        rethrow(ME)
    end
    
    mexFiles = dir('*.mexw64');
    for filetoMove = mexFiles'
        movefile(filetoMove.name, '../work');
    end

    cd(currentFolder);
end