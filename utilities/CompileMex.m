function CompileMex()
    currentFolder = pwd;
    project = simulinkproject;
    sourceFolder = fullfile(project.RootFolder,'/ccode');
    cd(sourceFolder);

    sourceFiles = dir('*.cpp');

    for fileToCompile = sourceFiles'
        mex(fileToCompile.name);
    end

    mexFiles = dir('*.mexw64');

    for filetoMove = mexFiles'
        movefile(filetoMove.name, '../work');
    end
    
    cd(currentFolder);
end