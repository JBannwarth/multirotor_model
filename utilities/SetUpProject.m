function SetUpProject()
%SetUpProject Configure the environment for this project
%
%   Set up the environment for the current project. This function is set to
%   Run at Startup.

%   Copyright 2011-2014 The MathWorks, Inc.

% Use Simulink Project API to get the current project:
project = simulinkproject;
projectRoot = project.RootFolder;

% Set the location of slprj to be the "work" folder of the current project:
myCacheFolder = fullfile(projectRoot, 'work');
if ~exist(myCacheFolder, 'dir')
    mkdir(myCacheFolder)
end
Simulink.fileGenControl('set', 'CacheFolder', myCacheFolder, ...
   'CodeGenFolder', myCacheFolder);

folders = { ...
    'libraries'         , ...
    'matlab_systems'    , ...
    'models_literature' , ...
    'models_archive'    , ...
    'models_test'       , ...
    'scripts_comparison', ...
    'scripts_literature', ...
    'scripts_quat'      , ...
    'scripts_misc'      , ...
    'scripts_test'      , ...
    'scripts_simulation', ...
    'scripts_plotting'  , ...
    'scripts_data'      , ...
    'scripts_init'      , ...
    'data_misc'         , ...
    'data_validation'   , ...
    'data_wind'         , ...
    'data_results'        ...
    };

% Add subfolders to path
for i = 1:length( folders )
    if ~exist(folders{i},'dir')
        mkdir(folders{i})
    end
    addpath(fullfile(projectRoot, folders{i}))
end

% Change working folder to the "work" folder:
%cd(myCacheFolder);
end