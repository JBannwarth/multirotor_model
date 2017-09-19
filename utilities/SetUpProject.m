function set_up_project()
%set_up_project  Configure the environment for this project
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

% Add subfolders to path
addpath(fullfile(projectRoot, 'ccode'))
addpath(fullfile(projectRoot, 'libraries'))
addpath(fullfile(projectRoot, 'scripts_misc'))
addpath(fullfile(projectRoot, 'scripts_test'))
addpath(fullfile(projectRoot, 'scripts_simulation'))
addpath(fullfile(projectRoot, 'scripts_plotting'))
addpath(fullfile(projectRoot, 'scripts_data'))
addpath(fullfile(projectRoot, 'data_misc'))
addpath(fullfile(projectRoot, 'data_validation'))
addpath(fullfile(projectRoot, 'data_wind'))
addpath(fullfile(projectRoot, 'data_results'))

% Change working folder to the "work" folder:
%cd(myCacheFolder);
end