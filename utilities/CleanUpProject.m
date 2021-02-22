function CleanUpProject()
%CleanUpProject   Clean up local customizations of the environment
% 
%   Clean up the environment for the current project. This function undoes
%   the settings applied in "set_up_project". It is set to Run at Shutdown.

%   Copyright 2011-2014 The MathWorks, Inc.

% Remove folders from path
project = simulinkproject;
projectRoot = project.RootFolder;

folders = { ...
    fullfile('libraries', 'quat')    , ...
    fullfile('libraries', 'format')  , ...
    'libraries'         , ...
    'matlab_systems'    , ...
    'models_literature' , ...
    'models_test'       , ...
    'models_tuning'     , ...
    'scripts_literature', ...
    'models_synthesis'  , ...
    'scripts_comparison', ...
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

for i = 1:length( folders )
    if exist(folders{i},'dir')
        rmpath(fullfile(projectRoot, folders{i}))
    end
end

% Reset the location where generated code and other temporary files are
% created (slprj) to the default:
Simulink.fileGenControl('reset');

end
