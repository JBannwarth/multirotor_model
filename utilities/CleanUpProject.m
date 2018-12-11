function clean_up_project()
%clean_up_project   Clean up local customizations of the environment
% 
%   Clean up the environment for the current project. This function undoes
%   the settings applied in "set_up_project". It is set to Run at Shutdown.

%   Copyright 2011-2014 The MathWorks, Inc.

% Remove folders from path
project = simulinkproject;
projectRoot = project.RootFolder;
rmpath(fullfile(projectRoot, 'libraries'))
rmpath(fullfile(projectRoot, 'scripts_misc'))
rmpath(fullfile(projectRoot, 'scripts_test'))
rmpath(fullfile(projectRoot, 'scripts_simulation'))
rmpath(fullfile(projectRoot, 'scripts_plotting'))
rmpath(fullfile(projectRoot, 'scripts_data'))
rmpath(fullfile(projectRoot, 'data_misc'))
rmpath(fullfile(projectRoot, 'data_validation'))
rmpath(fullfile(projectRoot, 'data_wind'))
rmpath(fullfile(projectRoot, 'scripts_quat'))
rmpath(fullfile(projectRoot, 'scripts_literature'))
rmpath(fullfile(projectRoot, 'models_literature'))
rmpath(fullfile(projectRoot, 'models_archive'))
rmpath(fullfile(projectRoot, 'models_test'))

% Reset the location where generated code and other temporary files are
% created (slprj) to the default:
Simulink.fileGenControl('reset');

end
