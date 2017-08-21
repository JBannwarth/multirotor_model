clear all; %#ok<CLALL>

%% 1) Load model and initialize the pool.
model = 'MultirotorSimulation';

if bdIsLoaded('MultirotorSimulation')
    close_system(model, 0);
end

load_system(model);

apool = gcp('nocreate');
if isempty(apool)
    apool = parpool('local');
end

%% 2) Set up the iterations that we want to compute.
motorModel1 = get_param('MultirotorSimulation/Motor model','Modelname');
motorModels = {motorModel1, 'MotorModelTest'};
iterations          = 2;
simout(iterations)  = Simulink.SimulationOutput;

%% 3) Switch workers to tempdir
% In case any code is generated for instance for StateFlow, or any other 
% file artifacts are created by the model.
% model=model;
spmd
    % Setup tempdir and cd into it
    currDir = pwd;
    addpath(currDir);
    tmpDir = tempname;
    mkdir(tmpDir);
    cd(tmpDir);
    disp('blup')
    % Load the model on the worker
    load_system(model);
end

%% 4) Loop over the number of iterations
parfor idx=1:iterations
    disp('blop')
    set_param([model '/Motor model'],'Modelname',motorModels{idx});
    simout(idx) = sim(model, 'SimulationMode', 'normal');
    
    testStruct = struct('Motormodel', motorModels{idx});
    
    simout(idx) = simout(idx).setUserData(testStruct);
end

%% 5) Switch all of the workers back to their original folder.
spmd
    cd(currDir);
    rmdir(tmpDir,'s');
    rmpath(currDir);
    close_system(model, 0);
end

close_system(model, 0);
delete(gcp('nocreate'));