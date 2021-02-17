%INITIALIZEHINFOCTO Initialize Octocopter model with H-Inf controller
%
%   Loads simulation parameters to perform manual tests on UAV model with
%   H-infinity controller. Requires HinfGain.mat to have been generated in
%   the work folder by CreateDOFHinfController.
%
%   See also CREATEDOFHINFCONTROLLER,
%   MULTIROTORSIMPX4v1_8FPHTFULLGAINMATRIX, INITIALIZEPX4V1_8FPHTFULLGAIN.
%
%   Written by: 2018/02/06, J.X.J. Bannwarth

clearvars
project = simulinkproject; projectRoot = project.RootFolder;

%% Configuration
% Simulation settings
Simulation.TS_MAX = 0.001;
Simulation.TS_OUT = 0.01;
Simulation.T_END = 30;

%% Set-up model
InitializePx4v1_8FPHTFullGain
set_param( [model '/Fixed wind input'], 'Value', mat2str( ULin ) )
set_param( [model '/Input choice'], 'Value', '2' )