%INITIALIZEHINFOCTO Initialize Octocopter model with H-Inf controller
%
%   Loads simulation parameters to perform manual tests on UAV model with
%   H-infinity controller. Requires HinfGain.mat to have been generated in
%   the work folder by CreateDOFHinfController.
%
%   See also CREATEDOFHINFCONTROLLER, INITIALIZEPX4,
%   MULTIROTORSIMPX4v1_8FPHTFULLGAINMATRIX.
%
%   Written by: 2018/02/06, J.X.J. Bannwarth

clearvars
project = simulinkproject; projectRoot = project.RootFolder;

%% Configuration
% Simulation settings
ctrlName = 'FPHTFullGain';
tEnd = 30;

%% Set-up model
[Aero, Ctrl, Initial, model, Motor, Simulation, Uav, windInput, toLoad] = InitializePx4( ctrlName, tEnd );

ULin = Ctrl.WIND_OPERATING;

FindOpPx4

set_param( [model '/Fixed wind input'], 'Value', mat2str( ULin ) )
set_param( [model '/Input choice'], 'Value', '2' )