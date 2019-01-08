# Multirotor Simulation Model
Created:      J.X.J. Bannwarth, 2016/06/30
Last updated: J.X.J. Bannwarth, 2019/01/08

## To-Do

- [ ] Rewrite the requirements
- [x] Add section on colour coding (2017/01/18)
- [x] Add intro
- [x] Add section about data

## Introduction

This repository contains a number of mulitotor models that I developed as part of my PhD.

The main simulation model is `MultirotorSimPx4`, in the `models` folder. It implements a multirotor dynamic model as well as a copy of the PX4 flight controller.

In order to make the model better encapsulated, the drag and thrust models are loaded from separate simulink files, also stored in the `models` folder.
Likewise, the quaternion blocks and PX4-related blocks are loaded from Simulink libraries stored in the `libraries` folder.
Simulink does have inbuilt quaternion blocks in the Aerospace toolbox, however it is not available on the default MATLAB installation at the University of Auckland, which is why a custom library was written.

Before using any of the models, make sure to open the Simulink Project file, `MultirotorModel.prj`. It will add all the necessary folders to the path.

In order to keep the size of the repository small, big data files are stored in a different location, as mentioned in the Data section of this README.

## Data

### AIAA Journal Paper Data


> [Aerodynamic Force Modeling of Multirotor Unmanned Aerial Vehicles](https://doi.org/10.2514/1.J057165)

> Jérémie X. J. Bannwarth, Z. Jeremy Chen, Karl A. Stol, Bruce A. MacDonald, and Peter J. Richards

> AIAA Journal 0 0:0, 1-10

The data used for above paper is available in the `UAV Wind Project` Google Drive (request access from me), under the following two folders:

- Data for the `data_validation` subfolder: `2018-12-14 All Validation Data for Github`
- Data for the `data_wind` subfolder: `2018-06-07 TurbSim Wind Profiles for AIAA Paper`

If you do not have access to the aforementioned Google Drive, the data is also stored on the University's research drive: `R:\MECH\Wind Disturbance Rejection UAV Project\Experiments`. 

## List of Requirements

Model must:

- Be well encapsulated - each component should be easily replaceable
- Be arbitrarily scalable - it should be easy to change from quadrotor to hexarotor to octorotor configuration (or any other feasible configuration)
- Use quaternions to cover any possible use case
- Include conversion blocks to run with quaternion or Euler controllers
- Run efficiently (ideally perform a ~60s simulation in 2-3 real time s)
- Be able to run in batch simulations
- Include comprehensive data logging and plotting
- Include ways to plot against experimental data

Code must:
- Be written using consistent naming conventions
	- Use underscore style for variables `variable_name`
	- Spell out greek letters `xi`, `omega`, etc.
	- Use `_vec` and `_mat` to denote vectors and matrices
	- Add contact address, name of creator, and date of modification to all files
	- Add paper reference/doi/url to file description for algorithms/models
- Simulink model must be clean and easy to understand
	- Name all lines with appropriate variable names
	- Name all block input/output appropriately
	- Use colour coding to match blocks of similar function
	- Add comment blocks to explain what is happening
- Be well organised in folders
- Be backed up on GitHub and kept up to date

## Color Coding

| Color       | Meaning                            |
| ----------- | ---------------------------------- |
| Grey        | Matlab Functions                   |
| Yellow      | Goto/from                          |
| Cyan        | Inports/outports                   |
| Green       | Source blocks                      |
| Light green | Sinks                              |
| Orange      | Model references                   |
| Light blue  | Subsystems                         |
| Pink        | Quaternion library blocks          |
| Purple      | PX4 library blocks                 |
| Dark green  | Integrators/differentiators/delays |
| White       | Normal blocks                      |