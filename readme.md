# Multirotor Simulation Model
Created:      J.X.J. Bannwarth, 2016/06/30
Last updated: J.X.J. Bannwarth, 2024/12/11 (added citation information and made the repository public)

## How to Cite

If you find this repository useful in your own work, please cite my PhD thesis:

- Bannwarth, Jérémie Xavier Joseph. “Aerodynamic Modelling and Wind Disturbance Rejection of Multirotor Unmanned Aerial Vehicles.” PhD Thesis, University of Auckland, 2022. https://hdl.handle.net/2292/58817.

## Introduction

This repository contains a number of multirotor models that I developed as part of my PhD.

The main simulation model is `MultirotorSimPx4`, in the `models` folder. It implements a multirotor dynamic model as well as a copy of the PX4 flight controller.

In order to make the model better encapsulated, the drag and thrust models are loaded from separate simulink files, also stored in the `models` folder.
Likewise, the quaternion blocks and PX4-related blocks are loaded from Simulink libraries stored in the `libraries` folder.
Simulink does have inbuilt quaternion blocks in the Aerospace toolbox. However, that toolbox was not available on the default MATLAB installation at the University of Auckland at the time of this model's creation. Hence, the this custom library was written.

Before using any of the models, make sure to open the Simulink Project file, `MultirotorModel.prj`. It will add all the necessary folders to the path.

## How to Run

To run a position hold simulation, run the following command to first open the `RunPositionHold` script and examine its contents.

```matlab
open RunPositionHold.m
```

This file runs a batch simulation, so if you just want to perform one run, put a breakpoint before Section 7 on line 106.

The simulation parameters can be changed by creating the following variables before running the simulation model:

- `uavType`: either `'octa_x'` (default) or `'quad_x'`. Determines whether octocopter or quadcopter parameters are loaded.
- `canted`: either `true` (default) or `false`. Determines wether the octocopter's rotors are canted or not. Has no effect if `uavType` is set to `quad_x`.

## Code structure

### Initialisation

For a full example of initialisation, take a look at `RunPositionHold` in the `scripts_simulation` folder. The UAV parameters are loaded by running either of the following two blocks of code:

```matlab
%% For quadcopter
model = 'MultirotorSimPx4';
load_system(model);
[Uav, Motor, Aero, Initial] = InitializeParametersQuadcopter( );
Simulation = InitializeModel( model, Initial, tEnd );
```

```matlab
%% For canted rotor octocopter with 31 degree cant
model = 'MultirotorSimPx4';
load_system(model);
[Uav, Motor, Aero, Initial] = InitializeParametersOctocopter( 31 );
Simulation = InitializeModel( model, Initial, tEnd );
```

`InitializeParametersQuadcopter()` and `InitializeParametersOctocopter()` load the parameters, while `InitializeModel()` sets up simulation parameters such as sampling time. Take a look at these functions to better understand how they work.

## Simulink Color Coding

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

## License

MIT License

Copyright (c) 2024 Jérémie Bannwarth

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
