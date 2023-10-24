# Holistic Path Planning for Multi-Drone Data Collection

This repository contains the solver for the data-muling problem described in the paper "Holistic Path Planning for Multi-Drone Data Collection". The repository also contains an older version of the autopilot used for simulations but the most current version of this autopilot and details on setting up the simulation have been forked to the [DroNS-3 Repository](https://github.com/pervasive-computing-systems-group/DroNS3).

## Solver Prerequisites

### Install Gurobi
Go to https://www.gurobi.com/academia/academic-program-and-licenses/, download the Linux version then get an academic license. Unpack the downloaded archive, start following the directions found in gurobi951/linux64/docs/quickstart_linux.pdf in the 'Software Installation Guide' section. This should get you started.

#### To Build Gurobi C++ Library
Follow these directions: https://stackoverflow.com/a/48867074

### Get LKH Solver

Go to http://webhotel4.ruc.dk/~keld/research/LKH-3/, download latest version
Build the executable, then move it to 
`~/bin`

Add the following line to your .bashrc file:

`export PATH="/home/$USER/bin:$PATH"`

## To Build

Clone this repository

`git clone https://github.com/JonD07/MinLatencyWSN.git`

### Build solver

We use cmake to build the MinLat-Solver. Cmake uses the `FindGUROBI.cmake` file to find the Gurobi solver library and link it with our solver during the build. You may need to update what Gurobi version you are using in `FindGUROBI.cmake`, specifically on the line that says something like "NAMES gurobi gurobi95". For any Gurobi version 10.0x, this line should be changed to "NAMES gurobi gurobi100". 

Create a new directory from the root directory of the solver to run build commands:

`cd MinLatencyWSN/MinLat_solver`

`mkdir build`

`cd build`

Create make file

`cmake ..`

Run make file

`make`

## To Run

Move into test folder

`cd ../test`

Run sar-solver executable

`../build/min-lat`

