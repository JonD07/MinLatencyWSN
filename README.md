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

Note: copy the LKH executable, NOT the LKH directory.

```
cp LKH-<VERSION>/LKH ~/bin
```

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

`../build/min-lat <testfile name> <energy budget> [algorithm] [number of UAVs] [node density]`

### Arguments

- Input File: 

    The test file defines the nodes to visit and the base station. These files are formatted as follows.

    ```
    [Number of Nodes]
    [Node X] [Node Y] [Connection Radius] [???] #I don't know what the 1's are??
    .
    .
    .
    [Base Station X] [Base Station Y]
    ```
- Energy Budget:

    The Energy budget defines the percentage of the total theoretical energy capacity of the UAVs. This total theoretical energy capacity is defined in `inc/defines.h` as `Q`. The Energy budget should be a floating point number between 0 and 1 (although budgets larger than 1 should still work).

- Algorithm:

    The Algorithm parameter defines which algorithm (and combination of algorithms) the solver uses to find the solution. Shown below are the available algorithms. Default of 1.

    | Alg Code      | Alg Number |
    | :----:        |    :----:   |
    | ALG_COMBO_AC_MILP_I |	0 |
    | ALG_COMBO_AC_NN_I	| 1 |
    | ALG_COMBO_AN_CL_NI | 2 |
    | ALG_COMBO_AN_CL_I	| 3 |
    | ALG_COMBO_AN_DG_NI |	4 |
    | ALG_COMBO_AN_DG_I	| 5 |
    | ALG_COMBO_AC_CL_NI | 6 |
    | ALG_COMBO_AC_CL_I	| 7 |
    | ALG_COMBO_AC_DG_NI | 8 |
    | ALG_COMBO_AC_DG_I	| 9 |
    | ALG_COMBO_AN_CL_NI_SC | 10 |
    | ALG_COMBO_AN_CL_I_SC | 11 |

    The algorithm codes follow the following format.

    * Select Hovering Locations:
        * AN - above node only
        * AC - all combos

    * Algorithm:
        * MILP - as advertised (not recommended)
        * NN   - nearest neighbor algorithm
        * CL   - clustering algorithm
        * DG   - divide-greedy algorithm

    * Post processing:
        * I  - improve tour
        * NI - do not improve tour

- Number of UAVs:

    The number of UAVs available for the algorithm. Default of 2.

- Node Density:
    
    The density of the nodes. Default is 150. I'm unsure how this is calculated.



## Output

In `inc/defines.h` there are several parameters to set what output the program should produce. 

### Print Results

This parameter produces a file that logs metadata from each run. The log file by default is stored in `alg_<algorithm>.dat`. Each line of the log file represents a single run of the algorithm, and is formatted as follows.
number of nodes, number of UAVs, node density, total duration, max latency, computation time
```
[Nodes] [UAVs] [Density] [Total Duration] [Max Latency] [Computation Time]
```

### Make Plot file

This parameter produces two files, by defaule `output_path.txt` and `output_graph.txt`.

`output_path.txt` defines the coordinates of the path all UAVs should take formatted as a coordinate pair. This file does not differentiate between UAVs and prints them as a single path.

`output_graph.txt` defines the input graph. This is effectively the same as the input file. It is formatted as follows.

```
[Number of Nodes]
[Node X] [Node Y] [Connection Radius] [???] #I don't know what the 1's are??
.
.
.
[Base Station X] [Base Station Y]
```

### Make Plan File
This parameter produces one or more `drone_[UAV Number]_[Route Number].pln` files. These are route files designed to work with the [DroNS-3 Repository](https://github.com/pervasive-computing-systems-group/DroNS3). Additional information on how these files are formatted can be found there.

