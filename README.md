# Minimizing Data Latency with Priority using Mobile Agents

## To Build

### Get LKH Solver

Go to http://webhotel4.ruc.dk/~keld/research/LKH-3/, download latest version
Build the executable, then move it to 
`~/bin`

Add the following line to your .bashrc file:

`export PATH="/home/$USER/bin:$PATH`

### Build solver
Create a new directory from the root directory to run build commands

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

