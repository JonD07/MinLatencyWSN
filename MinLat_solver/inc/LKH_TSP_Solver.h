/*
 * LKH_TSP_Solver.h
 *
 * Created by:	Peter Hall
 * On: 			10/11/2022
 *
 * Description:
 */

#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <stdlib.h>

#include "HoverLocation.h"
#include "UAV_Stop.h"

#define DEBUG_LKH	DEBUG || 0

class LKH_TSP_Solver {
public:
	LKH_TSP_Solver();
	~LKH_TSP_Solver();
	/*
	 * Runs the LKH heuristic TSP solver on the locations in vLoc and stores the
	 * results in vPath. vPath will be an ordered list of indexes that point to
	 * the locations in vLoc.
	 */
	void Solve_TSP(std::vector<UAV_Stop> &vLoc, std::vector<int> &vPath);

private:
	// Generates the config files to solve TSP on the locations in vLoc
	void write_LKH_Config(std::vector<UAV_Stop> &vLoc);
};

