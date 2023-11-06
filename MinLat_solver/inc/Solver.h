/*
 * Solver.h
 *
 * Created by:	Peter Hall
 * On: 			9/28/2022
 *
 * Description:
 */

#pragma once

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>

#include "HoverLocation.h"
#include "Graph.h"
#include "defines.h"
#include "gurobi_c++.h"
#include "Roots.h"
#include "UAV_Stop.h"
#include "Utilities.h"
#include "Solution.h"

class Solver {
public:
	Solver();
	Solver(double b);
	Solver(const Solver &s);
	virtual ~Solver();

	// Runs the underlying algorithm
	Solution* RunSolver(Graph* pG, int nV, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);
	double budget;

protected:
	virtual void solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS) = 0;
	
};

