/*
 * PathTSP_MIP_PathPlanner.h
 *
 * Created by:	Jonathan Diller
 * On: 			Mar 14, 2022
 *
 * Description: Solves a Path-TSP in each partition using a MIP on Gurobi with lazy
 * constraints for sub-tour elimination.
 */

#pragma once

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <vector>

#include "gurobi_c++.h"
#include "defines.h"
#include "Node.h"

#define DEBUG_MIPTSP		1 || DEBUG

class Subtour {
public:
	Subtour();
	virtual ~Subtour();

	void findsubtour(int n, double** sol, int* tourlenP, int* tour);
};

class SubtourElim : public GRBCallback {
public:
	SubtourElim(GRBVar** xvars, int xn);
	virtual ~SubtourElim();

	GRBVar** vars;
	int n;

protected:
	Subtour sb;

	void callback();
};


class PathTSP_MIP_PathPlanner {
public:
	PathTSP_MIP_PathPlanner();
	virtual ~PathTSP_MIP_PathPlanner();
	/*
	 * Runs the Path-TSP MIP solver for each partition in solution
	 */
	void RunAlgorithm(std::vector<Node> &tourNodes, int* tour);

private:
	// Takes in an integer and returns a string of said integer
	std::string itos(int i);
};
