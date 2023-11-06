/*
 * SolDivideGreedy.h
 *
 * Created by:	Jonathan Diller
 * On: 			Mar 13, 2022
 *
 * Description: Graph class
 */

#pragma once

#include "Solver.h"

#define DEBUG_DIV_GREED		DEBUG || 0

class SolDivideGreedy : public Solver {
public:

	SolDivideGreedy();
	SolDivideGreedy(double budget);
	SolDivideGreedy(const Solver &s);

protected:
	void solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);

};
