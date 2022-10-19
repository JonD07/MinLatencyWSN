//
// Created by peter on 9/28/2022.
//

#pragma once

#include "Solver.h"

class SolNearestNeighbor : public Solver {
public:

	SolNearestNeighbor();
	SolNearestNeighbor(const Solver &s);

protected:
	void solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);

};
