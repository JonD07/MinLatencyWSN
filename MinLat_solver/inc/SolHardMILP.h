//
// Created by peter on 9/28/2022.
//

#pragma once

#include "Solver.h"
#include "SolNearestNeighbor.h"

#define K	4

// Help solve faster?
#define MIN_MAX				false // This is the real objective
#define INITIAL_SOLUTION	true
#define PRIORITIES			false
#define CLIQUE_CUTS			false

class SolHardMILP : public Solver{
public:
	SolHardMILP();
	SolHardMILP(const Solver &s);

protected:
	void solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);
};
