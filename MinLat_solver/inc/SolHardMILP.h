//
// Created by peter on 9/28/2022.
//

#pragma once

#include "Solver.h"
#include "SolNearestNeighbor.h"
#include "SolClusters.h"

#define DEBUG_HMILP			DEBUG || 0
#define PRINT_GAP			1

#define K	4

// Help solve faster?
#define MIN_MAX				true // This is the real objective
#define INITIAL_SOLUTION	true
#define PRIORITIES			true
#define CLIQUE_CUTS			true

class SolHardMILP : public Solver{
public:
	SolHardMILP();
	SolHardMILP(double budget);
	SolHardMILP(const Solver &s);

protected:
	void solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);
};
