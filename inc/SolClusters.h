//
// Created by peter on 10/5/2022.
//

#pragma once

#include <sstream>
#include <fstream>
#include <list>

#include "Solver.h"
#include "HoverLocation.h"
#include "Utilities.h"
#include "defines.h"
#include "LKH_TSP_Solver.h"


class SolClusters : public Solver {

public:
	SolClusters();

	SolClusters(const Solver &s);



protected:
	void solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);

	std::vector<HoverLocation> hoverLocationOptimizer(std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);

};

