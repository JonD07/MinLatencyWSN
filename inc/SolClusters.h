//
// Created by peter on 10/5/2022.
//

#pragma once
#include "Solver.h"


class SolClusters : public Solver {

public:
	SolClusters();

	SolClusters(const Solver &s);

protected:
	void solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);

};

