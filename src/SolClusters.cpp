//
// Created by peter on 10/5/2022.
//

#include "SolClusters.h"

SolClusters::SolClusters() {}

SolClusters::SolClusters(const Solver &s): Solver(s) {}

void SolClusters::solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS){
	//TODO Run clustering algorithm


	//TODO Solve TSP on each cluster
}
