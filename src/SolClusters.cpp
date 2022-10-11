//
// Created by peter on 10/5/2022.
//

#include "SolClusters.h"

/*
 * SolClusters is a Solver.
 * Solves graph by first clustering the nodes into subtours, then solving TSP on those clusters.
 * In Development
 */

SolClusters::SolClusters() {}

SolClusters::SolClusters(const Solver &s): Solver(s) {}

void SolClusters::solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS){
	//TODO Find optimal hovering locations
	std::vector<HoverLocation> vHLs = hoverLocationOptimizer(vPotentialHL, vSPerHL, vHLPerS);

	//TODO Run clustering algorithm
	LKH_TSP_Solver o_TSP_Solver(vHLs);

	o_TSP_Solver.Write_LKH_Config();
	o_TSP_Solver.Solve_TSP();

	//TODO Solve TSP on each cluster
}

std::vector<HoverLocation> SolClusters::hoverLocationOptimizer(std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS){

	int nBestHL = -1;
	long unsigned int nBestHLSize = 0;

	for (long unsigned int i = 0; i < vSPerHL.size(); i++){
		if (vSPerHL[i].size() > nBestHLSize){
			nBestHL = i;
			nBestHLSize = vSPerHL[i].size();
		}
		else if(vSPerHL[i].size() == nBestHLSize){
			if (nBestHL == -1){
				nBestHL = i;
				nBestHLSize = vSPerHL[i].size();
			}
			else{

				float nDistance0 = distAtoB(vPotentialHL[0].fX, vPotentialHL[0].fY, vPotentialHL[nBestHL].fX, vPotentialHL[nBestHL].fY);
				float nDistance1 = distAtoB(vPotentialHL[0].fX, vPotentialHL[0].fY, vPotentialHL[i].fX, vPotentialHL[i].fY);

				if(nDistance0 < nDistance1){
					nBestHL = i;
					nBestHLSize = vSPerHL[i].size();
				}

			}
		}
	}
	if(DEBUG) {
		printf("Best Hl is %d with %ld sensors!\n", nBestHL, nBestHLSize);
	}
	std::vector<HoverLocation> vHLs;

	//Base case: vSPerHL is empty

	if(nBestHLSize == 0){
		return vHLs;
	}

	for (long unsigned int i = 0; i < vHLPerS.size(); i++) {

		vHLPerS[i].remove(nBestHL);

	}

	for (long unsigned int i = 0; i<vSPerHL.size(); i++){
		if((long unsigned int) nBestHL != i){
			for (auto hl = vSPerHL[nBestHL].begin(); hl != vSPerHL[nBestHL].end(); hl++){
				vSPerHL[i].remove(*hl);
			}
		}
	}

	vSPerHL[nBestHL].clear();


	std::vector<HoverLocation> vRecHLs = hoverLocationOptimizer(vPotentialHL, vSPerHL, vHLPerS);

	vRecHLs.insert(vRecHLs.begin(), vPotentialHL[nBestHL]);


	return vRecHLs;



}


