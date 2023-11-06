/*
 * SolClusters.h
 *
 * Created by:	Peter Hall
 * On: 			10/5/2022
 *
 * Description:
 */

#pragma once

#include <sstream>
#include <fstream>
#include <list>
#include <limits>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "Solver.h"
#include "SolverAlgs.h"
#include "HoverLocation.h"
#include "Utilities.h"
#include "defines.h"
#include "LKH_TSP_Solver.h"
#include "Graph_Theory.h"


#define DEBUG_SLCLUST	DEBUG || 0


class SolClusters : public Solver {
public:
	SolClusters();
	SolClusters(double budget);
	SolClusters(const Solver &s);

protected:
	// Run solver algorithm
	void solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);

private:
	// Determines which hovering locations are most ideal to collect all sensor data (greedy approach)
	void hoverLocationOptimizer(std::vector<HoverLocation> &vSelectedHL);
	/*
	 * Returns the utility yield of stopping at oHL. This considers only nodes that have not been
	 * marked and will remove sensor indexes from m_vSPerHL for sensors that have already been marked
	 */
	int utilityYeildOf(HoverLocation &oHL);
	// Returns true if there are still nodes that have not yet been serviced
	bool nodesNotServiced();
	// Determines the time required to complete this tour
	double timeForSubTour(std::list<UAV_Stop> tour);

	// Determine the cost of this tour
	double tourCost(std::list<UAV_Stop> &tour);

	Solution* m_pSolution;
	std::vector<HoverLocation> m_vPotentialHL;
	std::vector<std::list<int>> m_vSPerHL;
	std::vector<std::list<int>> m_vHLPerS;

};

