/*
 * SolClustImp.h
 *
 * Created by:	Jonathan Diller
 * On: 			02/01/2023
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


#define DEBUG_SLCLIMP	DEBUG || 0


class SolClustImp : public Solver {
public:
	SolClustImp(bool improvement);
	SolClustImp(const Solver &s);

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
	// Improve the found tour
	void improveRoute(std::vector<std::list<UAV_Stop>>& vTours);
	// Determine the cost of the tour given in tour, a list of UAV stops
	double tourDist(std::list<UAV_Stop> &tour);
	// Find points where the two comms radii overlap
	void findOverlapPoints(Node* v, Node* u, Roots* x_roots, Roots* y_roots);

	Solution* m_pSolution;
	std::vector<HoverLocation> m_vPotentialHL;
	std::vector<std::list<int>> m_vSPerHL;
	std::vector<std::list<int>> m_vHLPerS;
	bool m_bRunImprovement;

};

