/*
 * Solution.h
 *
 * Created by:	Jonathan Diller
 * On: 			Mar 13, 2022
 *
 * Description: Graph class
 */

#pragma once

#include <list>
#include <stdio.h>
#include <vector>
#include <sstream>
#include <fstream>

#include "Graph.h"
#include "defines.h"
#include "UAV_Stop.h"
#include "Utilities.h"

#define DEBUG_SOL	DEBUG || 0

class Solution {
public:
	Solution(Graph* G, int nV);
	Solution(const Solution &s);
	~Solution();

	void printResults(double compTime, bool printFile = false, int approach = 99, int nodeDensity=150);
	void printPlan();
	double GetWorstLatency();

	// List of tours. Sub-tours are organized as the order of each UAV
	std::vector<std::list<UAV_Stop>> vTours;
	// Pointer to the graph of this solution
	Graph* m_pG;
	// The number of UAVs
	long unsigned int m_nV;
protected:
};
