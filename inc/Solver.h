//
// Created by peter on 9/28/2022.
//

#ifndef MINLATENCYWSN_SOLVER_H
#define MINLATENCYWSN_SOLVER_H

#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>

#include "HoverLocation.h"
#include "Graph.h"
#include "defines.h"
#include "gurobi_c++.h"
#include "Roots.h"
#include "UAV_Stop.h"
#include "Utilities.h"

class Solver {
    Graph* G;
    std::vector<HoverLocation> &vPotentialHL;
    std::vector<std::list<int>> &vSPerHL;
    std::vector<std::list<int>> &vHLPerS;
    std::vector<std::list<UAV_Stop>> &vTours;
    // TODO: Adjust as needed
    unsigned long int V;
    unsigned long int K;

    // Help solve faster?
    bool MIN_MAX;	// This is the real objective
    bool INITIAL_SOLUTION;
    bool PRIORITIES;
    bool CLIQUE_CUTS;

public:
    Solver(Graph* G, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL,
           std::vector<std::list<int>> &vHLPerS, std::vector<std::list<UAV_Stop>> &vTours, unsigned long int V,
           unsigned long int K, bool MIN_MAX, bool INITIAL_SOLUTION, bool PRIORITIES, bool CLIQUE_CUTS);

    void runBasicMILP();
    void runNN();
    void runHardMILP();

    void printResults(bool bImproved);

};


#endif //MINLATENCYWSN_SOLVER_H
