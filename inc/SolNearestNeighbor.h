//
// Created by peter on 9/28/2022.
//

#ifndef MINLATENCYWSN_SOLNEARESTNEIGHBOR_H
#define MINLATENCYWSN_SOLNEARESTNEIGHBOR_H

#include "Solver.h"

class SolNearestNeighbor : public Solver {
public:

    SolNearestNeighbor(unsigned long int V, unsigned long int K, bool MIN_MAX, bool INITIAL_SOLUTION, bool PRIORITIES, bool CLIQUE_CUTS);
    SolNearestNeighbor(const Solver &s);

    void solve(Graph* G, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL,
               std::vector<std::list<int>> &vHLPerS, std::vector<std::list<UAV_Stop>> &vTours);


};


#endif //MINLATENCYWSN_SOLNEARESTNEIGHBOR_H
