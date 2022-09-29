//
// Created by peter on 9/28/2022.
//

#ifndef MINLATENCYWSN_BASICMILP_H
#define MINLATENCYWSN_BASICMILP_H

#include "Solver.h"


class SolBasicMILP: public Solver{

public:
    SolBasicMILP(unsigned long int V, unsigned long int K, bool MIN_MAX, bool INITIAL_SOLUTION, bool PRIORITIES, bool CLIQUE_CUTS);

    SolBasicMILP(const Solver &s);

    void solve(Graph* G, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL,
               std::vector<std::list<int>> &vHLPerS, std::vector<std::list<UAV_Stop>> &vTours);

};


#endif //MINLATENCYWSN_BASICMILP_H
