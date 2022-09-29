//
// Created by peter on 9/28/2022.
//

#include "../inc/Solver.h"


Solver::Solver(unsigned long int V, unsigned long int K, bool MIN_MAX, bool INITIAL_SOLUTION, bool PRIORITIES, bool CLIQUE_CUTS): V(V),
                K(K), MIN_MAX(MIN_MAX), INITIAL_SOLUTION(INITIAL_SOLUTION), PRIORITIES(PRIORITIES), CLIQUE_CUTS(CLIQUE_CUTS) {}

Solver::Solver(const Solver &s) {
    K = s.K;
    V = s.V;
    MIN_MAX = s.MIN_MAX;
    INITIAL_SOLUTION = s.INITIAL_SOLUTION;
    PRIORITIES = s.PRIORITIES;
    CLIQUE_CUTS =s.CLIQUE_CUTS;

}

Solver::~Solver(){}


void Solver::printResults(std::vector<std::list<UAV_Stop>> &vTours, Graph* G, bool bImproved){
    // Print results to file
    FILE * pOutputFile;
    char buff[100];
    int alg = bImproved ? ALGORITHM + 1 : ALGORITHM;
    sprintf(buff, DATA_LOG_LOCATION, alg);
    printf("%s\n", buff);
    pOutputFile = fopen(buff, "a");

    printf("\nFound tour:\n");
    double total_duration = 0;
    double max_latency = 0;
    double current_v_dur = 0;
    int vehicle = 0;
    for(unsigned long int i = 0; i < vTours.size(); i++) {
        if(vTours.at(i).size() > 2) {
            // Start with initial stop-time
            double duration = stopTime(i%K);
            std::list<UAV_Stop> l = vTours.at(i);
            if(l.size() > 1) {
                std::list<UAV_Stop>::iterator lst, nxt;
                lst = l.begin();
                nxt = l.begin();
                nxt++;

                // Run through the tour, add up distance from stop-to-stop
                while(nxt != l.end()) {
                    // Add time to move from lst to nxt
                    duration += lst -> edgeTime(*nxt);
                    // Add in time to talk to each sensor at nxt
                    for(int s : nxt->nodes) {
                        duration += G->vNodeLst.at(s).sensorTime(*nxt) ;
                    }

                    // Advance iterators
                    lst++;
                    nxt++;
                }
            }
            printf("%d-%ld: duration = %f\n ", vehicle, i%K, duration);

            total_duration += duration;
            current_v_dur += duration;
            for(UAV_Stop n : l) {
                printf(" (%f, %f)", n.fX, n.fY);
            }
            printf("\n");
        }

        if(i%K == K-1) {
            // Last sub-tour for this vehicle
            if(max_latency < current_v_dur) {
                max_latency = current_v_dur;
            }
            // Reset for next vehicle
            current_v_dur = 0;
            vehicle++;
        }
    }

    // Verify that we didn't loop-out early
    if(max_latency < current_v_dur) {
        max_latency = current_v_dur;
    }

    printf("Total duration = %f\n ", total_duration);
    printf("Worst latency = %f\n ", max_latency);
    fprintf(pOutputFile, "%ld %ld %f %f\n", V, G->vNodeLst.size(), total_duration, max_latency);

    fclose(pOutputFile);
}
