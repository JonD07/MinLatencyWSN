//
// Created by peter on 9/28/2022.
//

#include "../inc/SolNearestNeighbor.h"


SolNearestNeighbor::SolNearestNeighbor(unsigned long int V, unsigned long int K, bool MIN_MAX, bool INITIAL_SOLUTION, bool PRIORITIES, bool CLIQUE_CUTS):
        Solver(V, K, MIN_MAX, INITIAL_SOLUTION, PRIORITIES, CLIQUE_CUTS){}
SolNearestNeighbor::SolNearestNeighbor(const Solver &s): Solver(s) {}

void SolNearestNeighbor::solve(Graph* G, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL,
           std::vector<std::list<int>> &vHLPerS, std::vector<std::list<UAV_Stop>> &vTours){
    printf("\nRunning Nearest Neighbor\n");
    // Make a list of hovering locations
    std::list<HoverLocation> hlList;
    for(HoverLocation hl : vPotentialHL) {
        // Verify that this HL is worth stopping at...
        if(vSPerHL.at(hl.nID).size() > 0) {
            hlList.push_back(hl);
        }
    }

    // Make a serviced array
    bool* pServiced = new bool[G->vNodeLst.size()];
    for(unsigned long int i = 0; i < G->vNodeLst.size(); i++) {
        pServiced[i] = false;
    }

    // While there are still un-serviced sensors ...
    bool unservicedSensors = true;
    unsigned long int tourIndex = 0;
    unsigned long int vehicleIndex = 0;
    while(unservicedSensors) {
        printf("Starting a new tour\n");
        // Create a new tour
        std::list<UAV_Stop> tour;
        // Start tour at the base station
        tour.push_back(UAV_Stop(G->mBaseStation.getX(), G->mBaseStation.getY(), 0));

        bool underBudget = true;
        // While we are still under our energy budget..
        while(underBudget) {
            // Find a new HL to attempt to add to tour

            // Start-off with the first HL in our list
            std::list<HoverLocation>::iterator bstIt = hlList.begin();
            // Track closest distance
            double bestDist = INF;

            // Run through our list of HL's, pick closest to UAV's current location
            for(std::list<HoverLocation>::iterator hlIt = hlList.begin(); hlIt != hlList.end();) {
                // Determine if this HL services any un-serviced sensors
                bool usefulHL = false;
                // Check each HL for this sensor
                for(int l : vSPerHL.at(hlIt->nID)) {
                    usefulHL |= !pServiced[l];
                }

                // Check if HL is useful
                if(usefulHL) {
//					printf(" Found a useful HL: %d\n", hlIt->nID);
                    // HL services at least 1 sensor, consider adding to tour
                    double tempDist = distAtoB(tour.back().fX, tour.back().fY, hlIt->fX, hlIt->fY);
                    // Check to see if this HL is better than current best
                    if(tempDist < bestDist) {
//						printf("  new best\n");
                        // Found new best!
                        bstIt = hlIt;
                        bestDist = tempDist;
                    }

                    // Advance iterator
                    hlIt++;
                }
                else {
                    // HL is no longer helpful, remove from list (advances iterator)
//					printf(" Can't use HL: %d\n", hlIt->nID);
                    hlIt = hlList.erase(hlIt);
                }
            }

            // Check if we found a valid next-stop
            if(bestDist < INF) {
//				printf(" Attempting to add %d to tour\n", bstIt->nID);
                // Attempt to add this stop to the tour
                UAV_Stop nextStop(bstIt->fX, bstIt->fY, bstIt->nID);
                // Add un-serviced sensors to stop
                for(int l : vSPerHL.at(bstIt->nID)) {
                    if(!pServiced[l]) {
                        nextStop.nodes.push_back(l);
                    }
                }
                // Verify budget by returning to depot
                UAV_Stop lastStop(G->mBaseStation.getX(), G->mBaseStation.getY(), vPotentialHL.back().nID);
                // Add these stops to the tour
                tour.push_back(nextStop);
                tour.push_back(lastStop);

                // Check energy consumption of tour
                double budget = 0;
                std::list<UAV_Stop>::iterator lst, nxt;
                lst = tour.begin();
                nxt = tour.begin();
                nxt++;

                // Run through the tour, add up pst for each leg + HL stop
                while(nxt != tour.end()) {
                    // Add time to move from lst to nxt
                    budget += lst -> edgeCost(*nxt);
                    // Add in time to talk to each sensor at nxt
                    for(int s : nxt->nodes) {
                        budget += G->vNodeLst.at(s).sensorCost(*nxt) ;
                    }

                    // Advance iterators
                    lst++;
                    nxt++;
                }
//				printf(" New budget: %f\n", budget);

                // If good ...
                if(budget <= Q) {
//					printf(" Add to tour!\n");
                    // Mark sensors
                    for(int l : vSPerHL.at(bstIt->nID)) {
                        pServiced[l] = true;
                    }
                    // Remove depot
                    tour.pop_back();
                    // Continue running
                    underBudget = true;
                }
                else {
                    // Adding this stop put us over budget!
//					printf(" Went over budget\n");
                    // Remove stop
                    tour.pop_back();
                    tour.pop_back();
                    // Add depot
                    tour.push_back(lastStop);
                    // Finish this tour
                    underBudget = false;
                }
            }
            else {
                // No useful HLs => we have serviced all sensors
                printf("Serviced every sensor\n");
                // Add on depot
                UAV_Stop depot(G->mBaseStation.getX(), G->mBaseStation.getY(), vPotentialHL.back().nID);
                tour.push_back(depot);
                // Exit loop
                underBudget = false;
            }
        }

        // Determine which tour number this is
        unsigned long int vIndex = vehicleIndex*K + tourIndex;

        // Add this tour to the vector of tours
        vTours[vIndex] = tour;

        printf("Finished tour %ld\n", vIndex);
        vehicleIndex++;

        // If end of tour iteration for each vehicle, update indices
        if(vehicleIndex >= V) {
            vehicleIndex = 0;
            tourIndex++;
        }

        // Determine if there are still any un-serviced sensors
        unservicedSensors = false;
        for(unsigned long int i = 0; i < G->vNodeLst.size(); i++) {
            unservicedSensors |= !pServiced[i];
        }
    }
    printf("Tours complete!\n");

    printf("\nFiltered tour:\n");
    for(unsigned long int i = 0; i < vTours.size(); i++) {
        std::list<UAV_Stop> l = vTours.at(i);
        printf(" %ld: ", i);
        for(UAV_Stop n : l) {
            printf("(%f, %f) ", n.fX, n.fY);
        }
        printf("\n");
    }

    delete[] pServiced;
}