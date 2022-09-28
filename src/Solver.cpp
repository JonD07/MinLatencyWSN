//
// Created by peter on 9/28/2022.
//

#include "../inc/Solver.h"


Solver::Solver(Graph* G, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL,
               std::vector<std::list<int>> &vHLPerS, std::vector<std::list<UAV_Stop>> &vTours, unsigned long int V,
               unsigned long int K, bool MIN_MAX, bool INITIAL_SOLUTION, bool PRIORITIES, bool CLIQUE_CUTS):
               G(G), vPotentialHL(vPotentialHL), vSPerHL(vSPerHL), vHLPerS(vHLPerS), vTours(vTours), V(V), K(K),
               MIN_MAX(MIN_MAX), INITIAL_SOLUTION(INITIAL_SOLUTION), PRIORITIES(PRIORITIES), CLIQUE_CUTS(CLIQUE_CUTS) {}

void Solver::runBasicMILP() {
    unsigned long int M = K*V;

    try {
        // Create Gurobi environment
        GRBEnv env;
        GRBModel model = GRBModel(&env);

        /*
         * Sets:
         * i,j in H, set of hovering locations, |H|=N
         * l in S, set of sensors, |S|=L
         * k in K, set of tours (or vehicles), |K|=M
         * l in S_i, indexed set of sensors in-range of hovering location i
         * i in H_l, indexed set of hovering locations in-range of sensor l
         */

        unsigned long int N = vPotentialHL.size();
        unsigned long int L = G->vNodeLst.size();

        /// Define variables
        // Tracks edges between HLs
        GRBVar*** X = new GRBVar**[M];
        for(unsigned long int k = 0; k < M; k++) {
            X[k] = new GRBVar*[N];
            for(unsigned long int i = 0; i < N; i++) {
                X[k][i] = new GRBVar[N];
                for(unsigned long int j = 0; j < N; j++) {
                    X[k][i][j] = model.addVar(0.0, 1.0, vPotentialHL.at(i).edgeTime(vPotentialHL.at(j)),
                                              GRB_BINARY, "X_"+itos(k)+"_"+itos(i)+"_"+itos(j));
                }
            }
        }

        // Tracks when we talk to each sensor
        GRBVar*** Y = new GRBVar**[M];
        for(unsigned long int k = 0; k < M; k++) {
            Y[k] = new GRBVar*[N];
            for(unsigned long int i = 0; i < N; i++) {
                Y[k][i] = new GRBVar[L];
            }
        }

        // Create the indexed variables
        for(unsigned long int k = 0; k < M; k++) {
            for(unsigned long int i = 0; i < N; i++) {
                for(int l : vSPerHL.at(i)) {
                    Y[k][i][l] = model.addVar(0.0, 1.0,  G->vNodeLst.at(l).sensorTime(vPotentialHL.at(i)),
                                              GRB_BINARY, "Y_"+itos(k)+"_"+itos(i)+"_"+itos(l));
                }
            }
        }

        // Energy budget
        GRBVar** Z = new GRBVar*[M];
        for(unsigned long int k = 0; k < M; k++) {
            Z[k] = new GRBVar[N];
            for(unsigned long int i = 0; i < N; i++) {
                Z[k][i] = model.addVar(0.0, Q, 0.0, GRB_CONTINUOUS, "Z_"+itos(k)+"_"+itos(i));
            }
        }

        /// Constraints
        // Limit when Y can be turned on
        for(unsigned long int k = 0; k < M; k++) {
            for(unsigned long int i = 0; i < N; i++) {
                for(int l : vSPerHL.at(i)) {
                    // Y_kil
                    GRBLinExpr expr = Y[k][i][l];
                    // Minus X_kji (edges going into i on k)
                    for(unsigned long int j = 0; j < N; j++) {
                        expr -= X[k][j][i];
                    }
                    model.addConstr(expr <= 0, "Y_"+itos(k)+"_"+itos(i)+"_"+itos(l)+"_leq_X");
                }
            }
        }

        // Enforce budget/eliminate sub-tours (Upper Bound)
        for(unsigned long int k = 0; k < M; k++) {
            for(unsigned long int i = 0; i < N; i++) {
                for(unsigned long int j = 0; j < N; j++) {
                    // Budget at i, on k
                    GRBLinExpr expr = Z[k][i];
                    // Plus cost to talk to l from point i, on tour k
                    for(int l : vSPerHL.at(i)) {
                        expr += G->vNodeLst.at(l).sensorCost(vPotentialHL.at(i)) * Y[k][i][l];
                    }
                    // Plus cost to get to j from i
                    expr += vPotentialHL.at(i).edgeCost(vPotentialHL.at(j)) * X[k][i][j];
                    // Big-M, cancel-out expression if X == 0
                    expr -= Q*(1 - X[k][i][j]);
                    // Minus budget at j on k
                    expr -= Z[k][j];

                    model.addConstr(expr <= 0, "Z_"+itos(k)+"_"+itos(j)+"_geq_Z_"+itos(k)+"_"+itos(i));
                }
            }
        }

        // Enforce budget/eliminate sub-tours (Lower Bound)
        for(unsigned long int k = 0; k < M; k++) {
            for(unsigned long int i = 0; i < N; i++) {
                for(unsigned long int j = 0; j < N; j++) {
                    // Budget at i, on k
                    GRBLinExpr expr = Z[k][i];
                    // Plus cost to talk to l from point i, on tour k
                    for(int l : vSPerHL.at(i)) {
                        expr += G->vNodeLst.at(l).sensorCost(vPotentialHL.at(i)) * Y[k][i][l];
                    }
                    // Plus cost to get to j from i
                    expr +=vPotentialHL.at(i).edgeCost(vPotentialHL.at(j)) * X[k][i][j];
                    // Big-M, cancel-out expression if X == 0
                    expr += Q*(1 - X[k][i][j]);
                    // Minus budget at j on k
                    expr -= Z[k][j];

                    model.addConstr(expr >= 0, "Z_"+itos(k)+"_"+itos(j)+"_geq_Z_"+itos(k)+"_"+itos(i));
                }
            }
        }

        // Shut-off budget when you don't enter a hover location
        for(unsigned long int k = 0; k < M; k++) {
            for(unsigned long int i = 0; i < N; i++) {
                GRBLinExpr expr = 0;
                // Sum across all edges going into i
                for(unsigned long int j = 0; j < N; j++) {
                    expr += Q*X[k][j][i];
                }
                // Subtract the budget at i
                expr -= Z[k][i];

                model.addConstr(expr >= 0, "Z_"+itos(k)+"_"+itos(i)+"_leq_X_"+itos(k)+"_*_"+itos(i));
            }
        }

        // Must collect from each sensor
        for(Node l : G->vNodeLst) {
            int l_ID = l.getID();
            GRBLinExpr expr = 0;
            // Sum across tours
            for(unsigned long int k = 0; k < M; k++) {
                // Sum across hovering locations close to l
                for(int i : vHLPerS.at(l_ID)) {
                    expr += Y[k][i][l_ID];
                }
            }

            model.addConstr(expr == 1, "Y_k_i_"+itos(l_ID)+"_geq_1");
        }

        // Degree-in == degree-out
        for(unsigned long int k = 0; k < M; k++) {
            for(unsigned long int i = 1; i < N-1; i++) {
                GRBLinExpr expr = 0;
                // Sum across all edges going into i
                for(unsigned long int j = 0; j < N; j++) {
                    expr += X[k][j][i];
                }
                // Sum across all edges going out of i
                for(unsigned long int j = 0; j < N; j++) {
                    expr -= X[k][i][j];
                }

                model.addConstr(expr == 0, "X_"+itos(k)+"_*_"+itos(i)+"_eq_X_"+itos(k)+"_"+itos(i)+"_*");
            }
        }

        // Must exit from depot-initial
        for(unsigned long int k = 0; k < M; k++) {
            GRBLinExpr expr = 0;
            // Sum across all edges leaving depot on k
            for(unsigned long int i = 0; i < N; i++) {
                expr += X[k][0][i];
            }

            model.addConstr(expr <= 1, "X_"+itos(k)+"_0_i_eq_1");
        }

        // Must return to depot-final
        for(unsigned long int k = 0; k < M; k++) {
            GRBLinExpr expr = 0;
            // Sum across all edges leaving depot on k
            for(unsigned long int i = 0; i < N; i++) {
                expr += X[k][i][N - 1];
            }

            model.addConstr(expr <= 1, "X_"+itos(k)+"_i_i-hat_eq_1");
        }

//		for(unsigned long int k = 0; k < (M-1); k++) {
//			GRBLinExpr expr = 0;
//			for(unsigned long int i = 0; i < N; i++) {
//				for(unsigned long int j = 0; j < N; j++) {
//					expr += edgeTime(vPotentialHL.at(i), vPotentialHL.at(j))*X[k][i][j];
//					expr -= edgeTime(vPotentialHL.at(i), vPotentialHL.at(j))*X[k+1][i][j];
//				}
//			}
//			model.addConstr(expr >= 0, "X_"+itos(k)+"_leq_X_"+itos(k+1));
//		}

        // Initial budget is 0
        for(unsigned long int k = 0; k < M; k++) {
            model.addConstr(Z[k][0] == 0, "Z_"+itos(k)+"_0_eq_0");
        }

        // No self-loops
        for(unsigned long int k = 0; k < M; k++) {
            for(unsigned long int i = 0; i < N; i++) {
                GRBLinExpr expr = X[k][i][i];
                model.addConstr(expr == 0, "no_X_"+itos(k)+"_"+itos(i)+"_"+itos(i));
            }
        }

        // No going from depot '0' to depot 'N-1'
        for(unsigned long int k = 0; k < M; k++) {
            GRBLinExpr expr = X[k][0][N-1];
            model.addConstr(expr == 0, "no_X_"+itos(k)+"_0_N");
        }

        // Run for at most 1.5 hours
        model.set(GRB_DoubleParam_TimeLimit, 5400);
        // Use cuts aggressively
        model.set(GRB_INT_PAR_CUTS, "2");

        // Optimize model
        model.optimize();

        // Extract solution
        if (model.get(GRB_IntAttr_SolCount) > 0) {
            // Create arrays of doubles to collect results
            double*** Xsol = new double**[M];
            double*** Ysol = new double**[M];
            double** Zsol = new double*[M];

            // Extract the results from the solver
            for(unsigned long int k = 0; k < M; k++) {
                Xsol[k] = new double*[N];
                for(unsigned long int i = 0; i < N; i++) {
                    Xsol[k][i] = model.get(GRB_DoubleAttr_X, X[k][i], N);
                }
            }

            for(unsigned long int k = 0; k < M; k++) {
                Ysol[k] = new double*[N];
                for(unsigned long int i = 0; i < N; i++) {
                    Ysol[k][i] = new double[N];
                    for(unsigned long int j = 0; j < N; j++) {
                        Ysol[k][i][j] = 0;
                    }
                    for(int l : vSPerHL.at(i)) {
                        Ysol[k][i][l] = *model.get(GRB_DoubleAttr_X, &Y[k][i][l], 1);
                    }
                }
            }

            for(unsigned long int k = 0; k < M; k++) {
                Zsol[k] = model.get(GRB_DoubleAttr_X, Z[k], N);
            }

            // Print results
            printf("\nCollect tour:\n");
            for(unsigned long int k = 0; k < M; k++) {
                // Make list four sub-tour k
                std::list<UAV_Stop> tour_k;
                // Start at depot '0'
                unsigned long int prevHL = 0;
                // Put the depot on the list
                tour_k.push_back(UAV_Stop(vPotentialHL[prevHL].fX,vPotentialHL[prevHL].fY));
                // Find all stops on this tour
                while(prevHL != (N-1)) {
                    // Find the next stop
                    unsigned long int i = 0;
                    for(; i < N; i++) {
                        if(Xsol[k][prevHL][i] > 0.5) {
                            // Found next hovering-location
                            printf(" %ld: %ld -> %ld\n", k, prevHL, i);
                            // Create new UAV stop
                            UAV_Stop tempStop(vPotentialHL[i].fX,vPotentialHL[i].fY);
                            // Determine which sensors we talk to at this hovering location
                            for(unsigned long int l = 0; l < L; l++) {
                                if(Ysol[k][i][l] > 0.5) {
                                    printf(" %ld: @ %ld talk to %ld\n", k, i, l);
                                    tempStop.nodes.push_back(l);
                                }
                            }
                            printf(" %ld: Budget@%ld = %f\n", k, i, Zsol[k][i]);
                            // Update previous
                            prevHL = i;
                            // Add this node to tour
                            tour_k.push_back(tempStop);
                            break;
                        }
                    }
                    // Verify that nothing went wrong
                    if(i == N) {
                        if(tour_k.size() > 1) {
                            // Something went wrong...
                            fprintf(stderr, "[ERROR] Xsol[][][] does not contain complete graph!\n");
                            exit(1);
                        }
                        else {
                            // Found empty tour
                            printf("* Tour %ld is empty!\n", k);
                            prevHL = N-1;
                            tour_k.push_back(UAV_Stop(vPotentialHL[prevHL].fX,vPotentialHL[prevHL].fY));
                        }
                    }
                }
                vTours[k] = tour_k;
            }

            // Sanity print
            printf("\nFiltered tour:\n");
            for(unsigned long int i = 0; i < vTours.size(); i++) {
                std::list<UAV_Stop> l = vTours.at(i);
                printf(" %ld: ", i);
                for(UAV_Stop n : l) {
                    printf("(%f, %f) ", n.fX, n.fY);
                }
                printf("\n");
            }
            if(0) {

                printf("\nUn-Filtered tour:\n");
                for(unsigned long int k = 0; k < M; k++) {
                    for(unsigned long int i = 0; i < N; i++) {
                        for(unsigned long int j = 0; j < N; j++) {
                            if(Xsol[k][i][j] > 0.5) {
                                printf(" %ld: %ld -> %ld\n", k, i, j);
                                for(unsigned long int l = 0; l < L; l++) {
                                    if(Ysol[k][j][l] > 0.5) {
                                        printf(" %ld: @ %ld talk to %ld\n", k, j, l);
                                    }
                                }
                                printf(" %ld: Budget@%ld = %f\n", k, j, Zsol[k][j]);
                            }
                        }
                    }
                }

                printf("\nData collection:\n");
                for(unsigned long int k = 0; k < M; k++) {
                    for(unsigned long int i = 0; i < N; i++) {
                        for(unsigned long int l = 0; l < L; l++) {
                            if(Ysol[k][i][l] > 0.5) {
                                printf(" %ld: @ %ld talk to %ld\n", k, i, l);
                            }
                        }
                    }
                }

                printf("\nTotal Budget:\n");
                for(unsigned long int k = 0; k < M; k++) {
                    for(unsigned long int i = 0; i < N; i++) {
                        printf(" %ld: %ld : %f\n", k, i, Zsol[k][i]);
                    }
                }
            }

            // Clean-up memory
            for(unsigned long int k = 0; k < M; k++) {
                for(unsigned long int i = 0; i < N; i++) {
                    delete[] Xsol[k][i];
                    delete[] Ysol[k][i];
                }
                delete[] Xsol[k];
                delete[] Ysol[k];
                delete[] Zsol[k];
            }
            delete[] Xsol;
            delete[] Ysol;
            delete[] Zsol;
        }
        else {
            fprintf(stderr, "[ERROR] : Could not find valid solution!\n");
            exit(1);
        }

    } catch (GRBException& e) {
        printf("Error number: %d\n", e.getErrorCode());
        printf("%s\n", e.getMessage().c_str());
        exit(1);
    } catch (...) {
        printf("Error during optimization\n");
        exit(1);
    }
}

void Solver::runNN(){
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

void Solver::runHardMILP(){
    try {
        // Create Gurobi environment
        GRBEnv env;
        GRBModel model = GRBModel(&env);

        /*
         * Sets:
         * i,j in H, set of hovering locations, |H|=N
         * l in S, set of sensors, |S|=L
         * k in K, set of tours per vehicle, |K|=K
         * v in V, set of vehicles, |V|=V
         * k in KxV, all tours, |KxV|=M
         * l in S_i, indexed set of sensors in-range of hovering location i
         * i in H_l, indexed set of hovering locations in-range of sensor l
         */

        unsigned long int N = vPotentialHL.size();
        unsigned long int L = G->vNodeLst.size();

        /// Define variables
        // Tracks edges between HLs
        GRBVar**** X = new GRBVar***[N];
        for(unsigned long int i = 0; i < N; i++) {
            X[i] = new GRBVar**[N];
            for(unsigned long int j = 0; j < N; j++) {
                X[i][j] = new GRBVar*[K];
                for(unsigned long int k = 0; k < K; k++) {
                    X[i][j][k] = new GRBVar[V];
                    for(unsigned long int v = 0; v < V; v++) {
                        X[i][j][k][v] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "X_"+itos(i)+"_"+itos(j)+"_"+itos(k)+"_"+itos(v));
                    }
                }
            }
        }

        // Tracks when we talk to each sensor
        GRBVar**** Y = new GRBVar***[N];
        for(unsigned long int i = 0; i < N; i++) {
            Y[i] = new GRBVar**[L];
            for(unsigned long int l = 0; l < L; l++) {
                Y[i][l] = new GRBVar*[K];
                for(unsigned long int k = 0; k < K; k++) {
                    Y[i][l][k] = new GRBVar[V];
                }
            }
        }

        // Create the indexed variables
        for(unsigned long int i = 0; i < N; i++) {
            for(unsigned long int k = 0; k < K; k++) {
                for(unsigned long int v = 0; v < V; v++) {
                    for(int l : vSPerHL.at(i)) {
                        Y[i][l][k][v] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "Y_"+itos(i)+"_"+itos(l)+"_"+itos(k)+"_"+itos(v));
                    }
                }
            }
        }

        // Energy budget
        GRBVar*** Z = new GRBVar**[N];
        for(unsigned long int i = 0; i < N; i++) {
            Z[i] = new GRBVar*[K];
            for(unsigned long int k = 0; k < K; k++) {
                Z[i][k] = new GRBVar[V];
                for(unsigned long int v = 0; v < V; v++) {
                    Z[i][k][v] = model.addVar(0.0, Q, 0.0, GRB_CONTINUOUS, "Z_"+itos(i)+"_"+itos(k)+"_"+itos(v));
                }
            }
        }

        /// Attempt to improve solution

        // Auxiliary variables for Min ~ Max (this is the real objective function)
        if(MIN_MAX) {
            // Min ~ Max aux variable
            GRBVar W = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "W");
            for(unsigned long int v = 0; v < V; v++) {
                GRBLinExpr expr = W;

                // Sum on time-cost for each graph edge for each sub-tour
                for(unsigned long int i = 0; i < N; i++) {
                    for(unsigned long int j = 0; j < N; j++) {
                        for(unsigned long int k = 0; k < K; k++) {
                            expr -= vPotentialHL.at(i).edgeTime(vPotentialHL.at(j))*X[i][j][k][v];
                        }
                    }
                }

                // Sum on the time-cost to talk to each sensor, from each HL, for each sub-tour
                for(unsigned long int i = 0; i < N; i++) {
                    for(unsigned long int k = 0; k < K; k++) {
                        for(int l : vSPerHL.at(i)) {
                            expr -= G->vNodeLst.at(l).sensorTime(vPotentialHL.at(i)) *Y[i][l][k][v];
                        }
                    }
                }

                // Add in additional cost for subsequent tours
                for(unsigned long int i = 0; i < N; i++) {
                    for(unsigned long int k = 0; k < K; k++) {
                        expr -= stopTime(k)*X[0][i][k][v];
                    }
                }
                model.addConstr(expr >= 0, "W_geq_tot_"+itos(v));
            }
        }
        else {
            // Minimize total time
            GRBVar* W = new GRBVar[V];
            for(unsigned long int v = 0; v < V; v++) {
                W[v] = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "W_"+itos(v));
            }
            // W is the sum of time each UAV is used
            // Minimize total
            for(unsigned long int v = 0; v < V; v++) {
                GRBLinExpr expr = W[v];

                // Sum on time-cost for each graph edge for each sub-tour
                for(unsigned long int i = 0; i < N; i++) {
                    for(unsigned long int j = 0; j < N; j++) {
                        for(unsigned long int k = 0; k < K; k++) {
                            expr -= vPotentialHL.at(i).edgeTime(vPotentialHL.at(j))*X[i][j][k][v];
                        }
                    }
                }

                // Sum on the time-cost to talk to each sensor, from each HL, for each sub-tour
                for(unsigned long int i = 0; i < N; i++) {
                    for(unsigned long int k = 0; k < K; k++) {
                        for(int l : vSPerHL.at(i)) {
                            expr -=  G->vNodeLst.at(l).sensorTime(vPotentialHL.at(i))*Y[i][l][k][v];
                        }
                    }
                }

                // Add in additional cost for subsequent tours
                for(unsigned long int i = 0; i < N; i++) {
                    for(unsigned long int k = 0; k < K; k++) {
                        expr -= stopTime(k)*X[0][i][k][v];
                    }
                }
                model.addConstr(expr >= 0, "W_"+itos(v)+"_geq_tot");
            }
            delete[] W;
        }

        /// Add initial solution?
        if(INITIAL_SOLUTION) {
            // Create new vector of lists for greedy solution
            std::vector<std::list<UAV_Stop>> vTempTours;
            // Add empty tours to vTours
            for(unsigned long int v = 0; v < V; v++) {
                for(unsigned long int k = 0; k < K; k++) {
                    std::list<UAV_Stop> temp;
                    vTempTours.push_back(temp);
                }
            }
            std::vector<std::list<UAV_Stop>> original_tours = vTours;
            vTours = vTempTours;
            // Use greedy nearest-neighbor approach to find an initial solution
            runNN();

            vTempTours = vTours;
            vTours = original_tours;

            // Set defaults for all START values
            for(unsigned long int i = 0; i < N; i++) {
                for(unsigned long int j = 0; j < N; j++) {
                    for(unsigned long int k = 0; k < K; k++) {
                        for(unsigned long int v = 0; v < V; v++) {
                            X[i][j][k][v].set(GRB_DoubleAttr_Start, 0.0);
                        }
                    }
                }
            }
            for(unsigned long int i = 0; i < N; i++) {
                for(unsigned long int k = 0; k < K; k++) {
                    for(unsigned long int v = 0; v < V; v++) {
                        for(int l : vSPerHL.at(i)) {
                            Y[i][l][k][v].set(GRB_DoubleAttr_Start, 0.0);
                        }
                    }
                }
            }

            // Run through the solution, add the appropriate data to start attributes
            printf("Heuristic gave us:\n");
            for(unsigned long int i = 0; i < vTempTours.size(); i++) {
                std::list<UAV_Stop> lst = vTempTours.at(i);

                // Any valid solution has atleast 3 HLs
                if(lst.size() > 2) {
                    std::list<UAV_Stop>::iterator prev = lst.begin();
                    std::list<UAV_Stop>::iterator next = lst.begin();
                    next++;

                    // Determine which v and k this is
                    int v = i/K;
                    int k = i%K;
                    printf("%ld -> v=%d, k=%d\n", i, v, k);

                    // Step through tour
                    while(next != lst.end()) {
                        // Add this edge to solver
                        printf(" (%d, %d)", prev->nID, next->nID);
                        X[prev->nID][next->nID][k][v].set(GRB_DoubleAttr_Start, 1.0);

                        // Add which sensors to talk to
                        for(int l : next->nodes) {
                            Y[next->nID][l][k][v].set(GRB_DoubleAttr_Start, 1.0);
                        }

                        // Advance iterators
                        prev++;
                        next++;
                    }
                    printf("\n");
                }
            }
        }

        if(PRIORITIES) {
            // Add priorities to the branches that X_ijkv where j services the most sensors
            printf("Adding branching priorities to X\n");
            for(HoverLocation hl : vPotentialHL) {
                for(unsigned long int j = 0; j < N; j++) {
                    for(unsigned long int k = 0; k < K; k++) {
                        for(unsigned long int v = 0; v < V; v++) {
                            X[j][hl.nID][k][v].set(GRB_IntAttr_BranchPriority, vSPerHL.at(hl.nID).size());
                        }
                    }
                }
            }
        }

        if(CLIQUE_CUTS) {
            // Add a clique-cut to each HL so that at-most one in-bound X == 1
            printf("Adding clique-cuts to each hovering location\n");

            // Only let one UAV, on a single sub-tour enter i
            for(unsigned long int i = 1; i < N-1; i++) {
                GRBLinExpr expr = 0;
                for(unsigned long int j = 0; j < N; j++) {
                    for(unsigned long int k = 0; k < K; k++) {
                        for(unsigned long int v = 0; v < V; v++) {
                            expr += X[j][i][k][v];
                        }
                    }
                }
                model.addConstr(expr <= 1, "X_j_"+itos(i)+"_k_v_leq_1");
            }

            // For each sensor l
            for(Node l : G->vNodeLst) {
                GRBLinExpr expr = 0;
                printf("Clique on %d\n", l.getID());
                // For each HL i that services l
                for(int i : vHLPerS.at(l.getID())) {
                    // Check to see if this HL can only service l
                    if(vSPerHL.at(i).size() <= 1) {
                        printf(" %d", i);
                        // Sum on all edges, on all sub-tours, on all vehicles going into i
                        for(unsigned long int j = 0; j < N; j++) {
                            for(unsigned long int k = 0; k < K; k++) {
                                for(unsigned long int v = 0; v < V; v++) {
                                    expr += X[j][i][k][v];
                                }
                            }
                        }
                    }
                }
                printf("\n");
                model.addConstr(expr <= 1, "X_j_i_k_v_leq_1_for_"+itos(l.getID()));
            }

//			// For each sensor l
//			for(Node l : G->vNodeLst) {
//				GRBLinExpr expr = 0;
//				// For each HL i that services l
//				for(int i : vHLPerS.at(l.getID())) {
//					// Sum on all edges, on all sub-tours, on all vehicles going into i
//					for(unsigned long int j = 0; j < N; j++) {
//						for(unsigned long int k = 0; k < K; k++) {
//							for(unsigned long int v = 0; v < V; v++) {
//								expr += X[j][i][k][v];
//							}
//						}
//					}
//				}
//				model.addConstr(expr <= 1, "X_j_i_k_v_leq_1_for_"+itos(l.getID()));
//			}
        }

        /// Constraints

        // Limit when Y can be turned on
        for(unsigned long int i = 0; i < N; i++) {
            for(unsigned long int k = 0; k < K; k++) {
                for(unsigned long int v = 0; v < V; v++) {
                    for(int l : vSPerHL.at(i)) {
                        // For each eligible HL - sensor pair on tour k, vehicle v
                        GRBLinExpr expr = Y[i][l][k][v];
                        // Only turn-on Y if we used an appropriate X
                        for(unsigned long int j = 0; j < N; j++) {
                            expr -= X[j][i][k][v];
                        }

                        model.addConstr(expr <= 0, "Y_"+itos(i)+"_"+itos(l)+"_"+itos(k)+"_"+itos(v)+"_leq_X");
                    }
                }
            }
        }

        // Enforce budget/eliminate sub-tours (Upper Bound)
        for(unsigned long int i = 0; i < N; i++) {
            for(unsigned long int j = 0; j < N; j++) {
                for(unsigned long int k = 0; k < K; k++) {
                    for(unsigned long int v = 0; v < V; v++) {
                        // Budget at i, on k
                        GRBLinExpr expr = Z[i][k][v];
                        // Plus cost to talk to l from point i, on tour k
                        for(int l : vSPerHL.at(i)) {
                            expr += G->vNodeLst.at(l).sensorCost(vPotentialHL.at(i)) * Y[i][l][k][v];
                        }
                        // Plus cost to get to j from i
                        expr += vPotentialHL.at(i).edgeCost(vPotentialHL.at(j)) * X[i][j][k][v];
                        // Big-M, cancel-out expression if X == 0
                        expr += Q*(1 - X[i][j][k][v]);
                        // Minus budget at j on k
                        expr -= Z[j][k][v];

                        model.addConstr(expr >= 0, "Z_"+itos(j)+"_"+itos(k)+itos(v)+"_geq_Z_"+itos(i)+"_"+itos(k)+itos(v)+"_UB");
                    }
                }
            }
        }

        // Enforce budget/eliminate sub-tours (Lower Bound)
        for(unsigned long int i = 0; i < N; i++) {
            for(unsigned long int j = 0; j < N; j++) {
                for(unsigned long int k = 0; k < K; k++) {
                    for(unsigned long int v = 0; v < V; v++) {
                        // Budget at i, on k
                        GRBLinExpr expr = Z[i][k][v];
                        // Plus cost to talk to l from point i, on tour k
                        for(int l : vSPerHL.at(i)) {
                            expr += G->vNodeLst.at(l).sensorCost(vPotentialHL.at(i)) * Y[i][l][k][v];
                        }
                        // Plus cost to get to j from i
                        expr += vPotentialHL.at(i).edgeCost(vPotentialHL.at(j)) * X[i][j][k][v];
                        // Big-M, cancel-out expression if X == 0
                        expr -= Q*(1 - X[i][j][k][v]);
                        // Minus budget at j on k
                        expr -= Z[j][k][v];

                        model.addConstr(expr <= 0, "Z_"+itos(j)+"_"+itos(k)+itos(v)+"_geq_Z_"+itos(i)+"_"+itos(k)+itos(v)+"_LB");
                    }
                }
            }
        }

        // Shut-off budget when you don't enter a hover location
        for(unsigned long int i = 0; i < N; i++) {
            for(unsigned long int k = 0; k < K; k++) {
                for(unsigned long int v = 0; v < V; v++) {
                    GRBLinExpr expr = Z[i][k][v];
                    // Sum across all edges going into i
                    for(unsigned long int j = 0; j < N; j++) {
                        expr -= Q*X[j][i][k][v];
                    }

                    model.addConstr(expr <= 0, "Z_"+itos(i)+"_"+itos(k)+"_"+itos(v)+"_leq_X_*_"+itos(i)+"_"+itos(k)+"_"+itos(v));
                }
            }
        }

        // Must exit from depot-initial at most once
        for(unsigned long int k = 0; k < K; k++) {
            for(unsigned long int v = 0; v < V; v++) {
                GRBLinExpr expr = 0;
                // Sum across all edges leaving depot on k
                for(unsigned long int j = 0; j < N; j++) {
                    expr += X[0][j][k][v];
                }

                model.addConstr(expr <= 1, "X_0_j_"+itos(k)+"_"+itos(v)+"_leq_1");
            }
        }

        // Must return to depot-final at most once
        for(unsigned long int k = 0; k < K; k++) {
            for(unsigned long int v = 0; v < V; v++) {
                GRBLinExpr expr = 0;
                // Sum across all edges leaving depot on k
                for(unsigned long int j = 0; j < N; j++) {
                    expr += X[j][(N-1)][k][v];
                }

                model.addConstr(expr <= 1, "X_j_n_"+itos(k)+"_"+itos(v)+"_leq_1");
            }
        }

        // Degree-in == degree-out
        for(unsigned long int i = 1; i < N-1; i++) {
            for(unsigned long int k = 0; k < K; k++) {
                for(unsigned long int v = 0; v < V; v++) {
                    GRBLinExpr expr = 0;
                    // Sum across all edges going into i
                    for(unsigned long int j = 0; j < N; j++) {
                        expr += X[i][j][k][v];
                    }
                    // Sum across all edges going out of i
                    for(unsigned long int j = 0; j < N; j++) {
                        expr -= X[j][i][k][v];
                    }

                    model.addConstr(expr == 0, "X_"+itos(i)+"_*_"+itos(k)+"_"+itos(v)+"_eq_X_*_"+itos(i)+"_"+itos(k)+"_"+itos(v));
                }
            }
        }

        // Must collect from each sensor
        for(Node l : G->vNodeLst) {
            int l_ID = l.getID();
            GRBLinExpr expr = 0;
            // Sum across hovering locations close to l
            for(int i : vHLPerS.at(l_ID)) {
                // Sum across tours
                for(unsigned long int k = 0; k < K; k++) {
                    for(unsigned long int v = 0; v < V; v++) {
                        expr += Y[i][l_ID][k][v];
                    }
                }
            }

            model.addConstr(expr == 1, "Y_i_"+itos(l_ID)+"_k_v_eq_1");
        }

        // Initial budget is 0
        for(unsigned long int k = 0; k < K; k++) {
            for(unsigned long int v = 0; v < V; v++) {
                model.addConstr(Z[0][k][v] == 0, "Z_0_"+itos(k)+"_"+itos(v)+"_eq_0");
            }
        }

        // No self-loops
        for(unsigned long int i = 0; i < N; i++) {
            for(unsigned long int k = 0; k < K; k++) {
                for(unsigned long int v = 0; v < V; v++) {
                    GRBLinExpr expr = X[i][i][k][v];
                    model.addConstr(expr == 0, "no_X_"+itos(i)+"_"+itos(i)+"_"+itos(k)+"_"+itos(v));
                }
            }
        }

        // Remove symmetry in assigning routes to vehicles
//		for(unsigned long int k = 0; k < (M-1); k++) {
//			GRBLinExpr expr = 0;
//			for(unsigned long int i = 0; i < N; i++) {
//				for(unsigned long int j = 0; j < N; j++) {
//					expr += edgeTime(vPotentialHL.at(i), vPotentialHL.at(j))*X[k][i][j];
//					expr -= edgeTime(vPotentialHL.at(i), vPotentialHL.at(j))*X[k+1][i][j];
//				}
//			}
//			model.addConstr(expr >= 0, "X_"+itos(k)+"_leq_X_"+itos(k+1));
//		}


//			// No going from depot '0' to depot 'N-1'
//			for(unsigned long int k = 0; k < M; k++) {
//				GRBLinExpr expr = X[k][0][N-1];
//				model.addConstr(expr == 0, "no_X_"+itos(k)+"_0_N");
//			}

        // Run for at most 3 hours
        model.set(GRB_DoubleParam_TimeLimit, 10800);
        // Use cuts aggressively
        model.set(GRB_INT_PAR_CUTS, "2");

        // Optimize model
        model.optimize();

        // Extract solution
        if (model.get(GRB_IntAttr_SolCount) > 0) {
            // Get edge data
            double**** Xsol = new double***[N];
            for(unsigned long int i = 0; i < N; i++) {
                Xsol[i] = new double**[N];
                for(unsigned long int j = 0; j < N; j++) {
                    Xsol[i][j] = new double*[K];
                    for(unsigned long int k = 0; k < K; k++) {
                        Xsol[i][j][k] = model.get(GRB_DoubleAttr_X, X[i][j][k], V);
                    }
                }
            }

            // Get when to talk to each sensor (create variables)
            double**** Ysol = new double***[N];
            for(unsigned long int i = 0; i < N; i++) {
                Ysol[i] = new double**[L];
                for(unsigned long int l = 0; l < L; l++) {
                    Ysol[i][l] = new double*[K];
                    for(unsigned long int k = 0; k < K; k++) {
                        Ysol[i][l][k] = new double[V];
                        // Set all to 0
                        for(unsigned long int v = 0; v < V; v++) {
                            Ysol[i][l][k][v] = 0;
                        }
                    }
                }
            }

            // Get actual Y data
            for(unsigned long int i = 0; i < N; i++) {
                for(int l : vSPerHL.at(i)) {
                    for(unsigned long int k = 0; k < K; k++) {
                        for(unsigned long int v = 0; v < V; v++) {
                            Ysol[i][l][k][v] = *model.get(GRB_DoubleAttr_X, Y[i][l][k] + v, 1);
                        }
                    }
                }
            }

            // Get energy budget
            double*** Zsol = new double**[N];
            for(unsigned long int i = 0; i < N; i++) {
                Zsol[i] = new double*[K];
                for(unsigned long int k = 0; k < K; k++) {
                    Zsol[i][k] =  model.get(GRB_DoubleAttr_X, Z[i][k], V);
                }
            }

            // Get Min ~ Max value
//				double* WSol = model.get(GRB_DoubleAttr_X, W, V);

            // Print results
            printf("\nCollect tour:\n");
            for(unsigned long int v = 0; v < V; v++) {
                for(unsigned long int k = 0; k < K; k++) {
                    // Make list four sub-tour k
                    std::list<UAV_Stop> tour_k;
                    // Start at depot '0'
                    unsigned long int prevHL = 0;
                    // Put the depot on the list
                    tour_k.push_back(UAV_Stop(vPotentialHL[prevHL].fX, vPotentialHL[prevHL].fY));
                    // Find all stops on this tour
                    while(prevHL != (N-1)) {
                        // Find the next stop
                        unsigned long int i = 0;
                        for(; i < N; i++) {
                            if(Xsol[prevHL][i][k][v] > 0.5) {
                                // Found next hovering-location
                                printf(" %ld-%ld: %ld -> %ld\n", v, k, prevHL, i);
                                // Create new UAV stop
                                UAV_Stop tempStop(vPotentialHL[i].fX, vPotentialHL[i].fY);
                                // Determine which sensors we talk to at this hovering location
                                for(unsigned long int l = 0; l < L; l++) {
                                    if(Ysol[i][l][k][v] > 0.5) {
                                        printf(" %ld-%ld: @ %ld talk to %ld\n", v, k, i, l);
                                        tempStop.nodes.push_back(l);
                                    }
                                }
                                printf(" %ld-%ld: Budget@%ld = %f\n", v, k, i, Zsol[i][k][v]);
                                // Update previous
                                prevHL = i;
                                // Add this node to tour
                                tour_k.push_back(tempStop);
                                break;
                            }
                        }
                        // Verify that nothing went wrong
                        if(i == N) {
                            if(tour_k.size() > 1) {
                                // Something went wrong...
                                fprintf(stderr, "[ERROR] Xsol[][][] does not contain complete graph!\n");
                                exit(1);
                            }
                            else {
                                // Found empty tour
                                printf("* Tour %ld is empty!\n", (v*K + k));
                                prevHL = N-1;
                                tour_k.push_back(UAV_Stop(vPotentialHL[prevHL].fX,vPotentialHL[prevHL].fY));
                            }
                        }
                    }
                    vTours[v*K + k] = tour_k;
                }
            }

            // Sanity print
            printf("\nFiltered tour:\n");
            for(unsigned long int i = 0; i < vTours.size(); i++) {
                std::list<UAV_Stop> l = vTours.at(i);
                printf(" %ld: ", i);
                for(UAV_Stop n : l) {
                    printf("(%f, %f) ", n.fX, n.fY);
                }
                printf("\n");
            }

            // Clean-up memory
            for(unsigned long int i = 0; i < N; i++) {
                for(unsigned long int j = 0; j < N; j++) {
                    for(unsigned long int k = 0; k < K; k++) {
                        delete[] Xsol[i][j][k];
                    }
                    delete[] Xsol[i][j];
                }
                delete[] Xsol[i];
            }
            delete[] Xsol;

            for(unsigned long int i = 0; i < N; i++) {
                for(unsigned long int l = 0; l < L; l++) {
                    for(unsigned long int k = 0; k < K; k++) {
                        delete[] Ysol[i][l][k];
                    }
                    delete[] Ysol[i][l];
                }
                delete[] Ysol[i];
            }
            delete[] Ysol;

            for(unsigned long int i = 0; i < N; i++) {
                for(unsigned long int k = 0; k < K; k++) {
                    delete[] Zsol[i][k];
                }
                delete[] Zsol[i];
            }
            delete[] Zsol;
        }
        else {
            fprintf(stderr, "[ERROR] : Could not find valid solution!\n");
            exit(1);
        }

    } catch (GRBException& e) {
        printf("Error number: %d\n", e.getErrorCode());
        printf("%s\n", e.getMessage().c_str());
        exit(1);
    } catch (...) {
        printf("Error during optimization\n");
        exit(1);
    }
}

void Solver::printResults(bool bImproved){
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
