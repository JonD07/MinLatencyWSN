#include "SolHardMILP.h"


SolHardMILP::SolHardMILP() {}
SolHardMILP::SolHardMILP(double budget): Solver(budget) {}

SolHardMILP::SolHardMILP(const Solver &s): Solver(s) {}

void SolHardMILP::solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS) {
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
		unsigned long int L = solution->m_pG->vNodeLst.size();

		/// Define variables
		// Tracks edges between HLs
		GRBVar**** X = new GRBVar***[N];
		for(unsigned long int i = 0; i < N; i++) {
			X[i] = new GRBVar**[N];
			for(unsigned long int j = 0; j < N; j++) {
				X[i][j] = new GRBVar*[K];
				for(unsigned long int k = 0; k < K; k++) {
					X[i][j][k] = new GRBVar[solution->m_nV];
					for(unsigned long int v = 0; v < solution->m_nV; v++) {
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
					Y[i][l][k] = new GRBVar[solution->m_nV];
				}
			}
		}

		// Create the indexed variables
		for(unsigned long int i = 0; i < N; i++) {
			for(unsigned long int k = 0; k < K; k++) {
				for(unsigned long int v = 0; v < solution->m_nV; v++) {
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
				Z[i][k] = new GRBVar[solution->m_nV];
				for(unsigned long int v = 0; v < solution->m_nV; v++) {
					Z[i][k][v] = model.addVar(0.0, Q, 0.0, GRB_CONTINUOUS, "Z_"+itos(i)+"_"+itos(k)+"_"+itos(v));
				}
			}
		}

		/// Attempt to improve solution

		// Auxiliary variables for Min ~ Max (this is the real objective function)
		if(MIN_MAX) {
			// Min ~ Max aux variable
			GRBVar W = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "W");
			for(unsigned long int v = 0; v < solution->m_nV; v++) {
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
							expr -= solution->m_pG->vNodeLst.at(l).sensorTime(vPotentialHL.at(i)) *Y[i][l][k][v];
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
			GRBVar* W = new GRBVar[solution->m_nV];
			for(unsigned long int v = 0; v < solution->m_nV; v++) {
				W[v] = model.addVar(0.0, GRB_INFINITY, 1.0, GRB_CONTINUOUS, "W_"+itos(v));
			}
			// W is the sum of time each UAV is used
			// Minimize total
			for(unsigned long int v = 0; v < solution->m_nV; v++) {
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
							expr -=  solution->m_pG->vNodeLst.at(l).sensorTime(vPotentialHL.at(i))*Y[i][l][k][v];
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
			// Create a greed solution to base an initial solution off of SolClusters
			SolClusters greedy = SolClusters(*this);

			// Use greedy nearest-neighbor approach to find the initial solution
			Solution* greedySol = greedy.RunSolver(solution->m_pG, solution->m_nV, vPotentialHL, vSPerHL, vHLPerS);

			// Set defaults for all START values
			for(unsigned long int i = 0; i < N; i++) {
				for(unsigned long int j = 0; j < N; j++) {
					for(unsigned long int k = 0; k < K; k++) {
						for(unsigned long int v = 0; v < solution->m_nV; v++) {
							X[i][j][k][v].set(GRB_DoubleAttr_Start, 0.0);
						}
					}
				}
			}
			for(unsigned long int i = 0; i < N; i++) {
				for(unsigned long int k = 0; k < K; k++) {
					for(unsigned long int v = 0; v < solution->m_nV; v++) {
						for(int l : vSPerHL.at(i)) {
							Y[i][l][k][v].set(GRB_DoubleAttr_Start, 0.0);
						}
					}
				}
			}

			// Run through the solution, add the appropriate data to start attributes
			if(DEBUG_HMILP)
				printf("Heuristic gave us:\n");
			for(unsigned long int i = 0; i < greedySol->vTours.size(); i++) {
				std::list<UAV_Stop> lst = greedySol->vTours.at(i);

				// Any valid solution has atleast 3 HLs
				if(lst.size() > 2) {
					std::list<UAV_Stop>::iterator prev = lst.begin();
					std::list<UAV_Stop>::iterator next = lst.begin();
					next++;

					// Determine which v and k this is
					int v = i%solution->m_nV;
					int k = i/solution->m_nV;
					if(DEBUG_HMILP)
						printf("%ld -> v=%d, k=%d\n", i, v, k);

					bool first = true;
					// Step through tour
					while(next != lst.end()) {
						// Add this edge to solver
						if(DEBUG_HMILP)
							printf(" (%d, %d)", prev->nID, next->nID);
						if(first) {
							X[0][next->nID][k][v].set(GRB_DoubleAttr_Start, 1.0);
							first = false;
						}
						else {
							X[prev->nID][next->nID][k][v].set(GRB_DoubleAttr_Start, 1.0);
						}
//						X[prev->nID][next->nID][k][v].set(GRB_DoubleAttr_Start, 1.0);

						// Add which sensors to talk to
						for(int l : next->nodes) {
							Y[next->nID][l][k][v].set(GRB_DoubleAttr_Start, 1.0);
						}

						// Advance iterators
						prev++;
						next++;
					}
					if(DEBUG_HMILP)
						printf("\n");
				}
			}

			delete greedySol;
		}

		if(PRIORITIES) {
			// Add priorities to the branches that X_ijkv where j services the most sensors
			if(DEBUG_HMILP)
				printf("Adding branching priorities to X\n");
			for(HoverLocation hl : vPotentialHL) {
				for(unsigned long int j = 0; j < N; j++) {
					for(unsigned long int k = 0; k < K; k++) {
						for(unsigned long int v = 0; v < solution->m_nV; v++) {
							X[j][hl.nID][k][v].set(GRB_IntAttr_BranchPriority, vSPerHL.at(hl.nID).size());
						}
					}
				}
			}
		}

		if(CLIQUE_CUTS) {
			// Add a clique-cut to each HL so that at-most one in-bound X == 1
			if(DEBUG_HMILP)
				printf("Adding clique-cuts to each hovering location\n");

			// Only let one UAV, on a single sub-tour enter i
			for(unsigned long int i = 1; i < N-1; i++) {
				GRBLinExpr expr = 0;
				for(unsigned long int j = 0; j < N; j++) {
					for(unsigned long int k = 0; k < K; k++) {
						for(unsigned long int v = 0; v < solution->m_nV; v++) {
							expr += X[j][i][k][v];
						}
					}
				}
				model.addConstr(expr <= 1, "X_j_"+itos(i)+"_k_v_leq_1");
			}

			// For each sensor l
			for(Node l : solution->m_pG->vNodeLst) {
				GRBLinExpr expr = 0;
				if(DEBUG_HMILP)
					printf("Clique on %d\n", l.getID());
				// For each HL i that services l
				for(int i : vHLPerS.at(l.getID())) {
					// Check to see if this HL can only service l
					if(vSPerHL.at(i).size() <= 1) {
						printf(" %d", i);
						// Sum on all edges, on all sub-tours, on all vehicles going into i
						for(unsigned long int j = 0; j < N; j++) {
							for(unsigned long int k = 0; k < K; k++) {
								for(unsigned long int v = 0; v < solution->m_nV; v++) {
									expr += X[j][i][k][v];
								}
							}
						}
					}
				}
				if(DEBUG_HMILP)
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
				for(unsigned long int v = 0; v < solution->m_nV; v++) {
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
					for(unsigned long int v = 0; v < solution->m_nV; v++) {
						// Budget at i, on k
						GRBLinExpr expr = Z[i][k][v];
						// Plus cost to talk to l from point i, on tour k
						for(int l : vSPerHL.at(i)) {
							expr += solution->m_pG->vNodeLst.at(l).sensorCost(vPotentialHL.at(i)) * Y[i][l][k][v];
						}
						// Plus cost to get to j from i
						expr += vPotentialHL.at(i).edgeCost(vPotentialHL.at(j)) * X[i][j][k][v];
						// Big-M, cancel-out expression if X == 0
						expr += Q*budget*(1 - X[i][j][k][v]);
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
					for(unsigned long int v = 0; v < solution->m_nV; v++) {
						// Budget at i, on k
						GRBLinExpr expr = Z[i][k][v];
						// Plus cost to talk to l from point i, on tour k
						for(int l : vSPerHL.at(i)) {
							expr += solution->m_pG->vNodeLst.at(l).sensorCost(vPotentialHL.at(i)) * Y[i][l][k][v];
						}
						// Plus cost to get to j from i
						expr += vPotentialHL.at(i).edgeCost(vPotentialHL.at(j)) * X[i][j][k][v];
						// Big-M, cancel-out expression if X == 0
						expr -= Q*budget*(1 - X[i][j][k][v]);
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
				for(unsigned long int v = 0; v < solution->m_nV; v++) {
					GRBLinExpr expr = Z[i][k][v];
					// Sum across all edges going into i
					for(unsigned long int j = 0; j < N; j++) {
						expr -= Q*budget*X[j][i][k][v];
					}

					model.addConstr(expr <= 0, "Z_"+itos(i)+"_"+itos(k)+"_"+itos(v)+"_leq_X_*_"+itos(i)+"_"+itos(k)+"_"+itos(v));
				}
			}
		}

		// Must exit from depot-initial at most once
		for(unsigned long int k = 0; k < K; k++) {
			for(unsigned long int v = 0; v < solution->m_nV; v++) {
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
			for(unsigned long int v = 0; v < solution->m_nV; v++) {
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
				for(unsigned long int v = 0; v < solution->m_nV; v++) {
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
		for(Node l : solution->m_pG->vNodeLst) {
			int l_ID = l.getID();
			GRBLinExpr expr = 0;
			// Sum across hovering locations close to l
			for(int i : vHLPerS.at(l_ID)) {
				// Sum across tours
				for(unsigned long int k = 0; k < K; k++) {
					for(unsigned long int v = 0; v < solution->m_nV; v++) {
						expr += Y[i][l_ID][k][v];
					}
				}
			}

			model.addConstr(expr == 1, "Y_i_"+itos(l_ID)+"_k_v_eq_1");
		}

		// Initial budget is 0
		for(unsigned long int k = 0; k < K; k++) {
			for(unsigned long int v = 0; v < solution->m_nV; v++) {
				model.addConstr(Z[0][k][v] == 0, "Z_0_"+itos(k)+"_"+itos(v)+"_eq_0");
			}
		}

		// No self-loops
		for(unsigned long int i = 0; i < N; i++) {
			for(unsigned long int k = 0; k < K; k++) {
				for(unsigned long int v = 0; v < solution->m_nV; v++) {
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

		// Run for at most 5 minutes
		model.set(GRB_DoubleParam_TimeLimit, 300);
		// Use cuts aggressively
		model.set(GRB_INT_PAR_CUTS, "2");

		// Optimize model
		model.optimize();

		// Extract solution
		if (model.get(GRB_IntAttr_SolCount) > 0) {
			if(DEBUG_HMILP)
				printf("MIP gap: %f\n", model.get(GRB_DoubleAttr_MIPGap));

			if(PRINT_GAP) {
				// Print results to file
				FILE* pOutputFile;
				char buff[100];
				sprintf(buff, "Experiment2/MILP_GAP.dat");
				if(SANITY_PRINT)
					printf("%s\n", buff);
				pOutputFile = fopen(buff, "a");
				fprintf(pOutputFile, "%ld %f\n", solution->m_pG->vNodeLst.size(), model.get(GRB_DoubleAttr_MIPGap));
				fclose(pOutputFile);
			}

			// Get edge data
			double**** Xsol = new double***[N];
			for(unsigned long int i = 0; i < N; i++) {
				Xsol[i] = new double**[N];
				for(unsigned long int j = 0; j < N; j++) {
					Xsol[i][j] = new double*[K];
					for(unsigned long int k = 0; k < K; k++) {
						Xsol[i][j][k] = model.get(GRB_DoubleAttr_X, X[i][j][k], solution->m_nV);
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
						Ysol[i][l][k] = new double[solution->m_nV];
						// Set all to 0
						for(unsigned long int v = 0; v < solution->m_nV; v++) {
							Ysol[i][l][k][v] = 0;
						}
					}
				}
			}

			// Get actual Y data
			for(unsigned long int i = 0; i < N; i++) {
				for(int l : vSPerHL.at(i)) {
					for(unsigned long int k = 0; k < K; k++) {
						for(unsigned long int v = 0; v < solution->m_nV; v++) {
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
					Zsol[i][k] =  model.get(GRB_DoubleAttr_X, Z[i][k], solution->m_nV);
				}
			}

			// Get Min ~ Max value
//				double* WSol = model.get(GRB_DoubleAttr_X, W, V);

			// Print results
			if(DEBUG_HMILP)
				printf("\nCollect tour:\n");
			for(unsigned long int k = 0; k < K; k++) {
				for(unsigned long int v = 0; v < solution->m_nV; v++) {
					// Make list for sub-tour k
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
								if(DEBUG_HMILP)
									printf(" %ld-%ld: %ld -> %ld\n", v, k, prevHL, i);
								// Create new UAV stop
								UAV_Stop tempStop(vPotentialHL[i].fX, vPotentialHL[i].fY);
								// Determine which sensors we talk to at this hovering location
								for(unsigned long int l = 0; l < L; l++) {
									if(Ysol[i][l][k][v] > 0.5) {
										if(DEBUG_HMILP)
											printf(" %ld-%ld: @ %ld talk to %ld\n", v, k, i, l);
										tempStop.nodes.push_back(l);
									}
								}
								if(DEBUG_HMILP)
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
								if(DEBUG_HMILP)
									printf("* Tour %ld is empty!\n", (v*K + k));
								prevHL = N-1;
								tour_k.push_back(UAV_Stop(vPotentialHL[prevHL].fX,vPotentialHL[prevHL].fY));
							}
						}
					}

//					// If we found a valid tour, add it
//					if(tour_k.size() > 2) {
//						solution->vTours.push_back(tour_k);
//					}
					solution->vTours.push_back(tour_k);
				}
			}

			// Sanity print
			if(SANITY_PRINT) {
				printf("\nFiltered tour:\n");
				for(unsigned long int i = 0; i < solution->vTours.size(); i++) {
					std::list<UAV_Stop> l = solution->vTours.at(i);
					printf(" %ld: ", i);
					for(UAV_Stop n : l) {
						printf("(%f, %f) ", n.fX, n.fY);
					}
					printf("\n");
				}
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
