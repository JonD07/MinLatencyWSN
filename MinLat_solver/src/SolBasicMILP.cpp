#include "SolBasicMILP.h"


SolBasicMILP::SolBasicMILP() {}

SolBasicMILP::SolBasicMILP(const Solver &s): Solver(s) {}

void SolBasicMILP::solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS) {
	unsigned long int M = K*solution->m_nV;

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
		unsigned long int L = solution->m_pG->vNodeLst.size();

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
					Y[k][i][l] = model.addVar(0.0, 1.0,  solution->m_pG->vNodeLst.at(l).sensorTime(vPotentialHL.at(i)),
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
						expr += solution->m_pG->vNodeLst.at(l).sensorCost(vPotentialHL.at(i)) * Y[k][i][l];
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
						expr += solution->m_pG->vNodeLst.at(l).sensorCost(vPotentialHL.at(i)) * Y[k][i][l];
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
		for(Node l : solution->m_pG->vNodeLst) {
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
				// Make list for sub-tour k
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
				solution->vTours.push_back(tour_k);
			}

			// Sanity print
			printf("\nFiltered tour:\n");
			for(unsigned long int i = 0; i < solution->vTours.size(); i++) {
				std::list<UAV_Stop> l = solution->vTours.at(i);
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
