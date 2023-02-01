#include "SolClustImp.h"


/*
 * SolClusters is a Solver.
 * Solves graph by first clustering the nodes into subtours, then solving TSP on those clusters.
 * In Development
 */

SolClustImp::SolClustImp(bool improvement) : m_bRunImprovement(improvement) {}

SolClustImp::SolClustImp(const Solver &s): Solver(s) {}


//***********************************************************
// Protected Member Functions
//***********************************************************

// Run solver algorithm
void SolClustImp::solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL,
		std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS) {
	if(SANITY_PRINT)
		printf("* Running Divide / Conquer / Arrange Solution *\n");

	int m = 0;
	double bestWorstLat = std::numeric_limits<double>::max();
	bool runAgain = true;
	Solution* pBestSolution = NULL;
	SolverAlgs oSolAlg;

	// Store the given parameters, we will be manipulating them
	m_vPotentialHL = vPotentialHL;
	m_vSPerHL = vSPerHL;
	m_vHLPerS = vHLPerS;

	do {
		m++;
		m_pSolution = new Solution(*solution);
		m_pSolution->m_pG->ResetMarking();

		if(SANITY_PRINT)
			printf(" SolClusters() : Solve with m = %d\n", m);


		// Find ideal hovering locations
		std::vector<HoverLocation> vSelectedHL;
		hoverLocationOptimizer(vSelectedHL);

		// Vector of tour, before ordering
		std::vector<std::vector<UAV_Stop>> vUnsolvedTours;

		// Run clustering algorithm
		oSolAlg.kMeansClustering(vSelectedHL, vUnsolvedTours, m_vPotentialHL.back(), m);

		if(DEBUG_SLCLIMP) {
			printf("Tours so-far:\n");
			for(std::vector<UAV_Stop> tour : vUnsolvedTours) {
				for(UAV_Stop stp : tour) {
					printf(" %d (", stp.nID);
					for(auto n : stp.nodes) {
						printf(" %d", n);
					}
					printf("), ");
				}
				printf("\n");
			}
		}

		// Vector for solved subtours
		std::vector<std::list<UAV_Stop>> vSovledTours;

		// Solve TSP on each cluster
		for(std::vector<UAV_Stop> tour : vUnsolvedTours) {
			// Create a TSP solver
			LKH_TSP_Solver o_TSP_Solver;
			// Create an index array to store the solution
			std::vector<int> vPath;
			// Solve TSP
			o_TSP_Solver.Solve_TSP(tour, vPath);

			// New list to hold solution
			std::list<UAV_Stop> tempList;

			// Add the UAV stops to the final tour
			if(DEBUG_SLCLIMP)
				printf("Found path:\n ");
			for(int n : vPath) {
				UAV_Stop tempStop = tour.at(n);

				tempList.push_back(tempStop);
				if(DEBUG_SLCLIMP) {
					printf("%d ", tour.at(n).nID);
				}
			}
			if(DEBUG_SLCLIMP)
				printf("\n");

			// Add base station as last stop
			tempList.push_back(m_vPotentialHL.back());

			// Store the solved tour
			vSovledTours.push_back(tempList);
		}

		if(m_bRunImprovement) {
			// Improve tours
			improveRoute(vSovledTours);
		}

		// Verify that the cost of each tour is less than our energy budget
		bool underBudget = true;
		for(std::list<UAV_Stop> tour : m_pSolution->vTours) {
			if(tourCost(tour) > Q) {
				underBudget = false;
				if(SANITY_PRINT)
					printf(" *** Over budget!! ***\n");
			}
		}

		// Are all sub-tours under the energy budget?
		if(underBudget) {
			// Assign tours to UAVs
			if((m <= (int)m_pSolution->m_nV) || (1 == m_pSolution->m_nV)) {
				// No need to run assignment algorithm, assign tours directly
				for(std::list<UAV_Stop> tour : vSovledTours) {
					m_pSolution->vTours.push_back(tour);
				}
			}
			else {
				// Vector for the cost of each solved sub-tour
				std::vector<double> vCosts;
				// Need to match tours to UAVs
				std::vector<int> vOrder;

				// Determine the cost for each tour
				for(std::list<UAV_Stop> tour : vSovledTours) {
					// Store the cost in this vance cost vector
					vCosts.push_back(timeForSubTour(tour));
				}

				// Run job scheduling algorithm
				oSolAlg.minMaxMakespan(vCosts, (int)m_pSolution->m_nV, vOrder);

				// Sanity print
				if(DEBUG_SLCLIMP) {
					printf("Indexes:\n");
					for(int n : vOrder) {
						printf(" %d", n);
					}
					printf("\n");
				}

				// Push tours into the solution using the time orders
				int* uavIndex = new int[m_pSolution->m_nV];
				for(long unsigned int i = 0; i < m_pSolution->m_nV; i++) {
					uavIndex[i] = 0;
				}
				for(int i = 0; i < (int)vOrder.size(); i++) {
					// Flag to let us know if we found a valid tour
					bool pushedTour = false;
					// Determine which UAV we are on
					int uavID = i%m_pSolution->m_nV;
					// Number of jobs allotted to this UAV
					int numJobs = (int)vOrder.size()/m_pSolution->m_nV;
					// Determine where this UAV's job assignments start in vOrder
					int base = uavID*numJobs;

					// While there are still jobs in the vOrder for this UAV
					while(uavIndex[uavID] < numJobs) {
						int jobIndex = base+uavIndex[uavID];
						if(vOrder.at(jobIndex) == -1) {
							// Empty job, try again
							uavIndex[uavID]++;
						}
						else {
							// Valid job index, push back this tour
							m_pSolution->vTours.push_back(vSovledTours.at(vOrder.at(jobIndex)));
							uavIndex[uavID]++;
							pushedTour = true;
							break;
						}
					}

					// Ensure that we actually added a tour
					if(!pushedTour) {
						// Add empty tour
						m_pSolution->vTours.push_back(std::list<UAV_Stop>());
					}
				}
			}

			// Check to see if increasing the number of subtours improved the time
			double worstLat = m_pSolution->GetWorstLatency();
			if(bestWorstLat > worstLat) {
				if(pBestSolution != NULL) {
					// Memory cleanup
					delete pBestSolution;
				}
				// Made an improvement, store this as the best solution
				pBestSolution = m_pSolution;
				bestWorstLat = worstLat;
				// Continue to increase m
				runAgain = true;
			}
			else {
				if(m%solution->m_nV == 0) {
					// No longer improving the solution
					runAgain = false;
				}
				// Memory cleanup
				delete m_pSolution;
			}

			if(SANITY_PRINT)
				printf(" SolClusters() : Worst latency = %f\n", worstLat);
		}
		else {
			// Memory cleanup
			delete m_pSolution;
		}

		// Verify we have not gone too far...
		if(m >= (int)m_pSolution->m_pG->vNodeLst.size()) {
			// We have gone too far...
			runAgain = false;
			// Verify that we at least found one solution
			if(pBestSolution == NULL) {
				// Never found a solution, hard fail!
				fprintf(stderr, "ERROR : SolClusters::solve() failed to find a valid solution!\n");
			}
		}
	} while(runAgain);

	// Store the tours that we found in the original solution
	for(std::list<UAV_Stop> tour : pBestSolution->vTours) {
		solution->vTours.push_back(tour);
	}

	// Sanity print
	if(SANITY_PRINT)
		printf(" Final result: m = %ld, worst-lat = %f\n\n", pBestSolution->vTours.size(), bestWorstLat);

	// Memory cleanup
	delete pBestSolution;
}


//***********************************************************
// Private Member Functions
//***********************************************************


// Determines which hovering locations are most ideal to collect all sensor data (greedy approach)
void SolClustImp::hoverLocationOptimizer(std::vector<HoverLocation> &vSelectedHL) {
	// While there are still nodes that haven't been serviced
	while(nodesNotServiced()) {
		// Find hovering location that services the most sensors and is closest to the base station
		int nBestHL = -1;
		int nBestHLSize = 0;

		// For each of the potential hovering locations...
		for(HoverLocation candidate : m_vPotentialHL) {
			bool picked = false;
			// Check to see if we have already selected this HL
			for(HoverLocation selected : vSelectedHL) {
				if(selected.nID == candidate.nID) {
					picked = true;
				}
			}

			if(!picked) {
				// Not yet picked, find utility of this HL
				int utility = utilityYeildOf(candidate);
				// Does the HL service more sensors than the current best?
				if(utility > nBestHLSize) {
					// Found new "best" HL
					nBestHL = candidate.nID;
					nBestHLSize = utility;
				}
				// Does this HL match the best HL?
				else if(utility == nBestHLSize) {
					// HL matches best HL... which one is closer to base station?
					float fBestDist = m_vPotentialHL.back().distTo(m_vPotentialHL[nBestHL]);
					float fCandDist = m_vPotentialHL.back().distTo(m_vPotentialHL[candidate.nID]);
					if(fCandDist < fBestDist) {
						// Candidate HL is closer than previous best HL.. update
						nBestHL = candidate.nID;
						nBestHLSize = utility;
					}
				}
			}
		}

		if(DEBUG) {
			printf("Best Hl is %d with %d sensors!\n", nBestHL, nBestHLSize);
		}

		// Create HL for this position
		HoverLocation temp(m_vPotentialHL[nBestHL]);

		// Set which nodes the UAV should service at this stop
		for(int n : m_vSPerHL.at(nBestHL)) {
			if(!m_pSolution->m_pG->IsMark(n)) {
				temp.nodes.push_back(n);
			}
		}

		// Add this HL to our selected list of HLs
		vSelectedHL.push_back(temp);

		// Mark all nodes that are services by the HL
		for(int n : m_vSPerHL[nBestHL]) {
			m_pSolution->m_pG->MarkNode(n);
		}
	}
}

/*
 * Returns the utility yield of stopping at oHL. This considers only nodes that have not been
 * marked and will remove sensor indexes from vSPerHL for sensors that have already been marked
 */
int SolClustImp::utilityYeildOf(HoverLocation &oHL) {
	int yeild = 0;

	// Run through each sensor id in oHL's vSPerHL list
	std::list<int>::iterator itr = m_vSPerHL[oHL.nID].begin();
	while(itr != m_vSPerHL[oHL.nID].end()) {
		// Has the senor/node already been marked?
		if(m_pSolution->m_pG->IsMark(*itr)) {
			// Already marked for different HL, remove node from list
//			itr = m_vSPerHL[oHL.nID].erase(itr);
		}
		else {
			// Node has not been marked yet
			yeild++;
		}
		itr++;
	}

	return yeild;
}

// Returns true if there are still nodes that have not yet been serviced
bool SolClustImp::nodesNotServiced() {
	bool allNodesServiced = true;

	// Check each of the nodes in the solution graph
	for(Node n : m_pSolution->m_pG->vNodeLst) {
		if(!n.Locked()) {
			// Found a node that has not yet been serviced
			allNodesServiced = false;
		}
	}

	return !allNodesServiced;
}

// Determines the time required to complete this tour
double SolClustImp::timeForSubTour(std::list<UAV_Stop> tour) {
	if(DEBUG_SLCLIMP)
		printf("Evaluating sub-tour time\n");

	double tourDist = 0;
	std::list<UAV_Stop>::iterator lst, nxt;
	lst = tour.begin();
	nxt = tour.begin();
	nxt++;

	// Run through tour, add up distances
	while(nxt != tour.end()) {
		// Calculate distance to next stop
		tourDist += lst->distTo(*nxt);
		// Update iterator
		lst = nxt;
		nxt++;
	}
	// Add in distance to return to base station (first stop)
	tourDist += lst->distTo(tour.front());

	// Time-to-travel = dist-to-travel / V_max
	double travelTime = tourDist/V_MAX;

	if(DEBUG_SLCLIMP)
		printf(" Found distance: %f\n Found travel time: %f\n", tourDist, travelTime);

	// Add up the total hovering cost of these UAV stops
	double hoveringTime = 0;
	for(UAV_Stop hl : tour) {
		for(int n : hl.nodes) {
			hoveringTime += m_pSolution->m_pG->vNodeLst.at(n).sensorTime(hl);
		}
	}

	if(DEBUG_SLCLIMP)
		printf(" Found hovering time: %f\n", hoveringTime);

	// The weight is the cost of the min-spanning tree + 1/2 hovering cost
	double totalTime = travelTime + hoveringTime;
	if(DEBUG_SLCLIMP)
		printf(" Total time: %f\n", totalTime);
	return totalTime;
}

// Determine the cost of this tour
double SolClustImp::tourCost(std::list<UAV_Stop> &tour) {
	double cost = 0;
	std::list<UAV_Stop>::iterator nxt = tour.begin();
	std::list<UAV_Stop>::iterator lst = tour.begin();
	nxt++;

	if(DEBUG_SLCLIMP)
		printf("Calculating cost of tour\n");

	while(nxt != tour.end()) {
		if(DEBUG_SLCLIMP)
			printf(" nodes:");
		// Cost to service nodes at this stop
		for(int n : lst->nodes) {
			if(DEBUG_SLCLIMP)
				printf(" %d", n);
			cost += m_pSolution->m_pG->vNodeLst.at(n).sensorCost(*lst);
		}
		// Cost to move to the next stop
		cost += lst->edgeCost(*nxt);
		if(DEBUG_SLCLIMP)
			printf(" ; %d->%d, %f", lst->nID, nxt->nID, lst->edgeCost(*nxt));

		// Update iterators
		lst = nxt;
		nxt++;
	}
	if(DEBUG_SLCLIMP)
		printf("\n total cost = %f\n", cost);

	return cost;
}

void SolClustImp::improveRoute(std::vector<std::list<UAV_Stop>>& vTours) {
	if(SANITY_PRINT)
		printf("\nImprove route\n");
	for(unsigned long int i = 0; i < vTours.size(); i++) {
		bool runAgain = true;

		// While moving stops helped... run again!
		while(runAgain) {
			runAgain = false;
			if(DEBUG_SLCLIMP)
				printf(" %ld:\n",i);
			if(vTours.at(i).size() > 2) {
				std::list<UAV_Stop>::iterator back = vTours.at(i).begin();
				std::list<UAV_Stop>::iterator middle = vTours.at(i).begin();
				middle++;
				std::list<UAV_Stop>::iterator front = vTours.at(i).begin();
				front++;
				front++;

				while(front != vTours.at(i).end()) {
					// Attempt to remove the middle stop
					if(DEBUG_SLCLIMP)
						printf("  Pretending to remove (%f, %f)\n", middle->fX, middle->fY);

					bool madeChang = false;

					// For each sensor
					UAV_Stop tempOld = *middle;
					for(int n : tempOld.nodes) {
						double x_b = back->fX;
						double y_b = back->fY;
						double x_f = front->fX;
						double y_f = front->fY;
						double x_v = m_pSolution->m_pG->vNodeLst[n].getX();
						double y_v = m_pSolution->m_pG->vNodeLst[n].getY();

						// Find the point, v', closest to this sensor, v, on the line between stops back and front
						double m = (y_f - y_b)/(x_f - x_b);
						double b = y_f - m * x_f;
						double x_p = (x_v - m*(b - y_v))/(m*m + 1);
						double y_p = m*x_p + b;

						if(DEBUG_SLCLIMP)
							printf("   l = %d, (x_p, y_p) = (%f, %f)\n", n, x_p, y_p);

						// Find magnitude of vector u from v to v'
						double mag_u = sqrt(pow((x_v - x_p), 2) + pow((y_v - y_p), 2));

						// Check to see if v' is in-range of v
						if(mag_u <= m_pSolution->m_pG->vNodeLst[n].getR()) {
							// We are in-range
							if(DEBUG_SLCLIMP)
								printf("    this point is in-range of v!\n");
							// Get total distance of current tour
							double oldDist = tourDist(vTours.at(i));
							// Hold onto old stop
							UAV_Stop oldStop(*middle);
							// Create new UAV stop
							UAV_Stop tempStop(x_p, y_p);
							// Determine which sensors we talk to at this hovering location
							for(int n : oldStop.nodes) {
								if(distAtoB(m_pSolution->m_pG->vNodeLst[n].getX(), m_pSolution->m_pG->vNodeLst[n].getY(),
											tempStop.fX, tempStop.fY) <= (m_pSolution->m_pG->vNodeLst[n].getR() + EPSILON)) {
									if(DEBUG_SLCLIMP)
										printf("    talk to %d\n", n);
									tempStop.nodes.push_back(n);
								}
							}
							// Make sure that we have the entire list...
							if(oldStop.nodes.size() == tempStop.nodes.size()) {
								if(DEBUG_SLCLIMP)
									printf("     Easy swap!\n");
								// Add this stop to the tour, remove current middle
								vTours.at(i).erase(middle);
								middle = front;
								// Add new stop
								vTours.at(i).insert(middle, tempStop);
								// Correct iterator's position
								middle--;
								// Check new tour's distance
								double newDist = tourDist(vTours.at(i));
								// See if we made an improvement
								if(newDist < oldDist) {
									// Great!
									if(DEBUG_SLCLIMP)
										printf("    Update helped!\n");
									runAgain = true;
									madeChang = true;
									break;
								}
								else {
									// Adding the new stop didn't help.. remove it
									if(DEBUG_SLCLIMP)
										printf("    No improvement\n");
									vTours.at(i).erase(middle);
									middle = front;
									vTours.at(i).insert(middle, oldStop);
									middle--;
								}
							}
							else {
								// We can't easily swap out the two stops...
								if(DEBUG_SLCLIMP)
									printf("    No easy swap, try walking along f(x) = mx + b\n");

								// Vector from ( x_p, y_p ) to ( x_b, y_b )
								double a_1 = x_p - x_b;
								double a_2 = y_p - y_b;
								// Normalize the vector
								double mag_a = sqrt(a_1*a_1 + a_2*a_2);
								a_1 = a_1/mag_a;
								a_2 = a_2/mag_a;

								// Determine the distance to walk up/down the line
								double walk = sqrt(pow(m_pSolution->m_pG->vNodeLst[n].getR(),2) - mag_u*mag_u);

								// Attempt distance walk up the vector
								double x_pp = x_p + walk*a_1;
								double y_pp = y_p + walk*a_2;

								if(DEBUG_SLCLIMP)
									printf("     trying (x_p, y_p) = (%f, %f)\n", x_pp, y_pp);
								// Get total distance of current tour
								double oldDist = tourDist(vTours.at(i));
								// Hold onto old stop
								UAV_Stop oldStop(*middle);
								// Create new UAV stop
								UAV_Stop tempStop(x_pp, y_pp);
								// Determine which sensors we talk to at this hovering location
								for(int n : oldStop.nodes) {
									if(distAtoB(m_pSolution->m_pG->vNodeLst[n].getX(), m_pSolution->m_pG->vNodeLst[n].getY(),
												tempStop.fX, tempStop.fY) <= (m_pSolution->m_pG->vNodeLst[n].getR() + EPSILON)) {
										if(DEBUG_SLCLIMP)
											printf("      talk to %d\n", n);
										tempStop.nodes.push_back(n);
									}
								}
								// Make sure that we have the entire list...
								if(oldStop.nodes.size() == tempStop.nodes.size()) {
									if(DEBUG_SLCLIMP)
										printf("      Easy swap!\n");
									// Add this stop to the tour, remove current middle
									vTours.at(i).erase(middle);
									middle = front;
									// Add new stop
									vTours.at(i).insert(middle, tempStop);
									// Correct iterator's position
									middle--;
									// Check new tour's distance
									double newDist = tourDist(vTours.at(i));
									// See if we made an improvement
									if(newDist < oldDist) {
										// Great!
										if(DEBUG_SLCLIMP)
											printf("      * Update helped!\n");
										runAgain = true;
										madeChang = true;
										break;
									}
									else {
										// Adding the new stop didn't help.. remove it
										if(DEBUG_SLCLIMP)
											printf("     No improvement\n");
										vTours.at(i).erase(middle);
										middle = front;
										vTours.at(i).insert(middle, oldStop);
										middle--;
									}
								}
								else {
									// Attempt distance walk down the vector
									double x_pp = x_p - walk*a_1;
									double y_pp = y_p - walk*a_2;

									if(DEBUG_SLCLIMP)
										printf("     trying (x_p, y_p) = (%f, %f)\n", x_pp, y_pp);
									// Get total distance of current tour
									double oldDist = tourDist(vTours.at(i));
									// Hold onto old stop
									UAV_Stop oldStop(*middle);
									// Create new UAV stop
									UAV_Stop tempStop(x_pp, y_pp);
									// Determine which sensors we talk to at this hovering location
									for(int n : oldStop.nodes) {
										if(distAtoB(m_pSolution->m_pG->vNodeLst[n].getX(), m_pSolution->m_pG->vNodeLst[n].getY(),
													tempStop.fX, tempStop.fY) <= (m_pSolution->m_pG->vNodeLst[n].getR() + EPSILON)) {
											if(DEBUG_SLCLIMP)
												printf("      talk to %d\n", n);
											tempStop.nodes.push_back(n);
										}
									}
									// Make sure that we have the entire list...
									if(oldStop.nodes.size() == tempStop.nodes.size()) {
										if(DEBUG_SLCLIMP)
											printf("      Easy swap!\n");
										// Add this stop to the tour, remove current middle
										vTours.at(i).erase(middle);
										middle = front;
										// Add new stop
										vTours.at(i).insert(middle, tempStop);
										// Correct iterator's position
										middle--;
										// Check new tour's distance
										double newDist = tourDist(vTours.at(i));
										// See if we made an improvement
										if(newDist < oldDist) {
											// Great!
											if(DEBUG_SLCLIMP)
												printf("      * Update helped!\n");
											runAgain = true;
											madeChang = true;
											break;
										}
										else {
											// Adding the new stop didn't help.. remove it
											if(DEBUG_SLCLIMP)
												printf("     No improvement\n");
											vTours.at(i).erase(middle);
											middle = front;
											vTours.at(i).insert(middle, oldStop);
											middle--;
										}
									}
								}
							}
						}
						if(!madeChang) {
							// Find point along radius that is in-range
							double x_1 = x_p - x_v;
							double x_2 = y_p - y_v;
							double a = m_pSolution->m_pG->vNodeLst[n].getR()/(sqrt(pow(x_1, 2) + pow(x_2, 2)));
							double x_pp = x_v + x_1*a;
							double y_pp = y_v + x_2*a;

							if(DEBUG_SLCLIMP)
								printf("    (x_pp, y_pp) = (%f, %f)\n", x_pp, y_pp);

							// Get total distance of current tour
							double oldDist = tourDist(vTours.at(i));
							// Hold onto old stop
							UAV_Stop oldStop(*middle);
							// Create new UAV stop
							UAV_Stop tempStop(x_pp, y_pp);
							// Determine which sensors we talk to at this hovering location
							for(int n : oldStop.nodes) {
								if(distAtoB(m_pSolution->m_pG->vNodeLst[n].getX(), m_pSolution->m_pG->vNodeLst[n].getY(),
											tempStop.fX, tempStop.fY) <= (m_pSolution->m_pG->vNodeLst[n].getR() + EPSILON)) {
									if(DEBUG_SLCLIMP)
										printf("    talk to %d\n", n);
									tempStop.nodes.push_back(n);
								}
							}
							// Make sure that we have the entire list...
							if(oldStop.nodes.size() == tempStop.nodes.size()) {
								if(DEBUG_SLCLIMP)
									printf("     Easy swap!\n");
								// Add this stop to the tour, remove current middle
								vTours.at(i).erase(middle);
								middle = front;
								// Add new stop
								vTours.at(i).insert(middle, tempStop);
								// Correct iterator's position
								middle--;
								// Check new tour's distance
								double newDist = tourDist(vTours.at(i));
								// See if we made an improvement
								if(newDist < oldDist) {
									// Great!
									if(DEBUG_SLCLIMP)
										printf("    Update helped!\n");
									runAgain = true;
									madeChang = true;
									break;
								}
								else {
									// Adding the new stop didn't help.. remove it
									if(DEBUG_SLCLIMP)
										printf("    No improvement\n");
									vTours.at(i).erase(middle);
									middle = front;
									vTours.at(i).insert(middle, oldStop);
									middle--;
								}
							}
							else {
								// We can't easily swap out the two stops...
								if(DEBUG_SLCLIMP)
									printf("    No easy swap\n");
							}
						}
					}

					// If we still haven't found a good change...
					if(!madeChang) {
						// Check corners
						if(DEBUG_SLCLIMP)
							printf(" Try corners\n");

						for(std::list<int>::iterator itA = tempOld.nodes.begin(); itA != tempOld.nodes.end(); itA++) {
							for(std::list<int>::iterator itB = std::next(itA, 1); itB != tempOld.nodes.end(); itB++) {
								Node u = m_pSolution->m_pG->vNodeLst.at(*itA);
								Node v = m_pSolution->m_pG->vNodeLst.at(*itB);

								// Find overlapping points
								Roots y_roots;
								Roots x_roots;
								findOverlapPoints(&v, &u, &x_roots, &y_roots);

								// Add the two overlapping points to the solution
								if(!x_roots.imaginary & !y_roots.imaginary) {
									{
										// Try moving HL to these two points
										if(DEBUG_SLCLIMP)
											printf("  Trying corner @ (%f, %f)\n", x_roots.root1, y_roots.root1);
										// Get total distance of current tour
										double oldDist = tourDist(vTours.at(i));
										// Hold onto old stop
										UAV_Stop oldStop(*middle);
										// Create new UAV stop
										UAV_Stop tempStop(x_roots.root1, y_roots.root1);
										// Determine which sensors we talk to at this hovering location
										for(int n : oldStop.nodes) {
											if(distAtoB(m_pSolution->m_pG->vNodeLst[n].getX(), m_pSolution->m_pG->vNodeLst[n].getY(),
														tempStop.fX, tempStop.fY) <= (m_pSolution->m_pG->vNodeLst[n].getR() + EPSILON)) {
												if(DEBUG_SLCLIMP)
													printf("   talk to %d\n", n);
												tempStop.nodes.push_back(n);
											}
										}
										// Make sure that we have the entire list...
										if(oldStop.nodes.size() == tempStop.nodes.size()) {
											if(DEBUG_SLCLIMP)
												printf("  Easy swap!\n");
											// Add this stop to the tour, remove current middle
											vTours.at(i).erase(middle);
											middle = front;
											// Add new stop
											vTours.at(i).insert(middle, tempStop);
											// Correct iterator's position
											middle--;
											// Check new tour's distance
											double newDist = tourDist(vTours.at(i));
											// See if we made an improvement
											if(newDist < oldDist) {
												// Great!
												if(DEBUG_SLCLIMP)
													printf("   Update helped!\n");
												runAgain = true;
												madeChang = true;
												break;
											}
											else {
												// Adding the new stop didn't help.. remove it
												if(DEBUG_SLCLIMP)
													printf("   No improvement\n");
												vTours.at(i).erase(middle);
												middle = front;
												vTours.at(i).insert(middle, oldStop);
												middle--;
											}
										}
										else {
											// We can't easily swap out the two stops...
											if(DEBUG_SLCLIMP)
												printf("  Corner is out-of-range for other nodes\n");
										}
									}

									if(!madeChang) {
										// Try moving HL to these two points
										if(DEBUG_SLCLIMP)
											printf("  Trying corner @ (%f, %f)\n", x_roots.root2, y_roots.root2);
										// Get total distance of current tour
										double oldDist = tourDist(vTours.at(i));
										// Hold onto old stop
										UAV_Stop oldStop(*middle);
										// Create new UAV stop
										UAV_Stop tempStop(x_roots.root2, y_roots.root2);
										// Determine which sensors we talk to at this hovering location
										for(int n : oldStop.nodes) {
											if(distAtoB(m_pSolution->m_pG->vNodeLst[n].getX(), m_pSolution->m_pG->vNodeLst[n].getY(),
														tempStop.fX, tempStop.fY) <= (m_pSolution->m_pG->vNodeLst[n].getR() + EPSILON)) {
												if(DEBUG_SLCLIMP)
													printf("   talk to %d\n", n);
												tempStop.nodes.push_back(n);
											}
										}
										// Make sure that we have the entire list...
										if(oldStop.nodes.size() == tempStop.nodes.size()) {
											if(DEBUG_SLCLIMP)
												printf("  Easy swap!\n");
											// Add this stop to the tour, remove current middle
											vTours.at(i).erase(middle);
											middle = front;
											// Add new stop
											vTours.at(i).insert(middle, tempStop);
											// Correct iterator's position
											middle--;
											// Check new tour's distance
											double newDist = tourDist(vTours.at(i));
											// See if we made an improvement
											if(newDist < oldDist) {
												// Great!
												if(DEBUG_SLCLIMP)
													printf("   Update helped!\n");
												runAgain = true;
												madeChang = true;
												break;
											}
											else {
												// Adding the new stop didn't help.. remove it
												if(DEBUG_SLCLIMP)
													printf("   No improvement\n");
												vTours.at(i).erase(middle);
												middle = front;
												vTours.at(i).insert(middle, oldStop);
												middle--;
											}
										}
										else {
											// We can't easily swap out the two stops...
											if(DEBUG_SLCLIMP)
												printf("  Corner is out-of-range for other nodes\n");
										}
									}


//									vPotentialHL.push_back(HoverLocation(id++, x_roots.root1, y_roots.root1,
//																		 distAtoB(x_roots.root1, y_roots.root1, G->mBaseStation.getX(), G->mBaseStation.getY())));
//									vPotentialHL.push_back(HoverLocation(id++, x_roots.root2, y_roots.root2,
//																		 distAtoB(x_roots.root2, y_roots.root2, G->mBaseStation.getX(), G->mBaseStation.getY())));
								}
							}
						}



					}

					// Update iterators
					back++;
					middle++;
					front++;
				}
			}
		}
	}

	// Sanity print
	if(DEBUG_SLCLIMP) {
		printf("\nImproved tour:\n");
		int i = 0;
		for(std::list<UAV_Stop> l : vTours) {
			printf(" %d: ", i);
			for(UAV_Stop n : l) {
				printf("(%f, %f) ", n.fX, n.fY);
			}
			printf("\n");
			i++;
		}
	}
}


// Determine the cost of the tour given in tour, a list of UAV stops
double SolClustImp::tourDist(std::list<UAV_Stop> &tour) {
	double dist = 0;

	if(tour.size() > 1) {
		std::list<UAV_Stop>::iterator lst, nxt;
		lst = tour.begin();
		nxt = tour.begin();
		nxt++;

		// Run through the tour, add up distance from stop-to-stop
		while(nxt != tour.end()) {
			// Add distance
			dist += distAtoB(lst->fX, lst->fY, nxt->fX, nxt->fY);
			// Advance iterators
			lst++;
			nxt++;
		}
	}

	return dist;
}


// Find points where the two comm radii overlap
void SolClustImp::findOverlapPoints(Node* v, Node* u, Roots* x_roots, Roots* y_roots) {
	// Node ranges overlap, there are 2 potential hovering location here
	// Find overlapping points, math here: https://math.stackexchange.com/a/256123/661482
	double x1 = u->getX();
	double y1 = u->getY();
	double r1 = u->getR();
	double x2 = v->getX();
	double y2 = v->getY();
	double r2 = v->getR();

	// Find the y intercept values
	double c1 = -(y1 - y2)/(x1 - x2);
	double c2 = -(pow(r1,2) - pow(r2,2))/(2*(x1-x2))
				+ (pow(x1,2) - pow(x2,2))/(2*(x1 - x2))
				+ (pow(y1,2) - pow(y2,2))/(2*(x1 - x2));

	// ay^2 + by + c = 0
	double a = pow(c1,2) + 1;
	double b = (2*c1*(c2 - x1)-2*y1);
	double c = pow((c2 - x1),2) + pow(y1,2) - pow(r1,2);

	y_roots -> findRoots(a, b, c);

	// Find the x intercept values
	double c3 = -(x1 - x2)/(y1 - y2);
	double c4 = -(pow(r1,2) - pow(r2,2))/(2*(y1-y2))
				+ (pow(x1,2) - pow(x2,2))/(2*(y1 - y2))
				+ (pow(y1,2) - pow(y2,2))/(2*(y1 - y2));

	// ax^2 + bx + c = 0
	a = pow(c3,2) + 1;
	b = (2*c3*(c4 - y1)-2*x1);
	c = pow((c4 - y1),2) + pow(x1,2) - pow(r1,2);

	x_roots -> findRoots(a, b, c);

	if(isZero(pow(x_roots->root1 - x1, 2) + pow(y_roots->root1 - y1, 2) - pow(r1, 2))) {
		if(DEBUG_SLCLIMP)
			printf("x_roots.root1 pairs with y_roots.root1\n");
		// Do nothing
	}
	else if(isZero(pow(x_roots->root1 - x1, 2) + pow(y_roots->root2 - y1, 2) - pow(r1, 2))) {
		if(DEBUG_SLCLIMP)
			printf("x_roots.root1 pairs with y_roots.root2\n");
		// Swap the roots so they match
		double temp = y_roots->root2;
		y_roots->root2 = y_roots->root1;
		y_roots->root1 = temp;
	}
	else {
		if(DEBUG_SLCLIMP)
			printf("Pairing failed\n");
	}
}
