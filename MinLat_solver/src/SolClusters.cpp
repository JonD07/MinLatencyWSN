//
// Created by peter on 10/5/2022.
//

#include "SolClusters.h"


/*
 * SolClusters is a Solver.
 * Solves graph by first clustering the nodes into subtours, then solving TSP on those clusters.
 * In Development
 */

SolClusters::SolClusters() {}
SolClusters::SolClusters(double budget): Solver(budget) {}

SolClusters::SolClusters(const Solver &s): Solver(s) {}


//***********************************************************
// Protected Member Functions
//***********************************************************

// Run solver algorithm
void SolClusters::solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL,
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

		if(DEBUG_SLCLUST) {
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
		// Vector for the cost of each solved sub-tour
		std::vector<double> vCosts;

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
			if(DEBUG_SLCLUST)
				printf("Found path:\n ");
			for(int n : vPath) {
				UAV_Stop tempStop = tour.at(n);

				tempList.push_back(tempStop);
				if(DEBUG_SLCLUST) {
					printf("%d ", tour.at(n).nID);
				}
			}
			if(DEBUG_SLCLUST)
				printf("\n");

			// Add base station as last stop
			tempList.push_back(m_vPotentialHL.back());

			// Store the solved tour
			vSovledTours.push_back(tempList);
			vCosts.push_back(timeForSubTour(tempList));
		}

		// Assign tours to UAVs
		if((m <= (int)m_pSolution->m_nV) || (1 == m_pSolution->m_nV)) {
			// No need to run assignment algorithm, assign tours directly
			for(std::list<UAV_Stop> tour : vSovledTours) {
				m_pSolution->vTours.push_back(tour);
			}
		}
		else {
			// Need to match tours to UAVs
			std::vector<int> vOrder;
			// Run job scheduling algorithm
			oSolAlg.minMaxMakespan(vCosts, (int)m_pSolution->m_nV, vOrder);

			// Sanity print
			if(DEBUG_SLCLUST) {
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

		// Verify that the cost of each tour is less than our energy budget
		bool underBudget = true;
		for(std::list<UAV_Stop> tour : m_pSolution->vTours) {
			if(tourCost(tour) > Q*budget) {
				underBudget = false;
				if(SANITY_PRINT)
					printf(" *** Over budget!! ***\n");
			}
		}

		// Are all sub-tours under the energy budget?
		if(underBudget) {
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
void SolClusters::hoverLocationOptimizer(std::vector<HoverLocation> &vSelectedHL) {
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
int SolClusters::utilityYeildOf(HoverLocation &oHL) {
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
bool SolClusters::nodesNotServiced() {
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
double SolClusters::timeForSubTour(std::list<UAV_Stop> tour) {
	if(DEBUG_SLCLUST)
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

	if(DEBUG_SLCLUST)
		printf(" Found distance: %f\n Found travel time: %f\n", tourDist, travelTime);

	// Add up the total hovering cost of these UAV stops
	double hoveringTime = 0;
	for(UAV_Stop hl : tour) {
		for(int n : hl.nodes) {
			hoveringTime += m_pSolution->m_pG->vNodeLst.at(n).sensorTime(hl);
		}
	}

	if(DEBUG_SLCLUST)
		printf(" Found hovering time: %f\n", hoveringTime);

	// The weight is the cost of the min-spanning tree + 1/2 hovering cost
	double totalTime = travelTime + hoveringTime;
	if(DEBUG_SLCLUST)
		printf(" Total time: %f\n", totalTime);
	return totalTime;
}

// Determine the cost of this tour
double SolClusters::tourCost(std::list<UAV_Stop> &tour) {
	double cost = 0;
	std::list<UAV_Stop>::iterator nxt = tour.begin();
	std::list<UAV_Stop>::iterator lst = tour.begin();
	nxt++;

	if(DEBUG_SLCLUST)
		printf("Calculating cost of tour\n");

	while(nxt != tour.end()) {
		if(DEBUG_SLCLUST)
			printf(" nodes:");
		// Cost to service nodes at this stop
		for(int n : lst->nodes) {
			if(DEBUG_SLCLUST)
				printf(" %d", n);
			cost += m_pSolution->m_pG->vNodeLst.at(n).sensorCost(*lst);
		}
		// Cost to move to the next stop
		cost += lst->edgeCost(*nxt);
		if(DEBUG_SLCLUST)
			printf(" ; %d->%d, %f", lst->nID, nxt->nID, lst->edgeCost(*nxt));

		// Update iterators
		lst = nxt;
		nxt++;
	}
	if(DEBUG_SLCLUST)
		printf("\n total cost = %f\n", cost);

	return cost;
}
