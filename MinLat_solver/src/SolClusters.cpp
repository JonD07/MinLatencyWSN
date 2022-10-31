//
// Created by peter on 10/5/2022.
//

#include "SolClusters.h"

// Used to compare two blocks in the min-max makespan algorithm
bool compare_blocks(mmBlock* first, mmBlock* second) {
	return first->value < second->value;
}

/*
 * SolClusters is a Solver.
 * Solves graph by first clustering the nodes into subtours, then solving TSP on those clusters.
 * In Development
 */

SolClusters::SolClusters() {}

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

	// Store the given parameters, we will be manipulating them
	m_vPotentialHL = vPotentialHL;
	m_vSPerHL = vSPerHL;
	m_vHLPerS = vHLPerS;

	do{
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
		kMeansClustering(vSelectedHL, vUnsolvedTours, m);

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
			minMaxMakespan(vCosts, (int)m_pSolution->m_nV, vOrder);

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
			if(tourCost(tour) > Q) {
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
				// No longer improving the solution
				runAgain = false;
				// Memory cleanup
				delete m_pSolution;
			}

			if(SANITY_PRINT)
				printf(" SolClusters() : Worst latency = %f\n", worstLat);
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
		printf(" Final result: m = %d, worst-lat = %f\n\n", (m-1), bestWorstLat);

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

/*
 * Performs Lloyd's k-means clustering algorithm on the points in vLoc to create m
 * clusters, the centroids of each cluster is stored in centroids. The algorithm
 * runs for epocks iterations.
 */
void SolClusters::kMeansClustering(std::vector<HoverLocation> &vLoc, std::vector<std::vector<UAV_Stop>> &vTours, int m) {
	// max/min x/y coords
	double min_x = vLoc.at(0).fX;
	double max_x = vLoc.at(0).fX;
	double min_y = vLoc.at(0).fY;
	double max_y = vLoc.at(0).fY;

	// Create initial centroids
	std::vector<KPoint> centroids;
	// Create a vector of KPoint that hold hover locations
	std::vector<KPoint> points;

	// Put each HL in vLoc into a KPoint
	for(long unsigned int i = 0; i < vLoc.size(); i++) {
		points.push_back(KPoint(&vLoc.at(i)));

		// Check for max/min x/y coordinates
		if(vLoc.at(i).fX > max_x) {
			max_x = vLoc.at(i).fX;
		}
		if(vLoc.at(i).fX < min_x) {
			min_x = vLoc.at(i).fX;
		}
		if(vLoc.at(i).fY > max_y) {
			max_y = vLoc.at(i).fY;
		}
		if(vLoc.at(i).fY < min_y) {
			min_y = vLoc.at(i).fY;
		}
	}

	// Seed rand()
	srand(time(NULL));

	// Create m centroids using random points from max/min x/y coords
	for(int i = 0; i < m; i++) {
		double x = min_x + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_x-min_x)));
		double y = min_y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_y-min_y)));

		// Create centroid
		KPoint cPoint = KPoint(x, y);
		cPoint.cluster = i;
		centroids.push_back(cPoint);
	}

	// The index of the centroid within the centroids vector
	int k = (int)centroids.size();

	// Track if centroids have moved
	bool cent_moved = true;

	// Run clustering iterations
	for(int i = 0; i < KMEANS_ITERATIONS && cent_moved; ++i) {
		// Check every edge against every centroid
		for(std::vector<KPoint>::iterator it = points.begin(); it != points.end(); ++it) {
			// For each centroid, compute distance from the centroid to the point
			// and update the point's cluster if necessary
			for(std::vector<KPoint>::iterator c = centroids.begin(); c != centroids.end(); ++c) {
				// Get centroid id
				int clusterId = c->cluster;
				// Check to see if this centroid is better than the last centroid
				KPoint p = *it;
				double dist = c->distance(p);
				if(dist < p.minDist) {
					p.minDist = dist;
					p.cluster = clusterId;
				}
				*it = p;
			}
		}

		// Create vectors to keep track of data needed to compute means
		std::vector<int> nPoints;
		std::vector<double> sumX, sumY;
		for (int j = 0; j < k; ++j) {
			nPoints.push_back(0);
			sumX.push_back(0.0);
			sumY.push_back(0.0);
		}

		// Iterate over points to append data to centroids
		for(std::vector<KPoint>::iterator it = points.begin(); it != points.end(); ++it) {
			int clusterId = it->cluster;
			nPoints[clusterId] += 1;
			sumX[clusterId] += it->x;
			sumY[clusterId] += it->y;

			it->minDist = __DBL_MAX__;  // reset distance
		}

		cent_moved = false;

		// Compute the new centroids
		for(std::vector<KPoint>::iterator c = centroids.begin(); c != centroids.end(); ++c) {
			int clusterId = c->cluster;
			double x = sumX[clusterId] / nPoints[clusterId];
			double y = sumY[clusterId] / nPoints[clusterId];

			if( (abs(x - c->x) > MOVE_EPS) || (abs(y - c->y) > MOVE_EPS) ) {
				c->x = x;
				c->y = y;
				cent_moved = true;
			}
		}
	}

	if(DEBUG_SLCLUST) {
		if(!cent_moved) {
			printf("Clustering settled\n");
		}
		else {
			printf("Clustering timed-out\n");
		}
	}

	// Create subtours to store results
	std::vector<std::list<HoverLocation>> subTours;
	for(int i = 0; i < m; i++) {
		// Create a sub-tour
		std::list<HoverLocation> temp;
		// Add the base station to the tour
		temp.push_back(m_vPotentialHL.back());
		// Add subtour to vector of tours
		subTours.push_back(temp);
	}


//	/// Attempt to make equally weighted clusters
//	// Store the results in the subtours
//	for(KPoint pnt : points) {
//		subTours.at(pnt.cluster).push_back(*pnt.v);
//	}
//
//	// Check weight of each cluster
//	for(auto tour : subTours) {
//		double wt = weightOfSubTour(tour);
//		printf(" tour weight: %f\n", wt);
//	}


	// Create m tours
	for(int i = 0; i < m; i++) {
		vTours.push_back(std::vector<UAV_Stop>());
		// Add the base station as the first stop
		vTours.at(i).push_back(m_vPotentialHL.back());
	}

	// Store the results
	if(DEBUG_SLCLUST)
		printf("Clusters per point:\n");
	for(KPoint pt : points) {
		int clust = pt.cluster;
		vTours.at(clust).push_back(UAV_Stop(*pt.v));

		if(DEBUG_SLCLUST) {
			printf(" %d %d\n", pt.v->nID, clust);
		}
	}
	if(DEBUG_SLCLUST) {
		printf("Point per cluster:\n");
		for(long unsigned int i = 0; i < vTours.size(); i++) {
			printf(" %ld: ", i);
			for(UAV_Stop stp : vTours.at(i)) {
				if(DEBUG_SLCLUST) {
					printf("%d ", stp.nID);
				}
			}
			printf("\n");
		}
	}
}

//// Determines the weight of sub-tour tour
//double SolClusters::weightOfSubTour(std::list<HoverLocation> tour) {
//	printf("Evaluating sub-tour weight\n");
//	// Find the min-spanning tree of this tour
//	double tourDist = Graph_Theory::MST_Prims(tour);
//	printf(" Found distance: %f\n", tourDist);
//
//	// Add up the total hovering cost of these UAV stops
//	double hoveringCost = 0;
//	for(HoverLocation hl : tour) {
//		for(int n : hl.nodes) {
//			hoveringCost += this->m_pSolution->m_pG->vNodeLst.at(n).sensorCost(hl);
//		}
//	}
//
//	// The weight is the cost of the min-spanning tree + 1/2 hovering cost
//	return tourDist + 0.5*hoveringCost;
//}

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

/*
 * Min-max makespan algorithm. Takes the m job times in vTimes and finds an ordering
 * for them on n machines, the result is stored in vOrder.
 */
void SolClusters::minMaxMakespan(std::vector<double> &vTimes, int n, std::vector<int> &vOrder) {
	int m = (int)vTimes.size();
	if(DEBUG_SLCLUST)
		printf("min-max makespan\n");
	double temp1 = vTimes.size()/(double)n;
	double temp2 = log(temp1)/log(2.0);
	if(DEBUG_SLCLUST)
		printf(" temp1 = %f\n temp2 = %f\n", temp1, temp2);
	int l = ceil(temp2);
	if(DEBUG_SLCLUST)
		printf(" l = %d\n", l);
	// How many objects do we need?
	int obj = n*pow(2,l);
	if(DEBUG_SLCLUST)
		printf(" obj = %d\n create: %d\n", obj, obj - m);

//	// Create extra "empty" jobs
//	for(int i = 0; i < (obj - m); i++) {
//		vTimes.push_back(0.0);
//	}

	std::list<mmBlock*> lBlocks;

	// Put jobs into blocks with extra "empty" jobs
	for(int i = 0; i < obj; i++) {
		if(i < (int)vTimes.size()) {
			// Create block with job time/index
			lBlocks.push_back(new mmBlock(vTimes.at(i),i));
		}
		else {
			// Create empty block
			lBlocks.push_back(new mmBlock(0.0, -1));
		}
	}

	// Start recursive algorithm
	std::list<mmBlock*> orderedList = blockMatch(lBlocks, n);

	// Sanity print
	if(DEBUG_SLCLUST) {
		printf("Final times:\n");
		for(mmBlock* bl : orderedList) {
			printf(" %f\n", bl->value);
		}
	}

	// Extract the answer that we just found
	for(mmBlock* blck : orderedList) {
		fillOrderVector(blck, vOrder);
	}
}

// Runs block-matching algorithm. Builds up the min-max makespan pyramid, returns a final list of the n pyramids
std::list<mmBlock*> SolClusters::blockMatch(std::list<mmBlock*> &lBlocks, int n) {
	int lstLen = (int)lBlocks.size();
	if(lstLen == n) {
		/// Base case
		if(DEBUG_SLCLUST)
			printf(" Hit basecase\n");
		return lBlocks;
	}
	else {
		/// Recursive case
		if(DEBUG_SLCLUST)
			printf(" Recursive case, list-length = %d\n", lstLen);
		std::list<mmBlock*> nxtLayer;
		// Sort list
		lBlocks.sort(compare_blocks);
		// Build new layer, matching larger blocks with smallest blocks
		std::list<mmBlock*>::iterator itFront = lBlocks.begin();
		std::list<mmBlock*>::reverse_iterator itBack = lBlocks.rbegin();
		for(int i = 0; i < lstLen/2; i++, itFront++, itBack++) {
			nxtLayer.push_back(new mmBlock(*itFront, *itBack));
		}
		// Recursive call
		return blockMatch(nxtLayer, n);
	}
}

// Extracts the indexes from the pyramid, stores solution in vOrder
void SolClusters::fillOrderVector(mmBlock* blck, std::vector<int> &vOrder) {
	if(blck->child1 == NULL && blck->child2 == NULL) {
		// Found the bottom, add index to vOrder
		vOrder.push_back(blck->index);
	}
	else {
		// Not at base of pyramid, call fillOrderVector() on children
		fillOrderVector(blck->child1, vOrder);
		fillOrderVector(blck->child2, vOrder);
	}
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
