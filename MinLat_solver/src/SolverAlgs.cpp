#include "SolverAlgs.h"

// Used to compare two blocks in the min-max makespan algorithm
bool compare_blocks(mmBlock* first, mmBlock* second) {
	return first->value < second->value;
}


/*
 * SolClusters is a Solver.
 * Solves graph by first clustering the nodes into subtours, then solving TSP on those clusters.
 * In Development
 */

SolverAlgs::SolverAlgs() {}


//***********************************************************
// Public Member Functions
//***********************************************************


/*
 * Performs Lloyd's k-means clustering algorithm on the points in vLoc to create m
 * clusters, the centroids of each cluster is stored in centroids. The algorithm
 * runs for epocks iterations.
 */
void SolverAlgs::kMeansClustering(std::vector<HoverLocation> &vLoc, std::vector<std::vector<UAV_Stop>> &vTours, HoverLocation& hlBS, int m) {
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

	if(DEBUG_SOLALG) {
		if(!cent_moved) {
			printf("Clustering settled\n");
		}
		else {
			printf("Clustering timed-out\n");
		}
	}

	// Create m tours
	for(int i = 0; i < m; i++) {
		vTours.push_back(std::vector<UAV_Stop>());
		// Add the base station as the first stop
		vTours.at(i).push_back(hlBS);
	}

	// Store the results
	if(DEBUG_SOLALG)
		printf("Clusters per point:\n");
	for(KPoint pt : points) {
		int clust = pt.cluster;
		vTours.at(clust).push_back(UAV_Stop(*pt.v));

		if(DEBUG_SOLALG) {
			printf(" %d %d\n", pt.v->nID, clust);
		}
	}
	if(DEBUG_SOLALG) {
		printf("Point per cluster:\n");
		for(long unsigned int i = 0; i < vTours.size(); i++) {
			printf(" %ld: ", i);
			for(UAV_Stop stp : vTours.at(i)) {
				if(DEBUG_SOLALG) {
					printf("%d ", stp.nID);
				}
			}
			printf("\n");
		}
	}
}

/*
 * Min-max makespan algorithm. Takes the m job times in vTimes and finds an ordering
 * for them on n machines, the result is stored in vOrder.
 */
void SolverAlgs::minMaxMakespan(std::vector<double> &vTimes, int n, std::vector<int> &vOrder) {
	int m = (int)vTimes.size();
	if(DEBUG_SOLALG)
		printf("min-max makespan\n");
	double temp1 = vTimes.size()/(double)n;
	double temp2 = log(temp1)/log(2.0);
	if(DEBUG_SOLALG)
		printf(" temp1 = %f\n temp2 = %f\n", temp1, temp2);
	int l = ceil(temp2);
	if(DEBUG_SOLALG)
		printf(" l = %d\n", l);
	// How many objects do we need?
	int obj = n*pow(2,l);
	if(DEBUG_SOLALG)
		printf(" obj = %d\n create: %d\n", obj, obj - m);

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
	if(DEBUG_SOLALG) {
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



//***********************************************************
// Private Member Functions
//***********************************************************


// Runs block-matching algorithm. Builds up the min-max makespan pyramid, returns a final list of the n pyramids
std::list<mmBlock*> SolverAlgs::blockMatch(std::list<mmBlock*> &lBlocks, int n) {
	int lstLen = (int)lBlocks.size();
	if(lstLen == n) {
		/// Base case
		if(DEBUG_SOLALG)
			printf(" Hit basecase\n");
		return lBlocks;
	}
	else {
		/// Recursive case
		if(DEBUG_SOLALG)
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
void SolverAlgs::fillOrderVector(mmBlock* blck, std::vector<int> &vOrder) {
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

