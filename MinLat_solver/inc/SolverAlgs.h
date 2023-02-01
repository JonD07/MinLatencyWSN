/*
 * SolverAlgs.h
 *
 * Created by:	Jonathan Diller
 * On: 			02/01/2023
 *
 * Description: This class holds a variety of algorithms that are used by the different solvers
 */

#pragma once

#include <sstream>
#include <fstream>
#include <list>
#include <limits>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "Solver.h"
#include "HoverLocation.h"
#include "Utilities.h"
#include "defines.h"
#include "LKH_TSP_Solver.h"
#include "Graph_Theory.h"


#define DEBUG_SOLALG	DEBUG || 0

#define KMEANS_ITERATIONS	10000
#define MOVE_EPS			0.1

struct KPoint {
	double x, y;
	int cluster;
	bool locked;
	double minDist;
	HoverLocation* v;

	KPoint(double x, double y) : x(x), y(y), cluster(-1), locked(false), minDist(__DBL_MAX__), v(NULL) {}
	KPoint(HoverLocation* vrtx) : cluster(-1), locked(false), minDist(__DBL_MAX__), v(vrtx) {
		x = vrtx->fX;
		y = vrtx->fY;
	}

	/**
	* Computes the (square) Euclidean distance between this point and another
	*/
	double distance(KPoint p) {
		return (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);
	}
};

struct mmBlock {
	// The value of this block, which may be the sum of the
	// two children or may be a job processing time
	double value;
	// The index in the job list that this block represents,
	// or -1 if it is a non-base building block
	int index;
	// Used if this is non-base block, to point at the two
	// blocks beneath it
	mmBlock* child1;
	mmBlock* child2;

	mmBlock(double val, int i) {
		value = val;
		index = i;
		child1 = NULL;
		child2 = NULL;
	}

	mmBlock(mmBlock* bl1, mmBlock* bl2) {
		value = bl1->value + bl2->value;
		index = -1;
		child1 = bl1;
		child2 = bl2;
	}
};

class SolverAlgs {
public:
	SolverAlgs();

	/*
	 * Performs Lloyd's k-means clustering algorithm on the points in vLoc to create m
	 * clusters, the centroids of each cluster is stored in centroids. The algorithm
	 * runs for KMEANS_ITERATIONS iterations or until the centroids settle within 0.1,
	 * which ever comes first.
	 */
	void kMeansClustering(std::vector<HoverLocation> &vLoc, std::vector<std::vector<UAV_Stop>> &vTours, HoverLocation& hlBS, int m);
	/*
	 * Min-max makespan algorithm. Takes the job times in vTimes and finds an ordering
	 * for them on m machines, the result is stored in vOrder.
	 */
	void minMaxMakespan(std::vector<double> &vTimes, int m, std::vector<int> &vOrder);

private:
	// Runs block-matching algorithm. Builds up the min-max makespan pyramid, returns a final list of the n pyramids
	std::list<mmBlock*> blockMatch(std::list<mmBlock*> &lBlocks, int n);
	// Extracts the indexes from the pyramid, stores solution in vOrder
	void fillOrderVector(mmBlock* blck, std::vector<int> &vOrder);

};

