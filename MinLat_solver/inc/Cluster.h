//
// Created by peter on 10/5/2022.
//

#pragma once
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>


#include "HoverLocation.h"

/*
 * Cluster is a virtual parent class that implements clustering algorithms. Refer to child classes with the prefix "Clu" for specific implementations.
 */

class Cluster {
public:

	Cluster(int nPaths);

	Cluster(const Cluster &c);

	~Cluster();

	//Grouping generated by createCluster.
	std::vector<std::vector<HoverLocation>> vGrouping;

	//Number of groups within the grouping
	int nPaths;

	//General function to run the clustering algorithm. Enables consistent start procedures to the algorithm.
	std::vector<std::vector<HoverLocation>> RunCluster(std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);

protected:

	//Runs the clustering algorithm and populates vGrouping.
	virtual void createCluster(std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS) = 0;

};


