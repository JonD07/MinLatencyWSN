//
// Created by peter on 10/5/2022.
//

#include "Cluster.h"

Cluster::Cluster(int nPaths) : nPaths(nPaths) {



}

Cluster::Cluster(const Cluster &c){
	nPaths = c.nPaths;
}

Cluster::~Cluster(){}

//General function to run the clustering algorithm. Enables consistent start procedures to the algorithm.
std::vector<std::vector<HoverLocation>> Cluster::RunCluster(std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS){

	//Add additional functionality to all cluster classes here.

	createCluster(vPotentialHL,vSPerHL,vHLPerS);
	return vGrouping;

}
