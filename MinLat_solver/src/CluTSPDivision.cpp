//
// Created by peter on 10/5/2022.
//

#include "CluTSPDivision.h"


//***********************************************************
// Public Member Functions
//***********************************************************

CluTSPDivision::CluTSPDivision(int nPaths) : Cluster(nPaths) {}

CluTSPDivision::CluTSPDivision(const Cluster &c) : Cluster(c) {}


//***********************************************************
// Protected Member Functions
//***********************************************************

//Runs the clustering algorithm and populates vGrouping.
void CluTSPDivision::createCluster(std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS){
	//TODO Implement clustering algorithm
}
