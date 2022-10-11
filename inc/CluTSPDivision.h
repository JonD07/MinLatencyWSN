//
// Created by peter on 10/5/2022.
//

#pragma once
#include "Cluster.h"

/*
 * CluTSPDivision inherits from the Cluster class.
 *
 * TSPDivision separates the graphs into clusters by first finding a single route through all nodes, then splitting it into nPaths groupings.
 * Potential for some optimization by determining the highest cost nodes and redistributing them.
 * Under Development
 */

class CluTSPDivision : Cluster {
public:

	CluTSPDivision(int nPaths);

	CluTSPDivision(const Cluster &c);

protected:

	//Runs the clustering algorithm and populates vGrouping.
	void createCluster(std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS);

};

