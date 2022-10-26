/*
 * Graph_Theory.h
 *
 * Created by:	Jonathan Diller
 * On: 			Oct 22, 2022
 *
 * Description: Static class that holds common graph theory algorithms
 */

#pragma once

#include <sstream>
#include <fstream>
#include <vector>
#include <limits>

#include "defines.h"
#include "HoverLocation.h"

#define GRAPH_THEORY_DEBUG	1

class Graph_Theory {
public:
	Graph_Theory();
	~Graph_Theory();

	// Finds the distance of a Min-Spanning-Tree using Prim's Algorithm on complete graphs with n vertices
	static double MST_Prims(std::list<HoverLocation> tour);
private:
};
