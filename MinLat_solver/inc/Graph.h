/*
 * Graph.h
 *
 * Created by:	Jonathan Diller
 * On: 			Mar 13, 2022
 *
 * Description: Graph class
 */

#pragma once

#include <sstream>
#include <fstream>
#include <vector>

#include "Node.h"
#include "defines.h"

class Graph {
public:
	Graph(std::string graph_path);
	~Graph();

	void MarkNode(int id);

	// Vector to hold all of the nodes in this graph
	std::vector<Node> vNodeLst;
	// Base station node
	// TODO: should the base station be Node or one of the waypoint classes?
	Node mBaseStation;
private:
};
