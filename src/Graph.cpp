#include "Graph.h"

Graph::Graph(std::string graph_path) {
	printf("Creating graph\n");
	// Open file and read in data
	std::ifstream file(graph_path);
	std::string line;
	// Grab first line with n (number of sensors to visit)
	std::getline(file, line);
	std::stringstream lineStreamN(line);
	// Parse n from line
	int n;
	lineStreamN >> n;
	if(DEBUG)
		printf("Graph of n: %d\n", n);

	// Grab the next n lines, containing x,y,p
	for(int i = 0; i < n; i++) {
		std::getline(file, line);
		std::stringstream lineStream(line);
		// Parse line
		double x, y, r, p;
		lineStream >> x >> y >> r >> p;
		if(DEBUG)
			printf(" (%f, %f), R: %f, P: %f\n", x, y, r, p);
		vNodeLst.push_back(Node(i,x,y,r,p));
	}

	// get the location of the base station
	std::getline(file, line);
	std::stringstream lineStream(line);
	// Parse line
	double x,y;
	lineStream >> x >> y;
	mBaseStation = Node(n, x, y, 0, 0);
	if(DEBUG)
		printf(" bs: (%f, %f)\n |nodesLst| = %ld\n", mBaseStation.getX(), mBaseStation.getY(), vNodeLst.size());
}

Graph::~Graph() {}


void Graph::MarkNode(int id) {
	vNodeLst.at(id).setLocked();
}

