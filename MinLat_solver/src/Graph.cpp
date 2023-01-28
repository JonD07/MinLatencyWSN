#include "Graph.h"

Graph::Graph(std::string graph_path) {
	if(SANITY_PRINT)
		printf("Creating graph\n");
	sGFile = graph_path;
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
		double x, y, r;
		lineStream >> x >> y >> r;
		if(DEBUG)
			printf(" (%f, %f), R: %f\n", x, y, r);
		vNodeLst.push_back(Node(i,x,y,r));
	}

	// get the location of the base station
	std::getline(file, line);
	std::stringstream lineStream(line);
	// Parse line
	double x,y;
	lineStream >> x >> y;
	mBaseStation = Node(n, x, y, 0);
	if(DEBUG)
		printf(" bs: (%f, %f)\n |nodesLst| = %ld\n", mBaseStation.getX(), mBaseStation.getY(), vNodeLst.size());
}

Graph::~Graph() {}

void Graph::MarkNode(int id) {
	vNodeLst.at(id).setLocked();
}


// Returns true if node id has been marked
bool Graph::IsMark(int id) {
	return vNodeLst.at(id).Locked();
}

// Resets which nodes have been marked
void Graph::ResetMarking() {
	for(auto it = vNodeLst.begin(); it != vNodeLst.end(); it++) {
		it->unLock();
	}
}
