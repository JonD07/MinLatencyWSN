#include "Graph_Theory.h"

Graph_Theory::Graph_Theory() {}

Graph_Theory::~Graph_Theory() {}


// Finds the distance of a Min-Spanning-Tree using Prim's Algorithm
double Graph_Theory::MST_Prims(std::list<HoverLocation> tour) {
	if(GRAPH_THEORY_DEBUG) {
		printf("Running Prim's Algorithm\n");
	}

	double dist = 0;

	// Create two lists to hold the vertices
	std::list<HoverLocation> t_list;
	std::list<HoverLocation> g_list;

	// Add all vertices to graph-list
	for(HoverLocation stp : tour) {
		g_list.push_back(stp);
	}

	// To start, add a "random" vertex to the tree
	t_list.push_back(g_list.front());
	g_list.pop_front();

	if(GRAPH_THEORY_DEBUG) {
		printf("Ready to start, |g-list| = %ld, |t-list| = %ld\n", g_list.size(), t_list.size());
	}

	// Run until there are not more vertices in graph-list
	while(!g_list.empty()) {
		// Find shortest edge between tree-list and graph-list
		double shortest_leg = std::numeric_limits<float>::max();
		std::list<HoverLocation>::iterator t_it = t_list.begin(), best_v = g_list.begin();
		for (; t_it != t_list.end(); ++t_it) {
			std::list<HoverLocation>::iterator g_it = g_list.begin();
			for (; g_it != g_list.end(); ++g_it) {
				double dist_t_to_g = (*t_it).distTo(*g_it);
				if(dist_t_to_g < shortest_leg) {
					shortest_leg = dist_t_to_g;
					best_v = g_it;
				}
			}
		}

		// Sanity Print
		if(GRAPH_THEORY_DEBUG) {
			printf("Moving %d into tree\n", (*best_v).nID);
		}

		// Move the next closest vertex into the tree, remove from graph
		HoverLocation move_to_tree = *best_v;
		dist += shortest_leg;
		g_list.erase(best_v);
		t_list.push_back(move_to_tree);
	}

	// Sanity Print
	if(GRAPH_THEORY_DEBUG) {
		printf("Found MST distance of %f\n", dist);
	}

	return dist;
}
