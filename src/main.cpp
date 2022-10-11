#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>

#include "SolBasicMILP.h"
#include "pNode.h"
#include "HoverLocation.h"
#include "Graph.h"
#include "defines.h"
#include "gurobi_c++.h"
#include "Roots.h"
#include "UAV_Stop.h"
#include "Utilities.h"
#include "Solver.h"
#include "SolNearestNeighbor.h"
#include "SolHardMILP.h"
#include "Solution.h"
#include "SolClusters.h"



// Determines the distance of this tour
double tourDist(std::vector<Node> &tourNodes) {
	double dist = 0;
	Node nxt, lst;
	lst = tourNodes.front();

	// Add up distance between each node
	for(long unsigned int i = 1; i < tourNodes.size(); i++) {
		nxt = tourNodes.at(i);
		dist += lst.GetDistanceTo(&nxt);
		lst = nxt;
	}

	// Add distance to return to the first node
	nxt = tourNodes.front();
	dist += lst.GetDistanceTo(&nxt);

	return dist;
}

	double tourDist(std::list<UAV_Stop> &tour) {
	double dist = 0;

	if(tour.size() > 1) {
		std::list<UAV_Stop>::iterator lst, nxt;
		lst = tour.begin();
		nxt = tour.begin();
		nxt++;

		// Run through the tour, add up distance from stop-to-stop
		while(nxt != tour.end()) {
			// Add distance
			dist += distAtoB(lst->fX, lst->fY, nxt->fX, nxt->fY);
//			printf(" * add distance %f\n", distAtoB(lst->fX, lst->fY, nxt->fX, nxt->fY));
			// Advance iterators
			lst++;
			nxt++;
		}
	}

//	printf(" * total distance %f\n", dist);
	return dist;
}



void findOverlapPoints(Node* v, Node* u, Roots* x_roots, Roots* y_roots) {
	// Node ranges overlap, there are 2 potential hovering location here
	// Find overlapping points, math here: https://math.stackexchange.com/a/256123/661482
	double x1 = u->getX();
	double y1 = u->getY();
	double r1 = u->getR();
	double x2 = v->getX();
	double y2 = v->getY();
	double r2 = v->getR();

	// Find the y intercept values
	double c1 = -(y1 - y2)/(x1 - x2);
	double c2 = -(pow(r1,2) - pow(r2,2))/(2*(x1-x2))
				+ (pow(x1,2) - pow(x2,2))/(2*(x1 - x2))
				+ (pow(y1,2) - pow(y2,2))/(2*(x1 - x2));

	// ay^2 + by + c = 0
	double a = pow(c1,2) + 1;
	double b = (2*c1*(c2 - x1)-2*y1);
	double c = pow((c2 - x1),2) + pow(y1,2) - pow(r1,2);

	y_roots -> findRoots(a, b, c);

	// Find the x intercept values
	double c3 = -(x1 - x2)/(y1 - y2);
	double c4 = -(pow(r1,2) - pow(r2,2))/(2*(y1-y2))
				+ (pow(x1,2) - pow(x2,2))/(2*(y1 - y2))
				+ (pow(y1,2) - pow(y2,2))/(2*(y1 - y2));

	// ax^2 + bx + c = 0
	a = pow(c3,2) + 1;
	b = (2*c3*(c4 - y1)-2*x1);
	c = pow((c4 - y1),2) + pow(x1,2) - pow(r1,2);

	x_roots -> findRoots(a, b, c);

	if(isZero(pow(x_roots->root1 - x1, 2) + pow(y_roots->root1 - y1, 2) - pow(r1, 2))) {
		printf("x_roots.root1 pairs with y_roots.root1\n");
		// Do nothing
	}
	else if(isZero(pow(x_roots->root1 - x1, 2) + pow(y_roots->root2 - y1, 2) - pow(r1, 2))) {
		printf("x_roots.root1 pairs with y_roots.root2\n");
		// Swap the roots so they match
		double temp = y_roots->root2;
		y_roots->root2 = y_roots->root1;
		y_roots->root1 = temp;
	}
	else {
		printf("Pairing failed\n");
	}
}

void findRadiusPaths(Graph* G, int algorithm, int numUAVs) {
	int id = 0;
	std::vector<HoverLocation> vPotentialHL;

	// Add base-station as first hovering location
	vPotentialHL.push_back(HoverLocation(id++, G->mBaseStation.getX(), G->mBaseStation.getY(), 0));

	/// 1. Form list of potential hovering-locations
	for(Node v : G->vNodeLst) {
		/// Determine point within v's range closest to base station
//		double x_1 = G->mBaseStation.getX() - v.getX();
//		double x_2 = G->mBaseStation.getY() - v.getY();
//		double a = v.getR()/(sqrt(pow(x_1, 2) + pow(x_2, 2)));
//		double x_p = v.getX() + x_1*a;
//		double y_p = v.getY() + x_2*a;
//		// Determine the weight of visiting this location
//		double weight = distAtoB(x_p, y_p , G->mBaseStation.getX(), G->mBaseStation.getY());
//		// Create a potential hovering-location
//		printf("New hovering point at edge of R: %d @ (%f, %f), weight: %f\n", id, x_p, y_p, weight);
//		vPotentialHL.push_back(HoverLocation(id++, x_p, y_p, weight));

		/// Consider "directly above" as a potential hovering location
		printf("New hovering point above node: %d @ (%f, %f), weight: %f\n", id, v.getX(), v.getY(),
			   distAtoB(v.getX(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(HoverLocation(id++, v.getX(), v.getY(),
											 distAtoB(v.getX(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY())));

		/// Consider cardinal directions on radius as a potential hovering location
		// North
		printf("New hovering point north of node: %d @ (%f, %f), weight: %f\n", id, v.getX() + v.getR(), v.getY(),
			   distAtoB(v.getX() + v.getR(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(HoverLocation(id++, v.getX() + v.getR(), v.getY(),
											 distAtoB(v.getX() + v.getR(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY())));
		// South
		printf("New hovering point south of node: %d @ (%f, %f), weight: %f\n", id, v.getX() - v.getR(), v.getY(),
			   distAtoB(v.getX() - v.getR(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(HoverLocation(id++, v.getX() - v.getR(), v.getY(),
											 distAtoB(v.getX() - v.getR(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY())));
		// East
		printf("New hovering point east of node: %d @ (%f, %f), weight: %f\n", id, v.getX(), v.getY() + v.getR(),
			   distAtoB(v.getX(), v.getY() + v.getR(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(HoverLocation(id++, v.getX(), v.getY() + v.getR(),
											 distAtoB(v.getX(), v.getY() + v.getR(), G->mBaseStation.getX(), G->mBaseStation.getY())));
		// West
		printf("New hovering point east of node: %d @ (%f, %f), weight: %f\n", id, v.getX(), v.getY() - v.getR(),
			   distAtoB(v.getX(), v.getY() - v.getR(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(HoverLocation(id++, v.getX(), v.getY() - v.getR(),
											 distAtoB(v.getX(), v.getY() - v.getR(), G->mBaseStation.getX(), G->mBaseStation.getY())));

		/// Check all nodes to see if there is overlap
		for(long unsigned int i = (v.getID() + 1); i < G->vNodeLst.size(); i++) {
			Node u = G->vNodeLst.at(i);
			// Do communication radii overlap?
			if(u.GetDistanceTo(&v) <= (u.getR() + v.getR())) {
				// Find overlapping points
				Roots y_roots;
				Roots x_roots;
				findOverlapPoints(&v, &u, &x_roots, &y_roots);

				// Add the two overlapping points to the solution
				if(x_roots.imaginary || y_roots.imaginary) {
					// Repeated roots...
					double x_1 = u.getX() - v.getX();
					double x_2 = u.getY() - v.getY();
					double a = v.getR()/(sqrt(pow(x_1, 2) + pow(x_2, 2)));
					double x_p = v.getX() + x_1*a;
					double y_p = v.getY() + x_2*a;
					// Determine the weight of visiting this location
					double weight = distAtoB(x_p, y_p , u.getX(), u.getY());
					// Create a potential hovering-location
					printf("New hovering point at edge of two circles: %d @ (%f, %f), weight: %f\n", id, x_p, y_p, weight);
					vPotentialHL.push_back(HoverLocation(id++, x_p, y_p, weight));
				}
				else {
					// Add both roots
					printf("New hovering point from root1: %d @ (%f, %f), weight: %f\n", id, x_roots.root1, y_roots.root1,
						   distAtoB(x_roots.root1, y_roots.root1, G->mBaseStation.getX(), G->mBaseStation.getY()));
					vPotentialHL.push_back(HoverLocation(id++, x_roots.root1, y_roots.root1,
														 distAtoB(x_roots.root1, y_roots.root1, G->mBaseStation.getX(), G->mBaseStation.getY())));
					printf("New hovering point from root2: %d @ (%f, %f), weight: %f\n", id, x_roots.root2, y_roots.root2,
						   distAtoB(x_roots.root2, y_roots.root2, G->mBaseStation.getX(), G->mBaseStation.getY()));
					vPotentialHL.push_back(HoverLocation(id++, x_roots.root2, y_roots.root2,
														 distAtoB(x_roots.root2, y_roots.root2, G->mBaseStation.getX(), G->mBaseStation.getY())));
				}
			}
		}
	}

	// Add base-station as the last hovering location
	vPotentialHL.push_back(HoverLocation(id++, G->mBaseStation.getX(), G->mBaseStation.getY(), 0));

	/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
	// Make list of sensors for each hovering location
	std::vector<std::list<int>> vSPerHL;
	// For each hovering location
	for(HoverLocation hl : vPotentialHL) {
		printf("Considering HL: %d, adding sensors: ", hl.nID);
		// Make a list containing each node within range
		std::list<int> sList;
		for(Node n : G->vNodeLst) {
			// Check if n is in range of hl
			if(distAtoB(n.getX(), n.getY(), hl.fX, hl.fY) <= (n.getR() + EPSILON)) {
				printf("%d ", n.getID());
				// In range, add to list
				sList.push_back(n.getID());
			}
		}
		printf("\n");
		// Add nodes-list to collection of lists
		vSPerHL.push_back(sList);
	}

	// Make list of hovering locations for each node
	std::vector<std::list<int>> vHLPerS;
	// For each sensor
	for(Node n : G->vNodeLst) {
		printf("Considering Sensor: %d, adding HLs: ", n.getID());
		// Make a list containing each hovering location within range
		std::list<int> hlList;
		// For each hovering location
		for(HoverLocation hl : vPotentialHL) {
			// Check if hl is in range of n
			if(distAtoB(n.getX(), n.getY(), hl.fX, hl.fY) <= (n.getR() + EPSILON)) {
				printf("%d ", hl.nID);
				// In range, add to list
				hlList.push_back(hl.nID);
			}
		}
		printf("\n");
		// Add hl-list to collection of lists
		vHLPerS.push_back(hlList);
	}


	/// 3. Solve capacitated VRP
	Solver *solver;


	//TODO Change to be correct....
	if(true){
		solver = new SolClusters();
	}
	else if(algorithm == MILP_I) {
		solver = new SolBasicMILP();
	}
	else if(algorithm == GREEDY_NN) {
		solver = new SolNearestNeighbor();
	}
	else if(algorithm == MILP_II) {
		solver = new SolHardMILP();
	}

	Solution* solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);


	// Print Results before improvements
	if(PRINT_RESULTS) {
		solution->printResults();
	}


	/// 4. Improve route
	printf("\nImprove route\n");
	for(unsigned long int i = 0; i < solution->vTours.size(); i++) {
		bool runAgain = true;

		// While moving stops helped... run again!
		while(runAgain) {
			runAgain = false;
//			printf(" %ld:\n",i);
			if(solution->vTours.at(i).size() > 2) {
				std::list<UAV_Stop>::iterator back = solution->vTours.at(i).begin();
				std::list<UAV_Stop>::iterator middle = solution->vTours.at(i).begin();
				middle++;
				std::list<UAV_Stop>::iterator front = solution->vTours.at(i).begin();
				front++;
				front++;

				while(front != solution->vTours.at(i).end()) {
					// Attempt to remove the middle stop
//					printf("  Pretending to remove (%f, %f)\n", middle->fX, middle->fY);

					// For each sensor
					UAV_Stop tempOld = *middle;
					for(int n : tempOld.nodes) {
						double x_b = back->fX;
						double y_b = back->fY;
						double x_f = front->fX;
						double y_f = front->fY;
						double x_v = G->vNodeLst[n].getX();
						double y_v = G->vNodeLst[n].getY();

						// Find the point, v', closest to this sensor, v, on the line between stops back and front
						double m = (y_f - y_b)/(x_f - x_b);
						double b = y_f - m * x_f;
						double x_p = (x_v - m*(b - y_v))/(m*m + 1);
						double y_p = m*x_p + b;

//						printf("   l = %d, (x_p, y_p) = (%f, %f)\n", n, x_p, y_p);

						// Find magnitude of vector u from v to v'
						double mag_u = sqrt(pow((x_v - x_p), 2) + pow((y_v - y_p), 2));

						// Check to see if v' is in-range of v
						if(mag_u <= G->vNodeLst[n].getR()) {
							// We are in-range
//							printf("    this point is in-range of v!\n");
							// Get total distance of current tour
							double oldDist = tourDist(solution->vTours.at(i));
							// Hold onto old stop
							UAV_Stop oldStop(*middle);
							// Create new UAV stop
							UAV_Stop tempStop(x_p, y_p);
							// Determine which sensors we talk to at this hovering location
							for(int n : oldStop.nodes) {
								if(distAtoB(G->vNodeLst[n].getX(), G->vNodeLst[n].getY(),
											tempStop.fX, tempStop.fY) <= (G->vNodeLst[n].getR() + EPSILON)) {
//									printf("    talk to %d\n", n);
									tempStop.nodes.push_back(n);
								}
							}
							// Make sure that we have the entire list...
							if(oldStop.nodes.size() == tempStop.nodes.size()) {
//								printf("     Easy swap!\n");
								// Add this stop to the tour, remove current middle
								solution->vTours.at(i).erase(middle);
								middle = front;
								// Add new stop
								solution->vTours.at(i).insert(middle, tempStop);
								// Correct iterator's position
								middle--;
								// Check new tour's distance
								double newDist = tourDist(solution->vTours.at(i));
								// See if we made an improvement
								if(newDist < oldDist) {
									// Great!
//									printf("    Update helped!\n");
									runAgain = true;
									break;
								}
								else {
									// Adding the new stop didn't help.. remove it
//									printf("    No improvement\n");
									solution->vTours.at(i).erase(middle);
									middle = front;
									solution->vTours.at(i).insert(middle, oldStop);
									middle--;
								}
							}
							else {
								// We can't easily swap out the two stops...
//								printf("    No easy swap\n");
							}
						}
						else {
							// Find point along radius that is in-range
							double x_1 = x_p - x_v;
							double x_2 = y_p - y_v;
							double a = G->vNodeLst[n].getR()/(sqrt(pow(x_1, 2) + pow(x_2, 2)));
							double x_pp = x_v + x_1*a;
							double y_pp = y_v + x_2*a;

//							printf("    (x_pp, y_pp) = (%f, %f)\n", x_pp, y_pp);

							// Get total distance of current tour
							double oldDist = tourDist(solution->vTours.at(i));
							// Hold onto old stop
							UAV_Stop oldStop(*middle);
							// Create new UAV stop
							UAV_Stop tempStop(x_pp, y_pp);
							// Determine which sensors we talk to at this hovering location
							for(int n : oldStop.nodes) {
								if(distAtoB(G->vNodeLst[n].getX(), G->vNodeLst[n].getY(),
											tempStop.fX, tempStop.fY) <= (G->vNodeLst[n].getR() + EPSILON)) {
//									printf("    talk to %d\n", n);
									tempStop.nodes.push_back(n);
								}
							}
							// Make sure that we have the entire list...
							if(oldStop.nodes.size() == tempStop.nodes.size()) {
//								printf("     Easy swap!\n");
								// Add this stop to the tour, remove current middle
								solution->vTours.at(i).erase(middle);
								middle = front;
								// Add new stop
								solution->vTours.at(i).insert(middle, tempStop);
								// Correct iterator's position
								middle--;
								// Check new tour's distance
								double newDist = tourDist(solution->vTours.at(i));
								// See if we made an improvement
								if(newDist < oldDist) {
									// Great!
//									printf("    Update helped!\n");
									runAgain = true;
									break;
								}
								else {
									// Adding the new stop didn't help.. remove it
//									printf("    No improvement\n");
									solution->vTours.at(i).erase(middle);
									middle = front;
									solution->vTours.at(i).insert(middle, oldStop);
									middle--;
								}
							}
							else {
								// We can't easily swap out the two stops...
//								printf("    No easy swap\n");
							}
						}
					}

					// Update iterators
					back++;
					middle++;
					front++;
				}
			}
		}
	}

	// Sanity print
	{
		printf("\nImproved tour:\n");
		int i = 0;
		for(std::list<UAV_Stop> l : solution->vTours) {
			printf(" %d: ", i);
			for(UAV_Stop n : l) {
				printf("(%f, %f) ", n.fX, n.fY);
			}
			printf("\n");
			i++;
		}
	}

	// Print Results
	if(PRINT_RESULTS) {
		solution->printResults();
	}

	// Print Plot data
	if(MAKE_PLOT_FILE) {
		// Print found path to file
		FILE * pOutputFile;
		pOutputFile = fopen(PLOT_FILE_LOCATION, "w");
		printf("\nMaking Plot File\n");
		for(std::list<UAV_Stop> l : solution->vTours) {
			for(UAV_Stop n : l) {
				fprintf(pOutputFile, "%f %f\n", n.fX, n.fY);
			}
//			fprintf(pOutputFile, "\n");
		}
		fclose(pOutputFile);
	}


	delete solution;
}

int main(int argc, char *argv[]) {
	int algorithm = GREEDY_NN;
	int numUAVs = 2;

	// TODO: change the input arguments, we want: algorithm selection, num UAV, input file path
	// Verify user input
	if(argc != 2) {
		printf("Expected use:min-lat <file path> min-max? heuristic? Got %d args, expected 2.\n", argc);
		return 1;
	}

//	// Display given configurations
//	printf("\nUsing graph: %s\n", argv[1]);
//	if(atoi(argv[2])) {
//		printf(" Min-Max: true\n");
//		MIN_MAX = true;
//	}
//	else {
//		printf(" Min-Max: false\n");
//		MIN_MAX = false;
//	}
//
//	if(atoi(argv[3])) {
//		printf(" Initial Solution Heuristic: true\n");
//		INITIAL_SOLUTION = true;
//	}
//	else {
//		printf(" Initial Solution Heuristic: false\n");
//		INITIAL_SOLUTION = false;
//	}
//
//	if(atoi(argv[4])) {
//		printf(" Priority Branching: true\n");
//		PRIORITIES = true;
//	}
//	else {
//		printf(" Priority Branching: false\n");
//		PRIORITIES = false;
//	}
//
//	if(atoi(argv[5])) {
//		printf(" Add clique-cuts: true\n");
//		CLIQUE_CUTS = true;
//	}
//	else {
//		printf(" Add clique-cuts: false\n");
//		CLIQUE_CUTS = false;
//	}

	printf("\n\n");

	// Create the graph
	Graph G(argv[1]);

	findRadiusPaths(&G, algorithm, numUAVs);
}
