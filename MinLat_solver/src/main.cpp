#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>
#include <chrono>

#include "defines.h"
#include "gurobi_c++.h"

#include "pNode.h"
#include "HoverLocation.h"
#include "Graph.h"
#include "Roots.h"
#include "UAV_Stop.h"
#include "Utilities.h"
#include "Solver.h"
#include "Solution.h"

#include "SolBasicMILP.h"
#include "SolNearestNeighbor.h"
#include "SolHardMILP.h"
#include "SolClusters.h"
#include "SolDivideGreedy.h"


#define DEBUG_MAIN	DEBUG || 0
#define DEBUG_IMPR	DEBUG || 0


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
		if(DEBUG_MAIN)
			printf("x_roots.root1 pairs with y_roots.root1\n");
		// Do nothing
	}
	else if(isZero(pow(x_roots->root1 - x1, 2) + pow(y_roots->root2 - y1, 2) - pow(r1, 2))) {
		if(DEBUG_MAIN)
			printf("x_roots.root1 pairs with y_roots.root2\n");
		// Swap the roots so they match
		double temp = y_roots->root2;
		y_roots->root2 = y_roots->root1;
		y_roots->root1 = temp;
	}
	else {
		if(DEBUG_MAIN)
			printf("Pairing failed\n");
	}
}

void setHLAllCombos(Graph* G, std::vector<HoverLocation>& vPotentialHL, std::vector<std::list<int>>& vSPerHL, std::vector<std::list<int>>& vHLPerS) {
	// Hovering location ID tracker
	int id = 0;

	// Add base-station as first hovering location
	vPotentialHL.push_back(HoverLocation(id++, G->mBaseStation.getX(), G->mBaseStation.getY(), 0));

	// For each node
	for(Node v : G->vNodeLst) {
		/// Consider "directly above" as a potential hovering location
		if(DEBUG_MAIN)
			printf("New hovering point above node: %d @ (%f, %f), weight: %f\n", id, v.getX(), v.getY(),
					distAtoB(v.getX(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(HoverLocation(id++, v.getX(), v.getY(),
											 distAtoB(v.getX(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY())));

		/// Consider cardinal directions on radius as a potential hovering location
		// North
		if(DEBUG_MAIN)
			printf("New hovering point north of node: %d @ (%f, %f), weight: %f\n", id, v.getX() + v.getR(), v.getY(),
					distAtoB(v.getX() + v.getR(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(HoverLocation(id++, v.getX() + v.getR(), v.getY(),
											 distAtoB(v.getX() + v.getR(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY())));
		// South
		if(DEBUG_MAIN)
			printf("New hovering point south of node: %d @ (%f, %f), weight: %f\n", id, v.getX() - v.getR(), v.getY(),
					distAtoB(v.getX() - v.getR(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(HoverLocation(id++, v.getX() - v.getR(), v.getY(),
											 distAtoB(v.getX() - v.getR(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY())));
		// East
		if(DEBUG_MAIN)
			printf("New hovering point east of node: %d @ (%f, %f), weight: %f\n", id, v.getX(), v.getY() + v.getR(),
					distAtoB(v.getX(), v.getY() + v.getR(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(HoverLocation(id++, v.getX(), v.getY() + v.getR(),
											 distAtoB(v.getX(), v.getY() + v.getR(), G->mBaseStation.getX(), G->mBaseStation.getY())));
		// West
		if(DEBUG_MAIN)
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
					if(DEBUG_MAIN)
						printf("New hovering point at edge of two circles: %d @ (%f, %f), weight: %f\n", id, x_p, y_p, weight);
					vPotentialHL.push_back(HoverLocation(id++, x_p, y_p, weight));
				}
				else {
					// Add both roots
					if(DEBUG_MAIN)
						printf("New hovering point from root1: %d @ (%f, %f), weight: %f\n", id, x_roots.root1, y_roots.root1,
								distAtoB(x_roots.root1, y_roots.root1, G->mBaseStation.getX(), G->mBaseStation.getY()));
					vPotentialHL.push_back(HoverLocation(id++, x_roots.root1, y_roots.root1,
														 distAtoB(x_roots.root1, y_roots.root1, G->mBaseStation.getX(), G->mBaseStation.getY())));
					if(DEBUG_MAIN)
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

	// For each hovering location
	for(HoverLocation hl : vPotentialHL) {
		if(DEBUG_MAIN)
			printf("Considering HL: %d, adding sensors: ", hl.nID);
		// Make a list containing each node within range
		std::list<int> sList;
		for(Node n : G->vNodeLst) {
			// Check if n is in range of hl
			if(distAtoB(n.getX(), n.getY(), hl.fX, hl.fY) <= (n.getR() + EPSILON)) {
				if(DEBUG_MAIN)
					printf("%d ", n.getID());
				// In range, add to list
				sList.push_back(n.getID());
			}
		}
		if(DEBUG_MAIN)
			printf("\n");
		// Add nodes-list to collection of lists
		vSPerHL.push_back(sList);
	}

	// Make list of hovering locations for each node
	// For each sensor
	for(Node n : G->vNodeLst) {
		if(DEBUG_MAIN)
			printf("Considering Sensor: %d, adding HLs: ", n.getID());
		// Make a list containing each hovering location within range
		std::list<int> hlList;
		// For each hovering location
		for(HoverLocation hl : vPotentialHL) {
			// Check if hl is in range of n
			if(distAtoB(n.getX(), n.getY(), hl.fX, hl.fY) <= (n.getR() + EPSILON)) {
				if(DEBUG_MAIN)
					printf("%d ", hl.nID);
				// In range, add to list
				hlList.push_back(hl.nID);
			}
		}
		if(DEBUG_MAIN)
			printf("\n");
		// Add hl-list to collection of lists
		vHLPerS.push_back(hlList);
	}
}

void setHLAboveNodes(Graph* G, std::vector<HoverLocation>& vPotentialHL, std::vector<std::list<int>>& vSPerHL, std::vector<std::list<int>>& vHLPerS) {
	// Hovering location ID tracker
	int id = 0;

	// Add base-station as first hovering location
	{
		vPotentialHL.push_back(HoverLocation(id++, G->mBaseStation.getX(), G->mBaseStation.getY(), 0));
		// Make an empty list for nodes within range
		std::list<int> sList;
		// Add nodes-list to collection of lists
		vSPerHL.push_back(sList);
		// Make an empty list for hovering location within range
		std::list<int> hlList;
		// Add hl-list to collection of lists
		vHLPerS.push_back(hlList);
	}

	// For each node
	for(Node v : G->vNodeLst) {
		/// Consider "directly above" as a potential hovering location
		if(DEBUG_MAIN)
			printf("New hovering point above node: %d @ (%f, %f), weight: %f\n", id, v.getX(), v.getY(),
					distAtoB(v.getX(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY()));

		HoverLocation tmpHL(id++, v.getX(), v.getY(), distAtoB(v.getX(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(tmpHL);

		// Make a list containing each node within range
		std::list<int> sList;
		// Add directly above as HL
		sList.push_back(v.getID());
		// Add nodes-list to collection of lists
		vSPerHL.push_back(sList);

		// Make a list containing each hovering location within range
		std::list<int> hlList;
		// In range, add to list
		hlList.push_back(vPotentialHL.back().nID);
		// Add hl-list to collection of lists
		vHLPerS.push_back(hlList);
	}

	// Add base-station as the last hovering location
	{
		vPotentialHL.push_back(HoverLocation(id++, G->mBaseStation.getX(), G->mBaseStation.getY(), 0));
		// Make an empty list for nodes within range
		std::list<int> sList;
		// Add nodes-list to collection of lists
		vSPerHL.push_back(sList);
		// Make an empty list for hovering location within range
		std::list<int> hlList;
		// Add hl-list to collection of lists
		vHLPerS.push_back(hlList);
	}
}

void improveRoute(Graph* G, Solution* solution) {
	if(SANITY_PRINT)
		printf("\nImprove route\n");
	for(unsigned long int i = 0; i < solution->vTours.size(); i++) {
		bool runAgain = true;

		// While moving stops helped... run again!
		while(runAgain) {
			runAgain = false;
			if(DEBUG_IMPR)
				printf(" %ld:\n",i);
			if(solution->vTours.at(i).size() > 2) {
				std::list<UAV_Stop>::iterator back = solution->vTours.at(i).begin();
				std::list<UAV_Stop>::iterator middle = solution->vTours.at(i).begin();
				middle++;
				std::list<UAV_Stop>::iterator front = solution->vTours.at(i).begin();
				front++;
				front++;

				while(front != solution->vTours.at(i).end()) {
					// Attempt to remove the middle stop
					if(DEBUG_IMPR)
						printf("  Pretending to remove (%f, %f)\n", middle->fX, middle->fY);

					bool madeChang = false;

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

						if(DEBUG_IMPR)
							printf("   l = %d, (x_p, y_p) = (%f, %f)\n", n, x_p, y_p);

						// Find magnitude of vector u from v to v'
						double mag_u = sqrt(pow((x_v - x_p), 2) + pow((y_v - y_p), 2));

						// Check to see if v' is in-range of v
						if(mag_u <= G->vNodeLst[n].getR()) {
							// We are in-range
							if(DEBUG_IMPR)
								printf("    this point is in-range of v!\n");
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
									if(DEBUG_IMPR)
										printf("    talk to %d\n", n);
									tempStop.nodes.push_back(n);
								}
							}
							// Make sure that we have the entire list...
							if(oldStop.nodes.size() == tempStop.nodes.size()) {
								if(DEBUG_IMPR)
									printf("     Easy swap!\n");
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
									if(DEBUG_IMPR)
										printf("    Update helped!\n");
									runAgain = true;
									madeChang = true;
									break;
								}
								else {
									// Adding the new stop didn't help.. remove it
									if(DEBUG_IMPR)
										printf("    No improvement\n");
									solution->vTours.at(i).erase(middle);
									middle = front;
									solution->vTours.at(i).insert(middle, oldStop);
									middle--;
								}
							}
							else {
								// We can't easily swap out the two stops...
								if(DEBUG_IMPR)
									printf("    No easy swap, try walking along f(x) = mx + b\n");

								// Vector from ( x_p, y_p ) to ( x_b, y_b )
								double a_1 = x_p - x_b;
								double a_2 = y_p - y_b;
								// Normalize the vector
								double mag_a = sqrt(a_1*a_1 + a_2*a_2);
								a_1 = a_1/mag_a;
								a_2 = a_2/mag_a;

								// Determine the distance to walk up/down the line
								double walk = sqrt(pow(G->vNodeLst[n].getR(),2) - mag_u*mag_u);

								// Attempt distance walk up the vector
								double x_pp = x_p + walk*a_1;
								double y_pp = y_p + walk*a_2;

								if(DEBUG_IMPR)
									printf("     trying (x_p, y_p) = (%f, %f)\n", x_pp, y_pp);
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
										if(DEBUG_IMPR)
											printf("      talk to %d\n", n);
										tempStop.nodes.push_back(n);
									}
								}
								// Make sure that we have the entire list...
								if(oldStop.nodes.size() == tempStop.nodes.size()) {
									if(DEBUG_IMPR)
										printf("      Easy swap!\n");
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
										if(DEBUG_IMPR)
											printf("      * Update helped!\n");
										runAgain = true;
										madeChang = true;
										break;
									}
									else {
										// Adding the new stop didn't help.. remove it
										if(DEBUG_IMPR)
											printf("     No improvement\n");
										solution->vTours.at(i).erase(middle);
										middle = front;
										solution->vTours.at(i).insert(middle, oldStop);
										middle--;
									}
								}
								else {
									// Attempt distance walk down the vector
									double x_pp = x_p - walk*a_1;
									double y_pp = y_p - walk*a_2;

									if(DEBUG_IMPR)
										printf("     trying (x_p, y_p) = (%f, %f)\n", x_pp, y_pp);
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
											if(DEBUG_IMPR)
												printf("      talk to %d\n", n);
											tempStop.nodes.push_back(n);
										}
									}
									// Make sure that we have the entire list...
									if(oldStop.nodes.size() == tempStop.nodes.size()) {
										if(DEBUG_IMPR)
											printf("      Easy swap!\n");
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
											if(DEBUG_IMPR)
												printf("      * Update helped!\n");
											runAgain = true;
											madeChang = true;
											break;
										}
										else {
											// Adding the new stop didn't help.. remove it
											if(DEBUG_IMPR)
												printf("     No improvement\n");
											solution->vTours.at(i).erase(middle);
											middle = front;
											solution->vTours.at(i).insert(middle, oldStop);
											middle--;
										}
									}
								}
							}
						}
						if(!madeChang) {
							// Find point along radius that is in-range
							double x_1 = x_p - x_v;
							double x_2 = y_p - y_v;
							double a = G->vNodeLst[n].getR()/(sqrt(pow(x_1, 2) + pow(x_2, 2)));
							double x_pp = x_v + x_1*a;
							double y_pp = y_v + x_2*a;

							if(DEBUG_IMPR)
								printf("    (x_pp, y_pp) = (%f, %f)\n", x_pp, y_pp);

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
									if(DEBUG_IMPR)
										printf("    talk to %d\n", n);
									tempStop.nodes.push_back(n);
								}
							}
							// Make sure that we have the entire list...
							if(oldStop.nodes.size() == tempStop.nodes.size()) {
								if(DEBUG_IMPR)
									printf("     Easy swap!\n");
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
									if(DEBUG_IMPR)
										printf("    Update helped!\n");
									runAgain = true;
									madeChang = true;
									break;
								}
								else {
									// Adding the new stop didn't help.. remove it
									if(DEBUG_IMPR)
										printf("    No improvement\n");
									solution->vTours.at(i).erase(middle);
									middle = front;
									solution->vTours.at(i).insert(middle, oldStop);
									middle--;
								}
							}
							else {
								// We can't easily swap out the two stops...
								if(DEBUG_IMPR)
									printf("    No easy swap\n");
							}
						}
					}

					// If we still haven't found a good change...
					if(!madeChang) {
						// Check corners
						if(DEBUG_IMPR)
							printf(" Try corners\n");

						for(std::list<int>::iterator itA = tempOld.nodes.begin(); itA != tempOld.nodes.end(); itA++) {
							for(std::list<int>::iterator itB = std::next(itA, 1); itB != tempOld.nodes.end(); itB++) {
								Node u = G->vNodeLst.at(*itA);
								Node v = G->vNodeLst.at(*itB);

								// Find overlapping points
								Roots y_roots;
								Roots x_roots;
								findOverlapPoints(&v, &u, &x_roots, &y_roots);

								// Add the two overlapping points to the solution
								if(!x_roots.imaginary & !y_roots.imaginary) {
									{
										// Try moving HL to these two points
										if(DEBUG_MAIN)
											printf("  Trying corner @ (%f, %f)\n", x_roots.root1, y_roots.root1);
										// Get total distance of current tour
										double oldDist = tourDist(solution->vTours.at(i));
										// Hold onto old stop
										UAV_Stop oldStop(*middle);
										// Create new UAV stop
										UAV_Stop tempStop(x_roots.root1, y_roots.root1);
										// Determine which sensors we talk to at this hovering location
										for(int n : oldStop.nodes) {
											if(distAtoB(G->vNodeLst[n].getX(), G->vNodeLst[n].getY(),
														tempStop.fX, tempStop.fY) <= (G->vNodeLst[n].getR() + EPSILON)) {
												if(DEBUG_IMPR)
													printf("   talk to %d\n", n);
												tempStop.nodes.push_back(n);
											}
										}
										// Make sure that we have the entire list...
										if(oldStop.nodes.size() == tempStop.nodes.size()) {
											if(DEBUG_IMPR)
												printf("  Easy swap!\n");
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
												if(DEBUG_IMPR)
													printf("   Update helped!\n");
												runAgain = true;
												madeChang = true;
												break;
											}
											else {
												// Adding the new stop didn't help.. remove it
												if(DEBUG_IMPR)
													printf("   No improvement\n");
												solution->vTours.at(i).erase(middle);
												middle = front;
												solution->vTours.at(i).insert(middle, oldStop);
												middle--;
											}
										}
										else {
											// We can't easily swap out the two stops...
											if(DEBUG_IMPR)
												printf("  Corner is out-of-range for other nodes\n");
										}
									}

									if(!madeChang) {
										// Try moving HL to these two points
										if(DEBUG_MAIN)
											printf("  Trying corner @ (%f, %f)\n", x_roots.root2, y_roots.root2);
										// Get total distance of current tour
										double oldDist = tourDist(solution->vTours.at(i));
										// Hold onto old stop
										UAV_Stop oldStop(*middle);
										// Create new UAV stop
										UAV_Stop tempStop(x_roots.root2, y_roots.root2);
										// Determine which sensors we talk to at this hovering location
										for(int n : oldStop.nodes) {
											if(distAtoB(G->vNodeLst[n].getX(), G->vNodeLst[n].getY(),
														tempStop.fX, tempStop.fY) <= (G->vNodeLst[n].getR() + EPSILON)) {
												if(DEBUG_IMPR)
													printf("   talk to %d\n", n);
												tempStop.nodes.push_back(n);
											}
										}
										// Make sure that we have the entire list...
										if(oldStop.nodes.size() == tempStop.nodes.size()) {
											if(DEBUG_IMPR)
												printf("  Easy swap!\n");
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
												if(DEBUG_IMPR)
													printf("   Update helped!\n");
												runAgain = true;
												madeChang = true;
												break;
											}
											else {
												// Adding the new stop didn't help.. remove it
												if(DEBUG_IMPR)
													printf("   No improvement\n");
												solution->vTours.at(i).erase(middle);
												middle = front;
												solution->vTours.at(i).insert(middle, oldStop);
												middle--;
											}
										}
										else {
											// We can't easily swap out the two stops...
											if(DEBUG_IMPR)
												printf("  Corner is out-of-range for other nodes\n");
										}
									}


//									vPotentialHL.push_back(HoverLocation(id++, x_roots.root1, y_roots.root1,
//																		 distAtoB(x_roots.root1, y_roots.root1, G->mBaseStation.getX(), G->mBaseStation.getY())));
//									vPotentialHL.push_back(HoverLocation(id++, x_roots.root2, y_roots.root2,
//																		 distAtoB(x_roots.root2, y_roots.root2, G->mBaseStation.getX(), G->mBaseStation.getY())));
								}
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
	if(DEBUG_MAIN) {
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
}

void printResults(Solution* solution, int nApproach, double duration_s, int nodeDensity) {
	assert(solution != NULL);

	// Print Results
	solution->printResults(duration_s, PRINT_RESULTS, nApproach, nodeDensity);
	if(MAKE_PLAN_FILE) {
		// Create the autopilot plan file
		solution->printPlan();
	}

	// Print Plot data
	if(MAKE_PLOT_FILE) {
		// Print found path to file
		FILE * pOutputFile;
		pOutputFile = fopen(PLOT_FILE_LOCATION, "w");
		if(DEBUG_MAIN)
			printf("\nMaking Plot File\n");
		for(std::list<UAV_Stop> l : solution->vTours) {
			for(UAV_Stop n : l) {
				fprintf(pOutputFile, "%f %f\n", n.fX, n.fY);
			}
		}
		fclose(pOutputFile);
	}
}

void solveGraph(Graph* G, int algorithm, int numUAVs, int nodeDensity) {
	// Pointer to the solver
	Solver *solver;

	// Make list of hovering locations
	std::vector<HoverLocation> vPotentialHL;
	// Make list of sensors for each hovering location
	std::vector<std::list<int>> vSPerHL;
	// Make list of hovering locations for each node
	std::vector<std::list<int>> vHLPerS;

	// Capture start time
	auto start = std::chrono::high_resolution_clock::now();
	Solution* solution = NULL;

	// Run given algorithm combination
	switch(algorithm) {
	case ALG_COMBO_AC_MILP_I:
		/// 1. Form list of potential hovering-locations
		/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
		// Select hovering locations
		setHLAllCombos(G, vPotentialHL, vSPerHL, vHLPerS);

		/// 3. Solve capacitated VRP
		solver = new SolHardMILP();
		solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);

		// Print Results before improvements
		solution->printResults(0, false, algorithm);

		/// 4. Improve route
		improveRoute(G, solution);
		break;

	case ALG_COMBO_AC_NN_I:
		/// 1. Form list of potential hovering-locations
		/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
		// Select hovering locations
		setHLAllCombos(G, vPotentialHL, vSPerHL, vHLPerS);
		/// 3. Solve capacitated VRP
		solver = new SolNearestNeighbor();
		solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);

		// Print Results before improvements
		solution->printResults(0, false, algorithm);

		/// 4. Improve route
		improveRoute(G, solution);

		break;

	case ALG_COMBO_AN_CL_NI:
		/// 1. Form list of potential hovering-locations
		/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
		// Select hovering locations above the node only
		setHLAboveNodes(G, vPotentialHL, vSPerHL, vHLPerS);

		/// 3. Solve capacitated VRP
		solver = new SolClusters();
		solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);

		break;

	case ALG_COMBO_AN_CL_I:
		/// 1. Form list of potential hovering-locations
		/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
		// Select hovering locations above the node only
		setHLAboveNodes(G, vPotentialHL, vSPerHL, vHLPerS);

		/// 3. Solve capacitated VRP
		solver = new SolClusters();
		solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);

		// Print Results before improvements
		solution->printResults(0, false, algorithm);

		/// 4. Improve route
		improveRoute(G, solution);

		break;

	case ALG_COMBO_AN_DG_NI:
		/// 1. Form list of potential hovering-locations
		/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
		// Select hovering locations above the node only
		setHLAboveNodes(G, vPotentialHL, vSPerHL, vHLPerS);

		/// 3. Solve capacitated VRP
		solver = new SolDivideGreedy();
		solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);

		break;

	case ALG_COMBO_AN_DG_I:
		/// 1. Form list of potential hovering-locations
		/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
		// Select hovering locations above the node only
		setHLAboveNodes(G, vPotentialHL, vSPerHL, vHLPerS);

		/// 3. Solve capacitated VRP
		solver = new SolDivideGreedy();
		solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);

		// Print Results before improvements
		solution->printResults(0, false, algorithm);

		/// 4. Improve route
		improveRoute(G, solution);

		break;

	case ALG_COMBO_AC_CL_NI:
		/// 1. Form list of potential hovering-locations
		/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
		// Select hovering locations above the node only
		setHLAllCombos(G, vPotentialHL, vSPerHL, vHLPerS);

		/// 3. Solve capacitated VRP
		solver = new SolClusters();
		solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);

		break;

	case ALG_COMBO_AC_CL_I:
		/// 1. Form list of potential hovering-locations
		/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
		// Select hovering locations above the node only
		setHLAllCombos(G, vPotentialHL, vSPerHL, vHLPerS);

		/// 3. Solve capacitated VRP
		solver = new SolClusters();
		solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);

		// Print Results before improvements
		solution->printResults(0, false, algorithm);

		/// 4. Improve route
		improveRoute(G, solution);

		break;

	case ALG_COMBO_AC_DG_NI:
		/// 1. Form list of potential hovering-locations
		/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
		// Select hovering locations above the node only
		setHLAllCombos(G, vPotentialHL, vSPerHL, vHLPerS);

		/// 3. Solve capacitated VRP
		solver = new SolDivideGreedy();
		solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);

		break;

	case ALG_COMBO_AC_DG_I:
	default:
		/// 1. Form list of potential hovering-locations
		/// 2. Make neighborhood lists (sensors for each HL, HLs for each sensor)
		// Select hovering locations above the node only
		setHLAllCombos(G, vPotentialHL, vSPerHL, vHLPerS);

		/// 3. Solve capacitated VRP
		solver = new SolDivideGreedy();
		solution = solver->RunSolver(G, numUAVs, vPotentialHL, vSPerHL, vHLPerS);

		// Print Results before improvements
		solution->printResults(0, false, algorithm);

		/// 4. Improve route
		improveRoute(G, solution);
	}

	// Capture end time
	auto stop = std::chrono::high_resolution_clock::now();
	// Determine the time it took to solve this
	long long int duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
	// Runtime duration
	double duration_s = (double)duration/1000.0;
	printResults(solution, algorithm, duration_s, nodeDensity);

	// Memory cleanup
	delete solution;
	delete solver;
}

int main(int argc, char *argv[]) {
	int algorithm = ALG_COMBO_AC_NN_I;
	int numUAVs = 2;
	int density = 150;

	// Verify user input
	if(argc < 2) {
		printf("Received %d args, expected 2 or more.\nExpected use:\t./min-lat <file path> [algorithm] [number of UAVs] [node density]\n\n", (argc-1));
		return 1;
	}

	if(argc == 3) {
		algorithm = atoi(argv[2]);
	}
	else if(argc == 4) {
		algorithm = atoi(argv[2]);
		numUAVs = atoi(argv[3]);
	}
	else if(argc == 5) {
		algorithm = atoi(argv[2]);
		numUAVs = atoi(argv[3]);
		density = atoi(argv[4]);
	}

	// Create the graph
	Graph G(argv[1]);

	solveGraph(&G, algorithm, numUAVs, density);
}
