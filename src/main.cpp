#include <cassert>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>

#include "HoverLocation.h"
#include "Graph.h"
#include "defines.h"
#include "PathTSP_MIP_PathPlanner.h"
#include "gurobi_c++.h"


#define	MAX_UAV_DIST	10.0
//#define	UAV_WIFI_RANGE	0.75
#define EPSILON			0.0001
#define V_MAX			19.0
// TODO: Determine UAV energy budget
#define Q				10
#define I_HAT			6
#define INF				1000000000000


#define PRINT_RESULTS		false
#define DATA_LOG_LOCATION	"Output/alg_%d.dat"

// Algorithm types, should be odd numbers
#define MILP			1
#define GREEDY_NN		3
#define ALGORITHM		GREEDY_NN

struct pNode {
	int nID;
	int nPriority;

	pNode(int id, int p) {
		nID = id;
		nPriority = p;
	}

	friend bool operator<(const pNode& a, const pNode& b)
	{
		return a.nPriority < b.nPriority;
	}
};

// Struct to hold two roots of a quadratic equation
struct Roots {
	double root1;
	double root2;
	bool imaginary;

	Roots() {
		root1 = 0;
		root2 = 0;
		imaginary = false;
	}
};

// Struct to hold a UAV stop
struct UAV_Stop {
	double fX;
	double fY;
	std::list<int> nodes;

	UAV_Stop(double x, double y) {
		fX = x;
		fY = y;
	}

	UAV_Stop(const UAV_Stop &stp) {
		fX = stp.fX;
		fY = stp.fY;
		for(int n : stp.nodes) {
			nodes.push_back(n);
		}
	}
};

bool isZero(double c) {
	return (c < EPSILON) && (c > -EPSILON);
}

double distAtoB(double x_1, double y_1, double x_2, double y_2) {
	return sqrt(pow((x_1 - x_2), 2) + pow((y_1 - y_2), 2));
}

std::string itos(int i) {
	std::stringstream s;
	s << i;
	return s.str();
}

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

double edgeTime(HoverLocation& i, HoverLocation& j) {
	return distAtoB(i.fX, i.fY, j.fX, j.fY) / V_MAX;
}

double edgeTime(UAV_Stop& i, UAV_Stop& j) {
	return distAtoB(i.fX, i.fY, j.fX, j.fY) / V_MAX;
}

// TODO: These \/ \/
double sensorTime(HoverLocation& i, Node& l) {
	return 2;
}

double sensorTime(UAV_Stop& i, Node& l) {
	return 2;
}
// TODO: This is not correct!!
double edgeCost(HoverLocation& i, HoverLocation& j) {
	return distAtoB(i.fX, i.fY, j.fX, j.fY) * 0.1;
}

double sensorCost(HoverLocation& i, Node& l) {
	return 1;
}
double edgeCost(UAV_Stop& i, UAV_Stop& j) {
	return distAtoB(i.fX, i.fY, j.fX, j.fY) * 0.1;
}

double sensorCost(UAV_Stop& i, Node& l) {
	return 1;
}

void priorityTour(Graph* G) {
	// Current tour
	std::vector<Node> tourNodes;
	tourNodes.push_back(G->mBaseStation);

	// While there are still unlocked nodes
	bool run = true;
	while(run) {
		// To start, find the highest priority node
		std::priority_queue<pNode> mypq;

		// Find the next node with highest priority
		for(Node n : G->vNodeLst) {
			if(!n.Locked()) {
				mypq.push(pNode(n.getID(), n.getPriority()));
			}
		}

		if(mypq.size() > 0) {
			printf("\ntop id: %d\n", mypq.top().nID);

			// Try adding this node to the solution
			tourNodes.push_back(G->vNodeLst.at(mypq.top().nID));

			// Solve TSP
			int* tour = new int[tourNodes.size()];
			PathTSP_MIP_PathPlanner TSPsolver;
			printf("Calling TSP solver, |tourNodes| = %ld\n", tourNodes.size());
			TSPsolver.RunAlgorithm(tourNodes, tour);

			// Determine length of tour
			double d = tourDist(tourNodes);

			// Verify the distance
			if(d > MAX_UAV_DIST) {
				// This node makes the path too far
				tourNodes.pop_back();

				// Print tour
				printf("\n*Found toud:");
				for(Node n : tourNodes) {
					printf(" %d", n.getID());
				}
				printf("\n*New tour\n");

				// Reset tour
				tourNodes.clear();
				tourNodes.push_back(G->mBaseStation);
			}
			else {
				G->MarkNode(mypq.top().nID);
			}
		}
		else {
			printf("\nAll nodes marked\n");
			// Print tour
			printf("\n*Found toud:");
			for(Node n : tourNodes) {
				printf(" %d", n.getID());
			}
			printf("\n");
			// Stop algorithm
			run = false;
		}
	}

	printf("Done!\n");

}

// Find the roots of a quadratic equation, store roots in rt scruct.
// Assumes that we were given an actual quadratic equation (a != 0).
void findRoots(double a, double b, double c, Roots* rt) {
	double discriminant, realPart, imaginaryPart, x1, x2;
	// TODO: Don't compare floating point numbers to 0!
	if (a == 0) {
		printf("Requested to find root of non-quadratic\n");
		exit(1);
	}
	else {
		discriminant = b*b - 4*a*c;
		if (discriminant > 0) {
			x1 = (-b + sqrt(discriminant)) / (2*a);
			x2 = (-b - sqrt(discriminant)) / (2*a);
			printf("Roots are real and different.\n");
			printf("Root 1 = %f\n", x1);
			printf("Root 2 = %f\n", x2);

			rt->root1 = x1;
			rt->root2 = x2;
			rt->imaginary = false;
		}
		else if(discriminant == 0) {
			printf("Roots are real and same.\n");
			x1 = (-b + sqrt(discriminant)) / (2*a);
			printf("Root 1 = Root 2 = %f\n", x1);

			rt->root1 = rt->root2 = x1;
			rt->imaginary = false;
		}
		else {
			realPart = -b/(2*a);
			imaginaryPart = sqrt(-discriminant)/(2*a);
			printf("Roots are complex and different.\n");
			printf("Root 1 = %f + %fi\n", realPart, imaginaryPart);
			printf("Root 1 = %f - %fi\n", realPart, imaginaryPart);

			rt->root1 = realPart;
			rt->root2 = imaginaryPart;
			rt->imaginary = true;
		}
	}
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

	findRoots(a, b, c, y_roots);

	// Find the x intercept values
	double c3 = -(x1 - x2)/(y1 - y2);
	double c4 = -(pow(r1,2) - pow(r2,2))/(2*(y1-y2))
					+ (pow(x1,2) - pow(x2,2))/(2*(y1 - y2))
					+ (pow(y1,2) - pow(y2,2))/(2*(y1 - y2));

	// ax^2 + bx + c = 0
	a = pow(c3,2) + 1;
	b = (2*c3*(c4 - y1)-2*x1);
	c = pow((c4 - y1),2) + pow(x1,2) - pow(r1,2);

	findRoots(a, b, c, x_roots);

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

void findRadiusPaths(Graph* G) {
	int id = 0;
	std::vector<HoverLocation> vPotentialHL;

	// Add base-station as first hovering location
	vPotentialHL.push_back(HoverLocation(id++, G->mBaseStation.getX(), G->mBaseStation.getY(), 0));

	/// 1. Form list of potential hovering-locations
	for(Node v : G->vNodeLst) {
		/// Determine point within v's range closest to base station
		double x_1 = G->mBaseStation.getX() - v.getX();
		double x_2 = G->mBaseStation.getY() - v.getY();
		double a = v.getR()/(sqrt(pow(x_1, 2) + pow(x_2, 2)));
		double x_p = v.getX() + x_1*a;
		double y_p = v.getY() + x_2*a;
		// Determine the weight of visiting this location
		double weight = distAtoB(x_p, y_p , G->mBaseStation.getX(), G->mBaseStation.getY());
		// Create a potential hovering-location
		printf("New hovering point at edge of R: %d @ (%f, %f), weight: %f\n", id, x_p, y_p, weight);
		vPotentialHL.push_back(HoverLocation(id++, x_p, y_p, weight));

		/// Consider "directly above" as a potential hovering location
		printf("New hovering point above node: %d @ (%f, %f), weight: %f\n", id, v.getX(), v.getY(),
				distAtoB(v.getX(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY()));
		vPotentialHL.push_back(HoverLocation(id++, v.getX(), v.getY(),
				distAtoB(v.getX(), v.getY(), G->mBaseStation.getX(), G->mBaseStation.getY())));

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

	// Lists for each found tour
	std::vector<std::list<UAV_Stop>> vTours;


	/// 3. Solve capacitated VRP
	if(ALGORITHM == MILP) {
		unsigned long int M = 10;

		try {
			// Create Gurobi environment
			GRBEnv env;
			GRBModel model = GRBModel(&env);

			/*
			 * Sets:
			 * i,j in H, set of hovering locations, |H|=N
			 * l in S, set of sensors, |S|=L
			 * k in K, set of tours (or vehicles), |K|=M
			 * l in S_i, indexed set of sensors in-range of hovering location i
			 * i in H_l, indexed set of hovering locations in-range of sensor l
			 */

			unsigned long int N = vPotentialHL.size();
			unsigned long int L = G->vNodeLst.size();

			/// Define variables
			// Tracks edges between HLs
			GRBVar*** X = new GRBVar**[M];
			for(unsigned long int k = 0; k < M; k++) {
				X[k] = new GRBVar*[N];
				for(unsigned long int i = 0; i < N; i++) {
					X[k][i] = new GRBVar[N];
					for(unsigned long int j = 0; j < N; j++) {
						X[k][i][j] = model.addVar(0.0, 1.0, edgeTime(vPotentialHL.at(i), vPotentialHL.at(j)),
								GRB_BINARY, "X_"+itos(k)+"_"+itos(i)+"_"+itos(j));
					}
				}
			}

			// Tracks when we talk to each sensor
			GRBVar*** Y = new GRBVar**[M];
			for(unsigned long int k = 0; k < M; k++) {
				Y[k] = new GRBVar*[N];
				for(unsigned long int i = 0; i < N; i++) {
					Y[k][i] = new GRBVar[L];
				}
			}

			// Create the indexed variables
			for(unsigned long int k = 0; k < M; k++) {
				for(unsigned long int i = 0; i < N; i++) {
					for(int l : vSPerHL.at(i)) {
						Y[k][i][l] = model.addVar(0.0, 1.0, sensorTime(vPotentialHL.at(i), G->vNodeLst.at(l)),
								GRB_BINARY, "Y_"+itos(k)+"_"+itos(i)+"_"+itos(l));
					}
				}
			}

			// Energy budget
			GRBVar** Z = new GRBVar*[M];
			for(unsigned long int k = 0; k < M; k++) {
				Z[k] = new GRBVar[N];
				for(unsigned long int i = 0; i < N; i++) {
					Z[k][i] = model.addVar(0.0, Q, 0.0, GRB_CONTINUOUS, "Z_"+itos(k)+"_"+itos(i));
				}
			}

			/// Constraints
			// Limit when Y can be turned on
			for(unsigned long int k = 0; k < M; k++) {
				for(unsigned long int i = 0; i < N; i++) {
					for(int l : vSPerHL.at(i)) {
						// Y_kil
						GRBLinExpr expr = Y[k][i][l];
						// Minus X_kji (edges going into i on k)
						for(unsigned long int j = 0; j < N; j++) {
							expr -= X[k][j][i];
						}
						model.addConstr(expr <= 0, "Y_"+itos(k)+"_"+itos(i)+"_"+itos(l)+"_leq_X");
					}
				}
			}

			// Enforce budget/eliminate sub-tours (Upper Bound)
			for(unsigned long int k = 0; k < M; k++) {
				for(unsigned long int i = 0; i < N; i++) {
					for(unsigned long int j = 0; j < N; j++) {
						// Budget at i, on k
						GRBLinExpr expr = Z[k][i];
						// Plus cost to talk to l from point i, on tour k
						for(int l : vSPerHL.at(i)) {
							expr += sensorCost(vPotentialHL.at(i), G->vNodeLst.at(l)) * Y[k][i][l];
						}
						// Plus cost to get to j from i
						expr += edgeCost(vPotentialHL.at(i), vPotentialHL.at(j)) * X[k][i][j];
						// Big-M, cancel-out expression if X == 0
						expr -= Q*(1 - X[k][i][j]);
						// Minus budget at j on k
						expr -= Z[k][j];

						model.addConstr(expr <= 0, "Z_"+itos(k)+"_"+itos(j)+"_geq_Z_"+itos(k)+"_"+itos(i));
					}
				}
			}

			// Enforce budget/eliminate sub-tours (Lower Bound)
			for(unsigned long int k = 0; k < M; k++) {
				for(unsigned long int i = 0; i < N; i++) {
					for(unsigned long int j = 0; j < N; j++) {
						// Budget at i, on k
						GRBLinExpr expr = Z[k][i];
						// Plus cost to talk to l from point i, on tour k
						for(int l : vSPerHL.at(i)) {
							expr += sensorCost(vPotentialHL.at(i), G->vNodeLst.at(l)) * Y[k][i][l];
						}
						// Plus cost to get to j from i
						expr += edgeCost(vPotentialHL.at(i), vPotentialHL.at(j)) * X[k][i][j];
						// Big-M, cancel-out expression if X == 0
						expr += Q*(1 - X[k][i][j]);
						// Minus budget at j on k
						expr -= Z[k][j];

						model.addConstr(expr >= 0, "Z_"+itos(k)+"_"+itos(j)+"_geq_Z_"+itos(k)+"_"+itos(i));
					}
				}
			}

			// Shut-off budget when you don't enter a hover location
			for(unsigned long int k = 0; k < M; k++) {
				for(unsigned long int i = 0; i < N; i++) {
					GRBLinExpr expr = 0;
					// Sum across all edges going into i
					for(unsigned long int j = 0; j < N; j++) {
						expr += Q*X[k][j][i];
					}
					// Subtract the budget at i
					expr -= Z[k][i];

					model.addConstr(expr >= 0, "Z_"+itos(k)+"_"+itos(i)+"_leq_X_"+itos(k)+"_*_"+itos(i));
				}
			}

			// Must collect from each sensor
			for(Node l : G->vNodeLst) {
				int l_ID = l.getID();
				GRBLinExpr expr = 0;
				// Sum across tours
				for(unsigned long int k = 0; k < M; k++) {
					// Sum across hovering locations close to l
					for(int i : vHLPerS.at(l_ID)) {
						expr += Y[k][i][l_ID];
					}
				}

				model.addConstr(expr == 1, "Y_k_i_"+itos(l_ID)+"_geq_1");
			}

			// Degree-in == degree-out
			for(unsigned long int k = 0; k < M; k++) {
				for(unsigned long int i = 1; i < N-1; i++) {
					GRBLinExpr expr = 0;
					// Sum across all edges going into i
					for(unsigned long int j = 0; j < N; j++) {
						expr += X[k][j][i];
					}
					// Sum across all edges going out of i
					for(unsigned long int j = 0; j < N; j++) {
						expr -= X[k][i][j];
					}

					model.addConstr(expr == 0, "X_"+itos(k)+"_*_"+itos(i)+"_eq_X_"+itos(k)+"_"+itos(i)+"_*");
				}
			}

			// Must exit from depot-initial
			for(unsigned long int k = 0; k < M; k++) {
				GRBLinExpr expr = 0;
				// Sum across all edges leaving depot on k
				for(unsigned long int i = 0; i < N; i++) {
					expr += X[k][0][i];
				}

				model.addConstr(expr <= 1, "X_"+itos(k)+"_0_i_eq_1");
			}

			// Must return to depot-final
			for(unsigned long int k = 0; k < M; k++) {
				GRBLinExpr expr = 0;
				// Sum across all edges leaving depot on k
				for(unsigned long int i = 0; i < N; i++) {
					expr += X[k][i][N - 1];
				}

				model.addConstr(expr <= 1, "X_"+itos(k)+"_i_i-hat_eq_1");
			}

	//		for(unsigned long int k = 0; k < (M-1); k++) {
	//			GRBLinExpr expr = 0;
	//			for(unsigned long int i = 0; i < N; i++) {
	//				for(unsigned long int j = 0; j < N; j++) {
	//					expr += edgeTime(vPotentialHL.at(i), vPotentialHL.at(j))*X[k][i][j];
	//					expr -= edgeTime(vPotentialHL.at(i), vPotentialHL.at(j))*X[k+1][i][j];
	//				}
	//			}
	//			model.addConstr(expr >= 0, "X_"+itos(k)+"_leq_X_"+itos(k+1));
	//		}

			// Initial budget is 0
			for(unsigned long int k = 0; k < M; k++) {
				model.addConstr(Z[k][0] == 0, "Z_"+itos(k)+"_0_eq_0");
			}

			// No self-loops
			for(unsigned long int k = 0; k < M; k++) {
				for(unsigned long int i = 0; i < N; i++) {
					GRBLinExpr expr = X[k][i][i];
					model.addConstr(expr == 0, "no_X_"+itos(k)+"_"+itos(i)+"_"+itos(i));
				}
			}

			// No going from depot '0' to depot 'N-1'
			for(unsigned long int k = 0; k < M; k++) {
				GRBLinExpr expr = X[k][0][N-1];
				model.addConstr(expr == 0, "no_X_"+itos(k)+"_0_N");
			}

			// Run for at most 1.5 hours
			model.set(GRB_DoubleParam_TimeLimit, 5400);
			// Use cuts aggressively
			model.set(GRB_INT_PAR_CUTS, "2");

			// Optimize model
			model.optimize();

			// Extract solution
			if (model.get(GRB_IntAttr_SolCount) > 0) {
				// Create arrays of doubles to collect results
				double*** Xsol = new double**[M];
				double*** Ysol = new double**[M];
				double** Zsol = new double*[M];

				// Extract the results from the solver
				for(unsigned long int k = 0; k < M; k++) {
					Xsol[k] = new double*[N];
					for(unsigned long int i = 0; i < N; i++) {
						Xsol[k][i] = model.get(GRB_DoubleAttr_X, X[k][i], N);
					}
				}

				for(unsigned long int k = 0; k < M; k++) {
					Ysol[k] = new double*[N];
					for(unsigned long int i = 0; i < N; i++) {
						Ysol[k][i] = new double[N];
						for(unsigned long int j = 0; j < N; j++) {
							Ysol[k][i][j] = 0;
						}
						for(int l : vSPerHL.at(i)) {
							Ysol[k][i][l] = *model.get(GRB_DoubleAttr_X, &Y[k][i][l], 1);
						}
					}
				}

				for(unsigned long int k = 0; k < M; k++) {
					Zsol[k] = model.get(GRB_DoubleAttr_X, Z[k], N);
				}

				// Print results
				printf("\nCollect tour:\n");
				for(unsigned long int k = 0; k < M; k++) {
					// Make list four sub-tour k
					std::list<UAV_Stop> tour_k;
					// Start at depot '0'
					unsigned long int prevHL = 0;
					// Put the depot on the list
					tour_k.push_back(UAV_Stop(vPotentialHL[prevHL].fX,vPotentialHL[prevHL].fY));
					// Find all stops on this tour
					while(prevHL != (N-1)) {
						// Find the next stop
						unsigned long int i = 0;
						for(; i < N; i++) {
							if(Xsol[k][prevHL][i] > 0.5) {
								// Found next hovering-location
								printf(" %ld: %ld -> %ld\n", k, prevHL, i);
								// Create new UAV stop
								UAV_Stop tempStop(vPotentialHL[i].fX,vPotentialHL[i].fY);
								// Determine which sensors we talk to at this hovering location
								for(unsigned long int l = 0; l < L; l++) {
									if(Ysol[k][i][l] > 0.5) {
										printf(" %ld: @ %ld talk to %ld\n", k, i, l);
										tempStop.nodes.push_back(l);
									}
								}
								printf(" %ld: Budget@%ld = %f\n", k, i, Zsol[k][i]);
								// Update previous
								prevHL = i;
								// Add this node to tour
								tour_k.push_back(tempStop);
								break;
							}
						}
						// Verify that nothing went wrong
						if(i == N) {
							if(tour_k.size() > 1) {
								// Something went wrong...
								fprintf(stderr, "[ERROR] Xsol[][][] does not contain complete graph!\n");
								exit(1);
							}
							else {
								// Found empty tour
								printf("* Tour %ld is empty!\n", k);
								prevHL = N-1;
								tour_k.push_back(UAV_Stop(vPotentialHL[prevHL].fX,vPotentialHL[prevHL].fY));
							}
						}
					}
					vTours.push_back(tour_k);
				}

				// Sanity print
				printf("\nFiltered tour:\n");
				int i = 0;
				for(std::list<UAV_Stop> l : vTours) {
					printf(" %d: ", i);
					for(UAV_Stop n : l) {
						printf("(%f, %f) ", n.fX, n.fY);
					}
					printf("\n");
					i++;
				}
				if(0) {

					printf("\nUn-Filtered tour:\n");
					for(unsigned long int k = 0; k < M; k++) {
						for(unsigned long int i = 0; i < N; i++) {
							for(unsigned long int j = 0; j < N; j++) {
								if(Xsol[k][i][j] > 0.5) {
									printf(" %ld: %ld -> %ld\n", k, i, j);
									for(unsigned long int l = 0; l < L; l++) {
										if(Ysol[k][j][l] > 0.5) {
											printf(" %ld: @ %ld talk to %ld\n", k, j, l);
										}
									}
									printf(" %ld: Budget@%ld = %f\n", k, j, Zsol[k][j]);
								}
							}
						}
					}

					printf("\nData collection:\n");
					for(unsigned long int k = 0; k < M; k++) {
						for(unsigned long int i = 0; i < N; i++) {
							for(unsigned long int l = 0; l < L; l++) {
								if(Ysol[k][i][l] > 0.5) {
									printf(" %ld: @ %ld talk to %ld\n", k, i, l);
								}
							}
						}
					}

					printf("\nTotal Budget:\n");
					for(unsigned long int k = 0; k < M; k++) {
						for(unsigned long int i = 0; i < N; i++) {
							printf(" %ld: %ld : %f\n", k, i, Zsol[k][i]);
						}
					}
				}

				// Clean-up memory
				for(unsigned long int k = 0; k < M; k++) {
					for(unsigned long int i = 0; i < N; i++) {
						delete[] Xsol[k][i];
						delete[] Ysol[k][i];
					}
					delete[] Xsol[k];
					delete[] Ysol[k];
					delete[] Zsol[k];
				}
				delete[] Xsol;
				delete[] Ysol;
				delete[] Zsol;
			}
			else {
				fprintf(stderr, "[ERROR] : Could not find valid solution!\n");
				exit(1);
			}

		} catch (GRBException& e) {
			printf("Error number: %d\n", e.getErrorCode());
			printf("%s\n", e.getMessage().c_str());
			exit(1);
		} catch (...) {
			printf("Error during optimization\n");
			exit(1);
		}

	}
	else if(ALGORITHM == GREEDY_NN) {
		// Make a list of hovering locations
		std::list<HoverLocation> hlList;
		for(HoverLocation hl : vPotentialHL) {
			// Verify that this HL is worth stopping at...
			if(vSPerHL.at(hl.nID).size() > 0) {
				hlList.push_back(hl);
			}
		}

		// Make a serviced array
		bool* pServiced = new bool[G->vNodeLst.size()];
		for(unsigned long int i = 0; i < G->vNodeLst.size(); i++) {
			pServiced[i] = false;
		}

		// While there are still un-serviced sensors ...
		bool unservicedSensors = true;
		while(unservicedSensors) {
			printf("Starting a new tour\n");
			// Create a new tour
			std::list<UAV_Stop> tour;
			// Start tour at the base station
			tour.push_back(UAV_Stop(G->mBaseStation.getX(), G->mBaseStation.getY()));

			bool underBudget = true;
			// While we are still under our energy budget..
			while(underBudget) {
				// Find a new HL to attempt to add to tour

				// Start-off with the first HL in our list
				std::list<HoverLocation>::iterator bstIt = hlList.begin();
				// Track closest distance
				double bestDist = INF;

				// Run through our list of HL's, pick closest to UAV's current location
				for(std::list<HoverLocation>::iterator hlIt = hlList.begin(); hlIt != hlList.end();) {
					// Determine if this HL services any un-serviced sensors
					bool usefulHL = false;
					// Check each HL for this sensor
					for(int l : vSPerHL.at(hlIt->nID)) {
						usefulHL |= !pServiced[l];
					}

					// Check if HL is useful
					if(usefulHL) {
						printf(" Found a useful HL: %d\n", hlIt->nID);
						// HL services at least 1 sensor, consider adding to tour
						double tempDist = distAtoB(tour.back().fX, tour.back().fY, hlIt->fX, hlIt->fY);
						// Check to see if this HL is better than current best
						if(tempDist < bestDist) {
							printf("  new best\n");
							// Found new best!
							bstIt = hlIt;
							bestDist = tempDist;
						}

						// Advance iterator
						hlIt++;
					}
					else {
						// HL is no longer helpful, remove from list (advances iterator)
						printf(" Can't use HL: %d\n", hlIt->nID);
						hlIt = hlList.erase(hlIt);
					}
				}

				// Check if we found a valid next-stop
				if(bestDist < INF) {
					printf(" Attempting to add %d to tour\n", bstIt->nID);
					// Attempt to add this stop to the tour
					UAV_Stop nextStop(bstIt->fX, bstIt->fY);
					// Add un-serviced sensors to stop
					for(int l : vSPerHL.at(bstIt->nID)) {
						if(!pServiced[l]) {
							nextStop.nodes.push_back(l);
						}
					}
					// Verify budget by returning to depot
					UAV_Stop lastStop(G->mBaseStation.getX(), G->mBaseStation.getY());
					// Add these stops to the tour
					tour.push_back(nextStop);
					tour.push_back(lastStop);

					// Check energy consumption of tour
					double budget = 0;
					std::list<UAV_Stop>::iterator lst, nxt;
					lst = tour.begin();
					nxt = tour.begin();
					nxt++;

					// Run through the tour, add up pst for each leg + HL stop
					while(nxt != tour.end()) {
						// Add time to move from lst to nxt
						budget += edgeCost(*lst, *nxt);
						// Add in time to talk to each sensor at nxt
						for(int s : nxt->nodes) {
							budget += sensorCost(*nxt, G->vNodeLst.at(s));
						}

						// Advance iterators
						lst++;
						nxt++;
					}
					printf(" New budget: %f\n", budget);

					// If good ...
					if(budget <= Q) {
						printf(" Add to tour!\n");
						// Mark sensors
						for(int l : vSPerHL.at(bstIt->nID)) {
							pServiced[l] = true;
						}
						// Remove depot
						tour.pop_back();
						// Continue running
						underBudget = true;
					}
					else {
						// Adding this stop put us over budget!
						printf(" Went over budget\n");
						// Remove stop
						tour.pop_back();
						tour.pop_back();
						// Add depot
						tour.push_back(lastStop);
						// Finish this tour
						underBudget = false;
					}
				}
				else {
					// No useful HLs => we have serviced all sensors
					printf("Serviced every sensor\n");
					// Add on depot
					UAV_Stop depot(G->mBaseStation.getX(), G->mBaseStation.getY());
					tour.push_back(depot);
					// Exit loop
					underBudget = false;
				}
			}

			// Add this tour to the vector of tours
			vTours.push_back(tour);

			// Determine if there are still any un-serviced sensors
			unservicedSensors = false;
			for(unsigned long int i = 0; i < G->vNodeLst.size(); i++) {
				unservicedSensors |= !pServiced[i];
			}
			printf("Finished tour\n");
		}
		printf("Tours complete!\n");

		delete[] pServiced;
	}


	// Print Results before improvements
	if(PRINT_RESULTS) {
		// Print results to file
		FILE * pOutputFile;
		char buff[100];
		sprintf(buff, DATA_LOG_LOCATION, ALGORITHM);
		printf("%s\n", buff);
		pOutputFile = fopen(buff, "a");

		printf("\nFound tour:\n");
		int i = 0;
		double total_duration = 0;
		for(std::list<UAV_Stop> l : vTours) {
			double duration = 0;
			if(l.size() > 1) {
				std::list<UAV_Stop>::iterator lst, nxt;
				lst = l.begin();
				nxt = l.begin();
				nxt++;

				// Run through the tour, add up distance from stop-to-stop
				while(nxt != l.end()) {
					// Add time to move from lst to nxt
					duration += edgeTime(*lst, *nxt);
					// Add in time to talk to each sensor at nxt
					for(int s : nxt->nodes) {
						duration += sensorTime(*nxt, G->vNodeLst.at(s));
					}

					// Advance iterators
					lst++;
					nxt++;
				}
			}
			printf("%d: duration = %f\n ", i, duration);
			total_duration += duration;
			for(UAV_Stop n : l) {
				printf("(%f, %f) ", n.fX, n.fY);
			}
			printf("\n");
			i++;
		}
		printf("Total duration = %f\n ", total_duration);
		fprintf(pOutputFile, "%ld %f\n", G->vNodeLst.size(), total_duration);

		fclose(pOutputFile);
	}


	/// 4. Improve route
	printf("\nImprove route\n");
	for(unsigned long int i = 0; i < vTours.size(); i++) {
		bool runAgain = true;

		// While moving stops helped... run again!
		while(runAgain) {
			runAgain = false;
			printf(" %ld:\n",i);
			if(vTours.at(i).size() > 2) {
				std::list<UAV_Stop>::iterator back = vTours.at(i).begin();
				std::list<UAV_Stop>::iterator middle = vTours.at(i).begin();
				middle++;
				std::list<UAV_Stop>::iterator front = vTours.at(i).begin();
				front++;
				front++;

				while(front != vTours.at(i).end()) {
					// Attempt to remove the middle stop
					printf("  Pretending to remove (%f, %f)\n", middle->fX, middle->fY);

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

						printf("   l = %d, (x_p, y_p) = (%f, %f)\n", n, x_p, y_p);

						// Find magnitude of vector u from v to v'
						double mag_u = sqrt(pow((x_v - x_p), 2) + pow((y_v - y_p), 2));

						// Check to see if v' is in-range of v
						if(mag_u <= G->vNodeLst[n].getR()) {
							// We are in-range
							printf("    this point is in-range of v!\n");
							// Get total distance of current tour
							double oldDist = tourDist(vTours.at(i));
							// Hold onto old stop
							UAV_Stop oldStop(*middle);
							// Create new UAV stop
							UAV_Stop tempStop(x_p, y_p);
							// Determine which sensors we talk to at this hovering location
							for(int n : oldStop.nodes) {
								if(distAtoB(G->vNodeLst[n].getX(), G->vNodeLst[n].getY(),
										tempStop.fX, tempStop.fY) <= (G->vNodeLst[n].getR() + EPSILON)) {
									printf("    talk to %d\n", n);
									tempStop.nodes.push_back(n);
								}
							}
							// Make sure that we have the entire list...
							if(oldStop.nodes.size() == tempStop.nodes.size()) {
								printf("     Easy swap!\n");
								// Add this stop to the tour, remove current middle
								vTours.at(i).erase(middle);
								middle = front;
								// Add new stop
								vTours.at(i).insert(middle, tempStop);
								// Correct iterator's position
								middle--;
								// Check new tour's distance
								double newDist = tourDist(vTours.at(i));
								// See if we made an improvement
								if(newDist < oldDist) {
									// Great!
									printf("    Update helped!\n");
									runAgain = true;
									break;
								}
								else {
									// Adding the new stop didn't help.. remove it
									printf("    No improvement\n");
									vTours.at(i).erase(middle);
									middle = front;
									vTours.at(i).insert(middle, oldStop);
									middle--;
								}
							}
							else {
								// We can't easily swap out the two stops...
								printf("    No easy swap\n");
							}
						}
						else {
							// Find point along radius that is in-range
							double x_1 = x_p - x_v;
							double x_2 = y_p - y_v;
							double a = G->vNodeLst[n].getR()/(sqrt(pow(x_1, 2) + pow(x_2, 2)));
							double x_pp = x_v + x_1*a;
							double y_pp = y_v + x_2*a;

							printf("    (x_pp, y_pp) = (%f, %f)\n", x_pp, y_pp);

							// Get total distance of current tour
							double oldDist = tourDist(vTours.at(i));
							// Hold onto old stop
							UAV_Stop oldStop(*middle);
							// Create new UAV stop
							UAV_Stop tempStop(x_pp, y_pp);
							// Determine which sensors we talk to at this hovering location
							for(int n : oldStop.nodes) {
								if(distAtoB(G->vNodeLst[n].getX(), G->vNodeLst[n].getY(),
										tempStop.fX, tempStop.fY) <= (G->vNodeLst[n].getR() + EPSILON)) {
									printf("    talk to %d\n", n);
									tempStop.nodes.push_back(n);
								}
							}
							// Make sure that we have the entire list...
							if(oldStop.nodes.size() == tempStop.nodes.size()) {
								printf("     Easy swap!\n");
								// Add this stop to the tour, remove current middle
								vTours.at(i).erase(middle);
								middle = front;
								// Add new stop
								vTours.at(i).insert(middle, tempStop);
								// Correct iterator's position
								middle--;
								// Check new tour's distance
								double newDist = tourDist(vTours.at(i));
								// See if we made an improvement
								if(newDist < oldDist) {
									// Great!
									printf("    Update helped!\n");
									runAgain = true;
									break;
								}
								else {
									// Adding the new stop didn't help.. remove it
									printf("    No improvement\n");
									vTours.at(i).erase(middle);
									middle = front;
									vTours.at(i).insert(middle, oldStop);
									middle--;
								}
							}
							else {
								// We can't easily swap out the two stops...
								printf("    No easy swap\n");
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
		for(std::list<UAV_Stop> l : vTours) {
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
		// Print results to file
		FILE * pOutputFile;
		char buff[100];
		sprintf(buff, DATA_LOG_LOCATION, ALGORITHM + 1);
		printf("%s\n", buff);
		pOutputFile = fopen(buff, "a");

		printf("\nImproved tour:\n");
		int i = 0;
		double total_duration = 0;
		for(std::list<UAV_Stop> l : vTours) {
			double duration = 0;
			if(l.size() > 1) {
				std::list<UAV_Stop>::iterator lst, nxt;
				lst = l.begin();
				nxt = l.begin();
				nxt++;

				// Run through the tour, add up distance from stop-to-stop
				while(nxt != l.end()) {
					// Add time to move from lst to nxt
					duration += edgeTime(*lst, *nxt);
					// Add in time to talk to each sensor at nxt
					for(int s : nxt->nodes) {
						duration += sensorTime(*nxt, G->vNodeLst.at(s));
					}

					// Advance iterators
					lst++;
					nxt++;
				}
			}
			printf("%d: duration = %f\n ", i, duration);
			total_duration += duration;
			for(UAV_Stop n : l) {
				printf("(%f, %f) ", n.fX, n.fY);
			}
			printf("\n");
			i++;
		}
		printf("Total duration = %f\n ", total_duration);
		fprintf(pOutputFile, "%ld %f\n", G->vNodeLst.size(), total_duration);

		fclose(pOutputFile);
	}
}

int main(int argc, char *argv[]) {
	if(argc != 2) {
		printf("Expected use:min-lat <file path>\n");
		return 1;
	}

	// Create the graph
	Graph G(argv[1]);

//	priorityTour(&G);
	findRadiusPaths(&G);
}
