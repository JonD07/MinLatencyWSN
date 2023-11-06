#include "Node.h"

Node::Node() : nID(-1), fX(0), fY(0), fR(0), bLocked(false) {}

Node::Node(int id, double x, double y, double r, double b) : nID(id), fX(x), fY(y), fR(r), budget(b), bLocked(false){}

Node::Node(const Node &n) {
	nID = n.nID;
	fX = n.fX;
	fY = n.fY;
	fR = n.fR;
	budget = n.budget;
	bLocked = n.bLocked;
	
}

Node::~Node() {}

Node& Node::operator=(const Node& other)
{
	// Guard self assignment
	if (this == &other)
		return *this;

	this->nID = other.nID;
	this->fX = other.fX;
	this->fY = other.fY;

	return *this;
}

double Node::GetDistanceTo(Node* n) {
	return sqrt(pow((fX - n->fX), 2) + pow((fY - n->fY), 2));
}

// Budget cost to collect data from this node: t * s_m/s_h,
// which is [ t * (total time moving at V_MAX)/(total hovering time) ]
double Node::sensorCost(Location &i) {
	return sensorTime(i)*((Q*budget) / (HOVER_TIME));
}

// Actual hovering time, in seconds
double Node::sensorTime(Location &l) {return 5;}
