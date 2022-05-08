#include "Node.h"

Node::Node() : nID(-1), fX(0), fY(0), fR(0), fP(0), bLocked(false) {}

Node::Node(int id, double x, double y, double r, double p) : nID(id), fX(x), fY(y), fR(r), fP(p), bLocked(false) {}

Node::Node(const Node &n) {
	nID = n.nID;
	fX = n.fX;
	fY = n.fY;
	fR = n.fR;
	fP = n.fP;
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
	this->fP = other.fP;

	return *this;
}

double Node::GetDistanceTo(Node* n) {
	return sqrt(pow((fX - n->fX), 2) + pow((fY - n->fY), 2));
}
