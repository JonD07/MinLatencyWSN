/*
 * Node.h
 *
 * Created by:	Jonathan Diller
 * On: 			Mar 13, 2022
 *
 * Description: Node class, used to build graphs.
 */

#pragma once

#include <math.h>

#include "defines.h"

class Node {
public:
	Node();
	Node(int id, double x, double y, double r, double p);
	Node(const Node &n);
	~Node();

	// Returns the distance (in meters) from this vertex to v
	double GetDistanceTo(Node* v);
	// Get this node's ID
	int getID() { return nID; }
	// Get this node's x coordinate
	double getX() { return fX; }
	// Get this node's y coordinate
	double getY() { return fY; }
	// Get this node's WiFi range
	double getR() { return fR; }
	// Get this node's priority
	double getPriority() { return fP; }
	// Get node's locked condition
	bool Locked() { return bLocked; }
	// Set node's locked condition
	void setLocked() { bLocked = true; }

	// Overloaded assignment operator
	Node& operator=(const Node& other);

private:
	int nID;
	double fX, fY, fR, fP;
	bool bLocked;
};
