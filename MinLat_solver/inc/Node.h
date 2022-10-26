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
#include "Location.h"

class Node {
public:
	// TODO: Do we need an empty constructor?
	Node();
	Node(int id, double x, double y, double r);
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
	// Get node's locked condition
	bool Locked() { return bLocked; }
	// Set node's locked condition
	void setLocked() { bLocked = true; }
	// Set node's locked condition
	void unLock() { bLocked = false; }
    //Budget cost to communicate with sensor at location from hovering location l
    double sensorCost(Location& l);
    // Actual time to communicate with sensor from UAV stop l
    double sensorTime(Location& l);

	// Overloaded assignment operator
	Node& operator=(const Node& other);

private:
	int nID;
	double fX, fY, fR;
	bool bLocked;
};
