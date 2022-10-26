#include "Location.h"

Location::Location(int id, double x, double y): nID(id), fX(x), fY(y) {}

Location::Location(const Location &loc) {
	nID = loc.nID;
	fX = loc.fX;
	fY = loc.fY;
}

double Location::edgeTime(Location& i) {
    return distAtoB(i.fX, i.fY, fX, fY) / V_MAX;
}

double Location::edgeCost(Location& i) {
    // We are using pst, so this is the actual travel time
    return edgeTime(i);
}


// Measure the Euclidean distance to i
double Location::distTo(Location& i) {

	double x_1 = fX, y_1 = fY, x_2 = i.fX, y_2 = i.fY;

	return sqrt(pow((x_1 - x_2), 2) + pow((y_1 - y_2), 2));
}

Location::~Location() {}
