//
// Created by peter on 9/27/2022.
//

#include "../inc/Location.h"
Location::Location(int id, double x, double y): nID(id), fX(x), fY(y) {}

double Location::edgeTime(Location& i) {
    return distAtoB(i.fX, i.fY, fX, fY) / V_MAX;
}

double Location::edgeCost(Location& i) {
    // We are using pst, so this is the actual travel time
    return this -> edgeTime(i);
}

Location::~Location() {}