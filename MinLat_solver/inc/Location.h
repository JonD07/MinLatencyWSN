//
// Created by peter on 9/27/2022.
//

#ifndef MINLATENCYWSN_LOCATION_H
#define MINLATENCYWSN_LOCATION_H

#include "Utilities.h"


class Location {
public:
    Location(int id, double x, double y);
    Location(const Location &loc);
    ~Location();

    // Actual time to traverse between two locations
    double edgeTime(Location& i);
    // Budget cost to traverse between two hovering locations
    double edgeCost(Location& i);
    // Measure the Euclidean distance to i
    double distTo(Location& i);

    int nID;
    double fX;
    double fY;
};


#endif //MINLATENCYWSN_LOCATION_H
