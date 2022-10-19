//
// Created by peter on 9/27/2022.
//

#ifndef MINLATENCYWSN_LOCATION_H
#define MINLATENCYWSN_LOCATION_H

#include "Utilities.h"


class Location {
public:
    Location(int id, double x, double y);
    // Actual time to traverse between two locations
    double edgeTime(Location& i);
    // Budget cost to traverse between two hovering locations
    double edgeCost(Location& i);
    ~Location();
    int nID;
    double fX;
    double fY;
};


#endif //MINLATENCYWSN_LOCATION_H
