//
// Created by peter on 9/26/2022.
// Struct to hold UAV Stops
//

#ifndef MINLATENCYWSN_UAV_STOP_H
#define MINLATENCYWSN_UAV_STOP_H

#include <list>

#include "Location.h"


class UAV_Stop : public Location{
public:
    
    std::list<int> nodes;

    UAV_Stop(double x, double y);
    UAV_Stop(double x, double y, int mappedID);
    UAV_Stop(const UAV_Stop &stp);

};


#endif //MINLATENCYWSN_UAV_STOP_H