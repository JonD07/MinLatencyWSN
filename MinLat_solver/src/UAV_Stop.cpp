//
// Created by peter on 9/26/2022.
// Struct to hold UAV stops
//

#include "UAV_Stop.h"


UAV_Stop::UAV_Stop(double x, double y): Location(-1, x, y) {}

UAV_Stop::UAV_Stop(double x, double y, int mappedID): Location(mappedID, x, y) {}

UAV_Stop::UAV_Stop(const UAV_Stop &stp): Location(stp.nID, stp.fX, stp.fY) {
    for(int n : stp.nodes) {
        nodes.push_back(n);
    }
}

UAV_Stop::UAV_Stop(const HoverLocation &hl) : Location(hl) {
    for(int n : hl.nodes) {
        nodes.push_back(n);
    }
}

UAV_Stop::UAV_Stop(const Location &loc) : Location(loc) {}
