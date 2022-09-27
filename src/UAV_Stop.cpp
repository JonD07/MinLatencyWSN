//
// Created by peter on 9/26/2022.
// Struct to hold UAV stops
//

#include "../inc/UAV_Stop.h"


UAV_Stop::UAV_Stop(double x, double y) {
    nMappedID = -1;
    fX = x;
    fY = y;
}

UAV_Stop::UAV_Stop(double x, double y, int mappedID) {
    nMappedID = mappedID;
    fX = x;
    fY = y;
}

UAV_Stop::UAV_Stop(const UAV_Stop &stp) {
    nMappedID = stp.nMappedID;
    fX = stp.fX;
    fY = stp.fY;
    for(int n : stp.nodes) {
        nodes.push_back(n);
    }
}
