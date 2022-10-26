//
// Created by peter on 9/26/2022.
// Struct to hold two roots of a quadratic equation
//

#ifndef MINLATENCYWSN_ROOTS_H
#define MINLATENCYWSN_ROOTS_H

#include "Utilities.h"

#define DEBUG_ROOTS	DEBUG || 0

class Roots {
public:
    double root1;
    double root2;
    bool imaginary;

    // Find the roots of a quadratic equation, stored in root1 and root2
    // Assumes that we were given an actual quadratic equation (a != 0).
    void findRoots(double a, double b, double c);

    Roots();
};


#endif //MINLATENCYWSN_ROOTS_H
