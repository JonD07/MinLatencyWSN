//
// Created by peter on 9/28/2022.
//

#ifndef MINLATENCYWSN_UTILITIES_H
#define MINLATENCYWSN_UTILITIES_H

#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <queue>
#include <list>

#include "defines.h"


bool isZero(double c);

double distAtoB(double x_1, double y_1, double x_2, double y_2);

std::string itos(int i);

double stopTime(unsigned long int k);




#endif //MINLATENCYWSN_UTILITIES_H
