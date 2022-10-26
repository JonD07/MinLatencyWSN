#include "Utilities.h"


bool isZero(double c) {
    return (c < EPSILON) && (c > -EPSILON);
}

double distAtoB(double x_1, double y_1, double x_2, double y_2) {
	return sqrt(pow((x_1 - x_2), 2) + pow((y_1 - y_2), 2));
}

std::string itos(int i) {
    std::stringstream s;
    s << i;
    return s.str();
}

double stopTime(unsigned long int k) {
    if(k == 0) {
        // No penalty for first sub-tour
        return 0.0;
    }
    else {
        // Battery swap penalty
        return BAT_SWAP;
    }
}
