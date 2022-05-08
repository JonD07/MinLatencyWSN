#include "../inc/HoverLocation.h"

HoverLocation::HoverLocation() : nID(-1), fX(0), fY(0), fWeight(0) {}

HoverLocation::HoverLocation(int nID, double fX, double fY, double fWeight)
	: nID(nID), fX(fX), fY(fY), fWeight(fWeight) {}

HoverLocation::~HoverLocation() {}
