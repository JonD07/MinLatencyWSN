#include "HoverLocation.h"

HoverLocation::HoverLocation() : Location(-1, 0, 0), fWeight(0) {}

HoverLocation::HoverLocation(int nID, double fX, double fY, double fWeight): Location(nID, fX, fY), fWeight(fWeight){}

HoverLocation::HoverLocation(const HoverLocation &hl) : Location(hl), nodes(hl.nodes) {
	fWeight = hl.fWeight;
}

HoverLocation::~HoverLocation() {}
