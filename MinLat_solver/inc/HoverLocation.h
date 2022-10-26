/*
 * HoverPoint.h
 *
 * Created by:	Jonathan Diller
 * On: 			Apr 07, 2022
 *
 * Description: HoverPoint, a potential physical location for a UAV to go to collect data.
 */

#pragma once

#include "defines.h"
#include "Node.h"
#include "Location.h"

class HoverLocation: public Location {
public:
	HoverLocation();
	HoverLocation(int nID, double fX, double fY, double fWeight);
	HoverLocation(const HoverLocation &hl);
	~HoverLocation();

	//int nID;
	//double fX;
	//double fY;
	double fWeight;
    std::list<int> nodes;

private:
};
