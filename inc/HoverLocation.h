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

class HoverLocation {
public:
	HoverLocation();
	HoverLocation(int nID, double fX, double fY, double fWeight);
	~HoverLocation();

	int nID;
	double fX;
	double fY;
	double fWeight;

private:
};
