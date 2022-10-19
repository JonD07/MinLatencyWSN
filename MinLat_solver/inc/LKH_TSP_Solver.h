//
// Created by peter on 10/11/2022.
//

#pragma once

#include <vector>
#include <sstream>
#include <fstream>
#include <stdlib.h>

#include "HoverLocation.h"
class LKH_TSP_Solver {
public:
	LKH_TSP_Solver(std::vector<HoverLocation> HL);
	~LKH_TSP_Solver();

	void Write_LKH_Config();

	void Solve_TSP();

	std::vector<HoverLocation> vHL;
	std::vector<int> vPath;

};

