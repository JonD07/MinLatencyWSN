//
// Created by peter on 10/11/2022.
//

#include "LKH_TSP_Solver.h"


LKH_TSP_Solver::LKH_TSP_Solver(std::vector<HoverLocation> HL){
	vHL = HL;
}
LKH_TSP_Solver::~LKH_TSP_Solver(){

}

void LKH_TSP_Solver::Write_LKH_Config(){

	//Param file setup
	FILE * pParFile;
	char buff1[100];
	sprintf(buff1, "WSN_TSP.par");
	pParFile = fopen(buff1, "w");

	fprintf(pParFile, "PROBLEM_FILE = WSN_TSP.tsp\n");
	fprintf(pParFile, "COMMENT Solving Single Symmetric TSP for WSN Latency\n");
	fprintf(pParFile, "TOUR_FILE = LKH_output.dat\n");

	fclose(pParFile);

	//Data file setup
	FILE * pDataFile;
	char buff2[100];
	sprintf(buff2, "WSN_TSP.tsp");

	pDataFile = fopen(buff2, "w");

	fprintf(pDataFile, "NAME : WSN_TSP\n");
	fprintf(pDataFile, "COMMENT : Solving Single Symmetric TSP for WSN Latency\n");
	fprintf(pDataFile, "TYPE : TSP\n");
	fprintf(pDataFile, "DIMENSION : %ld\n", vHL.size());
	fprintf(pDataFile, "EDGE_WEIGHT_TYPE : EUC_2D\n");
	fprintf(pDataFile, "NODE_COORD_SECTION\n");

	for(long unsigned int i = 0; i < vHL.size(); i++){
		fprintf(pDataFile, "%ld %f %f\n", i+1, vHL[i].fX, vHL[i].fY);
	}

	fprintf(pDataFile, "EOF\n");
	fclose(pDataFile);

}

void LKH_TSP_Solver::Solve_TSP(){

	//Run LKH
	std::system("./LKH-3.0.6/LKH WSN_TSP.par");

	//Collect Results
	std::ifstream file("LKH_output.dat");

	//First few lines are header, remove them
	std::string line;
	for(int i = 0; i<6; i++){
		std::getline(file,line);
	}

	//Parse Data
	for(unsigned long int i = 0; i < vHL.size(); i++){
		std::getline(file, line);
		std::stringstream lineStreamN(line);

		//Get way-point from the line
		int n;
		lineStreamN >> n;
		vPath.push_back(n-1);
	}

}