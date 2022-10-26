#include "LKH_TSP_Solver.h"


LKH_TSP_Solver::LKH_TSP_Solver() {
}

LKH_TSP_Solver::~LKH_TSP_Solver() {

}

/*
 * Runs the LKH heuristic TSP solver on the locations in vLoc and stores the
 * results in vPath. vPath will be an ordered list of indexes that point to
 * the locations in vLoc.
 */
void LKH_TSP_Solver::Solve_TSP(std::vector<UAV_Stop> &vLoc, std::vector<int> &vPath) {
	// Make sure that this is worth solving
	if(vLoc.size() <= 2) {
		// Don't call solver
		int i = 0;
		for(auto n : vLoc) {
			vPath.push_back(i);
			i++;
		}
		return;
	}

	// Create the config files for vLoc
	write_LKH_Config(vLoc);

	if(DEBUG_LKH)
		printf("Running LKH\n");

	// Call solver
	std::system("LKH WSN_TSP.par > LKH_run-output.txt");

	if(DEBUG_LKH)
		printf("Found the following solution:\n");

	// Open file with results
	std::ifstream file("LKH_output.dat");

	// Remove the first few lines...
	std::string line;
	for(int i = 0; i < 6; i++) {
		std::getline(file, line);
	}

	// Start parsing the data
	for(long unsigned int i = 0; i < vLoc.size(); i++) {
		std::getline(file, line);
		std::stringstream lineStreamN(line);
		// Parse the way-point from the line
		int n;
		lineStreamN >> n;
		vPath.push_back(n-1);
		if(DEBUG_LKH)
			printf(" %d", n);
	}
	if(DEBUG_LKH)
		printf("\n");

	file.close();
}




// Generates the config files to solve TSP on the locations in vLoc
void LKH_TSP_Solver::write_LKH_Config(std::vector<UAV_Stop> &vLoc) {
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
	fprintf(pDataFile, "DIMENSION : %ld\n", vLoc.size());
	fprintf(pDataFile, "EDGE_WEIGHT_TYPE : EUC_2D\n");
	fprintf(pDataFile, "NODE_COORD_SECTION\n");

	 auto it = vLoc.begin();

	for(long unsigned int i = 0; i < vLoc.size() && it != vLoc.end(); i++, it++) {
		// Write location coordinate to file, must be "1" indexed
		fprintf(pDataFile, "%ld %f %f\n", i+1, it->fX, it->fY);
	}

	fprintf(pDataFile, "EOF\n");
	fclose(pDataFile);
}
