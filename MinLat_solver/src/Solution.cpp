#include "Solution.h"

Solution::Solution(Graph* G, int nV): m_pG(G), m_nV(nV) {}

Solution::Solution(const Solution &s) {
	m_pG = s.m_pG;
	m_nV = s.m_nV;
}

Solution::~Solution() {}


void Solution::printResults(double compTime, bool printFile, int approach, int nodeDensity, std::string file_flag) {
	if(DEBUG_SOL)
		printf("\nFound tour:\n");
	double total_duration = 0;
	double* uav_total_duration = new double[m_nV];
	for(unsigned long int i = 0; i < m_nV; i++) {
		uav_total_duration[i] = 0;
	}

	// Run through each sub-tour
	for(unsigned long int i = 0; i < vTours.size(); i++) {
		int UAV_ID = i%m_nV;
		int Tour_ID = i/m_nV;
		if(vTours.at(i).size() > 2) {
			// Start with initial stop-time
			double duration = stopTime(Tour_ID);
			std::list<UAV_Stop> l = vTours.at(i);
			if(l.size() > 1) {
				std::list<UAV_Stop>::iterator lst, nxt;
				lst = l.begin();
				nxt = l.begin();
				nxt++;

				// Run through the tour, add up distance from stop-to-stop
				while(nxt != l.end()) {
					// Add time to move from lst to nxt
					duration += lst -> edgeTime(*nxt);
					// Add in time to talk to each sensor at nxt
					for(int s : nxt->nodes) {
						duration += m_pG->vNodeLst.at(s).sensorTime(*nxt) ;
					}

					// Advance iterators
					lst++;
					nxt++;
				}
			}

			total_duration += duration;
			uav_total_duration[UAV_ID] += duration;

			if(DEBUG_SOL) {
				printf("%d-%d: duration = %f\n ", UAV_ID, Tour_ID, duration);
				for(UAV_Stop n : l) {
					printf(" (%f, %f)", n.fX, n.fY);
				}
				printf("\n");
			}

		}
	}

	// Identify which UAV ends last
	double max_latency = 0;
	for(unsigned long int i = 0; i < m_nV; i++) {
		if(uav_total_duration[i] > max_latency) {
			max_latency = uav_total_duration[i];
		}
	}

	if(SANITY_PRINT) {
		printf("Total duration = %f\n", total_duration);
		printf("Worst latency = %f\n", max_latency);
		printf("Comp Time = %f\n", compTime);
	}


	if(printFile) {
		// Print results to file
		FILE * pOutputFile;
		char buff[100];
		sprintf(buff, DATA_LOG_LOCATION, file_flag.c_str(), approach);
		if(SANITY_PRINT)
			printf("%s\n", buff);
		pOutputFile = fopen(buff, "a");
		// Each line in data file is: number of nodes, number of UAVs, node density, total duration, max latency, computation time
		fprintf(pOutputFile, "%ld %ld %d %f %f", m_pG->vNodeLst.size(), m_nV, nodeDensity, total_duration, max_latency);
		fprintf(pOutputFile, " %.4f\n", compTime);
		fclose(pOutputFile);
	}

	delete[] uav_total_duration;
}

// Create the autopilot plan for this solution
void Solution::printPlan() {
	// Loop through each tour
	for(long unsigned int i = 0; i < vTours.size(); i++) {
		int uavID = (int)i%m_nV;
		int tourID = (int)i/m_nV;

		// Open plan file
		FILE * pOutputFile;
		char buff[100];
		sprintf(buff, PLAN_FILE_LOCATION, uavID, tourID);
		printf("%s\n", buff);
		pOutputFile = fopen(buff, "w");

		// Set default altitude
		fprintf(pOutputFile, "2 50\n");

		// Walk through list, skipping the first stop (base station)
		std::list<UAV_Stop>::iterator it = vTours.at(i).begin();
		it++;

		// Write out plan
		while(it != vTours.at(i).end()) {
			// If BS, skip (BS has not nodes)
			if(it->nodes.size() != 0) {
				// Add HL as waypoint
				fprintf(pOutputFile, "0 %f %f 50\n", it->fX, it->fY);

				// Which nodes to talk to
				for(int n : it->nodes) {
					fprintf(pOutputFile, "1 %d\n", n);
				}
			}

			it++;
		}

		// Close file
		fclose(pOutputFile);
	}
}

double Solution::GetWorstLatency() {
	// Check each sub-tour to find worst latency
	double* uav_total_duration = new double[m_nV];
	for(unsigned long int i = 0; i < m_nV; i++) {
		uav_total_duration[i] = 0;
	}

	// Run through each sub-tour
	for(unsigned long int i = 0; i < vTours.size(); i++) {
		int UAV_ID = i%m_nV;
		int Tour_ID = i/m_nV;
		if(vTours.at(i).size() > 2) {
			// Start with initial stop-time
			double duration = stopTime(Tour_ID);
			std::list<UAV_Stop> l = vTours.at(i);
			if(l.size() > 1) {
				std::list<UAV_Stop>::iterator lst, nxt;
				lst = l.begin();
				nxt = l.begin();
				nxt++;

				// Run through the tour, add up distance from stop-to-stop
				while(nxt != l.end()) {
					// Add time to move from lst to nxt
					duration += lst -> edgeTime(*nxt);
					// Add in time to talk to each sensor at nxt
					for(int s : nxt->nodes) {
						duration += m_pG->vNodeLst.at(s).sensorTime(*nxt) ;
					}

					// Advance iterators
					lst++;
					nxt++;
				}
			}

			uav_total_duration[UAV_ID] += duration;
		}
	}

	// Identify which UAV ends last
	double max_latency = 0;
	for(unsigned long int i = 0; i < m_nV; i++) {
		if(uav_total_duration[i] > max_latency) {
			max_latency = uav_total_duration[i];
		}
	}

	if(DEBUG_SOL)
		printf("Worst latency = %f\n ", max_latency);

	delete[] uav_total_duration;

	return max_latency;
}
