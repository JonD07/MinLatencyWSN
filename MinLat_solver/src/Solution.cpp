#include "Solution.h"

Solution::Solution(Graph* G, int nV): m_pG(G), m_nV(nV) {}

Solution::Solution(const Solution &s) {
	m_pG = s.m_pG;
	m_nV = s.m_nV;
}

Solution::~Solution() {}


void Solution::printResults(double compTime, bool printFile, int approach) {
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
		sprintf(buff, DATA_LOG_LOCATION, approach);
		printf("%s\n", buff);
		pOutputFile = fopen(buff, "a");
		fprintf(pOutputFile, "%ld %ld %f %f", m_pG->vNodeLst.size(), m_nV, total_duration, max_latency);
		fprintf(pOutputFile, " %.4f\n", compTime);
		fclose(pOutputFile);
	}

	delete[] uav_total_duration;
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
