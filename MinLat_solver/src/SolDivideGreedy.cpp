#include "SolDivideGreedy.h"


SolDivideGreedy::SolDivideGreedy() {}
SolDivideGreedy::SolDivideGreedy(const Solver &s): Solver(s) {}

void SolDivideGreedy::solve(Solution* solution, std::vector<HoverLocation> &vPotentialHL,
		std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS) {
	if(SANITY_PRINT)
		printf("\nRunning Divided Greedy\n");

	// Extreme y-coordinates
	double max_y = -INF;
	double min_y = INF;

	// Find extreme points
	for(HoverLocation hl : vPotentialHL) {
		// Verify that this HL is worth stopping at...
		if(vSPerHL.at(hl.nID).size() > 0) {
			// Check to see if we have an update to min/max y
			if(hl.fY > max_y) {
				max_y = hl.fY;
			}
			if(hl.fY < min_y) {
				min_y = hl.fY;
			}
		}
	}

	// Find partition bounds
	double total_size = max_y - min_y;
	double part_size = total_size/solution->m_nV;
	double current_lower_bound = min_y - EPSILON;
	double current_upper_bound = min_y + part_size;

	// Store all HLs in lists for each UAV, and store the list in a vector
	std::vector<std::list<HoverLocation>> HLsPerUAV;

	if(DEBUG_DIV_GREED) {
		printf("Add HLs to UAV partitions\n");
	}

	// Divide up the HLs based on partition bounds
	for(long unsigned int i = 0; i < solution->m_nV; i++) {
		// Create a new list of HLs for this UAV
		std::list<HoverLocation> hlList;
		for(HoverLocation hl : vPotentialHL) {
			if((hl.fY <= current_upper_bound + EPSILON) && (hl.fY >= current_lower_bound - EPSILON)) {
				// This hl is in the UAV's partion
				hlList.push_back(hl);
				if(DEBUG_DIV_GREED) {
					printf(" %ld: add HL: %d\n", i, hl.nID);
				}
			}
		}
		HLsPerUAV.push_back(hlList);

		// Update the bounds
		current_lower_bound = current_upper_bound;
		current_upper_bound = current_upper_bound + part_size;
	}

	// Make a serviced array
	bool* pServiced = new bool[solution->m_pG->vNodeLst.size()];
	for(unsigned long int i = 0; i < solution->m_pG->vNodeLst.size(); i++) {
		pServiced[i] = false;
	}

	// While there are still un-serviced sensors ...
	bool unservicedSensors = true;
	unsigned long int tourIndex = 0;
	unsigned long int UAVIndex = 0;
	unsigned long int iteration = 0;
	while(unservicedSensors) {
		if(DEBUG_DIV_GREED)
			printf("Starting a new tour\n");
		// Create a new tour
		std::list<UAV_Stop> tour;
		// Start tour at the base station
		tour.push_back(UAV_Stop(solution->m_pG->mBaseStation.getX(), solution->m_pG->mBaseStation.getY(), 0));

		bool underBudget = true;
		// While we are still under our energy budget..
		while(underBudget) {
			// Find a new HL to attempt to add to tour

			// Start-off with the first HL in this UAV's HL list
			std::list<HoverLocation>::iterator bstIt = HLsPerUAV.at(UAVIndex).begin();
			// Track closest distance
			double bestDist = INF;

			// Run through our list of HL's, pick closest to UAV's current location
			for(std::list<HoverLocation>::iterator hlIt = HLsPerUAV.at(UAVIndex).begin(); hlIt != HLsPerUAV.at(UAVIndex).end();) {
				// Determine if this HL services any un-serviced sensors
				bool usefulHL = false;
				// Check each HL for this sensor
				for(int l : vSPerHL.at(hlIt->nID)) {
					usefulHL |= !pServiced[l];
				}

				// Check if HL is useful
				if(usefulHL) {
					if(DEBUG_DIV_GREED)
						printf(" Found a useful HL: %d\n", hlIt->nID);
					// HL services at least 1 sensor, consider adding to tour
					double tempDist = distAtoB(tour.back().fX, tour.back().fY, hlIt->fX, hlIt->fY);
					// Check to see if this HL is better than current best
					if(tempDist < bestDist) {
						if(DEBUG_DIV_GREED)
							printf("  new best\n");
						// Found new best!
						bstIt = hlIt;
						bestDist = tempDist;
					}

					// Advance iterator
					hlIt++;
				}
				else {
					// HL is no longer helpful, remove from list (advances iterator)
					if(DEBUG_DIV_GREED)
						printf(" Can't use HL: %d\n", hlIt->nID);
					hlIt = HLsPerUAV.at(UAVIndex).erase(hlIt);
				}
			}

			// Check if we found a valid next-stop
			if(bestDist < INF) {
				if(DEBUG_DIV_GREED)
					printf(" Attempting to add %d to tour\n", bstIt->nID);
				// Attempt to add this stop to the tour
				UAV_Stop nextStop(bstIt->fX, bstIt->fY, bstIt->nID);
				// Add un-serviced sensors to stop
				for(int l : vSPerHL.at(bstIt->nID)) {
					if(!pServiced[l]) {
						nextStop.nodes.push_back(l);
					}
				}
				// Verify budget by returning to depot
				UAV_Stop lastStop(solution->m_pG->mBaseStation.getX(), solution->m_pG->mBaseStation.getY(), vPotentialHL.back().nID);
				// Add these stops to the tour
				tour.push_back(nextStop);
				tour.push_back(lastStop);

				// Check energy consumption of tour
				double budget = 0;
				std::list<UAV_Stop>::iterator lst, nxt;
				lst = tour.begin();
				nxt = tour.begin();
				nxt++;

				// Run through the tour, add up pst for each leg + HL stop
				while(nxt != tour.end()) {
					// Add time to move from lst to nxt
					budget += lst -> edgeCost(*nxt);
					// Add in time to talk to each sensor at nxt
					for(int s : nxt->nodes) {
						budget += solution->m_pG->vNodeLst.at(s).sensorCost(*nxt) ;
					}

					// Advance iterators
					lst++;
					nxt++;
				}
				if(DEBUG_DIV_GREED)
					printf(" New budget: %f\n", budget);

				// If good ...
				if(budget <= Q) {
					if(DEBUG_DIV_GREED)
						printf(" Add to tour!\n");
					// Mark sensors
					for(int l : vSPerHL.at(bstIt->nID)) {
						pServiced[l] = true;
					}
					// Remove depot
					tour.pop_back();
					// Continue running
					underBudget = true;
				}
				else {
					// Adding this stop put us over budget!
					if(DEBUG_DIV_GREED)
						printf(" Went over budget\n");
					// Remove stop
					tour.pop_back();
					tour.pop_back();
					// Add depot
					tour.push_back(lastStop);
					// Finish this tour
					underBudget = false;
				}
			}
			else {
				// No useful HLs => we have serviced all sensors reachable in this UAV's partition
				if(DEBUG_DIV_GREED)
					printf("Serviced every sensor for UAV %ld\n", UAVIndex);
				// Add on depot
				UAV_Stop depot(solution->m_pG->mBaseStation.getX(), solution->m_pG->mBaseStation.getY(), vPotentialHL.back().nID);
				tour.push_back(depot);
				// Exit loop
				underBudget = false;
			}
		}

		// Add this tour to the vector of tours
		solution->vTours.push_back(tour);

		if(DEBUG_DIV_GREED)
			printf("Finished tour %ld\n", tourIndex);

		// Determine if there are still any un-serviced sensors
		unservicedSensors = false;
		for(unsigned long int i = 0; i < solution->m_pG->vNodeLst.size(); i++) {
			unservicedSensors |= !pServiced[i];
		}

		// Update indexes
		iteration++;
		tourIndex = iteration/solution->m_nV;
		UAVIndex = iteration%solution->m_nV;
	}
	if(DEBUG_DIV_GREED)
		printf("Tours complete!\n");

	if(SANITY_PRINT) {
		printf("\nFiltered tour:\n");
		for(unsigned long int i = 0; i < solution->vTours.size(); i++) {
			std::list<UAV_Stop> l = solution->vTours.at(i);
			printf(" %ld: ", i);
			for(UAV_Stop n : l) {
				printf("(%f, %f) ", n.fX, n.fY);
			}
			printf("\n");
		}
	}

	delete[] pServiced;
}
