/*
 * defines.h
 *
 * Created by:	Jonathan Diller
 * On: 			Mar 13, 2022
 *
 * Description: Global project defines.
 */

#pragma once

#define DEBUG			0
#define SANITY_PRINT	1


#define	MAX_UAV_DIST	10.0
#define EPSILON			0.0001
#define INF				1000000000000

// Max UAV velocity and max distance at max velocity
#define V_MAX			19.0
#define D_VM			3000.0
// UAV energy budget
#define Q				D_VM/V_MAX
// Battery swap time (in seconds)
#define BAT_SWAP		90

#define PRINT_RESULTS		true
#define DATA_LOG_LOCATION	"Experiment2/alg_%d.dat"
#define MAKE_PLOT_FILE		true
#define PLOT_FILE_LOCATION	"output_path.txt"

#define MILP_I			1
#define GREEDY_NN		2
#define MILP_II			3
#define CLUSTERING		4

