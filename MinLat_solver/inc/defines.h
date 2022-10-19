/*
 * defines.h
 *
 * Created by:	Jonathan Diller
 * On: 			Mar 13, 2022
 *
 * Description: Global project defines.
 */

#pragma once

#define DEBUG	1


#define	MAX_UAV_DIST	10.0
#define EPSILON			0.0001
#define INF				1000000000000

// Max UAV velocity and max distance at max velocity
#define V_MAX			19.0
#define D_VM			3000.0
// UAV energy budget
#define Q				D_VM/V_MAX

#define PRINT_RESULTS		true
#define DATA_LOG_LOCATION	"Output/alg_%d.dat"
#define MAKE_PLOT_FILE		true
#define PLOT_FILE_LOCATION	"output_path.txt"

#define MILP_I			1
#define GREEDY_NN		3
#define MILP_II			5
#define ALGORITHM		MILP_II

