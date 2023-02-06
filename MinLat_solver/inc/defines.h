/*
 * defines.h
 *
 * Created by:	Jonathan Diller
 * On: 			Mar 13, 2022
 *
 * Description: Global project defines.
 */

#pragma once

#define DEBUG			1
#define SANITY_PRINT	1


#define EPSILON			0.0001
#define INF				1000000000000

// Max UAV velocity and max distance at max velocity
// 19.0, smallsat: 10
#define V_MAX			10
// Old: 3000.0, smallsat: 7500.0
#define D_VM			7500.0
// Old: ?, smallsat: 14*60
#define HOVER_TIME		14*60
// UAV energy budget
#define Q				D_VM/V_MAX
// Battery swap time (in seconds)
#define BAT_SWAP		90

#define PRINT_RESULTS		1
#define DATA_LOG_LOCATION	"Experiment2/alg_%d.dat"
#define MAKE_PLOT_FILE		1
#define PLOT_FILE_LOCATION	"output_path.txt"
#define MAKE_PLAN_FILE		1
#define PLAN_FILE_LOCATION	"drone_%d_%d.pln"

#define MILP_I			1
#define GREEDY_NN		2
#define MILP_II			3
#define CLUSTERING		4
#define DIV_GREEDY		5

