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
#define BAT_SWAP		60

#define PRINT_RESULTS		1
#define DATA_LOG_LOCATION	"Experiment2/alg_%d.dat"
#define MAKE_PLOT_FILE		1
#define PLOT_FILE_LOCATION	"output_path.txt"
#define GRAPH_FILE_LOCATION	"output_graph.txt"
#define MAKE_PLAN_FILE		0
#define PLAN_FILE_LOCATION	"drone_%d_%d.pln"

/*
 * Algorithm combos
 *
 * Select Hovering Locations:
 * AN - above node only
 * AC - all combos
 *
 * Algorithm:
 * MILP - as advertised (I do not recommend)
 * NN   - nearest neighbor algorithm
 * CL   - clustering algorithm
 * DG   - divide-greedy algorithm
 *
 * Post processing:
 * I  - improve tour
 * NI - do not improve tour
 *
 */
#define ALG_COMBO_AC_MILP_I	0
#define ALG_COMBO_AC_NN_I	1
#define ALG_COMBO_AN_CL_NI	2
#define ALG_COMBO_AN_CL_I	3
#define ALG_COMBO_AN_DG_NI	4
#define ALG_COMBO_AN_DG_I	5
#define ALG_COMBO_AC_CL_NI	6
#define ALG_COMBO_AC_CL_I	7
#define ALG_COMBO_AC_DG_NI	8
#define ALG_COMBO_AC_DG_I	9


