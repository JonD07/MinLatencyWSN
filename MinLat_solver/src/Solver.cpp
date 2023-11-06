#include "Solver.h"


Solver::Solver() {}

Solver::Solver(const Solver &s) {
	budget = s.budget;

}
Solver::Solver(double b) {
	budget = b;
}

Solver::~Solver(){}


// Runs the underlying algorithm
Solution* Solver::RunSolver(Graph* pG, int nV, std::vector<HoverLocation> &vPotentialHL, std::vector<std::list<int>> &vSPerHL, std::vector<std::list<int>> &vHLPerS) {
	// Create a solution
	Solution* solution = new Solution(pG, nV);
	// Run the descendant class solve function
	solve(solution, vPotentialHL, vSPerHL, vHLPerS);
	return solution;
}
