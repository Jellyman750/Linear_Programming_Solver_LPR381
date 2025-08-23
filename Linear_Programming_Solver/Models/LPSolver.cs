using System;
using Linear_Programming_Solver.Models;

namespace Linear_Programming_Solver.Models
{
    public class LPSolver
    {
        /// <summary>
        /// Selects and runs the chosen LP algorithm.
        /// </summary>
        /// <param name="problem">The LP problem to solve.</param>
        /// <param name="algorithm">Name of the algorithm (e.g., "Primal Simplex").</param>
        /// <param name="updatePivot">Optional callback for pivot updates.</param>
        /// <returns>Solution as SimplexResult.</returns>
        public SimplexResult Solve(LPProblem problem, string algorithm, Action<string, bool[,]> updatePivot = null)
        {
            string algoKey = algorithm.Trim().ToLower();
            ILPAlgorithm algo = algoKey switch
            {
                "primal simplex" => new PrimalSimplex(),
                "revised primal simplex" => new RevisedPrimalSimplex(),
                "dual simplex" => new DualSimplex(),
                "branch and bound simplex" => new BranchAndBound(),
                _ => throw new Exception("Algorithm not supported")
            };

            return algo.Solve(problem, updatePivot);
        }
    }
}
