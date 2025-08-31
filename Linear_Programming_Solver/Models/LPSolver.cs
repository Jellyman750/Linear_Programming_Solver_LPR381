using System;
using System.Text.RegularExpressions;

namespace Linear_Programming_Solver.Models
{
    public class LPSolver
    {
        /// <summary>
        /// Stores the last final tableau for sensitivity analysis.
        /// </summary>
        public double[,] FinalTableau { get; private set; }

        /// <summary>
        /// Selects and runs the chosen LP algorithm.
        /// </summary>
        public SimplexResult Solve(LPProblem problem, string algorithm, Action<string, bool[,]> updatePivot = null)
        {
            string key = NormalizeAlgorithmKey(algorithm);

            ILPAlgorithm algo = key switch
            {
                // Primal simplex
                "primal simplex" => new PrimalSimplex(),
                "primal" => new PrimalSimplex(),

                // Revised primal simplex
                "revised primal simplex" => new RevisedPrimalSimplex(),
                "revised primal" => new RevisedPrimalSimplex(),

                // Dual simplex
                "dual simplex" => new DualSimplex(),
                "dual" => new DualSimplex(),

                // Branch & Bound (tableau-based)
                "branch and bound simplex" => new BranchAndBound(),
                "branch and bound" => new BranchAndBound(),
                "bnb" => new BranchAndBound(),

                _ => throw new Exception(
                    $"Algorithm not supported: '{algorithm}'. " +
                    "Try one of: Primal Simplex, Revised Primal Simplex, Dual Simplex, Branch and Bound Simplex."
                )
            };

            // Run the chosen algorithm
            var result = algo.Solve(problem, updatePivot);

            // Capture tableau if algorithm produced one
            if (result != null && result.Tableau != null)
            {
                FinalTableau = result.Tableau;
            }
            else
            {
                FinalTableau = null;
            }

            return result;
        }

        private static string NormalizeAlgorithmKey(string algorithm)
        {
            if (string.IsNullOrWhiteSpace(algorithm))
                throw new Exception("No algorithm selected.");

            // Lowercase, trim
            string key = algorithm.Trim().ToLowerInvariant();

            // Remove the word "algorithm" anywhere
            key = key.Replace("algorithm", "");

            // Collapse multiple spaces -> single space
            key = Regex.Replace(key, @"\s+", " ").Trim();

            return key;
        }
    }
}
