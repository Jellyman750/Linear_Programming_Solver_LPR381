
  
using System;
using System.Collections.Generic;
using System.Linq;

namespace Linear_Programming_Solver.Models
{
     public class BranchAndBound
    {
        private LPSolver solver = new LPSolver();
        private int subProblemCounter = 1;
        public double BestObjective { get; private set; } = double.NegativeInfinity;
        public double[] BestSolution { get; private set; }

        // Main entry
        public void Solve(LPProblem problem, string algorithm, Action<string> logCallback)
        {
            // Use LPSolver with the chosen algorithm for relaxed subproblems
            var solver = new LPSolver();
            var result = solver.Solve(problem, algorithm, (text, highlight) =>
            {
                logCallback?.Invoke(text);
            });

            logCallback?.Invoke("\nBranch & Bound Finished.\nFinal z = " + result.Summary);
        }

        private void SolveSubproblem(LPProblem problem, string name, Action<string> logCallback)
        {
            logCallback?.Invoke($"\n{name}: Solving LP relaxation...");

            // Solve LP relaxation
            var result = solver.Solve(problem, "Primal Simplex");

            // Extract solution vector
            double[] x = result.Summary.Contains("x* = [")
                ? ParseSolutionVector(result.Summary)
                : new double[problem.NumVars];

            double zValue = ParseObjectiveValue(result.Summary);

            logCallback?.Invoke($"{name} LP solution: z* = {zValue}, x* = [{string.Join(", ", x)}]");

            // Check if integer feasible
            int fractionalIndex = -1;
            for (int i = 0; i < x.Length; i++)
            {
                if (Math.Abs(x[i] - Math.Round(x[i])) > 1e-6)
                {
                    fractionalIndex = i;
                    break;
                }
            }

            if (fractionalIndex == -1)
            {
                // Integer solution → update best if better
                if (zValue > BestObjective)
                {
                    BestObjective = zValue;
                    BestSolution = x;
                }
                logCallback?.Invoke($"{name} is integer feasible. z* = {zValue}");
                return;
            }

            // Branch on fractional variable
            int varIndex = fractionalIndex;
            double fracValue = x[varIndex];

            // Subproblem 1: x <= floor
            var sub1 = problem.Clone();
            sub1.Constraints.Add(new Constraint
            {
                A = UnitVector(problem.NumVars, varIndex),
                Relation = Rel.LE,
                B = Math.Floor(fracValue)
            });

            // Subproblem 2: x >= ceil
            var sub2 = problem.Clone();
            sub2.Constraints.Add(new Constraint
            {
                A = UnitVector(problem.NumVars, varIndex),
                Relation = Rel.GE,
                B = Math.Ceiling(fracValue)
            });

            // Recursively solve subproblems
            SolveSubproblem(sub1, $"Subproblem {subProblemCounter++}: x{varIndex + 1} <= {Math.Floor(fracValue)}", logCallback);
            SolveSubproblem(sub2, $"Subproblem {subProblemCounter++}: x{varIndex + 1} >= {Math.Ceiling(fracValue)}", logCallback);
        }

        private static double[] UnitVector(int size, int index)
        {
            double[] v = new double[size];
            v[index] = 1.0;
            return v;
        }

        private static double[] ParseSolutionVector(string summary)
        {
            // Extract x* = [..] from summary
            int start = summary.IndexOf("x* = [") + 6;
            int end = summary.IndexOf("]", start);
            string vec = summary.Substring(start, end - start);
            return vec.Split(',').Select(s => double.Parse(s.Trim())).ToArray();
        }

        private static double ParseObjectiveValue(string summary)
        {
            // Extract z* = ... from summary
            foreach (var line in summary.Split('\n'))
            {
                if (line.StartsWith("z*"))
                {
                    var parts = line.Split('=');
                    if (parts.Length == 2 && double.TryParse(parts[1].Trim(), out double val))
                        return val;
                }
            }
            return 0;
        }
    }
}
