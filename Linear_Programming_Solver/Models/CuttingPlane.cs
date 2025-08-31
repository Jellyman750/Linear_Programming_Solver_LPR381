using System;
using System.Text;
using System.Linq;
using System.Globalization;
using Linear_Programming_Solver.Models;

namespace Linear_Programming_Solver.Models
{
    public class CuttingPlane : ILPAlgorithm
    {
        private const double Eps = 1e-9;

        public SimplexResult Solve(LPProblem problem, Action<string, bool[,]> updatePivot = null)
        {
            var simplex = new PrimalSimplex();
            var model = problem.Clone();
            var report = new StringBuilder();
            int iteration = 1;
            int maxIterations = 50;

            // Log problem
            report.AppendLine("=== Gomory Cutting Plane Algorithm ===");
            report.AppendLine($"Objective: Maximize {string.Join(" + ", problem.C.Select((c, i) => $"{c.ToString("F3", CultureInfo.InvariantCulture)}x{i + 1}"))}");
            report.AppendLine("Subject to:");
            for (int i = 0; i < problem.Constraints.Count; i++)
            {
                var c = problem.Constraints[i];
                report.AppendLine($"{string.Join(" + ", c.A.Select((a, j) => a != 0 ? $"{a.ToString("F3", CultureInfo.InvariantCulture)}x{j + 1}" : null).Where(s => s != null))} {c.Relation} {c.B.ToString("F3", CultureInfo.InvariantCulture)}");
            }
            report.AppendLine("x_j >= 0, integer");

            while (iteration <= maxIterations)
            {
                report.AppendLine($"\n--- Iteration {iteration} ---");

                // Solve LP
                SimplexResult lpResult;
                try
                {
                    lpResult = simplex.Solve(model, updatePivot);
                }
                catch (Exception ex)
                {
                    report.AppendLine($"Error in PrimalSimplex: {ex.Message}");
                    return new SimplexResult
                    {
                        Report = report.ToString(),
                        Summary = $"Error: {ex.Message}"
                    };
                }
                report.AppendLine(lpResult.Report);

                // Validate result
                if (lpResult.Tableau == null || lpResult.Basis == null || lpResult.Solution == null || lpResult.VarNames == null)
                {
                    report.AppendLine("Error: Invalid Simplex result.");
                    return new SimplexResult
                    {
                        Report = report.ToString(),
                        Summary = "Error: Invalid Simplex result"
                    };
                }

                // Extract solution (only decision variables)
                double[] solution = lpResult.Solution.Take(problem.NumVars).ToArray();
                if (solution.Length != problem.NumVars)
                {
                    report.AppendLine($"Error: Solution length ({solution.Length}) does not match NumVars ({problem.NumVars}).");
                    return new SimplexResult
                    {
                        Report = report.ToString(),
                        Summary = "Error: Invalid solution length"
                    };
                }
                report.AppendLine($"Current solution: x* = [{string.Join(", ", solution.Select(x => x.ToString("F3", CultureInfo.InvariantCulture)))}], z* = {lpResult.OptimalValue.ToString("F3", CultureInfo.InvariantCulture)}");

                // Check for integer solution
                int fracIndex = -1;
                double fracValue = 0;
                for (int i = 0; i < solution.Length; i++)
                {
                    double value = solution[i];
                    double frac = value - Math.Floor(value);
                    if (frac > Eps && frac < 1 - Eps)
                    {
                        fracIndex = i;
                        fracValue = value;
                        break;
                    }
                }

                if (fracIndex == -1)
                {
                    report.AppendLine("All variables integer. Optimal integer solution found.");
                    return new SimplexResult
                    {
                        Report = report.ToString(),
                        Summary = $"Status: OPTIMAL INTEGER\nz* = {lpResult.OptimalValue.ToString("F2", CultureInfo.InvariantCulture)}\nx* = [{string.Join(", ", solution.Select(x => x.ToString("F2", CultureInfo.InvariantCulture)))}]",
                        OptimalValue = lpResult.OptimalValue,
                        Solution = solution,
                        Tableau = lpResult.Tableau,
                        Basis = lpResult.Basis,
                        VarNames = lpResult.VarNames
                    };
                }

                // Find tableau row for basic variable x_fracIndex
                int row = -1;
                for (int i = 0; i < lpResult.Basis.Length; i++)
                {
                    if (lpResult.Basis[i] == fracIndex)
                    {
                        row = i + 1; // +1 because row 0 is objective
                        break;
                    }
                }
                if (row == -1)
                {
                    report.AppendLine($"Error: Variable x{fracIndex + 1} is not basic.");
                    return new SimplexResult
                    {
                        Report = report.ToString(),
                        Summary = "Error: Non-basic fractional variable"
                    };
                }

                // Generate Gomory cut
                var cut = GenerateGomoryCut(lpResult.Tableau, row, problem.NumVars, lpResult.VarNames);
                model.Constraints.Add(cut);
                report.AppendLine($"Added Gomory cut: {string.Join(" + ", cut.A.Select((a, i) => a != 0 ? $"{a.ToString("F3", CultureInfo.InvariantCulture)}x{i + 1}" : null).Where(s => s != null))} <= {cut.B.ToString("F3", CultureInfo.InvariantCulture)}");

                iteration++;
            }

            report.AppendLine("Iteration limit reached. Stopping.");
            return new SimplexResult
            {
                Report = report.ToString(),
                Summary = "Status: INCOMPLETE"
            };
        }

        private Constraint GenerateGomoryCut(double[,] tableau, int row, int numVars, string[] varNames)
        {
            var cut = new Constraint
            {
                A = new double[numVars],
                Relation = Rel.LE
            };

            // Gomory cut: sum(f_j * x_j) <= f_0, where f_j is fractional part of tableau[row,j]
            double rhs = tableau[row, tableau.GetLength(1) - 1];
            double f0 = rhs - Math.Floor(rhs);
            for (int j = 0; j < numVars; j++)
            {
                double aij = tableau[row, j];
                double fj = aij - Math.Floor(aij);
                if (fj > Eps)
                    cut.A[j] = fj;
            }
            cut.B = f0;

            return cut;
        }
    }
}