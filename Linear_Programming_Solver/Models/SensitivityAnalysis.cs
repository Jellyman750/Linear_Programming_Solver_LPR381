using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Globalization;
using Linear_Programming_Solver.Models;

namespace Linear_Programming_Solver.Analysis
{
    public class SensitivityAnalysis
    {
        private readonly LPProblem problem;
        private readonly SimplexResult result;
        private readonly double[,] tableau;
        private readonly int[] basis;
        private readonly string[] varNames;

        public SensitivityAnalysis(LPProblem problem, SimplexResult result)
        {
            this.problem = problem ?? throw new ArgumentNullException(nameof(problem));
            this.result = result ?? throw new ArgumentNullException(nameof(result));

            // Validate inputs
            if (result.Tableau == null)
                throw new ArgumentException("SimplexResult.Tableau cannot be null.");
            if (result.Basis == null)
                throw new ArgumentException("SimplexResult.Basis cannot be null.");
            if (result.VarNames == null)
                throw new ArgumentException("SimplexResult.VarNames cannot be null.");

            this.tableau = result.Tableau;
            this.basis = result.Basis;
            this.varNames = result.VarNames;

            // Validate tableau dimensions
            int m = problem.Constraints.Count;
            int n = problem.NumVars + m + 1; // Vars + slacks + RHS
            if (tableau.GetLength(0) != m + 1 || tableau.GetLength(1) != n)
                throw new ArgumentException($"Tableau dimensions invalid. Expected {m + 1} rows, {n} columns, got {tableau.GetLength(0)} rows, {tableau.GetLength(1)} columns.");
            if (basis.Length != m)
                throw new ArgumentException($"Basis length invalid. Expected {m}, got {basis.Length}.");
            if (varNames.Length < problem.NumVars + m)
                throw new ArgumentException($"VarNames length invalid. Expected at least {problem.NumVars + m}, got {varNames.Length}.");
        }

        // ===================== PUBLIC METHODS =====================
        public string GetRangeReport(string target)
        {
            if (string.IsNullOrWhiteSpace(target))
                throw new ArgumentException("Target cannot be empty.");

            if (target.StartsWith("Constraint"))
            {
                if (!int.TryParse(target.Split(' ')[1], out int index) || index < 1 || index > problem.Constraints.Count)
                    throw new ArgumentException($"Invalid constraint index in '{target}'. Expected 1 to {problem.Constraints.Count}.");
                index--; // 1-based to 0-based
                var range = GetConstraintRange(index);
                return $"{target}: {range.min:F3} ≤ B ≤ {range.max:F3}";
            }
            else
            {
                int col = Array.IndexOf(varNames, target);
                if (col < 0)
                    throw new ArgumentException($"Variable '{target}' not found in VarNames.");
                if (basis.Contains(col))
                {
                    var range = GetBasicVariableObjectiveRange(Array.IndexOf(basis, col));
                    return $"{target} (Basic): {range.min:F3} ≤ c ≤ {range.max:F3}";
                }
                else
                {
                    var range = GetNonBasicVariableRange(target);
                    return $"{target} (Non-Basic): {range.min:F3} ≤ c ≤ {range.max:F3}";
                }
            }
        }

        public string ApplyChange(string target, double value)
        {
            if (string.IsNullOrWhiteSpace(target))
                throw new ArgumentException("Target cannot be empty.");

            if (target.StartsWith("Constraint"))
            {
                if (!int.TryParse(target.Split(' ')[1], out int index) || index < 1 || index > problem.Constraints.Count)
                    throw new ArgumentException($"Invalid constraint index in '{target}'. Expected 1 to {problem.Constraints.Count}.");
                index--;
                problem.Constraints[index].B = value;
                return $"Constraint {index + 1} B-value updated to {value:F3}";
            }
            else
            {
                int col = Array.IndexOf(varNames, target);
                if (col < 0)
                    throw new ArgumentException($"Variable '{target}' not found in VarNames.");
                if (basis.Contains(col))
                {
                    problem.C[col] = value;
                    return $"Basic variable {target} objective coefficient updated to {value:F3}";
                }
                else
                {
                    problem.C[col] = value;
                    return $"Non-basic variable {target} objective coefficient updated to {value:F3}";
                }
            }
        }

        public string GetShadowPricesReport()
        {
            int m = problem.Constraints.Count;
            int nVars = problem.NumVars;

            if (tableau == null) throw new InvalidOperationException("Tableau is null.");
            if (tableau.GetLength(1) < nVars + m)
                throw new InvalidOperationException("Tableau does not contain expected slack columns.");

            var sb = new StringBuilder();
            sb.AppendLine("Shadow Prices:");
            for (int i = 0; i < m; i++)
            {
                double shadow = -tableau[0, nVars + i]; // Negative of reduced cost for slack variable
                sb.AppendLine($"  Constraint {i + 1}: {shadow.ToString("F3", CultureInfo.InvariantCulture)}");
            }

            return sb.ToString();
        }

        public SimplexResult SolveUsingDuality()
        {
            int m = problem.Constraints.Count;
            int n = problem.NumVars;

            // Dual objective coefficients = original RHS values (B)
            double[] dualC = new double[m];
            for (int i = 0; i < m; i++)
                dualC[i] = problem.Constraints[i].B;

            List<Constraint> dualConstraints = new List<Constraint>();
            for (int j = 0; j < n; j++)
            {
                double[] row = new double[m];
                for (int i = 0; i < m; i++)
                    row[i] = problem.Constraints[i].A[j];
                dualConstraints.Add(new Constraint
                {
                    A = row,
                    B = problem.C[j],
                    Relation = Rel.GE // Primal is maximize → dual is minimize
                });
            }

            LPProblem dualProblem = new LPProblem
            {
                C = dualC,
                Constraints = dualConstraints
            };

            var solver = new LPSolver();
            var traceSb = new StringBuilder();

            // Capture iterations
            Action<string, bool[,]> capture = (text, highlight) =>
            {
                if (!string.IsNullOrEmpty(text))
                    traceSb.AppendLine(text.TrimEnd());
            };

            // Solve with Dual Simplex
            SimplexResult dualResult;
            try
            {
                dualResult = solver.Solve(dualProblem, "Dual Simplex", capture);
                // Debug: Log raw solution and VarNames
                traceSb.AppendLine($"Debug: Raw Solution = [{string.Join(", ", (dualResult.Solution ?? new double[0]).Select(x => x.ToString("F3", CultureInfo.InvariantCulture)))}]");
                traceSb.AppendLine($"Debug: VarNames = [{string.Join(", ", dualResult.VarNames ?? new string[0])}]");
            }
            catch (Exception ex)
            {
                traceSb.AppendLine($"Error solving dual LP: {ex.Message}");
                return new SimplexResult
                {
                    Report = traceSb.ToString(),
                    Summary = "Error: " + ex.Message
                };
            }

            // Format report
            var report = new StringBuilder();
            report.AppendLine("=== Duality Algorithm Solution ===");
            report.AppendLine($"Dual Problem: Minimize {string.Join(" + ", dualProblem.C.Select((c, i) => $"{c.ToString("F3", CultureInfo.InvariantCulture)}y{i + 1}"))}");
            report.AppendLine("Subject to:");
            for (int i = 0; i < dualConstraints.Count; i++)
            {
                var c = dualConstraints[i];
                report.AppendLine($"{string.Join(" + ", c.A.Select((a, j) => $"{a.ToString("F3", CultureInfo.InvariantCulture)}y{j + 1}"))} {c.Relation} {c.B.ToString("F3", CultureInfo.InvariantCulture)}");
            }
            report.AppendLine("\n=== Duality Algorithm Iterations ===");
            report.AppendLine(traceSb.ToString().Trim());
            report.AppendLine("\n=== Final Result ===");
            report.AppendLine($"Status: {(dualResult.Summary?.Contains("OPTIMAL") ?? false ? "OPTIMAL" : "INFEASIBLE")}");
            report.AppendLine($"w* = {dualResult.OptimalValue.ToString("F3", CultureInfo.InvariantCulture)}");
            report.AppendLine($"y* = [{string.Join(", ", (dualResult.Solution ?? new double[0]).Take(n).Select(x => x.ToString("F3", CultureInfo.InvariantCulture)))}]");

            return new SimplexResult
            {
                Report = report.ToString(),
                Summary = dualResult.Summary,
                OptimalValue = dualResult.OptimalValue,
                Solution = dualResult.Solution?.Take(n).ToArray() ?? new double[n],
                Tableau = dualResult.Tableau,
                Basis = dualResult.Basis,
                VarNames = dualResult.VarNames
            };
        }

        // ===================== PRIVATE METHODS =====================
        private IEnumerable<string> GetNonBasicVariables()
        {
            for (int j = 0; j < problem.NumVars; j++)
                if (!basis.Contains(j))
                    yield return varNames[j];
        }

        private (double min, double max) GetNonBasicVariableRange(string varName)
        {
            int col = Array.IndexOf(varNames, varName);
            if (col < 0)
                throw new ArgumentException($"Variable '{varName}' not found in VarNames.");

            int m = tableau.GetLength(0) - 1;
            double current = problem.C[col]; // Current objective coefficient
            double reducedCost = tableau[0, col]; // Reduced cost in objective row

            double min = double.NegativeInfinity;
            double max = double.PositiveInfinity;

            if (reducedCost > 0)
                max = current + reducedCost; // Increase until reduced cost = 0
            else if (reducedCost < 0)
                min = current + reducedCost; // Decrease until reduced cost = 0

            return (min, max);
        }

        private (double min, double max) GetBasicVariableObjectiveRange(int basicVarRow)
        {
            int col = basis[basicVarRow];
            int m = tableau.GetLength(0) - 1;
            int n = tableau.GetLength(1) - 1;

            double current = problem.C[col];
            double min = double.NegativeInfinity;
            double max = double.PositiveInfinity;

            for (int j = 0; j < n; j++)
            {
                if (basis.Contains(j)) continue;
                double aij = tableau[basicVarRow + 1, j]; // +1 because row 0 is objective
                if (Math.Abs(aij) < 1e-9) continue;

                double reducedCost = tableau[0, j];
                double delta = -reducedCost / aij;
                if (aij > 0)
                    max = Math.Min(max, current + delta);
                else
                    min = Math.Max(min, current + delta);
            }

            return (min, max);
        }

        private (double min, double max) GetConstraintRange(int index)
        {
            int row = index + 1; // Objective row is 0
            int n = tableau.GetLength(1) - 1;
            double currentB = tableau[row, n]; // Current RHS

            double min = double.NegativeInfinity;
            double max = double.PositiveInfinity;

            for (int j = 0; j < problem.NumVars; j++)
            {
                if (basis.Contains(j)) continue;
                double aij = tableau[row, j];
                if (Math.Abs(aij) < 1e-9) continue;

                double delta = -tableau[row, n] / aij;
                if (aij > 0)
                    max = Math.Min(max, currentB + delta);
                else
                    min = Math.Max(min, currentB + delta);
            }

            return (min, max);
        }
    }
}