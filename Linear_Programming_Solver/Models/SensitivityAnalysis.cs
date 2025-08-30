using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Globalization;

namespace Linear_Programming_Solver.Models
{
    /// <summary>
    /// Performs sensitivity analysis on an optimal LP solution.
    /// Supports range analysis, changes to variables and constraints, adding new activities/constraints,
    /// shadow prices, and duality analysis.
    /// </summary>
    public class SensitivityAnalysis
    {
        private readonly LPSolver _solver = new LPSolver();
        private readonly LPProblem _problem;
        private const double EPS = 1e-6;

        public SensitivityAnalysis(LPProblem problem)
        {
            _problem = problem ?? throw new ArgumentNullException(nameof(problem));
        }

        public string GenerateReport()
        {
            var result = Analyze(_problem, null);
            return result.Report;
        }

        public SimplexResult Analyze(LPProblem problem, Action<string, bool[,]> updatePivot = null)
        {
            var sb = new StringBuilder();
            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);

            Log("Starting Sensitivity Analysis...");
            Log($"Problem: {problem.NumVars} variables, {problem.Constraints.Count} constraints");

            // Solve the primal problem to get the optimal tableau
            string algo = problem.Constraints.Any(c => c.Relation == Rel.GE || c.Relation == Rel.EQ)
                ? "Dual Simplex" : "Revised Primal Simplex";
            Log($"Solving primal problem with {algo}...");

            SimplexResult primalResult;
            try
            {
                primalResult = _solver.Solve(problem, algo, updatePivot);
                Log($"Primal solver completed. Summary length: {primalResult.Summary?.Length ?? 0}");
            }
            catch (Exception ex)
            {
                Log($"Primal problem infeasible or error: {ex.Message}");
                return new SimplexResult { Report = "Primal problem infeasible", Summary = "" };
            }

            double[] x = ParseSolutionVector(primalResult.Summary, problem.NumVars, updatePivot);
            double z = ParseObjectiveValue(primalResult.Summary);
            if (x.Length != problem.NumVars || double.IsNaN(z) || double.IsInfinity(z))
            {
                Log($"Failed to parse valid primal solution. x.Length={x.Length}, z={z}");
                return new SimplexResult { Report = "Invalid primal solution", Summary = "" };
            }

            Log($"Primal solution parsed: z* = {z}, x* = [{string.Join(", ", x)}]");

            double[,] finalTableau = ParseTableau(primalResult.Summary, problem, updatePivot);
            if (finalTableau == null)
            {
                Log("Failed to parse final tableau.");
                return new SimplexResult { Report = "Invalid tableau", Summary = "" };
            }

            sb.AppendLine($"Primal Solution: z* = {z:0.###}, x* = [{string.Join(", ", x.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");

            // Identify basic and non-basic variables
            var (basicVars, nonBasicVars) = GetBasicAndNonBasicVars(finalTableau, problem.NumVars, problem.Constraints.Count);
            Log($"Basic variables: [{string.Join(", ", basicVars.Select(i => $"x{i + 1}"))}]");
            Log($"Non-basic variables: [{string.Join(", ", nonBasicVars.Select(i => $"x{i + 1}"))}]");

            // 1. Display ranges for non-basic variables
            sb.AppendLine("\nNon-Basic Variable Objective Coefficient Ranges:");
            foreach (int j in nonBasicVars)
            {
                var (minCj, maxCj) = GetNonBasicVariableRange(finalTableau, j, basicVars, problem);
                sb.AppendLine($"x{j + 1}: [{minCj:0.###}, {maxCj:0.###}]");
            }

            // 2. Apply change to a non-basic variable (example: first non-basic, +1.0)
            if (nonBasicVars.Any())
            {
                int j = nonBasicVars.First();
                double newCj = problem.C[j] + 1.0;
                var newProblem = problem.Clone();
                newProblem.C[j] = newCj;
                sb.AppendLine($"\nApplying change to non-basic x{j + 1}: c{j + 1} = {newCj:0.###}");
                SimplexResult newResult = _solver.Solve(newProblem, algo, updatePivot);
                double[] newX = ParseSolutionVector(newResult.Summary, problem.NumVars, updatePivot);
                double newZ = ParseObjectiveValue(newResult.Summary);
                sb.AppendLine($"New solution: z* = {newZ:0.###}, x* = [{string.Join(", ", newX.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");
            }

            // 3. Display ranges for basic variables
            sb.AppendLine("\nBasic Variable Objective Coefficient Ranges:");
            foreach (int i in basicVars)
            {
                var (minCi, maxCi) = GetBasicVariableRange(finalTableau, i, basicVars, problem);
                sb.AppendLine($"x{i + 1}: [{minCi:0.###}, {maxCi:0.###}]");
            }

            // 4. Apply change to a basic variable (example: first basic, +1.0)
            if (basicVars.Any())
            {
                int i = basicVars.First();
                double newCi = problem.C[i] + 1.0;
                var newProblem = problem.Clone();
                newProblem.C[i] = newCi;
                sb.AppendLine($"\nApplying change to basic x{i + 1}: c{i + 1} = {newCi:0.###}");
                SimplexResult newResult = _solver.Solve(newProblem, algo, updatePivot);
                double[] newX = ParseSolutionVector(newResult.Summary, problem.NumVars, updatePivot);
                double newZ = ParseObjectiveValue(newResult.Summary);
                sb.AppendLine($"New solution: z* = {newZ:0.###}, x* = [{string.Join(", ", newX.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");
            }

            // 5. Display ranges for constraint RHS
            sb.AppendLine("\nConstraint RHS Ranges:");
            for (int i = 0; i < problem.Constraints.Count; i++)
            {
                var (minBi, maxBi) = GetRHSRange(finalTableau, i, basicVars, problem);
                sb.AppendLine($"Constraint {i + 1}: [{minBi:0.###}, {maxBi:0.###}]");
            }

            // 6. Apply change to a constraint RHS (example: first constraint, +1.0)
            if (problem.Constraints.Any())
            {
                int i = 0;
                double newBi = problem.Constraints[i].B + 1.0;
                var newProblem = problem.Clone();
                newProblem.Constraints[i].B = newBi;
                sb.AppendLine($"\nApplying change to constraint {i + 1} RHS: b{i + 1} = {newBi:0.###}");
                SimplexResult newResult = _solver.Solve(newProblem, algo, updatePivot);
                double[] newX = ParseSolutionVector(newResult.Summary, problem.NumVars, updatePivot);
                double newZ = ParseObjectiveValue(newResult.Summary);
                sb.AppendLine($"New solution: z* = {newZ:0.###}, x* = [{string.Join(", ", newX.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");
            }

            // 7. Display ranges for non-basic variable coefficients in constraints
            sb.AppendLine("\nNon-Basic Variable Constraint Coefficient Ranges:");
            foreach (int j in nonBasicVars)
            {
                for (int i = 0; i < problem.Constraints.Count; i++)
                {
                    var (minAij, maxAij) = GetNonBasicCoefficientRange(finalTableau, j, i, basicVars, problem);
                    sb.AppendLine($"x{j + 1} in constraint {i + 1}: [{minAij:0.###}, {maxAij:0.###}]");
                }
            }

            // 8. Apply change to a non-basic variable coefficient (example: first non-basic, first constraint)
            if (nonBasicVars.Any() && problem.Constraints.Any())
            {
                int j = nonBasicVars.First();
                int i = 0;
                double newAij = problem.Constraints[i].A[j] + 1.0;
                var newProblem = problem.Clone();
                newProblem.Constraints[i].A[j] = newAij;
                sb.AppendLine($"\nApplying change to x{j + 1} in constraint {i + 1}: a{i + 1},{j + 1} = {newAij:0.###}");
                SimplexResult newResult = _solver.Solve(newProblem, algo, updatePivot);
                double[] newX = ParseSolutionVector(newResult.Summary, problem.NumVars, updatePivot);
                double newZ = ParseObjectiveValue(newResult.Summary);
                sb.AppendLine($"New solution: z* = {newZ:0.###}, x* = [{string.Join(", ", newX.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");
            }

            // 9. Add a new activity (variable)
            int newVarIdx = problem.NumVars;
            double[] newC = new double[newVarIdx + 1];
            Array.Copy(problem.C, newC, newVarIdx);
            newC[newVarIdx] = 1.0; // Example coefficient
            var newConstraints = new List<Constraint>();
            foreach (var constraint in problem.Constraints)
            {
                var newA = new double[newVarIdx + 1];
                Array.Copy(constraint.A, newA, constraint.A.Length);
                newA[newVarIdx] = 1.0; // Example coefficient
                newConstraints.Add(new Constraint { A = newA, Relation = constraint.Relation, B = constraint.B });
            }
            var newActivityProblem = new LPProblem
            {
                C = newC,
                Constraints = newConstraints
            };
            sb.AppendLine($"\nAdding new activity x{newVarIdx + 1} with c{newVarIdx + 1} = 1, a_i,{newVarIdx + 1} = 1");
            SimplexResult newActivityResult = _solver.Solve(newActivityProblem, algo, updatePivot);
            double[] newActivityX = ParseSolutionVector(newActivityResult.Summary, newActivityProblem.NumVars, updatePivot);
            double newActivityZ = ParseObjectiveValue(newActivityResult.Summary);
            sb.AppendLine($"New solution: z* = {newActivityZ:0.###}, x* = [{string.Join(", ", newActivityX.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");

            // 10. Add a new constraint
            var newConstraintProblem = problem.Clone();
            var newConstraint = new Constraint
            {
                A = new double[problem.NumVars],
                Relation = Rel.LE,
                B = 10.0
            };
            for (int i = 0; i < problem.NumVars; i++) newConstraint.A[i] = 1.0;
            newConstraintProblem.Constraints.Add(newConstraint);
            sb.AppendLine($"\nAdding new constraint: {string.Join(" + ", newConstraint.A.Select((a, i) => $"{a:0.###}x{i + 1}"))} <= {newConstraint.B:0.###}");
            SimplexResult newConstraintResult = _solver.Solve(newConstraintProblem, "Dual Simplex", updatePivot);
            double[] newConstraintX = ParseSolutionVector(newConstraintResult.Summary, problem.NumVars, updatePivot);
            double newConstraintZ = ParseObjectiveValue(newConstraintResult.Summary);
            sb.AppendLine($"New solution: z* = {newConstraintZ:0.###}, x* = [{string.Join(", ", newConstraintX.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");

            // 11. Display shadow prices
            double[] shadowPrices = GetShadowPrices(finalTableau, problem);
            sb.AppendLine("\nShadow Prices:");
            for (int i = 0; i < shadowPrices.Length; i++)
            {
                sb.AppendLine($"Constraint {i + 1}: {shadowPrices[i]:0.###}");
            }

            // 12. Duality analysis
            var dualProblem = CreateDualProblem(problem);
            sb.AppendLine("\nDual Problem:");
            sb.AppendLine($"Minimize: {string.Join(" + ", dualProblem.C.Select((c, i) => $"{c:0.###}y{i + 1}"))}");
            sb.AppendLine("Subject to:");
            for (int i = 0; i < dualProblem.Constraints.Count; i++)
            {
                var c = dualProblem.Constraints[i];
                sb.AppendLine($"{string.Join(" + ", c.A.Select((a, j) => $"{a:0.###}y{j + 1}"))} {c.Relation} {c.B:0.###}");
            }

            SimplexResult dualResult;
            try
            {
                dualResult = _solver.Solve(dualProblem, "Dual Simplex", updatePivot);
            }
            catch (Exception ex)
            {
                Log($"Dual problem infeasible or error: {ex.Message}");
                sb.AppendLine("Dual problem infeasible.");
                return new SimplexResult { Report = sb.ToString(), Summary = sb.ToString() };
            }

            double[] y = ParseSolutionVector(dualResult.Summary, dualProblem.NumVars, updatePivot);
            double w = ParseObjectiveValue(dualResult.Summary);
            sb.AppendLine($"Dual Solution: w* = {w:0.###}, y* = [{string.Join(", ", y.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");

            // Verify strong or weak duality
            if (Math.Abs(z - w) < EPS)
            {
                sb.AppendLine("Strong Duality holds: Primal z* = Dual w*");
            }
            else
            {
                sb.AppendLine($"Weak Duality holds: Primal z* = {z:0.###}, Dual w* = {w:0.###}");
            }

            return new SimplexResult { Report = sb.ToString(), Summary = sb.ToString() };
        }

        private (List<int> basic, List<int> nonBasic) GetBasicAndNonBasicVars(double[,] tableau, int numVars, int numConstraints)
        {
            var basic = new List<int>();
            var nonBasic = Enumerable.Range(0, numVars).ToList();

            for (int i = 1; i <= numConstraints; i++)
            {
                for (int j = 0; j < numVars; j++)
                {
                    if (Math.Abs(tableau[i, j] - 1.0) < EPS && Enumerable.Range(0, numVars).Except(new[] { j }).All(k => Math.Abs(tableau[i, k]) < EPS))
                    {
                        basic.Add(j);
                        nonBasic.Remove(j);
                        break;
                    }
                }
            }

            return (basic, nonBasic);
        }

        private (double minCj, double maxCj) GetNonBasicVariableRange(double[,] tableau, int j, List<int> basicVars, LPProblem problem)
        {
            double cj = problem.C[j];
            double zj = tableau[0, j];
            double minDelta = double.NegativeInfinity;
            double maxDelta = double.PositiveInfinity;

            if (zj > -EPS)
            {
                maxDelta = zj;
            }
            else
            {
                minDelta = zj;
            }

            return (cj + minDelta, cj + maxDelta);
        }

        private (double minCi, double maxCi) GetBasicVariableRange(double[,] tableau, int i, List<int> basicVars, LPProblem problem)
        {
            double ci = problem.C[i];
            double minDelta = double.NegativeInfinity;
            double maxDelta = double.PositiveInfinity;

            int rowIdx = -1;
            for (int r = 1; r < tableau.GetLength(0); r++)
            {
                if (Math.Abs(tableau[r, i] - 1.0) < EPS && Enumerable.Range(0, problem.NumVars).Except(new[] { i }).All(k => Math.Abs(tableau[r, k]) < EPS))
                {
                    rowIdx = r;
                    break;
                }
            }

            if (rowIdx == -1) return (ci, ci);

            for (int j = 0; j < problem.NumVars; j++)
            {
                if (basicVars.Contains(j)) continue;
                double aij = tableau[rowIdx, j];
                if (Math.Abs(aij) < EPS) continue;

                double zj = tableau[0, j];
                double delta = -zj / aij;
                if (aij > EPS)
                {
                    if (delta < maxDelta) maxDelta = delta;
                }
                else
                {
                    if (delta > minDelta) minDelta = delta;
                }
            }

            return (ci + minDelta, ci + maxDelta);
        }

        private (double minBi, double maxBi) GetRHSRange(double[,] tableau, int constraintIdx, List<int> basicVars, LPProblem problem)
        {
            int rowIdx = constraintIdx + 1;
            double bi = tableau[rowIdx, tableau.GetLength(1) - 1];
            double minDelta = double.NegativeInfinity;
            double maxDelta = double.PositiveInfinity;

            for (int j = 0; j < problem.NumVars; j++)
            {
                if (basicVars.Contains(j)) continue;
                double aij = tableau[rowIdx, j];
                if (Math.Abs(aij) < EPS) continue;

                double xBj = tableau[rowIdx, tableau.GetLength(1) - 1];
                double delta = -xBj / aij;
                if (aij > EPS)
                {
                    if (delta < maxDelta) maxDelta = delta;
                }
                else
                {
                    if (delta > minDelta) minDelta = delta;
                }
            }

            return (bi + minDelta, bi + maxDelta);
        }

        private (double minAij, double maxAij) GetNonBasicCoefficientRange(double[,] tableau, int j, int constraintIdx, List<int> basicVars, LPProblem problem)
        {
            int rowIdx = constraintIdx + 1;
            double aij = tableau[rowIdx, j];
            double bi = tableau[rowIdx, tableau.GetLength(1) - 1];
            double minDelta = double.NegativeInfinity;
            double maxDelta = double.PositiveInfinity;

            for (int k = 0; k < problem.NumVars; k++)
            {
                if (basicVars.Contains(k)) continue;
                double aik = tableau[rowIdx, k];
                if (Math.Abs(aik) < EPS) continue;

                double delta = -bi / aik;
                if (aik > EPS)
                {
                    if (delta < maxDelta) maxDelta = delta;
                }
                else
                {
                    if (delta > minDelta) minDelta = delta;
                }
            }

            return (aij + minDelta, aij + maxDelta);
        }

        private double[] GetShadowPrices(double[,] tableau, LPProblem problem)
        {
            double[] shadowPrices = new double[problem.Constraints.Count];
            for (int i = 0; i < problem.Constraints.Count; i++)
            {
                int rowIdx = i + 1;
                for (int j = problem.NumVars; j < tableau.GetLength(1) - 1; j++)
                {
                    if (Math.Abs(tableau[rowIdx, j] - 1.0) < EPS)
                    {
                        shadowPrices[i] = -tableau[0, j];
                        break;
                    }
                }
            }
            return shadowPrices;
        }

        private LPProblem CreateDualProblem(LPProblem primal)
        {
            var dual = new LPProblem
            {
                C = primal.Constraints.Select(c => c.B).ToArray(),
                Constraints = new List<Constraint>()
            };

            for (int j = 0; j < primal.NumVars; j++)
            {
                var constraint = new Constraint
                {
                    A = primal.Constraints.Select(c => c.A[j]).ToArray(),
                    Relation = Rel.GE,
                    B = primal.C[j]
                };
                dual.Constraints.Add(constraint);
            }

            return dual;
        }

        private double[] ParseSolutionVector(string summary, int expectedNumVars, Action<string, bool[,]> updatePivot = null)
        {
            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);
            try
            {
                if (string.IsNullOrWhiteSpace(summary))
                {
                    Log("ParseSolutionVector: Summary is empty.");
                    return Array.Empty<double>();
                }

                int start = summary.IndexOf("x* = [", StringComparison.Ordinal);
                if (start < 0)
                {
                    Log("ParseSolutionVector: No 'x* = [' found in summary.");
                    return Array.Empty<double>();
                }
                start += 6;
                int end = summary.IndexOf("]", start, StringComparison.Ordinal);
                if (end < 0)
                {
                    Log("ParseSolutionVector: No closing ']' found in summary.");
                    return Array.Empty<double>();
                }
                string vec = summary.Substring(start, end - start).Trim();
                if (string.IsNullOrWhiteSpace(vec))
                {
                    Log("ParseSolutionVector: Empty vector string.");
                    return Array.Empty<double>();
                }
                Log($"ParseSolutionVector: Raw vector string = '{vec}'");

                var parts = vec.Split(new[] { ", " }, StringSplitOptions.RemoveEmptyEntries);
                var values = new double[expectedNumVars];
                for (int i = 0; i < Math.Min(parts.Length, expectedNumVars); i++)
                {
                    string numStr = parts[i].Trim().Replace(',', '.');
                    if (!double.TryParse(numStr, NumberStyles.Any, CultureInfo.InvariantCulture, out double val))
                    {
                        Log($"ParseSolutionVector: Failed to parse number '{numStr}' at index {i}.");
                        return Array.Empty<double>();
                    }
                    values[i] = val;
                }
                if (parts.Length != expectedNumVars)
                {
                    Log($"ParseSolutionVector: Expected {expectedNumVars} numbers, but found {parts.Length}.");
                    return Array.Empty<double>();
                }
                Log($"ParseSolutionVector: Parsed {values.Length} values: [{string.Join(", ", values.Select(v => v.ToString("0.######", CultureInfo.InvariantCulture)))}]");
                return values;
            }
            catch (Exception ex)
            {
                Log($"ParseSolutionVector error: {ex.Message}");
                return Array.Empty<double>();
            }
        }

        private double ParseObjectiveValue(string summary)
        {
            try
            {
                if (string.IsNullOrWhiteSpace(summary))
                    return double.NegativeInfinity;

                foreach (var raw in summary.Split('\n'))
                {
                    var line = raw.Trim();
                    if (line.StartsWith("z*") || line.StartsWith("w*"))
                    {
                        var parts = line.Split('=');
                        if (parts.Length == 2)
                        {
                            var s = parts[1].Trim().Replace(',', '.');
                            if (double.TryParse(s, NumberStyles.Any, CultureInfo.InvariantCulture, out double val))
                                return val;
                        }
                    }
                }
            }
            catch { }
            return double.NegativeInfinity;
        }

        private double[,] ParseTableau(string summary, LPProblem problem, Action<string, bool[,]> updatePivot = null)
        {
            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);

            if (string.IsNullOrWhiteSpace(summary))
            {
                Log("ParseTableau: Summary is empty.");
                return null;
            }

            var lines = summary.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            var tableauRows = new List<List<double>>();

            int expectedCols = problem.NumVars + problem.Constraints.Count + 1; // Variables + slacks + RHS
            int expectedRows = problem.Constraints.Count + 1; // Objective + constraints

            foreach (var line in lines)
            {
                var parts = Regex.Split(line.Trim(), @"\s+");
                var row = new List<double>();

                foreach (var p in parts)
                {
                    if (double.TryParse(p.Replace(',', '.'), NumberStyles.Any, CultureInfo.InvariantCulture, out double val))
                    {
                        row.Add(val);
                    }
                }

                if (row.Count == expectedCols)
                {
                    tableauRows.Add(row);
                    if (tableauRows.Count == expectedRows) break;
                }
            }

            if (tableauRows.Count != expectedRows)
            {
                Log($"ParseTableau: Expected {expectedRows} rows, found {tableauRows.Count}");
                return null;
            }

            var tableau = new double[expectedRows, expectedCols];
            for (int i = 0; i < expectedRows; i++)
                for (int j = 0; j < expectedCols; j++)
                    tableau[i, j] = tableauRows[i][j];

            Log("ParseTableau: Successfully parsed numeric tableau.");
            return tableau;
        }

    }
}