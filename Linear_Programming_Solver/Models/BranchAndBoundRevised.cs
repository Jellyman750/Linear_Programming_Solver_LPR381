using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Globalization;
using System.Text.RegularExpressions;

namespace Linear_Programming_Solver.Models
{
    /// <summary>
    /// Branch-and-Bound using Revised Primal Simplex for <= constraints and Dual Simplex for >= or = constraints.
    /// - Supports >= and <= constraints in subproblems.
    /// - Branches on variable with fractional part closest to 0.5, lowest subscript for ties.
    /// - Uses hierarchical subproblem naming (e.g., Subproblem 1.1).
    /// - Includes feasibility checks and tableau parsing.
    /// </summary>
    public class BranchAndBoundRevised : ILPAlgorithm
    {
        private readonly LPSolver _lpSolver = new LPSolver();
        private int _subCounter = 1;
        private const double EPS = 1e-6;
        private const int MaxDepth = 200;

        public double BestObjective { get; private set; } = double.NegativeInfinity;
        public double[] BestSolution { get; private set; }

        public SimplexResult Solve(LPProblem problem, Action<string, bool[,]> updatePivot = null)
        {
            _subCounter = 1;
            BestObjective = double.NegativeInfinity;
            BestSolution = null;

            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);

            Log("Revised Branch & Bound: starting at Root Problem.");

            // Choose algorithm for root problem
            string rootAlgo = ChooseAlgorithm(problem);
            Log($"Root Problem: Using {rootAlgo} for LP relaxation.");

            // Solve root LP relaxation
            SimplexResult rootRes;
            try
            {
                rootRes = _lpSolver.Solve(problem, rootAlgo, updatePivot);
            }
            catch (Exception ex)
            {
                Log($"Root Problem: LP relaxation infeasible or error: {ex.Message}");
                return new SimplexResult { Report = "LP relaxation infeasible", Summary = "" };
            }

            // Parse solution
            var xRoot = ParseSolutionVector(rootRes.Summary, problem.NumVars, updatePivot);
            if (xRoot.Length == 0 || xRoot.All(v => Math.Abs(v) < EPS))
            {
                xRoot = ParseSolutionVectorFromTableau(rootRes.Summary, problem.NumVars, updatePivot);
            }

            if (xRoot.Length != problem.NumVars)
            {
                Log($"Root Problem: Failed to parse valid solution vector. Expected {problem.NumVars} variables.");
                return new SimplexResult { Report = "Failed to parse root solution", Summary = "" };
            }

            var zRoot = ParseObjectiveValue(rootRes.Summary);
            Log($"Root Problem relaxation: z* = {zRoot:0.###}, x* = [{string.Join(", ", xRoot.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");

            // Check if root solution is integral and feasible
            if (IsIntegral(xRoot) && IsFeasible(xRoot, problem))
            {
                BestObjective = zRoot;
                BestSolution = xRoot.Select(RoundInt).ToArray();
                Log("Root Problem is integer feasible. Branch & Bound not required.");
            }
            else
            {
                Log("Root solution is fractional → starting Branch & Bound.");
                SolveSubproblem(problem, "Root Problem", "", updatePivot, 0);
            }

            var sb = new StringBuilder();
            sb.AppendLine("Revised Branch & Bound Finished.");
            if (BestSolution == null)
            {
                sb.AppendLine("No integer-feasible solution found.");
            }
            else
            {
                sb.AppendLine($"Best integer z* = {BestObjective:0.###}");
                sb.AppendLine("Best integer x* = [" + string.Join(", ", BestSolution.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture))) + "]");
            }

            return new SimplexResult
            {
                Report = sb.ToString(),
                Summary = sb.ToString()
            };
        }

        private void SolveSubproblem(LPProblem problem, string name, string parentId, Action<string, bool[,]> updatePivot, int depth)
        {
            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);

            if (depth > MaxDepth)
            {
                Log($"{name}: Maximum recursion depth reached → prune.");
                return;
            }

            // Log constraints
            Log($"{name}: Constraints: {string.Join("; ", problem.Constraints.Select(c => $"{string.Join(" + ", c.A.Select((a, i) => $"{a}x{i + 1}"))} {c.Relation} {c.B}"))}");

            // Choose algorithm
            string algo = ChooseAlgorithm(problem);
            Log($"\n{name}: Solving LP relaxation with {algo}...");

            SimplexResult res;
            try
            {
                res = _lpSolver.Solve(problem, algo, updatePivot);
            }
            catch (Exception ex)
            {
                Log($"{name}: LP relaxation infeasible or error: {ex.Message}");
                return;
            }

            // Parse solution
            var x = ParseSolutionVector(res.Summary, problem.NumVars, updatePivot);
            if (x.Length == 0 || x.All(v => Math.Abs(v) < EPS))
            {
                x = ParseSolutionVectorFromTableau(res.Summary, problem.NumVars, updatePivot);
            }

            if (x.Length != problem.NumVars)
            {
                Log($"{name}: Failed to parse valid solution vector. Expected {problem.NumVars} variables.");
                return;
            }

            var z = ParseObjectiveValue(res.Summary);
            if (double.IsNaN(z) || double.IsInfinity(z))
            {
                Log($"{name}: Invalid objective value → prune.");
                return;
            }

            Log($"{name} relaxation: z* = {z:0.###}, x* = [{string.Join(", ", x.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");

            // Check feasibility
            if (!IsFeasible(x, problem))
            {
                Log($"{name}: Solution x* = [{string.Join(", ", x.Select(v => v.ToString("0.###")))}] is infeasible for constraints.");
                return;
            }

            // Bounding (maximize)
            if (z <= BestObjective + EPS)
            {
                Log($"{name}: pruned by bound (z* ≤ current best {BestObjective:0.###}).");
                return;
            }

            // Check integrality
            if (IsIntegral(x))
            {
                BestObjective = z;
                BestSolution = x.Select(RoundInt).ToArray();
                Log($"{name}: integer feasible, updated incumbent: z* = {z:0.###}.");
                return;
            }

            // Branch on variable with fractional part closest to 0.5, lowest subscript for ties
            int fracIdx = -1;
            double minDist = double.MaxValue;
            for (int i = 0; i < x.Length; i++)
            {
                double fracPart = x[i] - Math.Floor(x[i]);
                if (fracPart > EPS && (1 - fracPart) > EPS)
                {
                    double dist = Math.Abs(fracPart - 0.5);
                    Log($"Checking x{i + 1} = {x[i]:0.######}, fracPart = {fracPart:0.######}, distance to 0.5 = {dist:0.######}");
                    if (dist < minDist || (dist == minDist && i < fracIdx))
                    {
                        minDist = dist;
                        fracIdx = i;
                    }
                }
            }

            if (fracIdx == -1)
            {
                Log($"{name}: No fractional variable found but solution not integral → prune.");
                return;
            }

            double xi = x[fracIdx];
            double floor = Math.Floor(xi);
            double ceil = Math.Ceiling(xi);

            // Generate hierarchical subproblem IDs
            string ceilId = string.IsNullOrEmpty(parentId) ? $"{_subCounter}" : $"{parentId}.{_subCounter - _subCounter + 1}";
            string floorId = string.IsNullOrEmpty(parentId) ? $"{_subCounter + 1}" : $"{parentId}.{_subCounter - _subCounter + 2}";
            _subCounter += 2;

            // Left branch: x_i ≤ floor(x_i)
            var left = problem.Clone();
            left.Constraints.Add(new Constraint
            {
                A = UnitVector(problem.NumVars, fracIdx),
                Relation = Rel.LE,
                B = floor
            });

            // Right branch: x_i ≥ ceil(x_i)
            var right = problem.Clone();
            right.Constraints.Add(new Constraint
            {
                A = UnitVector(problem.NumVars, fracIdx),
                Relation = Rel.GE,
                B = ceil
            });

            string leftName = $"Subproblem {floorId}: x{fracIdx + 1} ≤ {floor}";
            string rightName = $"Subproblem {ceilId}: x{fracIdx + 1} ≥ {ceil}";

            Log($"{name}: branching on x{fracIdx + 1} = {xi:0.###} (floor={floor}, ceil={ceil})");
            Log($" → {rightName} (ceil first)");
            Log($" → {leftName}");

            // Depth-first recursion, ceil first
            SolveSubproblem(right, rightName, ceilId, updatePivot, depth + 1);
            SolveSubproblem(left, leftName, floorId, updatePivot, depth + 1);
        }

        private static string ChooseAlgorithm(LPProblem p)
        {
            // Use Dual Simplex for >= or = constraints, Revised Primal Simplex for <=
            bool hasGEorEQ = p.Constraints.Any(c => c.Relation == Rel.GE || c.Relation == Rel.EQ);
            return hasGEorEQ ? "Dual Simplex" : "Revised Primal Simplex";
        }

        private static bool IsIntegral(double[] x)
        {
            return x.All(v => Math.Abs(v - Math.Round(v)) < EPS);
        }

        private static bool IsFeasible(double[] x, LPProblem problem)
        {
            foreach (var constraint in problem.Constraints)
            {
                double sum = 0;
                for (int i = 0; i < x.Length; i++)
                    sum += constraint.A[i] * x[i];
                if (constraint.Relation == Rel.LE && sum > constraint.B + EPS)
                    return false;
                if (constraint.Relation == Rel.GE && sum < constraint.B - EPS)
                    return false;
                if (constraint.Relation == Rel.EQ && Math.Abs(sum - constraint.B) > EPS)
                    return false;
            }
            // Check non-negativity
            return x.All(v => v >= -EPS);
        }

        private static double RoundInt(double v) => Math.Round(v);

        private static double[] UnitVector(int n, int index)
        {
            var a = new double[n];
            a[index] = 1.0;
            return a;
        }

        private static double[] ParseSolutionVector(string summary, int expectedNumVars, Action<string, bool[,]> updatePivot = null)
        {
            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);
            try
            {
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

                // Split on comma followed by space to handle locale-specific decimals
                var parts = vec.Split(new[] { ", " }, StringSplitOptions.RemoveEmptyEntries);
                var values = new double[expectedNumVars];
                for (int i = 0; i < Math.Min(parts.Length, expectedNumVars); i++)
                {
                    string numStr = parts[i].Trim().Replace(',', '.');
                    values[i] = double.Parse(numStr, NumberStyles.Any, CultureInfo.InvariantCulture);
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

        private static double[] ParseSolutionVectorFromTableau(string summary, int numVars, Action<string, bool[,]> updatePivot = null)
        {
            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);
            var values = new double[numVars];
            for (int i = 0; i < numVars; i++) values[i] = 0;

            var lines = summary.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            bool foundSolution = false;

            foreach (var line in lines)
            {
                var trimmed = line.Trim();
                // Match basic variables (x1, x2, ..., or v1, v2, ...)
                if (trimmed.StartsWith("x") || trimmed.StartsWith("v"))
                {
                    var parts = Regex.Split(trimmed, @"\s+");
                    if (parts.Length < 2) continue;

                    string varName = parts[0];
                    if (varName.Length < 2 || !(varName[0] == 'x' || varName[0] == 'v')) continue;

                    if (!int.TryParse(varName.Substring(1), out int varIndex)) continue;
                    if (varIndex < 1 || varIndex > numVars) continue;

                    string rhsStr = parts[parts.Length - 1].Replace(',', '.');
                    if (double.TryParse(rhsStr, NumberStyles.Any, CultureInfo.InvariantCulture, out double rhsVal))
                    {
                        values[varIndex - 1] = rhsVal;
                        foundSolution = true;
                    }
                }
            }

            if (!foundSolution)
            {
                Log("ParseSolutionVectorFromTableau: No valid solution found in tableau.");
                return new double[numVars]; // Return zeros to trigger feasibility check
            }

            Log($"ParseSolutionVectorFromTableau: Parsed values: [{string.Join(", ", values.Select(v => v.ToString("0.######", CultureInfo.InvariantCulture)))}]");
            return values;
        }

        private static double ParseObjectiveValue(string summary)
        {
            try
            {
                foreach (var raw in summary.Split('\n'))
                {
                    var line = raw.Trim();
                    if (line.StartsWith("z*"))
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
    }
}