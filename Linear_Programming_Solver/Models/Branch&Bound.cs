using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Globalization;
using System.Text.RegularExpressions;

namespace Linear_Programming_Solver.Models
{
    /// <summary>
    /// Branch-and-Bound for pure integer programs (all decision variables integral).
    /// - Root and each subproblem LP relaxation is solved with:
    ///   * "Primal Simplex" (PrimalSimplex.cs) if all constraints are <=
    ///   * "Dual Simplex" (DualSimplex.cs) if any constraint is >= or =
    /// - Branching variable: fractional part closest to 0.5, lowest subscript for ties.
    /// - Ceil branch explored first (good for maximization).
    /// - Stops immediately if root LP solution is integral.
    /// - Hierarchical subproblem naming (e.g., Subproblem 1.1, 1.2).
    /// - Detailed logging via updatePivot callback.
    /// </summary>
    public class BranchAndBound : ILPAlgorithm
    {
        private readonly LPSolver _solver = new LPSolver();
        private int _subProblemCounter = 1;
        private const double EPS = 1e-6;
        private const int MaxDepth = 200;

        public double BestObjective { get; private set; } = double.NegativeInfinity;
        public double[] BestSolution { get; private set; }

        public SimplexResult Solve(LPProblem problem, Action<string, bool[,]> updatePivot = null)
        {
            _subProblemCounter = 1;
            BestObjective = double.NegativeInfinity;
            BestSolution = null;

            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);

            // 1. Choose algorithm for root problem
            string rootAlgo = ChooseAlgorithm(problem);
            Log($"Branch & Bound: Using {rootAlgo} for the ROOT LP relaxation.");

            // 2. Solve root LP relaxation
            SimplexResult rootRes;
            try
            {
                rootRes = _solver.Solve(problem, rootAlgo, updatePivot);
            }
            catch (Exception ex)
            {
                Log($"Root Problem: LP relaxation infeasible or error: {ex.Message}");
                return new SimplexResult { Report = "LP relaxation infeasible", Summary = "" };
            }

            // Try to parse solution vector
            var xRoot = ParseSolutionVector(rootRes.Summary, problem.NumVars, updatePivot);

            // If parsing fails, try from tableau
            if (xRoot.Length == 0 || xRoot.All(v => Math.Abs(v) < EPS))
            {
                xRoot = ParseSolutionVectorFromTableau(rootRes.Summary, problem.NumVars, updatePivot);
            }

            // If still invalid, report error
            if (xRoot.Length != problem.NumVars)
            {
                Log($"Root Problem: Failed to parse valid solution vector. Expected {problem.NumVars} variables.");
                return new SimplexResult { Report = "Failed to parse root solution", Summary = "" };
            }

            var zRoot = ParseObjectiveValue(rootRes.Summary);

            Log($"Root Problem LP solution: z* = {zRoot:0.###}, x* = [{string.Join(", ", xRoot.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");
            Log("Root Problem optimal tableau displayed above.");

            // 3. Check if root solution is integral
            if (IsIntegral(xRoot) && IsFeasible(xRoot, problem))
            {
                BestObjective = zRoot;
                BestSolution = xRoot.Select(RoundInt).ToArray();
                Log("Root Problem is already integral and feasible. Branch & Bound not required.");
                return BuildReport();
            }

            // 4. Otherwise start branch and bound
            Log("Root solution is fractional → starting Branch & Bound.");
            SolveNode(problem, "Root Problem", "", updatePivot, 0);

            return BuildReport();

            SimplexResult BuildReport()
            {
                var sb = new StringBuilder();
                sb.AppendLine("Branch & Bound Finished.");
                if (BestSolution == null)
                {
                    sb.AppendLine("No integer-feasible solution found.");
                }
                else
                {
                    sb.AppendLine($"Best integer z* = {BestObjective:0.###}");
                    sb.AppendLine("Best integer x* = [" + string.Join(", ", BestSolution.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture))) + "]");
                }
                return new SimplexResult { Report = sb.ToString(), Summary = sb.ToString() };
            }
        }

        /// <summary>
        /// Recursive Branch & Bound node solver
        /// </summary>
        private void SolveNode(LPProblem problem, string name, string parentId, Action<string, bool[,]> updatePivot, int depth)
        {
            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);

            if (depth > MaxDepth)
            {
                Log($"{name}: Maximum recursion depth reached → prune.");
                return;
            }

            // Log constraints for debugging
            Log($"{name}: Constraints: {string.Join("; ", problem.Constraints.Select(c => $"{string.Join(" + ", c.A.Select((a, i) => $"{a}x{i + 1}"))} {c.Relation} {c.B}"))}");

            // Choose algorithm for this node
            string algo = ChooseAlgorithm(problem);
            Log($"\n{name}: Solving LP relaxation with {algo}...");

            SimplexResult res;
            try
            {
                res = _solver.Solve(problem, algo, updatePivot);
            }
            catch (Exception ex)
            {
                Log($"{name}: LP relaxation infeasible or error: {ex.Message}");
                return;
            }

            var x = ParseSolutionVector(res.Summary, problem.NumVars, updatePivot);

            // If parsing fails, try from tableau
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

            Log($"{name} LP solution: z* = {z:0.###}, x* = [{string.Join(", ", x.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]");

            // Validate solution feasibility
            if (!IsFeasible(x, problem))
            {
                Log($"{name}: Solution x* = [{string.Join(", ", x.Select(v => v.ToString("0.###")))}] is infeasible for constraints.");
                return;
            }

            // Bound pruning
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
                Log($"{name} is integer feasible. Updated BestObjective = {BestObjective:0.###}");
                return;
            }

            // Branching variable: fractional part closest to 0.5, lowest subscript for ties
            int fracIndex = -1;
            double minDist = double.MaxValue;
            for (int i = 0; i < x.Length; i++)
            {
                double fracPart = x[i] - Math.Floor(x[i]);
                if (fracPart > EPS && (1 - fracPart) > EPS) // fractional
                {
                    double dist = Math.Abs(fracPart - 0.5);
                    Log($"Checking x{i + 1} = {x[i]:0.######}, fracPart = {fracPart:0.######}, distance to 0.5 = {dist:0.######}");
                    if (dist < minDist || (dist == minDist && i < fracIndex))
                    {
                        minDist = dist;
                        fracIndex = i;
                    }
                }
            }

            if (fracIndex == -1)
            {
                Log($"{name}: No fractional variable found but solution not integral → prune.");
                return;
            }

            double fracVal = x[fracIndex];
            int floorVal = (int)Math.Floor(fracVal);
            int ceilVal = (int)Math.Ceiling(fracVal);

            Log($"{name}: branching on x{fracIndex + 1} = {fracVal:0.###} (floor={floorVal}, ceil={ceilVal})");

            // Generate hierarchical subproblem IDs
            string ceilId = string.IsNullOrEmpty(parentId) ? $"{_subProblemCounter}" : $"{parentId}.{_subProblemCounter - _subProblemCounter + 1}";
            string floorId = string.IsNullOrEmpty(parentId) ? $"{_subProblemCounter + 1}" : $"{parentId}.{_subProblemCounter - _subProblemCounter + 2}";
            _subProblemCounter += 2;

            // Left branch: xi ≤ floorVal
            var left = problem.Clone();
            left.Constraints.Add(new Constraint
            {
                A = UnitVector(problem.NumVars, fracIndex),
                Relation = Rel.LE,
                B = floorVal
            });

            // Right branch: xi ≥ ceilVal
            var right = problem.Clone();
            right.Constraints.Add(new Constraint
            {
                A = UnitVector(problem.NumVars, fracIndex),
                Relation = Rel.GE,
                B = ceilVal
            });

            string leftName = $"Subproblem {floorId}: x{fracIndex + 1} ≤ {floorVal}";
            string rightName = $"Subproblem {ceilId}: x{fracIndex + 1} ≥ {ceilVal}";

            Log($"{name}: → {rightName} (ceil first)");
            Log($"{name}: → {leftName}");

            SolveNode(right, rightName, ceilId, updatePivot, depth + 1);
            SolveNode(left, leftName, floorId, updatePivot, depth + 1);
        }

        // ------------------ Helpers ------------------------

        private static string ChooseAlgorithm(LPProblem p)
        {
            // If any constraint is >= or =, use DualSimplex.cs; else PrimalSimplex.cs
            bool hasGEorEQ = p.Constraints.Any(c => c.Relation == Rel.GE || c.Relation == Rel.EQ);
            return hasGEorEQ ? "Dual Simplex" : "Primal Simplex";
        }

        private static bool IsIntegral(double[] x)
        {
            for (int i = 0; i < x.Length; i++)
                if (Math.Abs(x[i] - Math.Round(x[i])) > EPS)
                    return false;
            return true;
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
            for (int i = 0; i < x.Length; i++)
                if (x[i] < -EPS)
                    return false;
            return true;
        }

        private static double RoundInt(double v) => Math.Round(v);

        private static double[] UnitVector(int size, int index)
        {
            var v = new double[size];
            v[index] = 1.0;
            return v;
        }

        private static double[] ParseSolutionVector(string summary, int expectedNumVars, Action<string, bool[,]> updatePivot = null)
        {
            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);
            try
            {
                int start = summary.IndexOf("x* = [");
                if (start < 0)
                {
                    Log("ParseSolutionVector: No 'x* = [' found in summary.");
                    return Array.Empty<double>();
                }
                start += 6;
                int end = summary.IndexOf("]", start);
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
            return 0;
        }
    }
}