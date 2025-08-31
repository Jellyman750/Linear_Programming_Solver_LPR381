using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Globalization;

namespace Linear_Programming_Solver.Models
{
    /// <summary>
    /// Branch-and-Bound for pure integer programs (all decision variables integral).
    /// - Root and each subproblem LP relaxation is solved with:
    ///   * "Primal Simplex" if all constraints are <=
    ///   * "Dual Simplex" if any constraint is >= or =
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

            // Log problem
            Log("=== Branch & Bound Algorithm ===");
            Log($"Objective: Maximize {string.Join(" + ", problem.C.Select((c, i) => $"{c.ToString("F3", CultureInfo.InvariantCulture)}x{i + 1}"))}");
            Log("Subject to:");
            for (int i = 0; i < problem.Constraints.Count; i++)
            {
                var c = problem.Constraints[i];
                Log($"{string.Join(" + ", c.A.Select((a, j) => a != 0 ? $"{a.ToString("F3", CultureInfo.InvariantCulture)}x{j + 1}" : null).Where(s => s != null))} {c.Relation} {c.B.ToString("F3", CultureInfo.InvariantCulture)}");
            }
            Log("x_j >= 0, integer");

            // Choose algorithm for root problem
            string rootAlgo = ChooseAlgorithm(problem);
            Log($"Branch & Bound: Using {rootAlgo} for the ROOT LP relaxation.");

            // Solve root LP relaxation
            SimplexResult rootRes;
            try
            {
                rootRes = _solver.Solve(problem, rootAlgo, updatePivot);
            }
            catch (Exception ex)
            {
                Log($"Root Problem: LP relaxation infeasible or error: {ex.Message}");
                return new SimplexResult { Report = "LP relaxation infeasible", Summary = "Error: Infeasible" };
            }

            // Validate result
            if (rootRes.Solution == null || rootRes.Tableau == null || rootRes.Basis == null || rootRes.VarNames == null)
            {
                Log("Root Problem: Invalid Simplex result (missing Solution, Tableau, Basis, or VarNames).");
                return new SimplexResult { Report = "Invalid Simplex result", Summary = "Error: Invalid result" };
            }

            // Extract solution
            double[] xRoot = rootRes.Solution.Take(problem.NumVars).ToArray();
            if (xRoot.Length != problem.NumVars)
            {
                Log($"Root Problem: Solution length ({xRoot.Length}) does not match NumVars ({problem.NumVars}).");
                return new SimplexResult { Report = "Invalid solution length", Summary = "Error: Invalid solution" };
            }
            double zRoot = rootRes.OptimalValue;

            Log($"Root Problem LP solution: z* = {zRoot.ToString("F3", CultureInfo.InvariantCulture)}, x* = [{string.Join(", ", xRoot.Select(v => v.ToString("F3", CultureInfo.InvariantCulture)))}]");
            Log("Root Problem optimal tableau displayed above.");

            // Check if root solution is integral
            if (IsIntegral(xRoot) && IsFeasible(xRoot, problem))
            {
                BestObjective = zRoot;
                BestSolution = xRoot.Select(RoundInt).ToArray();
                Log("Root Problem is already integral and feasible. Branch & Bound not required.");
                return BuildReport();
            }

            // Start branch and bound
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
                    sb.AppendLine($"Best integer z* = {BestObjective.ToString("F3", CultureInfo.InvariantCulture)}");
                    sb.AppendLine($"Best integer x* = [{string.Join(", ", BestSolution.Select(v => v.ToString("F3", CultureInfo.InvariantCulture)))}]");
                }
                return new SimplexResult
                {
                    Report = sb.ToString(),
                    Summary = sb.ToString(),
                    OptimalValue = BestObjective,
                    Solution = BestSolution,
                    Tableau = rootRes.Tableau,
                    Basis = rootRes.Basis,
                    VarNames = rootRes.VarNames
                };
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

            // Log constraints
            Log($"{name}: Constraints: {string.Join("; ", problem.Constraints.Select(c => $"{string.Join(" + ", c.A.Select((a, i) => a != 0 ? $"{a.ToString("F3", CultureInfo.InvariantCulture)}x{i + 1}" : null).Where(s => s != null))} {c.Relation} {c.B.ToString("F3", CultureInfo.InvariantCulture)}"))}");

            // Choose algorithm
            string algo = ChooseAlgorithm(problem);
            Log($"{name}: Solving LP relaxation with {algo}...");

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

            // Validate result
            if (res.Solution == null || res.Tableau == null || res.Basis == null || res.VarNames == null)
            {
                Log($"{name}: Invalid Simplex result (missing Solution, Tableau, Basis, or VarNames).");
                return;
            }

            // Extract solution
            double[] x = res.Solution.Take(problem.NumVars).ToArray();
            if (x.Length != problem.NumVars)
            {
                Log($"{name}: Solution length ({x.Length}) does not match NumVars ({problem.NumVars}).");
                return;
            }
            double z = res.OptimalValue;

            Log($"{name} LP solution: z* = {z.ToString("F3", CultureInfo.InvariantCulture)}, x* = [{string.Join(", ", x.Select(v => v.ToString("F3", CultureInfo.InvariantCulture)))}]");

            // Validate feasibility
            if (!IsFeasible(x, problem))
            {
                Log($"{name}: Solution x* = [{string.Join(", ", x.Select(v => v.ToString("F3", CultureInfo.InvariantCulture)))}] is infeasible for constraints.");
                return;
            }

            // Bound pruning
            if (z <= BestObjective + EPS)
            {
                Log($"{name}: Pruned by bound (z* ≤ current best {BestObjective.ToString("F3", CultureInfo.InvariantCulture)}).");
                return;
            }

            // Check integrality
            if (IsIntegral(x))
            {
                BestObjective = z;
                BestSolution = x.Select(RoundInt).ToArray();
                Log($"{name} is integer feasible. Updated BestObjective = {BestObjective.ToString("F3", CultureInfo.InvariantCulture)}");
                return;
            }

            // Branching variable: fractional part closest to 0.5, lowest subscript for ties
            int fracIndex = -1;
            double minDist = double.MaxValue;
            for (int i = 0; i < x.Length; i++)
            {
                double fracPart = x[i] - Math.Floor(x[i]);
                if (fracPart > EPS && (1 - fracPart) > EPS)
                {
                    double dist = Math.Abs(fracPart - 0.5);
                    Log($"Checking x{i + 1} = {x[i].ToString("F6", CultureInfo.InvariantCulture)}, fracPart = {fracPart.ToString("F6", CultureInfo.InvariantCulture)}, distance to 0.5 = {dist.ToString("F6", CultureInfo.InvariantCulture)}");
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

            Log($"{name}: Branching on x{fracIndex + 1} = {fracVal.ToString("F3", CultureInfo.InvariantCulture)} (floor={floorVal}, ceil={ceilVal})");

            // Generate hierarchical subproblem IDs
            string ceilId = string.IsNullOrEmpty(parentId) ? $"{_subProblemCounter}" : $"{parentId}.{_subProblemCounter}";
            string floorId = string.IsNullOrEmpty(parentId) ? $"{_subProblemCounter + 1}" : $"{parentId}.{_subProblemCounter + 1}";
            _subProblemCounter += 2;

            // Left branch: xi <= floorVal
            var left = problem.Clone();
            left.Constraints.Add(new Constraint
            {
                A = UnitVector(problem.NumVars, fracIndex),
                Relation = Rel.LE,
                B = floorVal
            });

            // Right branch: xi >= ceilVal
            var right = problem.Clone();
            right.Constraints.Add(new Constraint
            {
                A = UnitVector(problem.NumVars, fracIndex),
                Relation = Rel.GE,
                B = ceilVal
            });

            string leftName = $"Subproblem {floorId}: x{fracIndex + 1} <= {floorVal}";
            string rightName = $"Subproblem {ceilId}: x{fracIndex + 1} >= {ceilVal}";

            Log($"{name}: → {rightName} (ceil first)");
            Log($"{name}: → {leftName}");

            SolveNode(right, rightName, ceilId, updatePivot, depth + 1);
            SolveNode(left, leftName, floorId, updatePivot, depth + 1);
        }

        // ------------------ Helpers ------------------------

        private static string ChooseAlgorithm(LPProblem p)
        {
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
    }
}