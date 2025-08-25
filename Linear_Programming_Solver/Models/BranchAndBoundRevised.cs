using System;
using System.Linq;
using System.Text;

namespace Linear_Programming_Solver.Models
{
    /// <summary>
    /// Branch & Bound using the Revised Primal Simplex as the LP relaxation solver.
    /// Supports >= and <= constraints in subproblems.
    /// </summary>
    public class BranchAndBoundRevised : ILPAlgorithm
    {
        private readonly LPSolver _lpSolver = new LPSolver();
        private int _subCounter;
        private const double EPS = 1e-6;

        public double BestObjective { get; private set; } = double.NegativeInfinity;
        public double[] BestSolution { get; private set; }

        public SimplexResult Solve(LPProblem problem, Action<string, bool[,]> updatePivot = null)
        {
            _subCounter = 1;
            BestObjective = double.NegativeInfinity;
            BestSolution = null;

            void Log(string msg) => updatePivot?.Invoke(msg + Environment.NewLine, null);

            Log("Revised Branch & Bound: starting at Root Problem.");

            SolveSubproblem(problem, "Root Problem", Log);

            var sb = new StringBuilder();
            sb.AppendLine("Revised Branch & Bound Finished.");
            if (double.IsNegativeInfinity(BestObjective))
            {
                sb.AppendLine("No integer-feasible solution found.");
            }
            else
            {
                sb.AppendLine($"Best z* = {BestObjective:0.###}");
                sb.AppendLine("Best x* = [" + string.Join(", ", BestSolution.Select(v => v.ToString("0.###"))) + "]");
            }

            return new SimplexResult
            {
                Report = sb.ToString(),
                Summary = sb.ToString()
            };
        }

        private void SolveSubproblem(LPProblem problem, string name, Action<string> log)
        {
            log($"\n{name}: Solving LP relaxation (Revised Primal Simplex)...");

            SimplexResult res;
            try
            {
                // Solve LP relaxation using Revised Primal Simplex
                res = _lpSolver.Solve(problem, "Revised Primal Simplex");
            }
            catch (Exception ex)
            {
                log($"{name}: LP relaxation infeasible or error: {ex.Message}");
                return;
            }

            var x = ParseSolutionVector(res.Summary);
            var z = ParseObjectiveValue(res.Summary);

            if (x.Length == 0 || double.IsNaN(z) || double.IsInfinity(z))
            {
                log($"{name}: could not parse a valid relaxation solution → prune.");
                return;
            }

            log($"{name} relaxation: z* = {z:0.###}, x* = [{string.Join(", ", x.Select(v => v.ToString("0.###")))}]");

            // Bounding (maximize): prune if relaxation is worse than current best
            if (z <= BestObjective + EPS)
            {
                log($"{name}: pruned by bound (z* ≤ current best {BestObjective:0.###}).");
                return;
            }

            // Check integrality
            int fracIdx = -1;
            for (int i = 0; i < x.Length; i++)
            {
                if (Math.Abs(x[i] - Math.Round(x[i])) > EPS)
                {
                    fracIdx = i;
                    break;
                }
            }

            if (fracIdx == -1)
            {
                // Integer feasible → update incumbent if better
                if (z > BestObjective)
                {
                    BestObjective = z;
                    BestSolution = x;
                    log($"{name}: integer feasible, updated incumbent: z* = {z:0.###}.");
                }
                else
                {
                    log($"{name}: integer feasible but not better than incumbent.");
                }
                return;
            }

            // Branch on fractional variable
            double xi = x[fracIdx];
            double floor = Math.Floor(xi);
            double ceil = Math.Ceiling(xi);

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

            string leftName = $"Subproblem {_subCounter++}: x{fracIdx + 1} ≤ {floor}";
            string rightName = $"Subproblem {_subCounter++}: x{fracIdx + 1} ≥ {ceil}";

            log($"{name}: branching on x{fracIdx + 1} = {xi:0.###}");
            log($" → {leftName}");
            log($" → {rightName}");

            // Depth-first recursion
            SolveSubproblem(left, leftName, log);
            SolveSubproblem(right, rightName, log);
        }

        private static double[] UnitVector(int n, int index)
        {
            var a = new double[n];
            a[index] = 1.0;
            return a;
        }

        private static double[] ParseSolutionVector(string summary)
        {
            int start = summary.IndexOf("x* = [", StringComparison.Ordinal);
            if (start < 0) return Array.Empty<double>();
            start += 6;
            int end = summary.IndexOf("]", start, StringComparison.Ordinal);
            if (end < 0) return Array.Empty<double>();

            string vec = summary.Substring(start, end - start);
            var parts = vec.Split(',');
            var x = new double[parts.Length];
            for (int i = 0; i < parts.Length; i++)
            {
                if (!double.TryParse(parts[i].Trim(), out x[i])) x[i] = 0;
            }
            return x;
        }

        private static double ParseObjectiveValue(string summary)
        {
            foreach (var raw in summary.Split('\n'))
            {
                var line = raw.Trim();
                if (line.StartsWith("z*"))
                {
                    var parts = line.Split('=');
                    if (parts.Length == 2 && double.TryParse(parts[1].Trim(), out double val))
                        return val;
                }
            }
            return double.NegativeInfinity;
        }
    }
}
