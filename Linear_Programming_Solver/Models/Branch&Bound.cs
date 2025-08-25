using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using System.Globalization;
using System.Text.RegularExpressions;

namespace Linear_Programming_Solver.Models
{
    public class BranchAndBound : ILPAlgorithm
    {
        private LPSolver solver = new LPSolver();
        private int subProblemCounter = 1;
        private const double EPS = 1e-6;
        private const int MaxDepth = 100;

        public double BestObjective { get; private set; } = double.NegativeInfinity;
        public double[] BestSolution { get; private set; }
        private string _relaxationAlgorithm = "Primal Simplex";

        public void SetRelaxationAlgorithm(string algorithm)
        {
            if (!string.IsNullOrWhiteSpace(algorithm))
                _relaxationAlgorithm = algorithm.Trim();
        }

        public SimplexResult Solve(LPProblem problem, Action<string, bool[,]> updatePivot = null)
        {
            subProblemCounter = 1;
            BestObjective = double.NegativeInfinity;
            BestSolution = null;

            _relaxationAlgorithm = "Primal Simplex";
            if (problem.Constraints.Any(c => c.Relation == Rel.GE))
                updatePivot?.Invoke("Warning: GE constraints detected. Normalizing to LE for Primal Simplex.\n", null);

            updatePivot?.Invoke($"Branch & Bound: Using {_relaxationAlgorithm} for LP relaxations.\n", null);

            SimplexResult lpResult;
            try
            {
                lpResult = solver.Solve(problem, _relaxationAlgorithm, updatePivot);
            }
            catch (Exception ex)
            {
                updatePivot?.Invoke($"Root Problem LP relaxation infeasible or error: {ex.Message}\n", null);
                return new SimplexResult { Report = "LP relaxation infeasible", Summary = "" };
            }

            double[] xLP = ParseSolutionVector(lpResult.Summary, problem.NumVars, updatePivot);
            double zLP = ParseObjectiveValue(lpResult.Summary);

            updatePivot?.Invoke($"Root Problem LP optimal solution: z* = {zLP:0.###}, x* = [{string.Join(", ", xLP.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]\n", null);
            updatePivot?.Invoke("Root Problem optimal tableau displayed above.\n", null);

            SolveSubproblem(problem, "Root Problem (Branching)", updatePivot, 0);

            var sb = new StringBuilder();
            sb.AppendLine("Branch & Bound Finished.");
            if (BestSolution == null)
                sb.AppendLine("No integer-feasible solution found.");
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

        private void SolveSubproblem(LPProblem problem, string name, Action<string, bool[,]> updatePivot, int depth)
        {
            if (depth > MaxDepth)
            {
                updatePivot?.Invoke($"{name}: Maximum depth reached → prune.\n", null);
                return;
            }

            updatePivot?.Invoke($"\n{name}: Solving LP relaxation for branching...\n", null);

            // Normalize GE constraints to LE for Primal Simplex
            var normalized = problem.Clone();
            foreach (var c in normalized.Constraints)
            {
                if (c.Relation == Rel.GE)
                {
                    for (int i = 0; i < c.A.Length; i++) c.A[i] = -c.A[i];
                    c.B = -c.B;
                    c.Relation = Rel.LE;
                }
            }

            SimplexResult result;
            try
            {
                result = solver.Solve(normalized, _relaxationAlgorithm, updatePivot);
            }
            catch (Exception ex)
            {
                updatePivot?.Invoke($"{name}: LP relaxation infeasible or error: {ex.Message}\n", null);
                return;
            }

            double[] x = ParseSolutionVector(result.Summary, problem.NumVars, updatePivot);
            double zValue = ParseObjectiveValue(result.Summary);

            if (x.Length == 0)
            {
                updatePivot?.Invoke($"{name}: Could not parse solution → prune.\n", null);
                return;
            }

            updatePivot?.Invoke($"{name} LP solution: z* = {zValue:0.###}, x* = [{string.Join(", ", x.Select(v => v.ToString("0.###", CultureInfo.InvariantCulture)))}]\n", null);

            if (zValue <= BestObjective + EPS)
            {
                updatePivot?.Invoke($"{name} pruned: z* = {zValue:0.###} <= BestObjective = {BestObjective:0.###}\n", null);
                return;
            }

            updatePivot?.Invoke($"{name}: Checking integer feasibility for {x.Length} variables (expected {problem.NumVars})\n", null);
            int fracIndex = -1;
            double minDistanceToHalf = double.MaxValue;
            for (int i = 0; i < x.Length; i++)
            {
                double fracPart = x[i] - Math.Floor(x[i]);
                if (fracPart > EPS && fracPart < 1 - EPS)
                {
                    double distance = Math.Abs(fracPart - 0.5);
                    updatePivot?.Invoke($"Checking x{i + 1} = {x[i]:0.######}, fracPart = {fracPart:0.######}, distance to 0.5 = {distance:0.######}\n", null);
                    if (distance < minDistanceToHalf || (distance == minDistanceToHalf && i < fracIndex))
                    {
                        minDistanceToHalf = distance;
                        fracIndex = i;
                    }
                }
            }

            if (fracIndex == -1)
            {
                BestObjective = zValue;
                BestSolution = x;
                updatePivot?.Invoke($"{name} is integer feasible. Updated BestObjective = {BestObjective:0.###}\n", null);
                return;
            }

            double xi = x[fracIndex];
            double floor = Math.Floor(xi);
            double ceil = Math.Ceiling(xi);

            var left = problem.Clone();
            left.Constraints.Add(new Constraint
            {
                A = UnitVector(problem.NumVars, fracIndex),
                Relation = Rel.LE,
                B = floor
            });

            var right = problem.Clone();
            right.Constraints.Add(new Constraint
            {
                A = UnitVector(problem.NumVars, fracIndex),
                Relation = Rel.GE,
                B = ceil
            });

            string leftName = $"Subproblem {subProblemCounter++}: x{fracIndex + 1} <= {floor}";
            string rightName = $"Subproblem {subProblemCounter++}: x{fracIndex + 1} >= {ceil}";

            updatePivot?.Invoke($"{name}: Branching on x{fracIndex + 1} = {xi:0.###}\n → {leftName}\n → {rightName}\n", null);

            // Solve ceil branch first for maximization
            SolveSubproblem(right, rightName, updatePivot, depth + 1);
            SolveSubproblem(left, leftName, updatePivot, depth + 1);
        }

        private static double[] UnitVector(int size, int index)
        {
            double[] v = new double[size];
            v[index] = 1.0;
            return v;
        }

        private static double[] ParseSolutionVector(string summary, int expectedNumVars, Action<string, bool[,]> updatePivot = null)
        {
            try
            {
                int start = summary.IndexOf("x* = [");
                if (start < 0)
                {
                    updatePivot?.Invoke("ParseSolutionVector: No 'x* = [' found in summary.\n", null);
                    return Array.Empty<double>();
                }
                start += 6;
                int end = summary.IndexOf("]", start);
                if (end < 0)
                {
                    updatePivot?.Invoke("ParseSolutionVector: No closing ']' found in summary.\n", null);
                    return Array.Empty<double>();
                }
                string vec = summary.Substring(start, end - start).Trim();
                updatePivot?.Invoke($"ParseSolutionVector: Raw vector string = '{vec}'\n", null);

                // Extract numbers using Regex (matches integers or decimals with comma or period)
                var matches = Regex.Matches(vec, @"\d+[\.,]\d+|\d+");
                if (matches.Count != expectedNumVars)
                {
                    updatePivot?.Invoke($"ParseSolutionVector: Expected {expectedNumVars} numbers, but found {matches.Count}.\n", null);
                    return Array.Empty<double>();
                }

                var values = new double[matches.Count];
                for (int i = 0; i < matches.Count; i++)
                {
                    string numStr = matches[i].Value.Replace(',', '.');
                    values[i] = double.Parse(numStr, NumberStyles.Any, CultureInfo.InvariantCulture);
                }

                updatePivot?.Invoke($"ParseSolutionVector: Parsed {values.Length} values: [{string.Join(", ", values.Select(v => v.ToString("0.######", CultureInfo.InvariantCulture)))}]\n", null);

                return values;
            }
            catch (Exception ex)
            {
                updatePivot?.Invoke($"ParseSolutionVector error: {ex.Message}\n", null);
                Console.WriteLine($"ParseSolutionVector error: {ex.Message}");
                return Array.Empty<double>();
            }
        }

        private static double ParseObjectiveValue(string summary)
        {
            try
            {
                foreach (var line in summary.Split('\n'))
                {
                    if (line.StartsWith("z*"))
                    {
                        var parts = line.Split('=');
                        if (parts.Length == 2 && double.TryParse(parts[1].Trim().Replace(',', '.'), NumberStyles.Any, CultureInfo.InvariantCulture, out double val))
                            return val;
                    }
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"ParseObjectiveValue error: {ex.Message}");
            }
            return 0;
        }
    }
}