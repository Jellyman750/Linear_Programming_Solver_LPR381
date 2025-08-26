using System;
using System.Linq;
using System.Text;
using Linear_Programming_Solver.Models;


namespace Linear_Programming_Solver.Models
{
    public class CuttingPlaneRevised : ILPAlgorithm
    {
        private const double Eps = 1e-9;
        private const int MaxIterations = 50;

        public SimplexResult Solve(LPProblem problem, Action<string, bool[,]> updatePivot = null)
        {
            var solver = new RevisedPrimalSimplex();
            var model = problem.Clone();
            var report = new StringBuilder();
            int iter = 1;

            while (true)
            {
                var lpRes = solver.Solve(model, updatePivot);
                report.AppendLine($"--- Cutting-Plane Iteration {iter} ---");
                report.AppendLine(lpRes.Report);

                if (!lpRes.Summary.Contains("Status: OPTIMAL", StringComparison.OrdinalIgnoreCase))
                {
                    report.AppendLine("Stopping: LP not OPTIMAL; cannot continue cutting.");
                    return new SimplexResult
                    {
                        Report = report.ToString(),
                        Summary = "Terminated: LP not OPTIMAL; cutting-plane stopped."
                    };
                }

                var x = ExtractSolution(lpRes.Summary, problem.NumVars);
                if (x == null)
                {
                    report.AppendLine("Stopping: Could not parse primal solution.");
                    return new SimplexResult
                    {
                        Report = report.ToString(),
                        Summary = "Terminated: could not parse solution."
                    };
                }

                int fracIndex = FindFractionalIndex(x);
                if (fracIndex == -1)
                {
                    report.AppendLine("All decision variables are integer. Optimal integer solution found.");
                    return new SimplexResult
                    {
                        Report = report.ToString(),
                        Summary = lpRes.Summary.Replace("Status: OPTIMAL", "Status: OPTIMAL INTEGER")
                    };
                }

                double floorVal = Math.Floor(x[fracIndex] + 1e-12);
                var cut = new Constraint
                {
                    A = Enumerable.Range(0, model.NumVars).Select(j => j == fracIndex ? 1.0 : 0.0).ToArray(),
                    Relation = Rel.LE,
                    B = floorVal
                };
                model.Constraints.Add(cut);
                report.AppendLine($"Added cut: x{fracIndex + 1} ≤ {floorVal} (current x{fracIndex + 1} = {x[fracIndex]:0.###})");

                iter++;
                if (iter > MaxIterations)
                {
                    report.AppendLine("Iteration limit reached.");
                    return new SimplexResult
                    {
                        Report = report.ToString(),
                        Summary = "Iteration limit reached (solution may still be fractional)."
                    };
                }
            }
        }

        private static int FindFractionalIndex(double[] x)
        {
            for (int i = 0; i < x.Length; i++)
            {
                double v = x[i];
                double frac = v - Math.Floor(v);
                if (frac > Eps && frac < 1 - Eps) return i;
            }
            return -1;
        }

        private static double[]? ExtractSolution(string summary, int nVars)
        {
            var line = summary.Split('\n')
                              .FirstOrDefault(l => l.TrimStart().StartsWith("x* = ["));
            if (line == null) return null;

            int s = line.IndexOf('[');
            int e = line.IndexOf(']');
            if (s < 0 || e < 0 || e <= s) return null;

            var nums = line.Substring(s + 1, e - s - 1)
                           .Split(',')
                           .Select(t => double.Parse(t.Trim(), System.Globalization.CultureInfo.InvariantCulture))
                           .ToList();

            if (nums.Count < nVars) while (nums.Count < nVars) nums.Add(0.0);
            if (nums.Count > nVars) nums = nums.Take(nVars).ToList();
            return nums.ToArray();
        }
    }
}

