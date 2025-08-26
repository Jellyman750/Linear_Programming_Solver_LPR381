using System;
using System.Text;
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

            while (true)
            {
                var lpResult = simplex.Solve(model, updatePivot);
                report.AppendLine($"--- Iteration {iteration} ---");
                report.AppendLine(lpResult.Report);

                double[] solution = new double[problem.NumVars];
                string[] lines = lpResult.Summary.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
                for (int i = 0; i < lines.Length; i++)
                {
                    if (lines[i].StartsWith("x*"))
                    {
                        string s = lines[i].Split('=')[1].Trim().Trim('[', ']');
                        var parts = s.Split(new[] { ',', ' ' }, StringSplitOptions.RemoveEmptyEntries);
                        for (int j = 0; j < parts.Length; j++)
                            solution[j] = double.Parse(parts[j]);
                    }
                }

                int fracIndex = -1;
                for (int i = 0; i < solution.Length; i++)
                {
                    double value = solution[i];
                    double frac = value - Math.Floor(value);
                    if (frac > Eps && frac < 1 - Eps)
                    {
                        fracIndex = i;
                        break;
                    }
                }

                if (fracIndex == -1)
                {
                    report.AppendLine("All variables integer. Optimal integer solution found.");
                    return new SimplexResult
                    {
                        Report = report.ToString(),
                        Summary = lpResult.Summary.Replace("OPTIMAL", "OPTIMAL INTEGER")
                    };
                }
                var cut = new Constraint
                {
                    A = new double[model.NumVars],
                    Relation = Rel.LE,
                    B = Math.Floor(solution[fracIndex])
                };
                cut.A[fracIndex] = 1;
                model.Constraints.Add(cut);

                report.AppendLine($"Added Gomory cut: x{fracIndex + 1} ≤ {cut.B}");

                iteration++;
                if (iteration > 50)
                {
                    report.AppendLine("Iteration limit reached. Stopping.");
                    break;
                }
            }

            return new SimplexResult
            {
                Report = report.ToString(),
                Summary = "INCOMPLETE"
            };
        }
    }
}