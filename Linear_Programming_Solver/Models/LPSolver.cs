using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Linear_Programming_Solver.Models
{

    public enum Sense { Max, Min }
    public enum Rel { LE, GE, EQ }

    public class Constraint
    {
        public double[] A { get; set; } = Array.Empty<double>();
        public Rel Relation { get; set; }
        public double B { get; set; }

        public Constraint Clone() => new Constraint { A = (double[])A.Clone(), Relation = Relation, B = B };
    }

    public class LPProblem
    {
        public Sense ObjectiveSense { get; set; } = Sense.Max;
        public double[] C { get; set; } = Array.Empty<double>();
        public List<Constraint> Constraints { get; set; } = new List<Constraint>();
        public int NumVars => C.Length;

        public LPProblem Clone()
        {
            return new LPProblem
            {
                ObjectiveSense = ObjectiveSense,
                C = (double[])C.Clone(),
                Constraints = Constraints.Select(c => c.Clone()).ToList()
            };
        }
    }

    public class SimplexResult
    {
        public string Report { get; set; } = string.Empty;
        public string Summary { get; set; } = string.Empty;
    }

    public class LPSolver
    {
        private const int MaxIterations = 10000;
        private const double Eps = 1e-9;

        public SimplexResult Solve(LPProblem original)
        {
            var model = original.Clone();

            if (model.ObjectiveSense == Sense.Min)
                for (int i = 0; i < model.C.Length; i++) model.C[i] = -model.C[i];

            foreach (var cons in model.Constraints)
            {
                if (cons.Relation == Rel.GE)
                {
                    for (int j = 0; j < cons.A.Length; j++) cons.A[j] *= -1;
                    cons.B *= -1;
                    cons.Relation = Rel.LE;
                }
                if (cons.B < -Eps)
                {
                    for (int j = 0; j < cons.A.Length; j++) cons.A[j] *= -1;
                    cons.B *= -1;
                }
            }

            var tableauModel = ExpandEqualitiesToInequalities(model);
            var report = new StringBuilder();
            AppendCanonicalForm(report, tableauModel);

            var tableau = BuildTableau(tableauModel, out var basis, out var varNames);

            int iter = 0;
            while (true)
            {
                if (iter++ > MaxIterations)
                    throw new Exception("Iteration limit exceeded.");

                int entering = ChooseEntering(tableau);
                if (entering == -1) break; // optimal

                int leaving = ChooseLeaving(tableau, entering);
                if (leaving == -1)
                {
                    report.AppendLine("UNBOUNDED");
                    return FinalizeReport(report, tableau, basis, varNames, "UNBOUNDED");
                }

                Pivot(tableau, leaving, entering);
                basis[leaving] = entering;
                AppendTableau(report, tableau, basis, varNames, iter);
            }

            return FinalizeReport(report, tableau, basis, varNames, "OPTIMAL");
        }

        #region Helper Methods

        private static LPProblem ExpandEqualitiesToInequalities(LPProblem model)
        {
            var expanded = new LPProblem { C = (double[])model.C.Clone() };
            foreach (var cons in model.Constraints)
            {
                if (cons.Relation == Rel.EQ)
                {
                    expanded.Constraints.Add(new Constraint { A = (double[])cons.A.Clone(), Relation = Rel.LE, B = cons.B });
                    var neg = new Constraint { A = (double[])cons.A.Clone(), Relation = Rel.LE, B = cons.B };
                    for (int j = 0; j < neg.A.Length; j++) neg.A[j] *= -1;
                    neg.B *= -1;
                    expanded.Constraints.Add(neg);
                }
                else expanded.Constraints.Add(cons.Clone());
            }
            return expanded;
        }

        private static double[,] BuildTableau(LPProblem model, out int[] basis, out string[] varNames)
        {
            int m = model.Constraints.Count;
            int n = model.NumVars;
            int s = m;

            var T = new double[m + 1, n + s + 1];

            for (int i = 0; i < m; i++)
            {
                var row = model.Constraints[i];
                for (int j = 0; j < n; j++) T[i, j] = row.A[j];
                T[i, n + i] = 1.0;
                T[i, n + s] = row.B;
            }

            for (int j = 0; j < n; j++) T[m, j] = -model.C[j];

            basis = Enumerable.Range(n, s).ToArray();
            varNames = new string[n + s];
            for (int j = 0; j < n; j++) varNames[j] = $"x{j + 1}";
            for (int j = 0; j < s; j++) varNames[n + j] = $"s{j + 1}";

            return T;
        }
        

        private static int ChooseEntering(double[,] T)
        {
            int m = T.GetLength(0) - 1;
            int cols = T.GetLength(1) - 1;
            for (int j = 0; j < cols; j++)
                if (T[m, j] < -Eps) return j;
            return -1;
        }

        private static int ChooseLeaving(double[,] T, int entering)
        {
            int m = T.GetLength(0) - 1;
            int rhsCol = T.GetLength(1) - 1;
            double bestRatio = double.PositiveInfinity;
            int bestRow = -1;

            for (int i = 0; i < m; i++)
            {
                double aij = T[i, entering];
                if (aij > Eps)
                {
                    double ratio = T[i, rhsCol] / aij;
                    if (ratio < bestRatio - Eps)
                    {
                        bestRatio = ratio;
                        bestRow = i;
                    }
                }
            }
            return bestRow;
        }

        private static void Pivot(double[,] T, int row, int col)
        {
            int mPlus1 = T.GetLength(0);
            int nPlus1 = T.GetLength(1);
            double piv = T[row, col];
            for (int j = 0; j < nPlus1; j++) T[row, j] /= piv;
            for (int i = 0; i < mPlus1; i++)
            {
                if (i == row) continue;
                double factor = T[i, col];
                for (int j = 0; j < nPlus1; j++) T[i, j] -= factor * T[row, j];
            }
        }

        private static void AppendCanonicalForm(StringBuilder sb, LPProblem model)
        {
            sb.AppendLine("Objective: max " + string.Join(" ", model.C.Select((v, j) => $"{(v >= 0 ? "+" : "-")}{Math.Abs(v):0.###}x{j + 1}")));
            sb.AppendLine("Subject to:");
            for (int i = 0; i < model.Constraints.Count; i++)
            {
                var c = model.Constraints[i];
                string rel = c.Relation switch { Rel.LE => "<=", Rel.GE => ">=", _ => "=" };
                sb.AppendLine("  " + string.Join(" ", c.A.Select((v, j) => $"{(v >= 0 ? "+" : "-")}{Math.Abs(v):0.###}x{j + 1}")) + $" {rel} {c.B:0.###}");
            }
            sb.AppendLine("x >= 0");
        }

        private static void AppendTableau(StringBuilder sb, double[,] T, int[] basis, string[] varNames, int iter)
        {
            int m = T.GetLength(0) - 1;
            int nPlusS = T.GetLength(1) - 1;
            sb.AppendLine($"\nTABLEAU Iteration {iter}");
            sb.AppendLine("Basis | " + string.Join("", varNames.Select(v => v.PadLeft(8))) + " | RHS");
            sb.AppendLine(new string('-', 10 + 9 * (nPlusS + 1)));

            for (int i = 0; i < m; i++)
            {
                sb.Append(varNames[basis[i]].PadLeft(5) + " | ");
                for (int j = 0; j < nPlusS; j++) sb.Append(Math.Round(T[i, j], 3).ToString("0.###").PadLeft(8));
                sb.Append(" | ").Append(Math.Round(T[i, nPlusS], 3).ToString("0.###")).AppendLine();
            }

            sb.Append("   z  | ");
            for (int j = 0; j < nPlusS; j++) sb.Append(Math.Round(T[m, j], 3).ToString("0.###").PadLeft(8));
            sb.Append(" | ").Append(Math.Round(T[m, nPlusS], 3).ToString("0.###")).AppendLine();
        }

        private static SimplexResult FinalizeReport(StringBuilder sb, double[,] T, int[] basis, string[] varNames, string status)
        {
            int m = T.GetLength(0) - 1;
            int n = varNames.Count(v => v.StartsWith("x"));
            var x = new double[n];
            for (int i = 0; i < m; i++)
                if (basis[i] < n) x[basis[i]] = T[i, T.GetLength(1) - 1];

            double z = T[m, T.GetLength(1) - 1];
            sb.AppendLine($"\nStatus: {status}");
            for (int j = 0; j < n; j++) sb.AppendLine($"  x{j + 1} = {Math.Round(x[j], 3):0.###}");
            sb.AppendLine($"  z* = {Math.Round(z, 3):0.###}");

            var summary = new StringBuilder();
            summary.AppendLine($"Status: {status}");
            summary.AppendLine($"z* = {Math.Round(z, 3):0.###}");
            summary.AppendLine("x* = [" + string.Join(", ", x.Select(v => Math.Round(v, 3))) + "]");

            return new SimplexResult { Report = sb.ToString(), Summary = summary.ToString() };
        }
        #endregion

    }
}






