using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

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
        public string Report { get; set; }
        public string Summary { get; set; }
        public double OptimalValue { get; set; }
        public double[] Solution { get; set; }
        public double[,] Tableau { get; set; }

        // Add these
        public int[] Basis { get; set; }
        public string[] VarNames { get; set; }
    }


    internal class PrimalSimplex : ILPAlgorithm
    {
        private const int MaxIterations = 10000;
        private const double Eps = 1e-9;

        public SimplexResult Solve(LPProblem original, Action<string, bool[,]> updatePivot = null)
        {
            var model = original.Clone();

            // ✅ Convert minimization to maximization
            if (model.ObjectiveSense == Sense.Min)
                for (int i = 0; i < model.C.Length; i++) model.C[i] = -model.C[i];

            // ✅ Check for unsupported constraints
            foreach (var cons in model.Constraints)
            {
                if (cons.Relation == Rel.GE)
                {
                    throw new Exception("Constraint contains '>=' sign. The Primal Simplex method cannot handle this. Please try the Dual Simplex algorithm instead.");
                }

                if (cons.B < -1e-9)
                {
                    throw new Exception("Constraint has a negative RHS value. The Primal Simplex method cannot handle this. Please try the Dual Simplex algorithm instead.");
                }
            }

            // ✅ Expand equalities into two inequalities
            var tableauModel = ExpandEqualitiesToInequalities(model);
            var report = new StringBuilder();
            AppendCanonicalForm(report, tableauModel);

            // ✅ Build initial tableau
            var tableau = BuildTableau(tableauModel, out var basis, out var varNames);

            // Show initial tableau
            var sbIteration = new StringBuilder();
            AppendTableau(sbIteration, tableau, basis, varNames, 0);
            updatePivot?.Invoke(sbIteration.ToString(), null);

            int iter = 1;
            while (true)
            {
                if (iter > MaxIterations)
                    throw new Exception("Iteration limit exceeded.");

                int entering = ChooseEntering(tableau);
                if (entering == -1) break; // optimal reached

                int leaving = ChooseLeaving(tableau, entering);
                if (leaving == -1)
                {
                    report.AppendLine("UNBOUNDED");
                    return FinalizeReport(report, tableau, basis, varNames, "UNBOUNDED");
                }

                // Perform pivot
                Pivot(tableau, leaving, entering);
                basis[leaving] = entering;

                // Build string for this tableau
                sbIteration = new StringBuilder();
                AppendTableau(sbIteration, tableau, basis, varNames, iter);

                // Highlight pivot
                bool[,] highlight = new bool[tableau.GetLength(0), tableau.GetLength(1)];
                for (int j = 0; j < tableau.GetLength(1); j++) highlight[leaving, j] = true; // pivot row
                for (int i = 0; i < tableau.GetLength(0); i++) highlight[i, entering] = true; // pivot col

                updatePivot?.Invoke(sbIteration.ToString(), highlight);

                iter++;
            }

            return FinalizeReport(report, tableau, basis, varNames, "OPTIMAL");
        }

        // ---------- Helper methods ----------
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

            return new SimplexResult
            {
                Report = sb.ToString(),
                Summary = summary.ToString(),
                OptimalValue = z,
                Solution = x,
                Tableau = T,
                Basis = basis,
                VarNames = varNames
            };

        }

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
                T[i, n + i] = 1.0;   // slack
                T[i, n + s] = row.B;
            }

            for (int j = 0; j < n; j++) T[m, j] = -model.C[j];

            basis = Enumerable.Range(n, s).ToArray();

            varNames = new string[n + s];
            for (int j = 0; j < n; j++) varNames[j] = $"x{j + 1}";
            for (int j = 0; j < s; j++) varNames[n + j] = $"c{j + 1}";
            return T;
        }

        private static int ChooseEntering(double[,] T)
        {
            int m = T.GetLength(0) - 1;
            int cols = T.GetLength(1) - 1;
            int best = -1;
            double minVal = -Eps;
            for (int j = 0; j < cols; j++)
            {
                if (T[m, j] < minVal)
                {
                    minVal = T[m, j];
                    best = j;
                }
            }
            return best;
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
            int n = varNames.Count(v => v.StartsWith("x"));
            int ns = T.GetLength(1) - 1;
            int colWidth = 12;

            string PadString(string s) => s.PadLeft(colWidth);
            string PadDouble(double d) => d.ToString("0.###").PadLeft(colWidth);

            sb.AppendLine($"TABLEAU Iteration {iter}");

            sb.Append(PadString("Basis"));
            for (int j = 0; j < ns; j++) sb.Append(PadString(varNames[j]));
            sb.Append(PadString("RHS"));
            sb.AppendLine();

            string dash = new string('-', colWidth * (ns + 2));
            sb.AppendLine(dash);

            sb.Append(PadString("z"));
            for (int j = 0; j < ns; j++) sb.Append(PadDouble(T[m, j]));
            sb.Append(PadDouble(T[m, ns]));
            sb.AppendLine();

            for (int i = 0; i < m; i++)
            {
                sb.Append(PadString(varNames[basis[i]]));
                for (int j = 0; j < ns; j++) sb.Append(PadDouble(T[i, j]));
                sb.Append(PadDouble(T[i, ns]));
                sb.AppendLine();
            }
        }
    }

}

