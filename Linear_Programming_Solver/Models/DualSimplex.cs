using System;
using System.Linq;
using System.Text;

namespace Linear_Programming_Solver.Models
{
    /// <summary>
    /// Dual Simplex using tableau updates.
    /// Starts from a dual-feasible tableau (z-row nonnegative) and fixes infeasible RHS (negative b).
    /// </summary>
    internal class DualSimplex : ILPAlgorithm
    {
        private const double Eps = 1e-9;

        public SimplexResult Solve(LPProblem original, Action<string, bool[,]> updatePivot = null)
        {
            // Put in standard max form with <= and b >= 0 where possible; then build same tableau as primal solver.
            var model = PrepareForTableau(original);
            var T = BuildTableau(model, out var basis, out var varNames);

            // Make z-row dual feasible: if any negative reduced costs remain, run a few primal pivots to fix.
            ForceDualFeasibility(T);

            // Show initial tableau
            var sb0 = new StringBuilder();
            AppendTableau(sb0, T, basis, varNames, 0);
            updatePivot?.Invoke(sb0.ToString(), null);

            int iter = 1;
            while (true)
            {
                if (iter > 10000) throw new Exception("Iteration limit exceeded (Dual Simplex).");

                int m = T.GetLength(0) - 1;
                int nNoRhs = T.GetLength(1) - 1;

                // Choose leaving row: most negative RHS (infeasible basic)
                int rhsCol = nNoRhs;
                int leave = -1;
                double mostNeg = -Eps;
                for (int i = 0; i < m; i++)
                {
                    if (T[i, rhsCol] < mostNeg)
                    {
                        mostNeg = T[i, rhsCol];
                        leave = i;
                    }
                }
                if (leave == -1)
                {
                    // primal feasible now; optimal (dual feasible was maintained)
                    return FinalizeReport(new StringBuilder(), T, basis, varNames, "OPTIMAL");
                }

                // Choose entering col j that minimizes ratio (z_j / a_{leave,j}) over a_{leave,j} < 0.
                int enter = -1;
                double bestRatio = double.PositiveInfinity;
                for (int j = 0; j < nNoRhs; j++)
                {
                    double a = T[leave, j];
                    if (a < -Eps)
                    {
                        double ratio = T[m, j] / (-a); // (reduced cost)/|a|
                        if (ratio < bestRatio - 1e-12)
                        {
                            bestRatio = ratio;
                            enter = j;
                        }
                    }
                }
                if (enter == -1)
                {
                    return FinalizeReport(new StringBuilder().AppendLine("INFEASIBLE (dual step found no entering)"),
                        T, basis, varNames, "INFEASIBLE");
                }

                // Pivot
                Pivot(T, leave, enter);
                basis[leave] = enter;

                // Print iteration
                var sb = new StringBuilder();
                AppendTableau(sb, T, basis, varNames, iter);

                // highlight pivot row and column
                var highlight = new bool[m + 1, nNoRhs + 1]; // including z-row for uniformity in color pass
                for (int j = 0; j < nNoRhs + 1; j++) highlight[leave, j] = true;
                for (int i = 0; i < m + 1; i++) highlight[i, enter] = true;

                updatePivot?.Invoke(sb.ToString(), highlight);
                iter++;
            }
        }

        #region Prep / Tableau
        private static LPProblem PrepareForTableau(LPProblem original)
        {
            var model = original.Clone();

            // Convert to max
            if (model.ObjectiveSense == Sense.Min)
                for (int i = 0; i < model.C.Length; i++) model.C[i] = -model.C[i];

            // Convert GE to LE and make b >= 0 where possible (flip rows)
            var expanded = new LPProblem { C = (double[])model.C.Clone() };
            foreach (var cons in model.Constraints)
            {
                if (cons.Relation == Rel.EQ)
                {
                    expanded.Constraints.Add(new Constraint { A = (double[])cons.A.Clone(), B = cons.B, Relation = Rel.LE });
                    var neg = new Constraint { A = (double[])cons.A.Clone(), B = -cons.B, Relation = Rel.LE };
                    for (int j = 0; j < neg.A.Length; j++) neg.A[j] *= -1;
                    expanded.Constraints.Add(neg);
                }
                else
                {
                    var row = cons.Clone();
                    if (row.Relation == Rel.GE)
                    {
                        for (int j = 0; j < row.A.Length; j++) row.A[j] *= -1;
                        row.B *= -1;
                        row.Relation = Rel.LE;
                    }
                    if (row.B < -Eps)
                    {
                        for (int j = 0; j < row.A.Length; j++) row.A[j] *= -1;
                        row.B *= -1;
                    }
                    expanded.Constraints.Add(row);
                }
            }
            return expanded;
        }

        private static double[,] BuildTableau(LPProblem model, out int[] basis, out string[] varNames)
        {
            int m = model.Constraints.Count;
            int n = model.NumVars;
            int s = m;

            var T = new double[m + 1, n + s + 1];

            // rows
            for (int i = 0; i < m; i++)
            {
                var row = model.Constraints[i];
                for (int j = 0; j < n; j++) T[i, j] = row.A[j];
                T[i, n + i] = 1.0;
                T[i, n + s] = row.B;
            }
            // z row
            for (int j = 0; j < n; j++) T[m, j] = -model.C[j];

            // basis = slacks
            basis = Enumerable.Range(n, s).ToArray();

            varNames = new string[n + s];
            for (int j = 0; j < n; j++) varNames[j] = $"x{j + 1}";
            for (int j = 0; j < s; j++) varNames[n + j] = $"c{j + 1}";
            return T;
        }

        private static void ForceDualFeasibility(double[,] T)
        {
            // Ensure all z-row entries are >= 0 by performing a few primal pivots if needed.
            // Simple loop: while min z_j < 0, pivot with the usual primal rule (ratio test).
            int m = T.GetLength(0) - 1;
            int rhsCol = T.GetLength(1) - 1;

            for (int guard = 0; guard < 50; guard++)
            {
                int entering = -1;
                double minVal = -Eps;
                int nNoRhs = rhsCol;
                for (int j = 0; j < nNoRhs; j++)
                    if (T[m, j] < minVal) { minVal = T[m, j]; entering = j; }
                if (entering == -1) return; // dual feasible already

                int leave = -1;
                double bestRatio = double.PositiveInfinity;
                for (int i = 0; i < m; i++)
                {
                    if (T[i, entering] > Eps)
                    {
                        double ratio = T[i, rhsCol] / T[i, entering];
                        if (ratio < bestRatio - 1e-12) { bestRatio = ratio; leave = i; }
                    }
                }
                if (leave == -1) return; // unbounded; let dual simplex handle later
                Pivot(T, leave, entering);
            }
        }
        #endregion

        #region Core ops / printing (mirrors your PrimalSimplex style)
        private static void Pivot(double[,] T, int row, int col)
        {
            int R = T.GetLength(0);
            int C = T.GetLength(1);

            double piv = T[row, col];
            for (int j = 0; j < C; j++) T[row, j] /= piv;

            for (int i = 0; i < R; i++)
            {
                if (i == row) continue;
                double f = T[i, col];
                for (int j = 0; j < C; j++) T[i, j] -= f * T[row, j];
            }
        }

        private static void AppendTableau(StringBuilder sb, double[,] T, int[] basis, string[] varNames, int iter)
        {
            int m = T.GetLength(0) - 1; // constraints
            int ns = T.GetLength(1) - 1; // columns excluding RHS
            int colWidth = 12;

            string PadString(string s) => s.PadLeft(colWidth);
            string PadDouble(double d) => d.ToString("0.###").PadLeft(colWidth);

            sb.AppendLine($"DUAL SIMPLEX TABLEAU Iteration {iter}");

            sb.Append(PadString("Basis"));
            for (int j = 0; j < ns; j++) sb.Append(PadString(varNames[j]));
            sb.Append(PadString("RHS"));
            sb.AppendLine();

            string dash = new string('-', colWidth * (ns + 2));
            sb.AppendLine(dash);

            // z row
            sb.Append(PadString("z"));
            for (int j = 0; j < ns; j++) sb.Append(PadDouble(T[m, j]));
            sb.Append(PadDouble(T[m, ns]));
            sb.AppendLine();

            // constraints
            for (int i = 0; i < m; i++)
            {
                sb.Append(PadString(varNames[basis[i]]));
                for (int j = 0; j < ns; j++) sb.Append(PadDouble(T[i, j]));
                sb.Append(PadDouble(T[i, ns]));
                sb.AppendLine();
            }
        }

        private static SimplexResult FinalizeReport(StringBuilder report, double[,] T, int[] basis, string[] varNames, string status)
        {
            int m = T.GetLength(0) - 1;
            int n = varNames.Count(v => v.StartsWith("x"));
            var x = new double[n];

            for (int i = 0; i < m; i++)
                if (basis[i] < n) x[basis[i]] = T[i, T.GetLength(1) - 1];

            double z = T[m, T.GetLength(1) - 1];

            report.AppendLine($"\nStatus: {status}");
            for (int j = 0; j < n; j++) report.AppendLine($"  x{j + 1} = {Math.Round(x[j], 3):0.###}");
            report.AppendLine($"  z* = {Math.Round(z, 3):0.###}");

            var summary = new StringBuilder();
            summary.AppendLine($"Status: {status}");
            summary.AppendLine($"z* = {Math.Round(z, 3):0.###}");
            summary.AppendLine("x* = [" + string.Join(", ", x.Select(v => Math.Round(v, 3))) + "]");

            return new SimplexResult { Report = report.ToString(), Summary = summary.ToString() };
        }
        #endregion
    }
}