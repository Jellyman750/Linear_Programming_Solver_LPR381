using System;
using System.Linq;
using System.Text;

namespace Linear_Programming_Solver.Models
{
    /// <summary>
    /// Dual Simplex using tableau updates.
    /// Starts from a dual-feasible tableau (z-row >= 0) and fixes infeasible RHS (negative b).
    /// </summary>
    internal class DualSimplex : ILPAlgorithm
    {
        private const double Eps = 1e-9;

        public SimplexResult Solve(LPProblem original, Action<string, bool[,]> updatePivot = null)
        {
            // 1) Standardize to Max with only <= and b >= 0 (split '=' into two <=)
            var model = PrepareForTableau(original);

            // 2) Build primal-style tableau with slack basis
            var T = BuildTableau(model, out var basis, out var varNames);

            // 3) Make the z-row dual-feasible (all reduced costs >= 0) AND update basis during those pivots
            ForceDualFeasibility(T, basis);

            // 4) Show initial tableau
            // Show initial tableau (highlight z-row)
            // Show initial tableau
            var sb0 = new StringBuilder();
            AppendTableau(sb0, T, basis, varNames, 0);
            updatePivot?.Invoke(sb0.ToString(), null);



            // 5) Dual simplex iterations: fix negative RHS while maintaining z-row >= 0
            int iter = 1;
            while (true)
            {
                if (iter > 10000) throw new Exception("Iteration limit exceeded (Dual Simplex).");

                int m = T.GetLength(0) - 1;     // constraint rows
                int nNoRhs = T.GetLength(1) - 1; // columns excluding RHS
                int rhsCol = nNoRhs;

                // Choose leaving row: most negative RHS
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

                // If none negative -> primal feasible; since z-row is >= 0, we're optimal
                if (leave == -1)
                {

                    // primal feasible now; optimal (dual feasible was maintained)
                    // print final tableau with Z row (row 0) highlighted
                    var sbFinal = new StringBuilder();
                    AppendTableau(sbFinal, T, basis, varNames, iter);

                    bool[,] highlightFinal = new bool[T.GetLength(0), T.GetLength(1)];
                    int zRowIndex = 0; // <-- Z row is the first row
                    for (int j = 0; j < T.GetLength(1); j++)
                        highlightFinal[zRowIndex, j] = true;

                    updatePivot?.Invoke(sbFinal.ToString(), highlightFinal);

                    return FinalizeReport(new StringBuilder(), T, basis, varNames, "OPTIMAL");
                }

                // Choose entering column j minimizing z_j / |a_{leave,j}| over a_{leave,j} < 0
                int enter = -1;
                double bestRatio = double.PositiveInfinity;
                for (int j = 0; j < nNoRhs; j++)
                {
                    double a = T[leave, j];
                    if (a < -Eps)
                    {
                        double ratio = T[m, j] / (-a);
                        if (ratio < bestRatio - 1e-12)
                        {
                            bestRatio = ratio;
                            enter = j;
                        }
                    }
                }
                if (enter == -1)
                {
                    return FinalizeReport(new StringBuilder().AppendLine("INFEASIBLE (no entering column found)"),
                        T, basis, varNames, "INFEASIBLE");
                }

                // Pivot and update basis
                Pivot(T, leave, enter);
                basis[leave] = enter;

                // Print iteration with highlight
                var sb = new StringBuilder();
                AppendTableau(sb, T, basis, varNames, iter);

                var highlight = new bool[m + 1, nNoRhs + 1];
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

            // Convert to Max
            if (model.ObjectiveSense == Sense.Min)
                for (int i = 0; i < model.C.Length; i++) model.C[i] = -model.C[i];

            // Build expanded model with only <= and b >= 0; split '='
            var expanded = new LPProblem { C = (double[])model.C.Clone(), ObjectiveSense = Sense.Max };
            foreach (var cons in model.Constraints)
            {
                if (cons.Relation == Rel.EQ)
                {
                    // +row <= b
                    expanded.Constraints.Add(new Constraint { A = (double[])cons.A.Clone(), B = cons.B, Relation = Rel.LE });
                    // -row <= -b
                    var neg = new Constraint { A = (double[])cons.A.Clone(), B = -cons.B, Relation = Rel.LE };
                    for (int j = 0; j < neg.A.Length; j++) neg.A[j] *= -1;
                    expanded.Constraints.Add(neg);
                }
                else
                {
                    var row = cons.Clone();
                    if (row.Relation == Rel.GE)
                    {
                        // multiply by -1 to turn >= into <=
                        for (int j = 0; j < row.A.Length; j++) row.A[j] *= -1;
                        row.B *= -1;
                        row.Relation = Rel.LE;
                    }
                    if (row.B < -Eps)
                    {
                        // ensure b >= 0
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

            // constraints
            for (int i = 0; i < m; i++)
            {
                var row = model.Constraints[i];
                for (int j = 0; j < n; j++) T[i, j] = row.A[j];
                T[i, n + i] = 1.0;                // slack
                T[i, n + s] = row.B;              // RHS
            }

            // z row = -c for decision vars
            for (int j = 0; j < n; j++) T[m, j] = -model.C[j];

            // initial basis = slacks
            basis = Enumerable.Range(n, s).ToArray();

            // variable names (x then c)
            varNames = new string[n + s];
            for (int j = 0; j < n; j++) varNames[j] = $"x{j + 1}";
            for (int j = 0; j < s; j++) varNames[n + j] = $"c{j + 1}";

            return T;
        }

        /// <summary>
        /// Ensures z-row >= 0 by performing primal pivots if needed.
        /// IMPORTANT: updates the basis array alongside tableau.
        /// </summary>
        private static void ForceDualFeasibility(double[,] T, int[] basis)
        {
            int m = T.GetLength(0) - 1;
            int rhsCol = T.GetLength(1) - 1;
            int nNoRhs = rhsCol;

            // Loop until all reduced costs are >= 0 or until we hit a guard
            for (int guard = 0; guard < 100; guard++)
            {
                int entering = -1;
                double minVal = -Eps;
                for (int j = 0; j < nNoRhs; j++)
                    if (T[m, j] < minVal) { minVal = T[m, j]; entering = j; }

                if (entering == -1) return; // already dual-feasible

                // choose leaving by primal ratio test (like primal simplex)
                int leave = -1;
                double bestRatio = double.PositiveInfinity;
                for (int i = 0; i < m; i++)
                {
                    double a = T[i, entering];
                    if (a > Eps)
                    {
                        double ratio = T[i, rhsCol] / a;
                        if (ratio < bestRatio - 1e-12) { bestRatio = ratio; leave = i; }
                    }
                }
                if (leave == -1) return; // cannot fix via primal step; stop and let dual phase handle

                Pivot(T, leave, entering);
                basis[leave] = entering;  // ✅ keep basis consistent with tableau
            }
        }
        #endregion

        #region Core ops / printing
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
            int m = T.GetLength(0) - 1;         // constraints
            int ns = T.GetLength(1) - 1;        // columns excluding RHS
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
            int n = varNames.Count(v => v.StartsWith("x")); // decision variables
            var x = new double[n];

            // Assign RHS to any decision variable that is currently basic
            for (int i = 0; i < m; i++)
            {
                int basicCol = basis[i];
                if (basicCol < n)
                {
                    x[basicCol] = T[i, T.GetLength(1) - 1];
                }
            }

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
