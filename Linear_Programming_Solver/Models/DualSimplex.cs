using System;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;

namespace Linear_Programming_Solver.Models
{
    /// <summary>
    /// Dual Simplex using tableau updates.
    /// Starts from a dual-feasible tableau (z-row nonnegative) and fixes infeasible RHS (negative b).
    /// </summary>
    internal class DualSimplex : ILPAlgorithm
    {
        private const double Eps = 1e-9;
        private const int MaxIterations = 10000;

        public SimplexResult Solve(LPProblem original, Action<string, bool[,]> updatePivot = null)
        {
            // Prepare problem: convert to max, <= constraints, b >= 0
            var model = PrepareForTableau(original);
            var T = BuildTableau(model, out var basis, out var varNames);

            // Ensure dual feasibility (z-row nonnegative reduced costs)
            ForceDualFeasibility(T, updatePivot);

            // Show initial tableau
            var sb0 = new StringBuilder();
            AppendTableau(sb0, T, basis, varNames, 0);
            updatePivot?.Invoke(sb0.ToString(), null);

            int iter = 1;
            while (true)
            {
                if (iter > MaxIterations)
                    throw new Exception("Iteration limit exceeded (Dual Simplex).");

                int m = T.GetLength(0) - 1;       // number of constraints
                int nNoRhs = T.GetLength(1) - 1;  // number of variables + slacks

                // Choose leaving row: most negative RHS (infeasible basic variable)
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
                    // Primal feasible now; optimal (dual feasible maintained)
                    return FinalizeReport(new StringBuilder(), T, basis, varNames, "OPTIMAL");
                }

                // Choose entering column j that minimizes ratio (reduced cost / |a_leave,j|) over a_leave,j < 0
                int enter = -1;
                double bestRatio = double.PositiveInfinity;
                for (int j = 0; j < nNoRhs; j++)
                {
                    double a = T[leave, j];
                    if (a < -Eps)
                    {
                        double ratio = T[m, j] / (-a); // reduced cost / |a|
                        if (ratio < bestRatio - 1e-12)
                        {
                            bestRatio = ratio;
                            enter = j;
                        }
                    }
                }

                if (enter == -1)
                {
                    // No entering variable found → problem infeasible
                    var report = new StringBuilder();
                    report.AppendLine("INFEASIBLE (dual step found no entering variable)");
                    return FinalizeReport(report, T, basis, varNames, "INFEASIBLE");
                }

                // Perform pivot operation
                Pivot(T, leave, enter);
                basis[leave] = enter;

                // Build tableau string with highlights for pivot row and column
                var sb = new StringBuilder();
                AppendTableau(sb, T, basis, varNames, iter);

                var highlight = new bool[m + 1, nNoRhs + 1]; // including z-row
                for (int j = 0; j < nNoRhs + 1; j++) highlight[leave, j] = true;
                for (int i = 0; i < m + 1; i++) highlight[i, enter] = true;

                updatePivot?.Invoke(sb.ToString(), highlight);

                iter++;
            }
        }

        #region Preparation and Tableau Construction

        private static LPProblem PrepareForTableau(LPProblem original)
        {
            var model = original.Clone();

            // Convert minimization to maximization by negating objective coefficients
            if (model.ObjectiveSense == Sense.Min)
                for (int i = 0; i < model.C.Length; i++)
                    model.C[i] = -model.C[i];

            // Convert all constraints to <= form with b >= 0 by flipping rows if needed
            var expanded = new LPProblem { C = (double[])model.C.Clone() };
            foreach (var cons in model.Constraints)
            {
                if (cons.Relation == Rel.EQ)
                {
                    // Equality constraint split into two inequalities
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
                        // Flip GE to LE by multiplying by -1
                        for (int j = 0; j < row.A.Length; j++) row.A[j] *= -1;
                        row.B *= -1;
                        row.Relation = Rel.LE;
                    }
                    if (row.B < -Eps)
                    {
                        // Flip row to ensure b >= 0
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
            int s = m; // number of slack variables

            var T = new double[m + 1, n + s + 1];

            // Fill constraints rows
            for (int i = 0; i < m; i++)
            {
                var row = model.Constraints[i];
                for (int j = 0; j < n; j++)
                    T[i, j] = row.A[j];
                T[i, n + i] = 1.0; // slack variable coefficient
                T[i, n + s] = row.B; // RHS
            }

            // Fill objective row (z-row)
            for (int j = 0; j < n; j++)
                T[m, j] = -model.C[j];

            // Initial basis is slack variables
            basis = Enumerable.Range(n, s).ToArray();

            // Variable names for display
            varNames = new string[n + s];
            for (int j = 0; j < n; j++) varNames[j] = $"x{j + 1}";
            for (int j = 0; j < s; j++) varNames[n + j] = $"c{j + 1}";

            return T;
        }

        /// <summary>
        /// Ensures dual feasibility by performing primal pivots if z-row has negative reduced costs.
        /// </summary>
        private static void ForceDualFeasibility(double[,] T, Action<string, bool[,]> updatePivot = null)
        {
            int m = T.GetLength(0) - 1;
            int rhsCol = T.GetLength(1) - 1;
            int nNoRhs = rhsCol;

            for (int guard = 0; guard < 50; guard++)
            {
                int entering = -1;
                double minVal = -Eps;
                for (int j = 0; j < nNoRhs; j++)
                {
                    if (T[m, j] < minVal)
                    {
                        minVal = T[m, j];
                        entering = j;
                    }
                }
                if (entering == -1) return; // dual feasible

                int leave = -1;
                double bestRatio = double.PositiveInfinity;
                for (int i = 0; i < m; i++)
                {
                    if (T[i, entering] > Eps)
                    {
                        double ratio = T[i, rhsCol] / T[i, entering];
                        if (ratio < bestRatio - 1e-12)
                        {
                            bestRatio = ratio;
                            leave = i;
                        }
                    }
                }
                if (leave == -1)
                {
                    // Unbounded in primal, but dual simplex will handle later
                    return;
                }

                Pivot(T, leave, entering);

                // Optional: log pivot during force dual feasibility
                if (updatePivot != null)
                {
                    var sb = new StringBuilder();
                    AppendTableau(sb, T, Enumerable.Range(T.GetLength(1) - 1 - m, m).ToArray(), null, -1);
                    updatePivot(sb.ToString(), null);
                }
            }
        }

        #endregion

        #region Core Operations and Output

        private static void Pivot(double[,] T, int row, int col)
        {
            int R = T.GetLength(0);
            int C = T.GetLength(1);

            double piv = T[row, col];
            if (Math.Abs(piv) < Eps)
                throw new Exception("Pivot element is zero or too close to zero.");

            for (int j = 0; j < C; j++)
                T[row, j] /= piv;

            for (int i = 0; i < R; i++)
            {
                if (i == row) continue;
                double f = T[i, col];
                for (int j = 0; j < C; j++)
                    T[i, j] -= f * T[row, j];
            }
        }

        private static void AppendTableau(StringBuilder sb, double[,] T, int[] basis, string[] varNames, int iter)
        {
            int m = T.GetLength(0) - 1; // constraints count
            int ns = T.GetLength(1) - 1; // variables + slacks count
            int colWidth = 12;

            string PadString(string s) => s.PadLeft(colWidth);
            string PadDouble(double d) => d.ToString("0.###").PadLeft(colWidth);

            sb.AppendLine($"DUAL SIMPLEX TABLEAU Iteration {iter}");

            sb.Append(PadString("Basis"));
            for (int j = 0; j < ns; j++)
                sb.Append(PadString(varNames != null ? varNames[j] : $"v{j + 1}"));
            sb.Append(PadString("RHS"));
            sb.AppendLine();

            string dash = new string('-', colWidth * (ns + 2));
            sb.AppendLine(dash);

            // z row
            sb.Append(PadString("z"));
            for (int j = 0; j < ns; j++)
                sb.Append(PadDouble(T[m, j]));
            sb.Append(PadDouble(T[m, ns]));
            sb.AppendLine();

            // constraints rows
            for (int i = 0; i < m; i++)
            {
                string basisName = (basis != null && varNames != null && basis.Length > i && basis[i] < varNames.Length)
                    ? varNames[basis[i]]
                    : $"b{i + 1}";
                sb.Append(PadString(basisName));
                for (int j = 0; j < ns; j++)
                    sb.Append(PadDouble(T[i, j]));
                sb.Append(PadDouble(T[i, ns]));
                sb.AppendLine();
            }
        }

        private static SimplexResult FinalizeReport(StringBuilder report, double[,] T, int[] basis, string[] varNames, string status)
        {
            int m = T.GetLength(0) - 1;
            int n = varNames?.Count(v => v.StartsWith("x")) ?? 0;
            var x = new double[n];

            for (int i = 0; i < m; i++)
            {
                if (basis[i] < n)
                    x[basis[i]] = T[i, T.GetLength(1) - 1];
            }

            double z = T[m, T.GetLength(1) - 1];

            report.AppendLine($"\nStatus: {status}");
            for (int j = 0; j < n; j++)
                report.AppendLine($"  x{j + 1} = {Math.Round(x[j], 6):0.######}");
            report.AppendLine($"  z* = {Math.Round(z, 6):0.######}");

            var summary = new StringBuilder();
            summary.AppendLine($"Status: {status}");
            summary.AppendLine($"z* = {Math.Round(z, 6):0.######}");
            summary.AppendLine("x* = [" + string.Join(", ", x.Select(v => Math.Round(v, 6))) + "]");

            return new SimplexResult { Report = report.ToString(), Summary = summary.ToString() };
        }

        #endregion
    }
}