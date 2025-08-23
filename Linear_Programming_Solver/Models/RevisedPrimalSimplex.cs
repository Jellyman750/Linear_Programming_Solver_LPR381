using System;
using System.Linq;
using System.Text;
using System.Collections.Generic;

namespace Linear_Programming_Solver.Models
{
    /// <summary>
    /// Revised Primal Simplex (price-out). 
    /// Shows product-form details by printing B^{-1}, reduced costs (pricing), search direction (d), ratios, and the basic solution each iteration.
    /// </summary>
    internal class RevisedPrimalSimplex : ILPAlgorithm
    {
        private const double Eps = 1e-9;
        private const int MaxIterations = 10000;

        public SimplexResult Solve(LPProblem original, Action<string, bool[,]> updatePivot = null)
        {
            // 1) Make a working clone and convert to Max with <= and b >= 0 (slacks start basic)
            var model = Standardize(original);

            // Build A|I, c, b
            int m = model.Constraints.Count;
            int n = model.NumVars;          // original variables (x)
            int Ntot = n + m;               // x + slacks
            double[,] A = new double[m, Ntot];
            double[] c = new double[Ntot];
            double[] b = new double[m];

            // Fill A for decision vars
            for (int i = 0; i < m; i++)
            {
                var row = model.Constraints[i];
                for (int j = 0; j < n; j++) A[i, j] = row.A[j];
                A[i, n + i] = 1.0; // slack
                b[i] = row.B;
            }
            for (int j = 0; j < n; j++) c[j] = model.C[j];
            // c for slacks are 0 by default

            // Initial basis = slacks
            var Bidx = Enumerable.Range(n, m).ToArray();      // basic column indices
            var Nidx = Enumerable.Range(0, n).ToList();       // nonbasic start with decision vars

            var names = new string[Ntot];
            for (int j = 0; j < n; j++) names[j] = $"x{j + 1}";
            for (int j = 0; j < m; j++) names[n + j] = $"c{j + 1}";

            var report = new StringBuilder();
            report.AppendLine("=== Revised Primal Simplex (Product-Form / Price-Out) ===");

            // Initial B^{-1}, x_B, z
            var Binv = Invert(GetSubmatrix(A, Bidx)); // Gauss-Jordan (simple & robust)
            var xB = Multiply(Binv, b);
            var cB = GetSubvector(c, Bidx);
            double z = Dot(cB, xB);

            // Show iteration 0
            updatePivot?.Invoke(BuildIterationBlock(0, Bidx, Nidx, names, Binv, xB, z, null, null, null, null), null);

            for (int iter = 1; iter <= MaxIterations; iter++)
            {
                // Price out: r_N = c_N - c_B^T B^{-1} N
                var Nmat = GetSubmatrix(A, Nidx.ToArray());
                var cN = GetSubvector(c, Nidx.ToArray());
                var piT = MultiplyRow(cB, Binv);          // c_B^T * B^{-1}  -> vector length m
                var rN = Subtract(cN, MultiplyRow(piT, Nmat)); // vector length |N|


                // Choose entering with most negative reduced cost
                int enteringPos = -1; // position in Nidx list
                double minRC = -Eps;
                for (int j = 0; j < rN.Length; j++)
                    if (rN[j] < minRC)
                    {
                        minRC = rN[j];
                        enteringPos = j;
                    }
                if (enteringPos == -1)
                {
                    // Optimal
                    var final = BuildFinalSummary(Bidx, names, xB, z, "OPTIMAL");
                    report.Append(final.Report);
                    return final;
                }

                int entering = Nidx[enteringPos];

                // Direction d = B^{-1} * A[:, entering]
                var aEntering = GetColumn(A, entering);
                var d = Multiply(Binv, aEntering);

                // Ratio test theta = min_i xB_i / d_i where d_i > 0
                int leaveRow = -1;
                double bestTheta = double.PositiveInfinity;
                for (int i = 0; i < m; i++)
                {
                    if (d[i] > Eps)
                    {
                        double theta = xB[i] / d[i];
                        if (theta < bestTheta - 1e-12)
                        {
                            bestTheta = theta;
                            leaveRow = i;
                        }
                    }
                }
                if (leaveRow == -1)
                {
                    var final = BuildFinalSummary(Bidx, names, xB, z, "UNBOUNDED");
                    report.Append(final.Report);
                    return final;
                }

                // Update basis indices (pivot)
                int leaving = Bidx[leaveRow];
                Bidx[leaveRow] = entering;
                Nidx.RemoveAt(enteringPos);
                Nidx.Add(leaving);

                // Product-form update (implemented by recomputing B^{-1} for clarity & stability)
                // In teaching mode we still print it as "product form (updated B^{-1})"
                Binv = Invert(GetSubmatrix(A, Bidx)); // For production, swap in Eta-matrix chain if desired

                // Update x_B and objective
                cB = GetSubvector(c, Bidx);
                xB = Multiply(Binv, b);
                z = Dot(cB, xB);

                // Show iteration block
                var highlight = new bool[m, 4]; // 4 columns in our printed vector table below; this is only to make pivot row stand out
                for (int j = 0; j < 4; j++) highlight[leaveRow, j] = true;
                updatePivot?.Invoke(
                    BuildIterationBlock(iter, Bidx, Nidx, names, Binv, xB, z, rN, entering, d, bestTheta),
                    highlight
                );
            }

            throw new Exception("Iteration limit exceeded in Revised Primal Simplex.");
        }

        #region Standardization
        private static LPProblem Standardize(LPProblem original)
        {
            var model = original.Clone();

            // Maximize
            if (model.ObjectiveSense == Sense.Min)
                for (int i = 0; i < model.C.Length; i++) model.C[i] = -model.C[i];

            // Convert GE to LE by multiplying by -1; ensure b >= 0
            for (int i = 0; i < model.Constraints.Count; i++)
            {
                var c = model.Constraints[i];
                if (c.Relation == Rel.GE)
                {
                    for (int j = 0; j < c.A.Length; j++) c.A[j] *= -1;
                    c.B *= -1;
                    c.Relation = Rel.LE;
                }
                if (c.Relation == Rel.EQ)
                {
                    // split equality into two <= rows
                    var pos = new Constraint { A = (double[])c.A.Clone(), B = c.B, Relation = Rel.LE };
                    var neg = new Constraint { A = (double[])c.A.Clone(), B = -c.B, Relation = Rel.LE };
                    for (int j = 0; j < neg.A.Length; j++) neg.A[j] *= -1;
                    model.Constraints.RemoveAt(i);
                    model.Constraints.Insert(i, neg);
                    model.Constraints.Insert(i, pos);
                    i++; // skip over the inserted pair
                    continue;
                }
                if (c.B < -Eps)
                {
                    for (int j = 0; j < c.A.Length; j++) c.A[j] *= -1;
                    c.B *= -1;
                }
            }
            return model;
        }
        #endregion

        #region Printing
        private static string BuildIterationBlock(
            int iter,
            int[] Bidx,
            List<int> Nidx,
            string[] names,
            double[,] Binv,
            double[] xB,
            double z,
            double[] rN,                 // may be null on iteration 0
            int? entering,               // may be null on iteration 0
            double[] d,                  // may be null on iteration 0
            double? bestTheta)           // may be null on iteration 0
        {
            var sb = new StringBuilder();
            sb.AppendLine($"=== Revised Simplex Iteration {iter} ===");

            // Basis summary
            sb.AppendLine("Basis: " + string.Join(", ", Bidx.Select(k => names[k])));
            sb.AppendLine("Nonbasic: " + string.Join(", ", Nidx.Select(k => names[k])));

            // B^{-1}
            sb.AppendLine("\nProduct-form: current B^{-1}");
            sb.Append(MatrixToString(Binv));

            // Basic solution & objective
            sb.AppendLine("x_B = [" + string.Join(", ", xB.Select(v => v.ToString("0.###"))) + "]");
            sb.AppendLine($"z = {z:0.###}");

            // Price-out info
            if (rN != null)
            {
                sb.AppendLine("\nReduced costs r_N = c_N - c_B^T B^{-1} N:");
                for (int j = 0; j < rN.Length; j++)
                    sb.AppendLine($"  r({Nidx[j]}:{names[Nidx[j]]}) = {rN[j]:0.###}");
            }

            if (entering.HasValue)
                sb.AppendLine($"\nEntering variable: {names[entering.Value]}");

            if (d != null)
            {
                sb.AppendLine("Direction d = B^{-1} * a_entering:");
                sb.AppendLine("  d = [" + string.Join(", ", d.Select(v => v.ToString("0.###"))) + "]");

                sb.AppendLine("\nRatio test (theta):");
                // We cannot print xB/d per-row with names unless we know which row corresponds to which basic
                for (int i = 0; i < d.Length; i++)
                {
                    if (d[i] > Eps) sb.AppendLine($"  row {i + 1}: {xB[i]:0.###} / {d[i]:0.###} = {(xB[i] / d[i]):0.###}");
                    else sb.AppendLine($"  row {i + 1}: d_i <= 0 (skip)");
                }
                if (bestTheta.HasValue) sb.AppendLine($"Chosen theta* = {bestTheta.Value:0.###}");
            }

            sb.AppendLine();
            return sb.ToString();
        }

        private static string MatrixToString(double[,] M)
        {
            int r = M.GetLength(0), c = M.GetLength(1);
            int w = 12;
            var sb = new StringBuilder();
            for (int i = 0; i < r; i++)
            {
                for (int j = 0; j < c; j++)
                    sb.Append(M[i, j].ToString("0.###").PadLeft(w));
                sb.AppendLine();
            }
            return sb.ToString();
        }

        private static SimplexResult BuildFinalSummary(int[] Bidx, string[] names, double[] xB, double z, string status)
        {
            // Map basic x_B back to x variables only
            int nVars = names.Count(s => s.StartsWith("x"));
            var x = new double[nVars];
            for (int i = 0; i < Bidx.Length; i++)
            {
                int col = Bidx[i];
                if (col < nVars) x[col] = xB[i];
            }

            var sb = new StringBuilder();
            sb.AppendLine($"\nStatus: {status}");
            for (int j = 0; j < nVars; j++) sb.AppendLine($"  x{j + 1} = {Math.Round(x[j], 3):0.###}");
            sb.AppendLine($"  z* = {Math.Round(z, 3):0.###}");

            var summary = new StringBuilder();
            summary.AppendLine($"Status: {status}");
            summary.AppendLine($"z* = {Math.Round(z, 3):0.###}");
            summary.AppendLine("x* = [" + string.Join(", ", x.Select(v => Math.Round(v, 3))) + "]");

            return new SimplexResult { Report = sb.ToString(), Summary = summary.ToString() };
        }
        #endregion

        #region Linear Algebra helpers
        private static double[,] GetSubmatrix(double[,] A, int[] cols)
        {
            int m = A.GetLength(0);
            int k = cols.Length;
            var M = new double[m, k];
            for (int i = 0; i < m; i++)
                for (int j = 0; j < k; j++)
                    M[i, j] = A[i, cols[j]];
            return M;
        }

        private static double[] GetSubvector(double[] v, int[] idx)
        {
            var r = new double[idx.Length];
            for (int i = 0; i < idx.Length; i++) r[i] = v[idx[i]];
            return r;
        }

        private static double[] GetColumn(double[,] A, int col)
        {
            int m = A.GetLength(0);
            var v = new double[m];
            for (int i = 0; i < m; i++) v[i] = A[i, col];
            return v;
        }

        private static double[] Multiply(double[,] A, double[] x)
        {
            int r = A.GetLength(0), c = A.GetLength(1);
            var y = new double[r];
            for (int i = 0; i < r; i++)
            {
                double s = 0;
                for (int j = 0; j < c; j++) s += A[i, j] * x[j];
                y[i] = s;
            }
            return y;
        }

        private static double[,] Multiply(double[] row, double[,] A)
        {
            // row (1xm) * A (mxn) => (1xn) as row-vector in 2D [1,n]
            int m = A.GetLength(0), n = A.GetLength(1);
            var y = new double[1, n];
            for (int j = 0; j < n; j++)
            {
                double s = 0;
                for (int i = 0; i < m; i++) s += row[i] * A[i, j];
                y[0, j] = s;
            }
            return y;
        }

        private static double[] Multiply(double[,] row1xn, double[,] A) // where first is actually [1,n]
        {
            int n = row1xn.GetLength(1);
            int m = A.GetLength(1); // columns count (since 1 x n * n x m)
            var y = new double[m];
            for (int j = 0; j < m; j++)
            {
                double s = 0;
                for (int i = 0; i < n; i++) s += row1xn[0, i] * A[i, j];
                y[j] = s;
            }
            return y;
        }
        private static double[] MultiplyRow(double[] row, double[,] A)
        {
            // row(1xm) * A(mxn) => vector length n
            int m = A.GetLength(0), n = A.GetLength(1);
            if (row.Length != m) throw new Exception("Row length mismatch in MultiplyRow.");
            var y = new double[n];
            for (int j = 0; j < n; j++)
            {
                double s = 0;
                for (int i = 0; i < m; i++) s += row[i] * A[i, j];
                y[j] = s;
            }
            return y;
        }


        private static double Dot(double[] a, double[] b)
        {
            double s = 0;
            for (int i = 0; i < a.Length; i++) s += a[i] * b[i];
            return s;
        }

        private static double[] Subtract(double[] a, double[] b)
        {
            var r = new double[a.Length];
            for (int i = 0; i < a.Length; i++) r[i] = a[i] - b[i];
            return r;
        }

        private static double[,] Transpose(double[] v) // returns [1,n] row
        {
            var M = new double[1, v.Length];
            for (int j = 0; j < v.Length; j++) M[0, j] = v[j];
            return M;
        }

        private static double[,] Invert(double[,] M)
        {
            int n = M.GetLength(0);
            int m = M.GetLength(1);
            if (n != m) throw new Exception("Matrix not square; cannot invert B.");

            // Augment with identity and Gauss-Jordan
            var A = new double[n, 2 * n];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++) A[i, j] = M[i, j];
                A[i, n + i] = 1.0;
            }

            for (int col = 0; col < n; col++)
            {
                // Pivot
                int pivotRow = col;
                double best = Math.Abs(A[pivotRow, col]);
                for (int r = col + 1; r < n; r++)
                {
                    double v = Math.Abs(A[r, col]);
                    if (v > best) { best = v; pivotRow = r; }
                }
                if (Math.Abs(A[pivotRow, col]) < Eps) throw new Exception("Singular basis encountered.");

                if (pivotRow != col)
                    for (int j = 0; j < 2 * n; j++)
                    {
                        double tmp = A[col, j];
                        A[col, j] = A[pivotRow, j];
                        A[pivotRow, j] = tmp;
                    }

                // Scale to 1
                double piv = A[col, col];
                for (int j = 0; j < 2 * n; j++) A[col, j] /= piv;

                // Eliminate
                for (int r = 0; r < n; r++)
                {
                    if (r == col) continue;
                    double factor = A[r, col];
                    for (int j = 0; j < 2 * n; j++) A[r, j] -= factor * A[col, j];
                }
            }

            // Extract right half
            var inv = new double[n, n];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    inv[i, j] = A[i, n + j];

            return inv;
        }
        #endregion
    }
}