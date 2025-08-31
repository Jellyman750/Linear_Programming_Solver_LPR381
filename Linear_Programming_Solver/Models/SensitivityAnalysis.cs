using System;
using System.Collections.Generic;
using System.Linq;
using Linear_Programming_Solver.Models;

namespace Linear_Programming_Solver.Analysis
{
    public class SensitivityAnalysis
    {
        private readonly LPProblem problem;
        private readonly SimplexResult result;
        private readonly double[,] tableau;
        private readonly int[] basis;
        private readonly string[] varNames;

        public SensitivityAnalysis(LPProblem problem, SimplexResult result)
        {
            this.problem = problem ?? throw new ArgumentNullException(nameof(problem));
            this.result = result ?? throw new ArgumentNullException(nameof(result));
            this.tableau = result.Tableau;
            this.basis = result.Basis;
            this.varNames = result.VarNames;
        }

        // ===================== PUBLIC METHODS =====================
        public string GetRangeReport(string target)
        {
            if (target.StartsWith("Constraint"))
            {
                int index = int.Parse(target.Split(' ')[1]) - 1;
                var range = GetConstraintRange(index);
                return $"{target}: {range.min:F3} ≤ B ≤ {range.max:F3}";
            }
            else
            {
                int col = Array.IndexOf(varNames, target);
                if (basis.Contains(col))
                {
                    var range = GetBasicVariableObjectiveRange(Array.IndexOf(basis, col));
                    return $"{target} (Basic): {range.min:F3} ≤ c ≤ {range.max:F3}";
                }
                else
                {
                    var range = GetNonBasicVariableRange(target);
                    return $"{target} (Non-Basic): {range.min:F3} ≤ c ≤ {range.max:F3}";
                }
            }
        }

        public string ApplyChange(string target, double value)
        {
            if (target.StartsWith("Constraint"))
            {
                int index = int.Parse(target.Split(' ')[1]) - 1;
                problem.Constraints[index].B = value;
                return $"Constraint {index + 1} B-value updated to {value}";
            }
            else
            {
                int col = Array.IndexOf(varNames, target);
                if (basis.Contains(col))
                    return $"Basic variable {target} change not implemented for objective coefficients.";
                else
                    return $"Non-basic variable {target} change not implemented for objective coefficients.";
            }
        }

        public string GetShadowPricesReport()
        {
            int m = tableau.GetLength(0) - 1;
            string report = "Shadow Prices:\n";
            for (int i = 0; i < basis.Length; i++)
            {
                string bVar = varNames[basis[i]];
                double shadow = tableau[m, basis[i]];
                report += $"  {bVar}: {shadow:F3}\n";
            }
            return report;
        }

        public SimplexResult SolveDualLP()
        {
            int m = problem.Constraints.Count;
            int n = problem.C.Length;

            // Dual objective coefficients = original RHS values (B)
            double[] dualC = new double[m];
            for (int i = 0; i < m; i++)
                dualC[i] = problem.Constraints[i].B;

            List<Constraint> dualConstraints = new List<Constraint>();
            for (int j = 0; j < n; j++)
            {
                double[] row = new double[m];
                for (int i = 0; i < m; i++)
                    row[i] = problem.Constraints[i].A[j];

                dualConstraints.Add(new Constraint
                {
                    A = row,
                    B = problem.C[j],
                    Relation = Rel.LE// assume primal is maximize; adjust if needed
                });
            }

            // Construct dual LP using constructor or object initializer
            LPProblem dualProblem = new LPProblem
            {
                C = dualC,
                Constraints = dualConstraints
            };

            var solver = new LPSolver();
            return solver.Solve(dualProblem, "Primal Simplex", (text, highlight) => { });
        }

        // ===================== PRIVATE METHODS =====================
        private IEnumerable<string> GetNonBasicVariables()
        {
            int n = problem.NumVars;
            for (int j = 0; j < n; j++)
                if (!basis.Contains(j))
                    yield return varNames[j];
        }

        private (double min, double max) GetNonBasicVariableRange(string varName)
        {
            int col = Array.IndexOf(varNames, varName);
            int m = tableau.GetLength(0) - 1;
            double current = tableau[m, col];

            double min = double.NegativeInfinity;
            double max = double.PositiveInfinity;

            for (int i = 0; i < m; i++)
            {
                double coeff = tableau[i, col];
                double rhs = tableau[i, tableau.GetLength(1) - 1];

                if (Math.Abs(coeff) < 1e-9) continue;

                double val = current / coeff;
                if (coeff > 0) max = Math.Min(max, current + val);
                else min = Math.Max(min, current + val);
            }

            return (min, max);
        }

        private (double min, double max) GetBasicVariableObjectiveRange(int basicVarRow)
        {
            int col = basis[basicVarRow];
            int m = tableau.GetLength(0) - 1;
            int n = tableau.GetLength(1) - 1;

            double min = double.NegativeInfinity;
            double max = double.PositiveInfinity;

            for (int j = 0; j < n; j++)
            {
                if (basis.Contains(j)) continue;
                double cj = problem.C[j];
                double aij = tableau[basicVarRow, j];
                double reduced = tableau[m, j];

                if (Math.Abs(aij) < 1e-9) continue;

                double val = reduced / aij;
                if (aij > 0) max = Math.Min(max, cj + val);
                else min = Math.Max(min, cj + val);
            }

            return (min, max);
        }

        private (double min, double max) GetConstraintRange(int index)
        {
            int m = tableau.GetLength(0) - 1;
            int n = tableau.GetLength(1) - 1;

            double[] bCol = new double[basis.Length];
            for (int i = 0; i < basis.Length; i++) bCol[i] = tableau[i, n];

            double min = double.NegativeInfinity;
            double max = double.PositiveInfinity;

            int row = index;

            for (int j = 0; j < basis.Length; j++)
            {
                double aij = tableau[row, basis[j]];
                if (Math.Abs(aij) < 1e-9) continue;

                double val = bCol[j] / aij;
                if (aij > 0) max = Math.Min(max, val);
                else min = Math.Max(min, val);
            }

            return (min, max);
        }
    }
}
