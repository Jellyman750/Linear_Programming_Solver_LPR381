using System.Text;
using Linear_Programming_Solver.Models;
using System.Linq;
using System;

public class BranchAndBound : ILPAlgorithm
{
    private LPSolver solver = new LPSolver();
    private int subProblemCounter = 1;
    private const double EPS = 1e-6;

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

        // Choose the algorithm based on constraints
        _relaxationAlgorithm = problem.Constraints.Any(c => c.Relation == Rel.GE)
            ? "Dual Simplex"
            : "Primal Simplex";

        updatePivot?.Invoke($"Branch & Bound: Using {_relaxationAlgorithm} for initial LP relaxation.\n", null);

        // Step 1: Solve the LP relaxation first and display the optimal tableau
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

        double[] xLP = ParseSolutionVector(lpResult.Summary);
        double zLP = ParseObjectiveValue(lpResult.Summary);

        updatePivot?.Invoke($"Root Problem LP optimal solution: z* = {zLP:0.###}, x* = [{string.Join(", ", xLP.Select(v => v.ToString("0.###")))}]\n", null);
        updatePivot?.Invoke("Root Problem optimal tableau displayed above.\n", null);

        // Step 2: Begin Branch & Bound recursion to find integer solutions
        SolveSubproblem(problem, "Root Problem (Branching)", updatePivot);

        // Step 3: Build final report
        var sb = new StringBuilder();
        sb.AppendLine("Branch & Bound Finished.");
        if (BestSolution == null)
            sb.AppendLine("No integer-feasible solution found.");
        else
        {
            sb.AppendLine($"Best integer z* = {BestObjective:0.###}");
            sb.AppendLine("Best integer x* = [" + string.Join(", ", BestSolution.Select(v => v.ToString("0.###"))) + "]");
        }

        return new SimplexResult
        {
            Report = sb.ToString(),
            Summary = sb.ToString()
        };
    }

    private void SolveSubproblem(LPProblem problem, string name, Action<string, bool[,]> updatePivot)
    {
        updatePivot?.Invoke($"\n{name}: Solving LP relaxation for branching...\n", null);

        SimplexResult result;
        try
        {
            result = solver.Solve(problem, _relaxationAlgorithm, updatePivot);
        }
        catch (Exception ex)
        {
            updatePivot?.Invoke($"{name}: LP relaxation infeasible or error: {ex.Message}\n", null);
            return;
        }

        double[] x = ParseSolutionVector(result.Summary);
        double zValue = ParseObjectiveValue(result.Summary);

        if (x.Length == 0)
        {
            updatePivot?.Invoke($"{name}: Could not parse solution → prune.\n", null);
            return;
        }

        updatePivot?.Invoke($"{name} LP solution: z* = {zValue:0.###}, x* = [{string.Join(", ", x.Select(v => v.ToString("0.###")))}]\n", null);

        // Prune if objective worse than current best
        if (zValue <= BestObjective + EPS)
        {
            updatePivot?.Invoke($"{name} pruned: z* = {zValue:0.###} <= BestObjective = {BestObjective:0.###}\n", null);
            return;
        }

        // Check integer feasibility
        int fracIndex = -1;
        for (int i = 0; i < x.Length; i++)
        {
            if (Math.Abs(x[i] - Math.Round(x[i])) > EPS)
            {
                fracIndex = i;
                break;
            }
        }

        if (fracIndex == -1)
        {
            // Integer feasible solution
            BestObjective = zValue;
            BestSolution = x;
            updatePivot?.Invoke($"{name} is integer feasible. Updated BestObjective = {BestObjective:0.###}\n", null);
            return;
        }

        // Branch on fractional variable
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

        SolveSubproblem(left, leftName, updatePivot);
        SolveSubproblem(right, rightName, updatePivot);
    }

    private static double[] UnitVector(int size, int index)
    {
        double[] v = new double[size];
        v[index] = 1.0;
        return v;
    }

    private static double[] ParseSolutionVector(string summary)
    {
        int start = summary.IndexOf("x* = [");
        if (start < 0) return Array.Empty<double>();
        start += 6;
        int end = summary.IndexOf("]", start);
        if (end < 0) return Array.Empty<double>();
        string vec = summary.Substring(start, end - start);
        return vec.Split(',').Select(s => double.Parse(s.Trim())).ToArray();
    }

    private static double ParseObjectiveValue(string summary)
    {
        foreach (var line in summary.Split('\n'))
        {
            if (line.StartsWith("z*"))
            {
                var parts = line.Split('=');
                if (parts.Length == 2 && double.TryParse(parts[1].Trim(), out double val))
                    return val;
            }
        }
        return 0;
    }
}
