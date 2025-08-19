using System.Text;
using Linear_Programming_Solver.Models;

public class BranchAndBound : ILPAlgorithm
{
    private LPSolver solver = new LPSolver();
    private int subProblemCounter = 1;
    public double BestObjective { get; private set; } = double.NegativeInfinity;
    public double[] BestSolution { get; private set; }
    private string _relaxationAlgorithm = "Primal Simplex";

    public SimplexResult Solve(LPProblem problem, Action<string, bool[,]> updatePivot = null)
    {
        subProblemCounter = 1;
        BestObjective = double.NegativeInfinity;
        BestSolution = null;

        SolveSubproblem(problem, "Root Problem", updatePivot);

        var sb = new StringBuilder();
        sb.AppendLine("Branch & Bound Finished.");
        sb.AppendLine($"Best z* = {BestObjective:0.###}");
        sb.AppendLine("Best x* = [" + (BestSolution != null ? string.Join(", ", BestSolution.Select(v => v.ToString("0.###"))) : "") + "]");

        return new SimplexResult
        {
            Report = sb.ToString(),
            Summary = sb.ToString()
        };
    }

    public void SetRelaxationAlgorithm(string algorithm)
    {
        _relaxationAlgorithm = algorithm.Trim();
    }

    private void SolveSubproblem(LPProblem problem, string name, Action<string, bool[,]> updatePivot)
    {
        updatePivot?.Invoke($"\n{name}: Solving LP relaxation...\n", null);

        // Solve LP relaxation with chosen algorithm
        var result = solver.Solve(problem, _relaxationAlgorithm, updatePivot);

        // Extract solution vector and objective value
        double[] x = ParseSolutionVector(result.Summary);
        double zValue = ParseObjectiveValue(result.Summary);

        updatePivot?.Invoke($"{name} LP solution: z* = {zValue:0.###}, x* = [{string.Join(", ", x.Select(v => v.ToString("0.###")))}]\n", null);

        // Prune if objective is worse than best known integer solution
        if (zValue < BestObjective - 1e-6)
        {
            updatePivot?.Invoke($"{name} pruned: z* = {zValue:0.###} < BestObjective = {BestObjective:0.###}\n", null);
            return;
        }

        // Check if integer feasible
        int fractionalIndex = -1;
        for (int i = 0; i < x.Length; i++)
        {
            if (Math.Abs(x[i] - Math.Round(x[i])) > 1e-6)
            {
                fractionalIndex = i;
                break;
            }
        }

        if (fractionalIndex == -1)
        {
            // Integer solution → update best if better
            if (zValue > BestObjective)
            {
                BestObjective = zValue;
                BestSolution = x;
                updatePivot?.Invoke($"{name} is integer feasible. z* = {zValue:0.###}\n", null);
            }
            return;
        }

        // Branch on fractional variable
        int varIndex = fractionalIndex;
        double fracValue = x[varIndex];

        // Subproblem 1: x <= floor
        var sub1 = problem.Clone();
        sub1.Constraints.Add(new Constraint
        {
            A = UnitVector(problem.NumVars, varIndex),
            Relation = Rel.LE,
            B = Math.Floor(fracValue)
        });

        // Subproblem 2: x >= ceil
        var sub2 = problem.Clone();
        sub2.Constraints.Add(new Constraint
        {
            A = UnitVector(problem.NumVars, varIndex),
            Relation = Rel.GE,
            B = Math.Ceiling(fracValue)
        });

        // Recursively solve subproblems
        SolveSubproblem(sub1, $"Subproblem {subProblemCounter++}: x{varIndex + 1} <= {Math.Floor(fracValue)}", updatePivot);
        SolveSubproblem(sub2, $"Subproblem {subProblemCounter++}: x{varIndex + 1} >= {Math.Ceiling(fracValue)}", updatePivot);
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
