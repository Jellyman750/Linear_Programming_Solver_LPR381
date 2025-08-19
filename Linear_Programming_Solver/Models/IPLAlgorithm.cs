using System;

namespace Linear_Programming_Solver.Models
{
    public interface ILPAlgorithm
    {
        SimplexResult Solve(LPProblem problem, Action<string, bool[,]> updatePivot = null);
    }
}

