using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Linear_Programming_Solver.Models;

namespace Linear_Programming_Solver.Controllers
{
    public static class LPController
    {
        public static SimplexResult SolvePrimalSimplex(LPProblem problem)
        {
            var solver = new LPSolver();
            return solver.Solve(problem);
        }
    }
}

