using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Linear_Programming_Solver.Models
{
    
        public class SubProblem
        {
            public LPProblem Problem { get; set; }
            public string Name { get; set; }  // e.g., "Subproblem 1"
            public double? ObjectiveValue { get; set; } // Store LP relaxation solution
        }
    }

