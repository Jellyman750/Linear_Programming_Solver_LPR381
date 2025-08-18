# Linear & Integer Programming Solver – C# Console Application

##  Project Overview
This project is a **menu-driven C# console application** (`solve.exe`) that solves:
- **Linear Programming (LP) problems**
- **Integer Programming (IP) problems** (binary, integer)
- **Knapsack problems**
- **Sensitivity Analysis** & **Duality**

It implements multiple algorithms, reads input from a text file, and outputs results to a text file in the required format.

---

##  Features
- **File I/O Support** – Reads model specifications from an input file and writes results to an output file.
- **Multiple Algorithms**:
  - Primal Simplex Algorithm
  - Revised Primal Simplex Algorithm
  - Branch & Bound Simplex Algorithm
  - Cutting Plane Algorithm (optional)
  - Branch & Bound Knapsack Algorithm
- **Sensitivity Analysis** with multiple operations
- **Duality Checking** – Solve dual models and verify strong/weak duality
- **Special Case Handling** – Detects infeasible and unbounded models
- **Bonus Feature** – Non-linear problem solving (optional)
- **User-Friendly Menu System** in the terminal

---

## Technology Stack
- **Language:** C# (.NET)
- **Project Type:** Console Application
- **IDE:** Visual Studio
- **Executable Output:** `solve.exe`

---

📂 Project Structure
/LinearProgrammingSolver
│
├── Program.cs # Main entry point with menu system
├── FileHandler.cs # Handles input/output file reading/writing
├── Algorithms/
│ ├── PrimalSimplex.cs
│ ├── RevisedPrimalSimplex.cs
│ ├── BranchBoundSimplex.cs
│ ├── BranchBoundKnapsack.cs
│ └── SensitivityAnalysis.cs
├── Input/
│ └── example_input.txt # Example problem definition
├── Output/
│ └── example_output.txt # Example solution output
└── README.md # This file
