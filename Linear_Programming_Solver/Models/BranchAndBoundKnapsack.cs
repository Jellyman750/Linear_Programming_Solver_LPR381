using System.Text;

namespace Linear_Programming_Solver.Models
{
    /// <summary>
    /// Branch & Bound for 0/1 Knapsack (best-bound / backtracking style).
    /// - Expects exactly one <= constraint (weights / capacity).
    /// - Objective coefficients are profits (problem.C).
    /// - Uses fractional knapsack for upper bounds; expands nodes in best-bound order.
    /// - Logs each subproblem's fractional relaxation, branching, candidates and best candidate.
    /// </summary>
    public class BranchAndBoundKnapsack : ILPAlgorithm
    {
        private class Item
        {
            public int Index;
            public double Profit;
            public double Weight;
            public double Ratio => Weight > 0 ? Profit / Weight : double.PositiveInfinity;
        }

        // Node for best-first search
        private class Node
        {
            public int[] Assigned; // -1 undecided, 0 fixed 0, 1 fixed 1 (original index order)
            public string Label;   // hierarchical label (e.g., "0", "1", "1.2")
            public double Bound;   // fractional relaxation upper bound
            public double RelaxProfit; // relaxation profit (same as bound)
            public double RelaxWeight;
        }

        private StringBuilder _report;
        private Action<string, bool[,]> _log;
        private int _flushPos = 0;

        // Only send the “new” portion of the report each time
        private void FlushDelta()
        {
            if (_log == null) return;

            string full = _report.ToString();
            if (full.Length > _flushPos)
            {
                string delta = full.Substring(_flushPos);
                _flushPos = full.Length;
                _log(delta, null);
            }
        }

        private double _capacity;
        private List<Item> _itemsByRatio;
        private int _n;

        private double _bestValue;
        private int[] _bestX; // 0/1 solution
        private const double EPS = 1e-9;

        public SimplexResult Solve(LPProblem problem, Action<string, bool[,]> updatePivot = null)
        {
            _log = updatePivot ?? ((s, h) => { }); // safe no-op if null
            _report = new StringBuilder();
            _flushPos = 0;

            // Validate
            if (problem == null) throw new ArgumentNullException(nameof(problem));
            if (problem.Constraints == null || problem.Constraints.Count != 1)
                throw new Exception("Knapsack solver requires exactly one constraint (weights and capacity).");
            var cons = problem.Constraints[0];
            if (cons.Relation != Rel.LE) throw new Exception("Knapsack solver requires a <= constraint.");

            _n = problem.NumVars;
            _capacity = cons.B;

            // Build items sorted by ratio (profit/weight) descending
            _itemsByRatio = Enumerable.Range(0, _n)
                .Select(i => new Item { Index = i, Profit = problem.C[i], Weight = cons.A[i] })
                .OrderByDescending(it => it.Ratio)
                .ThenByDescending(it => it.Profit)
                .ToList();

            // Initialize best
            _bestValue = double.NegativeInfinity;
            _bestX = new int[_n];

            // Header
            _report.AppendLine("Branch and Bound Knapsack Algorithm");
            _report.AppendLine("=================================================");
            _report.AppendLine("Ratio Test:");
            _report.AppendLine("Item\tci/ai\tRank");
            for (int i = 0; i < _itemsByRatio.Count; i++)
            {
                var it = _itemsByRatio[i];
                _report.AppendLine($"{it.Index + 1}\t{it.Ratio:0.###}\t{i + 1}");
            }
            _report.AppendLine();
            _report.AppendLine("-------------------------------------------------");

            // Priority queue (max-heap) by bound
            var pq = new SimpleMaxHeap<Node>((a, b) => a.Bound.CompareTo(b.Bound));

            // Root node: all undecided
            var root = new Node
            {
                Assigned = Enumerable.Repeat(-1, _n).ToArray(),
                Label = "0"
            };
            // compute root relaxation
            var rootRelax = ComputeRelaxation(root.Assigned);
            root.Bound = rootRelax.bound;
            root.RelaxProfit = rootRelax.profit;
            root.RelaxWeight = rootRelax.weight;

            pq.Push(root);

            int nodeCounter = 0;

            // main loop
            while (pq.Count > 0)
            {
                var node = pq.Pop();
                nodeCounter++;

                // If bound cannot beat current best, skip
                if (node.Bound <= _bestValue + EPS) continue;

                // Compute relaxation again (should be same as stored, but ensures consistency)
                var (relaxed, bound, fractionalSortedIdx, profitRel, weightRel) = ComputeRelaxation(node.Assigned);

                // Log node header + relaxation
                string header = node.Label == "0" ? "Sub-Problem 0" : $"Sub-Problem {node.Label}";
                _report.AppendLine(header);
                _report.AppendLine();

                // fractional variable original index if any
                int fractionalOrigIndex = -1;
                if (fractionalSortedIdx >= 0) fractionalOrigIndex = _itemsByRatio[fractionalSortedIdx].Index;

                // print relaxed vector (original variable order)
                for (int i = 0; i < _n; i++)
                {
                    string prefix = (i == fractionalOrigIndex) ? ">" : " ";
                    _report.AppendLine($"{prefix}\tx{i + 1}\t=\t{relaxed[i]:0.###}");
                }
                _report.AppendLine();

                // If no fractional variable: integer candidate (all 0/1)
                if (fractionalSortedIdx == -1)
                {
                    // check feasibility
                    if (weightRel <= _capacity + EPS)
                    {
                        double candidateValue = profitRel;
                        var candX = new int[_n];
                        for (int i = 0; i < _n; i++) candX[i] = relaxed[i] >= 0.5 ? 1 : 0;

                        _report.AppendLine($"\tz = {Math.Round(candidateValue, 6):0.###}");
                        if (candidateValue > _bestValue + EPS)
                        {
                            _bestValue = candidateValue;
                            Array.Copy(candX, _bestX, _n);
                            _report.AppendLine("\tBEST CANDIDATE");
                        }
                        else
                        {
                            _report.AppendLine("\tCANDIDATE");
                        }
                    }
                    else
                    {
                        _report.AppendLine("\tINFEASIBLE");
                    }

                    _report.AppendLine("------------------------------------------------");
                    FlushDelta();

                    continue;
                }

                // Otherwise we will branch on fractional item (the first fractional in ratio order)
                int origIdx = _itemsByRatio[fractionalSortedIdx].Index;

                // Create child labels
                string leftLabel = node.Label == "0" ? $"{node.Label}.1" : node.Label + ".1";
                string rightLabel = node.Label == "0" ? $"{node.Label}.2" : node.Label + ".2";
                // For nicer numbers, if root label "0" produce "1" and "2"
                if (node.Label == "0")
                {
                    leftLabel = "1";
                    rightLabel = "2";
                }
                /*
                  // Print branching choices
                  _report.AppendLine($"\tSub-P {leftLabel}: x{origIdx + 1} = 0");
                  _report.AppendLine();
                  _report.AppendLine($"\tSub-P {rightLabel}: x{origIdx + 1} = 1");
                */
                _report.AppendLine("------------------------------------------------");
                _report.AppendLine();



                // push UI update so user sees traversal incrementally
                FlushDelta();


                // LEFT child: fix to 0
                var leftAssigned = (int[])node.Assigned.Clone();
                leftAssigned[origIdx] = 0;
                var leftRelax = ComputeRelaxation(leftAssigned);

                // Always log child relaxation briefly for clarity
                _report.AppendLine($"-- Node {leftLabel} branching (x{origIdx + 1}=0):");
                LogRelaxedVector(leftRelax.relaxed, leftRelax.fractionalSortedIdx);
                _report.AppendLine($"\tBound = {leftRelax.bound:0.###}, Capacity = {leftRelax.weight:0.###}");
                if (leftRelax.weight > _capacity + EPS)
                {
                    _report.AppendLine("\tINFEASIBLE");
                    _report.AppendLine("------------------------------------------------");
                    _report.AppendLine();

                    FlushDelta();
                }
                else if (leftRelax.bound > _bestValue + EPS)
                {
                    bool allInt = leftRelax.relaxed.All(v => Math.Abs(v - Math.Round(v)) < EPS);
                    bool feasible = leftRelax.weight <= _capacity + EPS;

                    if (allInt && feasible)
                    {
                        _report.AppendLine($"\tCANDIDATE {leftLabel}");
                        if (leftRelax.profit > _bestValue + EPS)
                        {
                            _bestValue = leftRelax.profit;
                            for (int i = 0; i < _n; i++) _bestX[i] = (int)Math.Round(leftRelax.relaxed[i]);
                        }
                    }
                    else
                    {
                        var leftNode = new Node
                        {
                            Assigned = leftAssigned,
                            Label = leftLabel,
                            Bound = leftRelax.bound,
                            RelaxProfit = leftRelax.profit,
                            RelaxWeight = leftRelax.weight
                        };
                        pq.Push(leftNode);
                        //  _report.AppendLine($"\tEnqueued Node {leftLabel} (bound {leftRelax.bound:0.###})");
                    }

                    _report.AppendLine("------------------------------------------------");
                    _report.AppendLine();

                    FlushDelta();
                }

                else
                {
                    _report.AppendLine($"\tCANDIDATE {leftLabel}");
                    _report.AppendLine("------------------------------------------------");
                    _report.AppendLine();

                    FlushDelta();
                }

                // RIGHT child: fix to 1
                var rightAssigned = (int[])node.Assigned.Clone();
                rightAssigned[origIdx] = 1;
                var rightRelax = ComputeRelaxation(rightAssigned);

                // Always log right child relaxation
                _report.AppendLine($"-- Node {rightLabel} branching (x{origIdx + 1}=1):");
                LogRelaxedVector(rightRelax.relaxed, rightRelax.fractionalSortedIdx);
                _report.AppendLine($"\tBound = {rightRelax.bound:0.###}, Capacity = {rightRelax.weight:0.###}");

                // if direct infeasible (weight > capacity) -> log infeasible and do not push
                if (rightRelax.weight > _capacity + EPS)
                {
                    _report.AppendLine("\tINFEASIBLE ");
                    _report.AppendLine("------------------------------------------------");
                    _report.AppendLine();

                    FlushDelta();
                }
                else if (rightRelax.bound > _bestValue + EPS)
                {
                    bool allInt = rightRelax.relaxed.All(v => Math.Abs(v - Math.Round(v)) < EPS);
                    bool feasible = rightRelax.weight <= _capacity + EPS;

                    if (allInt && feasible)
                    {
                        _report.AppendLine($"\tCANDIDATE {rightLabel}");
                        if (rightRelax.profit > _bestValue + EPS)
                        {
                            _bestValue = rightRelax.profit;
                            for (int i = 0; i < _n; i++) _bestX[i] = (int)Math.Round(rightRelax.relaxed[i]);
                        }
                    }
                    else
                    {
                        var rightNode = new Node
                        {
                            Assigned = rightAssigned,
                            Label = rightLabel,
                            Bound = rightRelax.bound,
                            RelaxProfit = rightRelax.profit,
                            RelaxWeight = rightRelax.weight
                        };

                        pq.Push(rightNode);
                        // _report.AppendLine($"\tEnqueued Node {rightLabel} (bound {rightRelax.bound:0.###})");
                    }

                    _report.AppendLine("------------------------------------------------");
                    _report.AppendLine();

                    FlushDelta();
                }

                else
                {
                    _report.AppendLine($"\tCANDIDATE {rightLabel}");
                    _report.AppendLine("------------------------------------------------");
                    _report.AppendLine();

                    FlushDelta();
                }
            } // end loop

            // After loop, append final formatted report and summary exactly as requested

            // Canonical form (Max expression + constraint)
            _report.AppendLine();
            _report.AppendLine("Final Report:");
            _report.AppendLine("Branch & Bound Knapsack Finished.");
            _report.AppendLine();

           

            // Status and solution
            if (double.IsNegativeInfinity(_bestValue))
            {
                _report.AppendLine("Status: NO FEASIBLE CANDIDATE");
            }
            else
            {
                _report.AppendLine("Status: BEST CANDIDATE FOUND");
                for (int j = 0; j < _n; j++)
                    _report.AppendLine($"  x{j + 1} = {_bestX[j]}");
                _report.AppendLine($"  z* = {Math.Round(_bestValue, 6):0.###}");
            }

            _report.AppendLine();
            _report.AppendLine();
            _report.AppendLine("Summary:");
            if (double.IsNegativeInfinity(_bestValue))
            {
                _report.AppendLine("No feasible candidate found.");
            }
            else
            {
                _report.AppendLine($"Best Candidate = {Math.Round(_bestValue, 6):0.###}");
                _report.AppendLine("Best x* = [" + string.Join(", ", _bestX) + "]");
            }

            FlushDelta();

            // After loop, build final report
            var finalReport = new StringBuilder();
            finalReport.AppendLine("Final Report:");
            finalReport.AppendLine("Branch & Bound Knapsack Finished.");
            finalReport.AppendLine();
           

            if (double.IsNegativeInfinity(_bestValue))
            {
                finalReport.AppendLine("Status: INFEASIBLE");
            }
            else
            {
                finalReport.AppendLine("Status: BEST CANDIDATE FOUND");
                for (int i = 0; i < _n; i++)
                {
                    finalReport.AppendLine($"  x{i + 1} = {_bestX[i]}");
                }
                finalReport.AppendLine($"  z* = {_bestValue:0.###}");
            }

            finalReport.AppendLine();
            finalReport.AppendLine("Summary:");
            if (double.IsNegativeInfinity(_bestValue))
            {
                finalReport.AppendLine("No feasible candidate found.");
            }
            else
            {
                finalReport.AppendLine($"Best Candidate = {_bestValue:0.###}");
                finalReport.AppendLine("Best x* = [" + string.Join(", ", _bestX) + "]");
            }

            return new SimplexResult
            {
                Report = finalReport.ToString(),  // ✅ only final report
                Summary = ""                      // ✅ no duplicate
            };

        }

        /// <summary>
        /// Helper to log relaxed vector into _report (original variable order).
        /// Marks the fractional variable with a leading '>'.
        /// </summary>
        private void LogRelaxedVector(double[] relaxed, int fractionalSortedIdx)
        {
            int fractionalOrigIndex = -1;
            if (fractionalSortedIdx >= 0) fractionalOrigIndex = _itemsByRatio[fractionalSortedIdx].Index;

            for (int i = 0; i < _n; i++)
            {
                string prefix = (i == fractionalOrigIndex) ? ">" : " ";
                _report.AppendLine($"{prefix}\tx{i + 1}\t=\t{relaxed[i]:0.###}");
            }
        }

        /// <summary>
        /// Compute fractional relaxation for a given assignment (original indices).
        /// - First add all items fixed to 1 (hard constraint)
        /// - If that exceeds capacity => infeasible (weight > capacity)
        /// - Otherwise, pack undecided items greedily by ratio (whole then fractional)
        /// </summary>
        private (double[] relaxed, double bound, int fractionalSortedIdx, double profit, double weight)
            ComputeRelaxation(int[] assigned)
        {
            var relaxed = new double[_n];
            for (int i = 0; i < _n; i++) relaxed[i] = 0.0;

            double weight = 0.0;
            double profit = 0.0;
            int fractionalSortedIdx = -1;

            // 1) Add all items fixed to 1 (regardless of ratio order)
            for (int i = 0; i < _n; i++)
            {
                if (assigned[i] == 1)
                {
                    // find the item object for original index i
                    var it = _itemsByRatio.First(x => x.Index == i);
                    relaxed[i] = 1.0;
                    weight += it.Weight;
                    profit += it.Profit;
                }
            }

            // If fixed items alone exceed capacity → infeasible relaxation
            if (weight > _capacity + EPS)
                return (relaxed, profit, -1, profit, weight);

            // 2) Greedily pack undecided items in ratio order
            for (int s = 0; s < _itemsByRatio.Count; s++)
            {
                var it = _itemsByRatio[s];
                int orig = it.Index;

                if (assigned[orig] == 1) continue;     // already counted
                if (assigned[orig] == 0) { relaxed[orig] = 0.0; continue; } // fixed out

                // undecided
                if (weight + it.Weight <= _capacity + EPS)
                {
                    relaxed[orig] = 1.0;
                    weight += it.Weight;
                    profit += it.Profit;
                }
                else
                {
                    double remain = _capacity - weight;
                    if (remain > EPS && it.Weight > EPS)
                    {
                        double frac = remain / it.Weight;
                        relaxed[orig] = frac;
                        profit += it.Profit * frac;
                        weight += it.Weight * frac;
                        fractionalSortedIdx = s;
                    }
                    // after first fractional, nothing else can fit
                    break;
                }
            }

            return (relaxed, profit, fractionalSortedIdx, profit, weight);
        }

        // Simple max-heap on Node using provided compare function (cmp returns positive if a > b)
        private class SimpleMaxHeap<T>
        {
            private readonly List<T> data = new List<T>();
            private readonly Comparison<T> cmp;
            public SimpleMaxHeap(Comparison<T> compare) { cmp = compare; }
            public int Count => data.Count;

            public void Push(T item)
            {
                data.Add(item);
                int ci = data.Count - 1;
                while (ci > 0)
                {
                    int pi = (ci - 1) / 2;
                    if (cmp(data[ci], data[pi]) <= 0) break;
                    Swap(ci, pi);
                    ci = pi;
                }
            }

            public T Pop()
            {
                if (data.Count == 0) throw new InvalidOperationException("Heap empty");
                int li = data.Count - 1;
                Swap(0, li);
                T ret = data[li];
                data.RemoveAt(li);
                Heapify(0);
                return ret;
            }

            private void Heapify(int i)
            {
                int li = data.Count - 1;
                while (true)
                {
                    int l = 2 * i + 1;
                    int r = 2 * i + 2;
                    int largest = i;
                    if (l <= li && cmp(data[l], data[largest]) > 0) largest = l;
                    if (r <= li && cmp(data[r], data[largest]) > 0) largest = r;
                    if (largest == i) break;
                    Swap(i, largest);
                    i = largest;
                }
            }

            private void Swap(int a, int b)
            {
                var tmp = data[a];
                data[a] = data[b];
                data[b] = tmp;
            }
        }
    }
}
