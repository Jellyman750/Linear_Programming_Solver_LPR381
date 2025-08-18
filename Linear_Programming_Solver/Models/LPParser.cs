using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using Linear_Programming_Solver.Models;

public static class LPParser
{
    public static LPProblem ParseFromText(string input)
    {
        var lines = input.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries)
                         .Select(l => l.Trim())
                         .Where(l => !string.IsNullOrWhiteSpace(l))
                         .ToArray();

        if (lines.Length < 2) throw new Exception("Input must contain an objective and at least one constraint.");

        // Parse Objective
        var objMatch = Regex.Match(lines[0], @"^(max|min)\s*:\s*(.+)$", RegexOptions.IgnoreCase);
        if (!objMatch.Success) throw new Exception("Objective format incorrect. Example: Max: 3x1 + 5x2");

        Sense sense = objMatch.Groups[1].Value.Trim().ToLower() == "max" ? Sense.Max : Sense.Min;
        string objExpr = objMatch.Groups[2].Value;
        double[] c = ParseCoefficients(objExpr);

        var problem = new LPProblem
        {
            ObjectiveSense = sense,
            C = c
        };

        // Parse Constraints
        for (int i = 1; i < lines.Length; i++)
        {
            var consLine = lines[i];
            var match = Regex.Match(consLine, @"^(.+?)(<=|>=|=)(.+)$"); // non-greedy LHS
            if (!match.Success) throw new Exception($"Constraint format incorrect: {consLine}");

            string lhs = match.Groups[1].Value.Trim();
            string relStr = match.Groups[2].Value.Trim();
            string rhsStr = match.Groups[3].Value.Trim();

            double[] A = ParseCoefficients(lhs);

            Rel relation = relStr switch
            {
                "<=" => Rel.LE,
                ">=" => Rel.GE,
                "=" => Rel.EQ,
                _ => throw new Exception($"Unknown relation: {relStr}")
            };

            if (!double.TryParse(rhsStr, out double B)) throw new Exception($"Invalid RHS number: {rhsStr}");

            problem.Constraints.Add(new Constraint { A = A, Relation = relation, B = B });
        }

        return problem;
    }

    private static double[] ParseCoefficients(string expr)
    {
        expr = expr.Replace("-", "+-").Replace(" ", ""); // normalize and remove spaces
        var parts = expr.Split(new[] { '+' }, StringSplitOptions.RemoveEmptyEntries);

        var coefficients = new List<double>();
        foreach (var part in parts)
        {
            var match = Regex.Match(part.Trim(), @"^([-]?\d*\.?\d*)x\d+$");
            if (!match.Success) throw new Exception($"Cannot parse coefficient: {part}");

            string valStr = match.Groups[1].Value;
            double val = string.IsNullOrEmpty(valStr) ? 1 :
                         valStr == "-" ? -1 : double.Parse(valStr);
            coefficients.Add(val);
        }

        return coefficients.ToArray();
    }
}
