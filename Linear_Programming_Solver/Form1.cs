using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using Linear_Programming_Solver.Analysis;
using Linear_Programming_Solver.Models;

namespace Linear_Programming_Solver
{
    public partial class Form1 : Form
    {
        private Label titleLabel;
        private Button importButton, exportButton, solveButton, sensitivityButton;
        private TextBox lpInputTextBox;
        private RichTextBox iterationOutputTextBox;
        private ComboBox algorithmDropdown;

        // Sensitivity UI
        private ComboBox sensitivityTargetDropdown;
        private ComboBox sensitivityOperationDropdown;
        private TextBox sensitivityValueTextBox;
        private Button sensitivityExecuteButton;

        private LPProblem currentProblem;
        private SimplexResult currentResult;

        public Form1()
        {
            InitializeComponent();
            InitializeCustomComponents();
        }

        private void InitializeCustomComponents()
        {
            this.BackColor = Color.LightBlue;
            this.Text = "Linear Programming Solver";
            this.Size = new Size(1200, 1000);
            this.FormBorderStyle = FormBorderStyle.Sizable;
            this.MaximizeBox = true;

            // Title Label
            titleLabel = new Label
            {
                Text = "LP Solver",
                Font = new Font("Arial", 20, FontStyle.Bold | FontStyle.Underline),
                AutoSize = true,
                Location = new Point((this.ClientSize.Width - 100) / 2, 20),
                Anchor = AnchorStyles.Top
            };
            this.Controls.Add(titleLabel);

            // Import Button
            importButton = new Button { Text = "Import", Size = new Size(115, 55), Location = new Point((this.ClientSize.Width / 2) - 130, 70) };
            importButton.Click += BtnUpload_Click;
            this.Controls.Add(importButton);

            // Export Button
            exportButton = new Button { Text = "Export", Size = new Size(115, 55), Location = new Point((this.ClientSize.Width / 2) + 15, 70) };
            exportButton.Click += BtnExport_Click;
            this.Controls.Add(exportButton);

            // Algorithm Dropdown
            algorithmDropdown = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList };
            algorithmDropdown.Items.AddRange(new string[]
            {
                "Primal Simplex", "Revised Primal Simplex", "Dual Simplex",
                "Branch and Bound", "Revised Branch and Bound", "Cutting Plane",
                "Revised Cutting Plane", "Branch and Bound Knapsack"
            });
            algorithmDropdown.Location = new Point((this.ClientSize.Width / 2) - 150, 150);
            algorithmDropdown.Width = 300;
            this.Controls.Add(algorithmDropdown);

            // LP Input TextBox
            lpInputTextBox = new TextBox { Multiline = true, Size = new Size(350, 100), Location = new Point((this.ClientSize.Width / 2) - 175, 200), PlaceholderText = "Enter LP here..." };
            this.Controls.Add(lpInputTextBox);

            // Iteration Output TextBox
            iterationOutputTextBox = new RichTextBox
            {
                Multiline = true,
                Size = new Size(900, 400),
                Location = new Point((this.ClientSize.Width / 2) - 450, 320),
                ScrollBars = RichTextBoxScrollBars.Both,
                Font = new Font("Consolas", 9),
                WordWrap = false
            };
            this.Controls.Add(iterationOutputTextBox);

            // Solve Button
            solveButton = new Button { Text = "Solve LP", Size = new Size(115, 55), Location = new Point((this.ClientSize.Width / 2) - 180, 750) };
            solveButton.Click += btnSolve_Click;
            this.Controls.Add(solveButton);

            // Sensitivity Analysis Button
            sensitivityButton = new Button
            {
                Text = "Sensitivity Analysis",
                Size = new Size(180, 55),
                Location = new Point((this.ClientSize.Width / 2) + 20, 750)
            };
            sensitivityButton.Click += SensitivityButton_Click;
            this.Controls.Add(sensitivityButton);

            // Sensitivity Analysis UI (Hidden Initially)
            sensitivityTargetDropdown = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Location = new Point((this.ClientSize.Width / 2) - 200, 820), Width = 150, Visible = false };
            sensitivityOperationDropdown = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Location = new Point((this.ClientSize.Width / 2) - 40, 820), Width = 150, Visible = false };
            sensitivityOperationDropdown.Items.AddRange(new string[]
            {
                "Display Range",
                "Apply Change",
                "Show Shadow Prices",
                "Solve using Duality"
            });
            sensitivityValueTextBox = new TextBox { Location = new Point((this.ClientSize.Width / 2) + 130, 820), Width = 80, PlaceholderText = "Value", Visible = false };
            sensitivityExecuteButton = new Button { Text = "Execute", Location = new Point((this.ClientSize.Width / 2) + 220, 820), Size = new Size(80, 30), Visible = false };
            sensitivityExecuteButton.Click += SensitivityExecuteButton_Click;

            this.Controls.Add(sensitivityTargetDropdown);
            this.Controls.Add(sensitivityOperationDropdown);
            this.Controls.Add(sensitivityValueTextBox);
            this.Controls.Add(sensitivityExecuteButton);
        }


        private void SensitivityButton_Click(object sender, EventArgs e)
        {
            if (currentProblem == null || currentResult == null)
            {
                MessageBox.Show("Solve an LP first to perform sensitivity analysis.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            // Show Sensitivity Analysis Controls
            sensitivityTargetDropdown.Visible = true;
            sensitivityOperationDropdown.Visible = true;
            sensitivityValueTextBox.Visible = true;
            sensitivityExecuteButton.Visible = true;

            // Populate Sensitivity Target Dropdown
            sensitivityTargetDropdown.Items.Clear();
            sensitivityTargetDropdown.Items.AddRange(currentResult.VarNames);
            for (int i = 0; i < currentProblem.Constraints.Count; i++)
                sensitivityTargetDropdown.Items.Add($"Constraint {i + 1}");

            // Insert a marker/header in the RichTextBox
            iterationOutputTextBox.AppendText("\n=== Sensitivity Analysis ===\n");
        }

        private void SensitivityExecuteButton_Click(object sender, EventArgs e)
        {
            if (currentProblem == null || currentResult == null)
            {
                MessageBox.Show("Solve an LP first before performing sensitivity analysis.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            string target = sensitivityTargetDropdown.SelectedItem?.ToString();
            string operation = sensitivityOperationDropdown.SelectedItem?.ToString();

            if (string.IsNullOrWhiteSpace(target))
            {
                MessageBox.Show("Please select a variable or constraint for sensitivity analysis.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            if (string.IsNullOrWhiteSpace(operation))
            {
                MessageBox.Show("Please select a sensitivity operation.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            double value = 0;
            if (operation == "Apply Change")
            {
                if (!double.TryParse(sensitivityValueTextBox.Text, out value))
                {
                    MessageBox.Show("Enter a valid numeric value for Apply Change.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }
            }

            // Clear only the previous sensitivity results while keeping the LP solution
            string marker = "=== Sensitivity Analysis ===";
            int markerIndex = iterationOutputTextBox.Text.IndexOf(marker);
            if (markerIndex >= 0)
            {
                iterationOutputTextBox.Text = iterationOutputTextBox.Text.Substring(0, markerIndex + marker.Length) + "\n";
            }
            else
            {
                // Insert marker if not present
                iterationOutputTextBox.AppendText("\n\n" + marker + "\n");
            }

            try
            {
                var analysis = new SensitivityAnalysis(currentProblem, currentResult);
                iterationOutputTextBox.AppendText("\n=== Sensitivity Analysis Operation ===\n");

                switch (operation)
                {
                    case "Display Range":
                        iterationOutputTextBox.AppendText(analysis.GetRangeReport(target) + "\n");
                        break;

                    case "Apply Change":
                        iterationOutputTextBox.AppendText(analysis.ApplyChange(target, value) + "\n");
                        break;

                    case "Show Shadow Prices":
                        iterationOutputTextBox.AppendText(analysis.GetShadowPricesReport() + "\n");
                        break;

                    case "Solve using Duality":
                        var dualResult = analysis.SolveUsingDuality();
                        iterationOutputTextBox.AppendText("Solve using Duality:\n" + dualResult.Summary + "\n");
                        break;

                    default:
                        iterationOutputTextBox.AppendText("Operation not recognized.\n");
                        break;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error during sensitivity analysis: " + ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void btnSolve_Click(object sender, EventArgs e)
        {
            iterationOutputTextBox.Clear();
            currentProblem = LPParser.ParseFromText(lpInputTextBox.Text);

            string selectedAlgorithm = algorithmDropdown.SelectedItem?.ToString() ?? "";

            switch (selectedAlgorithm)
            {
                case "Primal Simplex":
                case "Revised Primal Simplex":
                case "Dual Simplex":
                    {
                        var solver = new LPSolver();
                        currentResult = solver.Solve(currentProblem, selectedAlgorithm, (text, highlight) => AppendPivotRow(text, highlight));
                        break;
                    }

                case "Cutting Plane":
                    {
                        var cpSolver = new CuttingPlane();
                        currentResult = cpSolver.Solve(currentProblem, (text, highlight) => AppendPivotRow(text, highlight));
                        break;
                    }

                case "Revised Cutting Plane":
                    {
                        var cpRevisedSolver = new CuttingPlaneRevised();
                        currentResult = cpRevisedSolver.Solve(currentProblem, (text, highlight) => AppendPivotRow(text, highlight));
                        break;
                    }

                case "Branch and Bound":
                case "Revised Branch and Bound":
                case "Branch and Bound Knapsack":
                    {
                        var bbSolver = new BranchAndBound();
                        currentResult = bbSolver.Solve(currentProblem, (text, highlight) => AppendPivotRow(text, highlight));
                        break;
                    }

                default:
                    MessageBox.Show("Please select a valid algorithm.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    return;
            }

            iterationOutputTextBox.AppendText("\n\nFinal Report:\n" + currentResult.Report);
            iterationOutputTextBox.AppendText("\n\nSummary:\n" + currentResult.Summary);
        }




        private void BtnUpload_Click(object sender, EventArgs e)
        {
            using (OpenFileDialog ofd = new OpenFileDialog())
            {
                ofd.Filter = "Text Files (*.txt)|*.txt|All Files (*.*)|*.*";

                if (ofd.ShowDialog() == DialogResult.OK)
                {
                    try { lpInputTextBox.Text = File.ReadAllText(ofd.FileName); }
                    catch (Exception ex) { MessageBox.Show("Error reading file: " + ex.Message, "File Error", MessageBoxButtons.OK, MessageBoxIcon.Error); }
                }
            }
        }

        private void BtnExport_Click(object sender, EventArgs e)
        {
            using (SaveFileDialog sfd = new SaveFileDialog())
            {
                sfd.Filter = "Text Files (*.txt)|*.txt|All Files (*.*)|*.*";

                if (sfd.ShowDialog() == DialogResult.OK)
                {
                    try
                    {
                        using (StreamWriter writer = new StreamWriter(sfd.FileName))
                        {
                            writer.WriteLine("Linear Program:");
                            writer.WriteLine(lpInputTextBox.Text);
                            writer.WriteLine();
                            writer.WriteLine("Iterations:");
                            writer.WriteLine(iterationOutputTextBox.Text);
                        }
                        MessageBox.Show("File exported successfully!", "Success", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show("Error writing file: " + ex.Message, "File Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
                }
            }
        }

        private void AppendPivotRow(string text, bool[,] highlight)
        {
            iterationOutputTextBox.SuspendLayout();
            int startPos = iterationOutputTextBox.TextLength;
            iterationOutputTextBox.AppendText(text);

            if (highlight != null)
            {
                string[] lines = text.Split(new[] { Environment.NewLine }, StringSplitOptions.None);
                int colWidth = 12;
                int firstDataLineIdx = 3;
                int numDataRows = highlight.GetLength(0);
                int numDataCols = highlight.GetLength(1);

                int pos = startPos;
                for (int l = 0; l < lines.Length; l++)
                {
                    string line = lines[l];
                    int lineStart = pos;
                    pos += line.Length + (l < lines.Length - 1 ? Environment.NewLine.Length : 0);

                    if (l < firstDataLineIdx || l >= firstDataLineIdx + numDataRows) continue;

                    int hi = l - firstDataLineIdx;
                    for (int hj = 0; hj < numDataCols; hj++)
                    {
                        if (!highlight[hi, hj]) continue;

                        int dcol = hj + 1;
                        int cellStart = lineStart + dcol * colWidth;
                        iterationOutputTextBox.Select(cellStart, colWidth);
                        iterationOutputTextBox.SelectionColor = Color.Orange;
                        iterationOutputTextBox.SelectionFont = new Font(iterationOutputTextBox.Font, FontStyle.Bold);
                    }
                }
            }

            iterationOutputTextBox.SelectionStart = iterationOutputTextBox.TextLength;
            iterationOutputTextBox.SelectionLength = 0;
            iterationOutputTextBox.SelectionColor = iterationOutputTextBox.ForeColor;
            iterationOutputTextBox.SelectionFont = iterationOutputTextBox.Font;
            iterationOutputTextBox.ResumeLayout();
        }
    


public void AddIteration(string step)
        {
            if (!string.IsNullOrWhiteSpace(step))
            {
                iterationOutputTextBox.AppendText(step + Environment.NewLine);
            }
        }

        private void Form1_Load(object sender, EventArgs e) { }
    }
}
