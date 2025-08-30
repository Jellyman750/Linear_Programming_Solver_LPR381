using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;
using Linear_Programming_Solver.Models;

namespace Linear_Programming_Solver
{
    public partial class Form1 : Form
    {
        private Label titleLabel;
        private Button importButton;
        private Button exportButton;
        private Button solveButton;
        private TextBox lpInputTextBox;
        private RichTextBox iterationOutputTextBox;
        private ComboBox algorithmDropdown;
        private Button sensitivityButton;

        public Form1()
        {
            InitializeComponent();
            InitializeCustomComponents();
        }

        private void InitializeCustomComponents()
        {
            // Set form properties
            this.BackColor = Color.LightBlue;
            this.Text = "Linear Programming Solver";
            this.Size = new Size(1200, 1000);
            this.FormBorderStyle = FormBorderStyle.Sizable;
            this.MaximizeBox = true;

            // Title Label
            titleLabel = new Label();
            titleLabel.Text = "LP Solver";
            titleLabel.Font = new Font("Arial", 20, FontStyle.Bold | FontStyle.Underline);
            titleLabel.AutoSize = true;
            titleLabel.Location = new Point((this.ClientSize.Width - titleLabel.Width) / 2, 20);
            titleLabel.Anchor = AnchorStyles.Top;
            this.Controls.Add(titleLabel);

            // Import Button
            importButton = new Button();
            importButton.Text = "Import";
            importButton.Size = new Size(115, 55);
            importButton.Location = new Point((this.ClientSize.Width / 2) - 130, 70);
            importButton.Click += BtnUpload_Click;
            this.Controls.Add(importButton);

            // Export Button
            exportButton = new Button();
            exportButton.Text = "Export";
            exportButton.Size = new Size(115, 55);
            exportButton.Location = new Point((this.ClientSize.Width / 2) + 15, 70);
            exportButton.Click += BtnExport_Click;
            this.Controls.Add(exportButton);

            // Algorithm Dropdown
            algorithmDropdown = new ComboBox();
            algorithmDropdown.DropDownStyle = ComboBoxStyle.DropDownList;
            algorithmDropdown.Items.AddRange(new string[]
            {
                "Primal Simplex",
                "Revised Primal Simplex",
                "Dual Simplex",
                "Branch and Bound",
                "Revised Branch and Bound",
                "Cutting Plane",
                "Revised Cutting Plane",
                "Branch and Bound Knapsack"
            });
            algorithmDropdown.Location = new Point((this.ClientSize.Width / 2) - 150, 150);
            algorithmDropdown.Width = 300;
            this.Controls.Add(algorithmDropdown);

            // LP Input TextBox
            lpInputTextBox = new TextBox();
            lpInputTextBox.Multiline = true;
            lpInputTextBox.Size = new Size(350, 100);
            lpInputTextBox.Location = new Point((this.ClientSize.Width / 2) - 175, 200);
            lpInputTextBox.PlaceholderText = "Enter LP here...";
            this.Controls.Add(lpInputTextBox);

            // Iteration Output TextBox
            iterationOutputTextBox = new RichTextBox();
            iterationOutputTextBox.Multiline = true;
            iterationOutputTextBox.Size = new Size(900, 400);
            iterationOutputTextBox.Location = new Point((this.ClientSize.Width / 2) - 450, 320);
            iterationOutputTextBox.ScrollBars = RichTextBoxScrollBars.Both;
            iterationOutputTextBox.Font = new Font("Consolas", 9);
            iterationOutputTextBox.WordWrap = false;
            this.Controls.Add(iterationOutputTextBox);

            // Solve Button
            solveButton = new Button();
            solveButton.Text = "Solve LP";
            solveButton.Size = new Size(115, 55);
            solveButton.Location = new Point((this.ClientSize.Width / 2) - 180, 750);
            solveButton.Click += btnSolve_Click;
            this.Controls.Add(solveButton);

            // Sensitivity Analysis Button
            sensitivityButton = new Button();
            sensitivityButton.Text = "Sensitivity Analysis";
            sensitivityButton.Size = new Size(180, 55);
            sensitivityButton.Location = new Point((this.ClientSize.Width / 2) + 20, 750);
            sensitivityButton.Click += btnSensitivityAnalysis_Click; // ? hooked correctly
            this.Controls.Add(sensitivityButton);
        }

        private void btnSensitivityAnalysis_Click(object sender, EventArgs e)
        {
            try
            {
                iterationOutputTextBox.Clear();
                var lpText = lpInputTextBox.Text;
                var problem = LPParser.ParseFromText(lpText);

                var analysis = new SensitivityAnalysis(problem);
                string report = analysis.GenerateReport();

                iterationOutputTextBox.AppendText("=== Sensitivity Analysis Report ===\n\n");
                iterationOutputTextBox.AppendText(report);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error running sensitivity analysis: " + ex.Message,
                                "Analysis Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
        }

        private void btnSolve_Click(object sender, EventArgs e)
        {
            try
            {
                iterationOutputTextBox.Clear();
                var lpText = lpInputTextBox.Text;
                var problem = LPParser.ParseFromText(lpText);
                string selectedAlgorithm = algorithmDropdown.SelectedItem?.ToString() ?? "";

                switch (selectedAlgorithm)
                {
                    case "Primal Simplex":
                    case "Revised Primal Simplex":
                    case "Dual Simplex":
                    case "Cutting Plane":
                    case "Revised Cutting Plane":
                        {
                            var solver = new LPSolver();
                            var result = solver.Solve(problem, selectedAlgorithm, (text, highlight) => AppendPivotRow(text, highlight));
                            iterationOutputTextBox.AppendText("\n\nFinal Report:\n" + result.Report);
                            iterationOutputTextBox.AppendText("\n\nSummary:\n" + result.Summary);
                            break;
                        }
                    case "Branch and Bound":
                    case "Revised Branch and Bound":
                    case "Branch and Bound Knapsack":
                        {
                            var bbSolver = new BranchAndBound();
                            var result = bbSolver.Solve(problem, (text, highlight) => AppendPivotRow(text, highlight));
                            iterationOutputTextBox.AppendText("\n\nFinal Report:\n" + result.Report);
                            iterationOutputTextBox.AppendText("\n\nSummary:\n" + result.Summary);
                            break;
                        }
                    default:
                        iterationOutputTextBox.Text = "Error: Algorithm not supported.";
                        break;
                }
            }
            catch (Exception ex)
            {
                iterationOutputTextBox.Text = "Error: " + ex.Message;
            }
        }

        private void BtnUpload_Click(object sender, EventArgs e)
        {
            using (OpenFileDialog ofd = new OpenFileDialog())
            {
                ofd.Filter = "Text Files (*.txt)|*.txt|All Files (*.*)|*.*";

                if (ofd.ShowDialog() == DialogResult.OK)
                {
                    try
                    {
                        lpInputTextBox.Text = File.ReadAllText(ofd.FileName);
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show("Error reading file: " + ex.Message, "File Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
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

            if (highlight == null)
            {
                iterationOutputTextBox.ResumeLayout();
                return;
            }

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
                    int cellLen = colWidth;

                    iterationOutputTextBox.Select(cellStart, cellLen);
                    iterationOutputTextBox.SelectionColor = Color.Orange;
                    iterationOutputTextBox.SelectionFont = new Font(iterationOutputTextBox.Font, FontStyle.Bold);
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
