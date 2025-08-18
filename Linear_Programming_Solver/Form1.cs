using System;
using System.Drawing;
using System.IO;
using System.Windows.Forms;

namespace Linear_Programming_Solver
{
    public partial class Form1 : Form
    {
        private Label titleLabel;
        private Button importButton;
        private Button exportButton;
        private TextBox lpInputTextBox;
        private TextBox iterationOutputTextBox;
        private ComboBox algorithmDropdown;

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
            this.Size = new Size(900, 800);

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
            importButton.TextAlign = ContentAlignment.MiddleCenter;
            importButton.Click += BtnUpload_Click;
            this.Controls.Add(importButton);

            // Export Button
            exportButton = new Button();
            exportButton.Text = "Export";
            exportButton.Size = new Size(115, 55);
            exportButton.Location = new Point((this.ClientSize.Width / 2) + 15, 70);
            exportButton.TextAlign = ContentAlignment.MiddleCenter;
            exportButton.Click += BtnExport_Click;
            this.Controls.Add(exportButton);

            // Algorithm Dropdown
            algorithmDropdown = new ComboBox();
            algorithmDropdown.DropDownStyle = ComboBoxStyle.DropDownList;
            algorithmDropdown.Items.AddRange(new string[]
            {
                "Primal Simplex Algorithm",
                "Revised Primal Simplex Algorithm",
                "Branch and Bound Simplex Algorithm",
                "Revised Branch and Bound Simplex Algorithm",
                "Cutting Plane Algorithm",
                "Revised Cutting Plane Algorithm",
                "Branch and Bound Knapsack Algorithm"
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
            iterationOutputTextBox = new TextBox();
            iterationOutputTextBox.Multiline = true;
            iterationOutputTextBox.ScrollBars = ScrollBars.Vertical;
            iterationOutputTextBox.Size = new Size(600, 250);
            iterationOutputTextBox.Location = new Point((this.ClientSize.Width / 2) - 300, 320);
            iterationOutputTextBox.PlaceholderText = "Iterations of Tableau will appear here...";
            this.Controls.Add(iterationOutputTextBox);
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

        // Example function: Add iteration steps manually
        public void AddIteration(string step)
        {
            if (!string.IsNullOrWhiteSpace(step))
            {
                iterationOutputTextBox.AppendText(step + Environment.NewLine);
            }
        }
    }
}
