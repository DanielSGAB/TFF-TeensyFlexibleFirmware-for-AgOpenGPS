using System;
using System.Windows.Forms;

namespace TeensyTool.App
{
    /// <summary>
    /// Fristående popup-fönster (samma mönster som
    /// NumericKeypadForm/TextKeyboardForm — ett eget, litet fönster,
    /// inte inbyggt i huvudflikarna) för att öppna och analysera en
    /// tidigare skapad loggfil. Ren läsning — skriver aldrig till
    /// filen, skickar aldrig något till Teensyn.
    /// </summary>
    public class LogAnalyzerForm : Form
    {
        private readonly TextBox _txtSummary;
        private readonly string _initialDirectory;

        public LogAnalyzerForm(string initialDirectory)
        {
            _initialDirectory = initialDirectory;

            Text = "Analyze Log File";
            Width = 560;
            Height = 520;
            StartPosition = FormStartPosition.CenterParent;

            var btnOpen = new Button
            {
                Text = "Open Log File...",
                Left = 15, Top = 15, Width = 150, Height = 30
            };
            btnOpen.Click += BtnOpen_Click;
            Controls.Add(btnOpen);

            var lblHint = new Label
            {
                Text = "Pick a .txt log file created by this tool's \"Enable logging\" checkbox — "
                    + "you'll get a short summary of any problems found in it, not the raw log text.",
                Left = 180, Top = 20, Width = 360, Height = 40,
                ForeColor = System.Drawing.Color.Gray
            };
            Controls.Add(lblHint);

            _txtSummary = new TextBox
            {
                Left = 15, Top = 65, Width = 515, Height = 410,
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Vertical,
                Font = new System.Drawing.Font("Consolas", 9.5f),
                Text = "No file opened yet — click \"Open Log File...\" above to get started."
            };
            Controls.Add(_txtSummary);
        }

        private void BtnOpen_Click(object sender, EventArgs e)
        {
            // OpenFileDialog är en FÄRDIG, inbyggd WinForms-
            // dialogruta — samma princip som FolderBrowserDialog
            // används för loggmappsvalet: ingen egen filväljar-UI
            // byggd här.
            using (var dialog = new OpenFileDialog
            {
                Filter = "Log files (*.txt)|*.txt|All files (*.*)|*.*",
                InitialDirectory = _initialDirectory,
                Title = "Choose a log file to analyze"
            })
            {
                if (dialog.ShowDialog() != DialogResult.OK) return;

                _txtSummary.Text = "Analyzing...";
                Refresh();  // visar "Analyzing..." direkt, inte bara efter Analyze() redan kört klart
                _txtSummary.Text = LogAnalyzer.Analyze(dialog.FileName);
            }
        }
    }
}
