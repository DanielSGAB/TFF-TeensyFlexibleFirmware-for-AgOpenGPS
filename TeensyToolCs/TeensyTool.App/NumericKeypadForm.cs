using System;
using System.Drawing;
using System.Windows.Forms;
using TeensyTool.Protocol;
using Keypad;

namespace TeensyTool.App
{
    /// <summary>
    /// En egen, modal popup-dialog — men SJÄLVA KNAPPSATSEN är nu
    /// AGO:s egen, RIKTIGA NumKeypad-kontroll (från det återanvända
    /// Keypad-projektet, "using Keypad;" ovan), inte en egenbyggd
    /// kopia. Bara skalet runt den (display, min/max-etiketter, OK/
    /// Cancel-hantering) är vårt eget — motsvarande, minimala
    /// motsvarighet till AGO:s egen FormNumeric.cs, som vi MEDVETET
    /// inte återanvänder rakt av (den beror på
    /// AgOpenGPS.Core.Translations bara för en enda felsträng —
    /// oproportionerligt att dra in för det).
    ///
    /// Valideringen (min/max) återanvänder VÅR EGEN, redan testade
    /// NumericValidator — AGO:s egen FormNumeric gör i praktiken samma
    /// sak, bara med sin egen, separata kod för det.
    /// </summary>
    public class NumericKeypadForm : Form
    {
        private readonly double _min;
        private readonly double _max;
        private readonly TextBox _display;
        private readonly Label _lblMin;
        private readonly Label _lblMax;
        private bool _isFirstKey = true;

        public double ReturnValue { get; private set; }

        public NumericKeypadForm(double min, double max, double currentValue)
        {
            _min = min;
            _max = max;

            Text = "Enter Value";
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = false;
            MinimizeBox = false;
            StartPosition = FormStartPosition.CenterParent;
            // Storleken anpassad efter NumKeypads EGEN, faktiska
            // storlek (456x430, sett direkt i dess NumKeypad.Designer.cs)
            // plus utrymme för vår display/etiketter ovanför — inte
            // gissad, avläst.
            Width = 500;
            Height = 570;

            _display = new TextBox
            {
                Text = currentValue.ToString("0.####"),
                Left = 10, Top = 10, Width = 440,
                Font = new System.Drawing.Font("Tahoma", 20F, System.Drawing.FontStyle.Bold),
                TextAlign = HorizontalAlignment.Right,
                ReadOnly = true   // bara knappsatsen ska kunna ändra innehållet
            };
            Controls.Add(_display);

            _lblMin = new Label { Text = $"Min: {min}", Left = 10, Top = 45, Width = 200 };
            _lblMax = new Label { Text = $"Max: {max}", Left = 250, Top = 45, Width = 200, TextAlign = ContentAlignment.MiddleRight };
            Controls.Add(_lblMin);
            Controls.Add(_lblMax);

            // Här är den ENDA raden som skiljer sig från att bygga
            // knapparna själv — en riktig, oförändrad NumKeypad-
            // instans, AGO:s egen kod, inte vår tolkning av den.
            var numKeypad = new NumKeypad { Left = 10, Top = 75 };
            numKeypad.ButtonPressed += NumKeypad_ButtonPressed;
            Controls.Add(numKeypad);
        }

        /// <summary>
        /// Motsvarar EXAKT AGO:s egen RegisterKeypad1_ButtonPressed i
        /// FormNumeric.cs — samma teckenhantering (samma
        /// specialtecken: 'B'=backspace, '.'=decimal, '-'=plus/minus,
        /// 'C'=clear, 'X'=cancel, 'K'=OK), eftersom det är NumKeypads
        /// EGNA, redan bestämda konvention (se dess RaiseButtonPressed-
        /// anrop) — vi följer den, uppfinner inte en egen.
        /// </summary>
        private void NumKeypad_ButtonPressed(object sender, KeyPressEventArgs e)
        {
            char key = e.KeyChar;

            if (_isFirstKey && char.IsDigit(key))
            {
                _display.Text = "";
                _isFirstKey = false;
            }

            if (char.IsDigit(key))
            {
                _display.Text += key;
            }
            else if (key == 'B')
            {
                if (_display.Text.Length > 0)
                    _display.Text = _display.Text.Remove(_display.Text.Length - 1);
            }
            else if (key == '.')
            {
                if (!_display.Text.Contains("."))
                {
                    _display.Text += ".";
                    if (_display.Text.StartsWith(".")) _display.Text = "0" + _display.Text;
                }
            }
            else if (key == '-')
            {
                if (_display.Text.StartsWith("-"))
                    _display.Text = _display.Text.Substring(1);
                else
                    _display.Text = "-" + _display.Text;
            }
            else if (key == 'C')
            {
                _display.Text = "";
                _isFirstKey = true;
            }
            else if (key == 'X')
            {
                DialogResult = DialogResult.Cancel;
                Close();
            }
            else if (key == 'K')
            {
                // Återanvänder VÅR EGEN, redan testade
                // NumericValidator — se klassens egen kommentar högst
                // upp för varför.
                bool ok = NumericValidator.TryValidate(_display.Text, _min, _max, out double value, out string error);
                if (!ok)
                {
                    _lblMin.ForeColor = System.Drawing.Color.Red;
                    _lblMax.ForeColor = System.Drawing.Color.Red;
                    _display.Text = error;
                    _isFirstKey = true;
                    return;
                }

                ReturnValue = value;
                DialogResult = DialogResult.OK;
                Close();
            }
        }
    }
}
