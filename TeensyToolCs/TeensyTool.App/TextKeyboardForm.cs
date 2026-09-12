using System;
using System.Windows.Forms;
using Keypad;

namespace TeensyTool.App
{
    /// <summary>
    /// Samma mönster som NumericKeypadForm — ett eget, litet dialogskal
    /// runt AGO:s RIKTIGA, oförändrade Keyboard-kontroll (ärver samma
    /// GenericKeypad-bas som NumKeypad, se dess egen kommentar). Byggd
    /// separat från AGO:s FormKeyboard.cs av samma skäl som
    /// NumericKeypadForm byggdes separat från FormNumeric.cs — den
    /// beror på AgOpenGPS.Core.Translations bara för en enda
    /// felsträng, oproportionerligt att dra in för det.
    ///
    /// Keyboard använder EN ANNAN teckenkonvention än NumKeypad —
    /// unicode-kontrolltecken istället för bokstäver ('\u0008'=
    /// backspace, '\u0005'=clear, '\u0027'=cancel, '\u0004'=OK) — helt
    /// logiskt, eftersom ett fullt tangentbord har RIKTIGA bokstäver
    /// som 'K'/'X'/'B'/'C' som vanliga, tryckbara tangenter, till
    /// skillnad från NumKeypad där bara siffror förekommer.
    /// </summary>
    public class TextKeyboardForm : Form
    {
        private readonly TextBox _display;

        public string ReturnValue { get; private set; }

        public TextKeyboardForm(string currentValue)
        {
            Text = "Enter Text";
            FormBorderStyle = FormBorderStyle.FixedDialog;
            MaximizeBox = false;
            MinimizeBox = false;
            StartPosition = FormStartPosition.CenterParent;
            // Keyboard är betydligt större än NumKeypad (fler tangenter
            // per rad) — 925x525, verifierat direkt i
            // Keyboard.Designer.cs egen "this.Size = new Size(925, 525)"-
            // rad. (Min första gissning här var 750x350 — för liten,
            // hade skurit av knappsatsen. Rättad efter att faktiskt ha
            // kontrollerat, inte bara antagit.)
            Width = 950;
            Height = 620;

            _display = new TextBox
            {
                Text = currentValue,
                Left = 10, Top = 10, Width = 910,
                Font = new System.Drawing.Font("Tahoma", 14F, System.Drawing.FontStyle.Bold),
                ReadOnly = true
            };
            Controls.Add(_display);

            var keyboard = new Keyboard { Left = 10, Top = 45 };
            keyboard.ButtonPressed += Keyboard_ButtonPressed;
            Controls.Add(keyboard);
        }

        /// <summary>
        /// Handles every keypress from the embedded, real Keyboard
        /// control (Keypad project, AGO's own unchanged code) — see
        /// this class's own header comment for the full unicode-
        /// control-character convention (backspace/clear/cancel/OK)
        /// this switches on. Plain characters (letters, digits,
        /// punctuation) fall through to the final else and are just
        /// appended, no special handling needed for those.
        /// </summary>
        private void Keyboard_ButtonPressed(object sender, KeyPressEventArgs e)
        {
            char key = e.KeyChar;

            if (key == '\u0008')   // backspace
            {
                if (_display.Text.Length > 0)
                    _display.Text = _display.Text.Remove(_display.Text.Length - 1);
            }
            else if (key == '\u0005')   // clear
            {
                _display.Text = "";
            }
            else if (key == '\u0027')   // cancel
            {
                DialogResult = DialogResult.Cancel;
                Close();
            }
            else if (key == '\u0004')   // OK
            {
                ReturnValue = _display.Text;
                DialogResult = DialogResult.OK;
                Close();
            }
            else
            {
                // Vanlig bokstav/siffra/mellanslag — läggs bara till
                // rakt av, ingen validering behövs för fri text (till
                // skillnad från NumericKeypadForm, som kräver ett
                // giltigt tal inom min/max).
                _display.Text += key;
            }
        }
    }
}
