using System;
using System.Windows.Forms;
using TeensyTool.Protocol;

namespace TeensyTool.App
{
    /// <summary>
    /// En ny, egen kontrolltyp — INTE en återanvändning av
    /// NumericSettingControl, av ett genuint skäl: SETROLLKALMAN/
    /// SETHEADINGKALMAN skickar TRE kommaseparerade tal i ETT enda
    /// kommando (mea,est,q), och firmware svarar aldrig med dessa
    /// värden i PDIAG — ingen "Current"-rad går att visa/synka, till
    /// skillnad från VARJE annan sifferinställning vi byggt hittills.
    /// Matchar Python-verktygets egen make_kalman_row(): tre textfält
    /// med hårdkodade startvärden, en Set-knapp, aldrig uppdaterade
    /// efter första visningen.
    /// </summary>
    public class KalmanTuningControl : UserControl
    {
        private readonly TextBox _txtMea;
        private readonly TextBox _txtEst;
        private readonly TextBox _txtQ;
        private readonly Label _lblError;

        /// <summary>
        /// Ger tillbaka alla tre redan validerade tal tillsammans, inte
        /// ett i taget — firmware kräver alla tre samtidigt i SAMMA
        /// kommando (sscanf-formatet "%f,%f,%f"), till skillnad från
        /// varje annan inställning där ett värde skickas för sig.
        /// </summary>
        public event EventHandler<(double mea, double est, double q)> ValuesSubmitted;

        public KalmanTuningControl(string label, double defaultMea, double defaultEst, double defaultQ)
        {
            Width = 420;
            Height = 30;

            var lbl = new Label { Text = label, Left = 0, Top = 5, Width = 65 };

            Label MakeSubLabel(string text, int left) =>
                new Label { Text = text, Left = left, Top = 5, Width = 20 };

            // Panel-med-Padding-experimentet backat helt — se
            // NumericSettingControl.cs egen kommentar om varför.
            // Tillbaka till den ursprungliga, enkla TextBox-formen,
            // oförändrad bredd (45px, samma som den redan hade).
            _txtMea = new TextBox { Left = 90, Top = 2, Width = 45, Text = defaultMea.ToString("0.####") };
            _txtEst = new TextBox { Left = 165, Top = 2, Width = 45, Text = defaultEst.ToString("0.####") };
            _txtQ = new TextBox { Left = 240, Top = 2, Width = 45, Text = defaultQ.ToString("0.####") };

            // Samma touch-vänliga knappsats som NumericSettingControl
            // — alla tre fält delar samma min/max (strikt > 0), given
            // att firmware kräver det identiskt för mea/est/q.
            _txtMea.Click += (s, e) => OpenKeypadFor(_txtMea);
            _txtEst.Click += (s, e) => OpenKeypadFor(_txtEst);
            _txtQ.Click += (s, e) => OpenKeypadFor(_txtQ);

            var btnSet = new Button { Text = "Set", Left = 295, Top = 0, Width = 55 };
            btnSet.Click += BtnSet_Click;

            _lblError = new Label { Text = "", Left = 0, Top = 30, Width = 420, ForeColor = System.Drawing.Color.Red };

            Controls.Add(lbl);
            Controls.Add(MakeSubLabel("mea", 65));
            Controls.Add(_txtMea);
            Controls.Add(MakeSubLabel("est", 140));
            Controls.Add(_txtEst);
            Controls.Add(MakeSubLabel("q", 215));
            Controls.Add(_txtQ);
            Controls.Add(btnSet);
            Controls.Add(_lblError);

            // Höjden justerad för att rymma felraden under — annars
            // identisk layouttanke som NumericSettingControl.
            Height = 50;
        }

        /// <summary>
        /// Öppnar samma knappsats som NumericSettingControl använder,
        /// för VILKET som helst av de tre textfälten (mea/est/q) —
        /// "for TextBox target" som parameter gör metoden generell
        /// istället för att behöva tre nästan identiska kopior.
        /// </summary>
        private void OpenKeypadFor(TextBox target)
        {
            // Samma komma-till-punkt-hantering som NumericValidator
            // (se den filens egen, utförliga kommentar för varför) —
            // utan den skulle detta falla tillbaka på datorns EGEN
            // regioninställning (CurrentCulture, C#:s standardbeteende
            // när ingen kultur anges explicit) istället för ett
            // konsekvent, förutsägbart beteende. Bara kosmetisk risk
            // här (avgör bara knappsatsens FÖRIFYLLDA startvärde, inte
            // den faktiska valideringen/sändningen — den går alltid
            // via NumericValidator ändå) — men värt att göra
            // konsekvent snarare än att låta två olika
            // tolkningsregler leva kvar sida vid sida i samma projekt.
            double currentValue = double.TryParse(
                target.Text.Replace(',', '.'), System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out double v) ? v : 0.0001;

            using (var keypad = new NumericKeypadForm(0.0001, 1000, currentValue))
            {
                if (keypad.ShowDialog() == DialogResult.OK)
                {
                    target.Text = keypad.ReturnValue.ToString("0.####");
                }
            }
        }

        /// <summary>
        /// Validates and sends all three mea/est/q values together,
        /// as ONE SETROLLKALMAN/SETHEADINGKALMAN command — never
        /// individually, since firmware's sscanf() expects all three
        /// in a single comma-separated command. See the inline
        /// comment below for the strict-positive validation range.
        /// </summary>
        private void BtnSet_Click(object sender, EventArgs e)
        {
            // Firmware kräver STRIKT större än noll för alla tre
            // (mea > 0.0f && est > 0.0f && q > 0.0f, se
            // SETROLLKALMAN/SETHEADINGKALMAN i zHandlers.ino) — inte
            // "noll eller mer". Återanvänder NumericValidator med ett
            // mycket litet, praktiskt taget-noll min-värde (0.0001)
            // snarare än en helt ny valideringsmetod, eftersom den
            // redan är testad och pålitlig. Ingen övre gräns i
            // firmware, men 1000 satt här som ett rimligt, praktiskt
            // tak — inget riktigt tuningvärde skulle rimligen behöva
            // vara större.
            bool okMea = NumericValidator.TryValidate(_txtMea.Text, 0.0001, 1000, out double mea, out string errMea);
            bool okEst = NumericValidator.TryValidate(_txtEst.Text, 0.0001, 1000, out double est, out string errEst);
            bool okQ = NumericValidator.TryValidate(_txtQ.Text, 0.0001, 1000, out double q, out string errQ);

            if (!okMea) { _lblError.Text = "mea: " + errMea; return; }
            if (!okEst) { _lblError.Text = "est: " + errEst; return; }
            if (!okQ) { _lblError.Text = "q: " + errQ; return; }

            _lblError.Text = "";
            ValuesSubmitted?.Invoke(this, (mea, est, q));
        }
    }
}
