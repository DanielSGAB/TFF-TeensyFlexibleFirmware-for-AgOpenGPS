using System;
using System.Windows.Forms;
using TeensyTool.Protocol;

namespace TeensyTool.App
{
    /// <summary>
    /// En återanvändbar, sammansatt kontroll — "UserControl" är C#:s
    /// sätt att bygga en egen, liten "widget-klass" som kombinerar
    /// flera inbyggda kontroller (Label, TextBox, Button) till EN enda
    /// enhet man kan lägga in flera gånger, med samma beteende varje
    /// gång. Motsvarar ungefär att skriva en egen funktion i Python som
    /// bygger upp en hel liten panel och returnerar den — fast här är
    /// resultatet en riktig, egen klass, inte bara en funktion.
    ///
    /// Byggd MEDVETET separat från AGO:s egen FormNumeric-dialog (som
    /// vi undersökte och bestämde inte passade vårt behov direkt) —
    /// men NumericValidator, logiken den använder under huven, är
    /// samma oavsett. Den här UserControl-klassen är alltså den delen
    /// vi vet kommer bytas ut senare (mot en riktig touchskärms-
    /// knappsats i AGO-stil); NumericValidator är den delen som
    /// förblir densamma.
    /// </summary>
    public class NumericSettingControl : UserControl
    {
        private readonly Label _lblName;
        private readonly Label _lblCurrent;
        private readonly TextBox _txtNewValue;
        private readonly Button _btnSet;
        private readonly Label _lblError;

        private readonly double _min;
        private readonly double _max;

        /// <summary>
        /// "event EventHandler&lt;double&gt; ValueSubmitted" —
        /// utlöses bara när användaren skrivit in ETT GILTIGT värde
        /// och klickat Set. Den som lägger till kontrollen i sitt
        /// fönster (MainForm) prenumererar på den här händelsen för
        /// att faktiskt SKICKA rätt SETxxx-kommando — den här klassen
        /// vet ingenting om TeensyConnection eller kommandonamn alls,
        /// helt medvetet, för att hålla den generellt återanvändbar
        /// för VILKEN siffer-inställning som helst.
        /// </summary>
        public event EventHandler<double> ValueSubmitted;

        public NumericSettingControl(string settingName, double min, double max)
        {
            _min = min;
            _max = max;

            // Höjd/bredd satt här, en gång — den som ANVÄNDER
            // kontrollen (MainForm) bestämmer bara VAR den ska sitta
            // (.Left/.Top), inte hur den ser ut invändigt.
            Width = 380;
            Height = 65;

            // AutoSize=false + generösare Width (250 -> 280) — samma
            // fix som redan gjorts på flera andra etiketter genom
            // hela projektet, missad här. Längsta förväntade text
            // ("IMU Roll Offset (matches IMU to Dual)") klipptes.
            _lblName = new Label
            {
                Text = settingName, Left = 0, Top = 0, Width = 280, Height = 18,
                AutoSize = false
            };
            // Avstånd till _lblName ovanför ökat rejält (20 -> 30 för
            // _lblCurrent/_txtNewValue/_btnSet, +10px), efter att
            // varken font-vikt, BorderStyle, Panel-ram eller
            // AutoSize-fixen löste den rapporterade "brutna konturen"
            // — ett mer generöst avstånd till texten ovanför är nästa,
            // enklare sak att pröva. Height 55 -> 65 i takt (nedan) för
            // att rymma det.
            //
            // Current-värdet medvetet FÖRSTORAT relativt resten av
            // kontrollen — matchar AGO:s eget, återkommande mönster
            // att göra det faktiska, viktiga siffervärdet visuellt mer
            // framträdande än sin egen etikett (t.ex. NTRIP-dialogens
            // Caster Port-siffra i 20.25pt fetstil, mot omgivande
            // etiketter i 14.25pt).
            _lblCurrent = new Label
            {
                // AutoSize=false — SAKNADES helt tidigare. Utan den
                // ignoreras Width=200 helt (WinForms standard är
                // AutoSize=true), och ett längre värde hade kunnat få
                // denna 12pt FETSTILTA etikett att växa förbi sin egen
                // deklarerade bredd, rakt in i TextBox-fältets
                // vänsterkant.
                Text = "Current: --", Left = 0, Top = 30, Width = 200, Height = 22,
                AutoSize = false,
                Font = new System.Drawing.Font("Tahoma", 12F, System.Drawing.FontStyle.Bold)
            };
            _txtNewValue = new TextBox
            {
                // Width 80 -> 45, matchande Kalman-fältens bredd (de
                // har alltid sett korrekta ut). Font-override till
                // normalvikt togs bort helt — ärver nu MainForm:s
                // fetstil precis som Kalman-fälten, se historiken för
                // hela resonemanget bakom det.
                Left = 210, Top = 32, Width = 45
            };
            // Klick i fältet öppnar den stora, touch-vänliga
            // knappsatsen (NumericKeypadForm) som ett bekvämt
            // alternativ — själva TextBox-fältet förblir ändå
            // skrivbart med vanligt tangentbord också, inget tas bort,
            // bara ett extra sätt läggs till.
            _txtNewValue.Click += (s, e) => OpenKeypad();

            _btnSet = new Button { Text = "Set", Left = 295, Top = 30, Width = 60 };
            _lblError = new Label
            {
                Text = "", Left = 0, Top = 55, Width = 380,
                ForeColor = System.Drawing.Color.Red
            };

            _btnSet.Click += BtnSet_Click;

            Controls.Add(_lblName);
            Controls.Add(_lblCurrent);
            Controls.Add(_txtNewValue);
            Controls.Add(_btnSet);
            Controls.Add(_lblError);
        }

        /// <summary>
        /// Anropas av MainForm när ny data anländer från Teensyn —
        /// motsvarar exakt vad Pythons "Current: {value}"-uppdatering
        /// gjorde, fast här som en egen, tydlig publik metod istället
        /// för att MainForm rör kontrollens INVÄNDIGA etiketter direkt.
        /// </summary>
        public void UpdateCurrentValue(double value)
        {
            _lblCurrent.Text = "Current: " + value.ToString("0.####");
        }

        // Har inget att göra med UpdateCurrentValue ovan — den rör
        // bara "Current"-etiketten, aldrig textfältet själv, ett
        // beteende som redan gällt genomgående för ALLA
        // NumericSettingControl-instanser sedan de först byggdes (inte
        // bara Keya). Given att C#-fältet dessutom ALLTID startar TOMT
        // (till skillnad från Python, som förifyller det) fanns aldrig
        // samma "tyst, felaktig överskrivning"-risk här — men samma
        // synk-princip är ändå värd att erbjuda, så fältet visar det
        // verkliga, sparade värdet direkt istället för att kräva att
        // användaren skriver om det bara för att justera lite.
        private bool _syncedOnce = false;

        /// <summary>
        /// Fyller textfältet med det FAKTISKA, mottagna värdet — men
        /// bara EN gång per session (samma engångsprincip som Python-
        /// versionens motsvarande fix), inte varje gång ny data
        /// kommer in var 2:e sekund, vilket annars skulle skriva över
        /// vad användaren aktivt håller på att skriva innan de hinner
        /// klicka Set.
        /// </summary>
        public void SyncEntryOnce(double value)
        {
            if (_syncedOnce) return;
            _txtNewValue.Text = value.ToString("0.####");
            _syncedOnce = true;
        }

        /// <summary>
        /// Fyller textfältet med ETT ANGIVET default-värde — anropas
        /// bara av en explicit "Load Defaults"-knapp, ALDRIG
        /// automatiskt. Skickar INGET till firmware — kräver
        /// fortfarande ett separat Set-klick, precis som varje annan
        /// ändring i det här fältet.
        /// </summary>
        public void LoadDefault(double defaultValue)
        {
            _txtNewValue.Text = defaultValue.ToString("0.####");
        }

        private void BtnSet_Click(object sender, EventArgs e)
        {
            bool ok = NumericValidator.TryValidate(
                _txtNewValue.Text, _min, _max, out double value, out string error);

            if (!ok)
            {
                _lblError.Text = error;
                return;
            }

            _lblError.Text = "";

            // "ValueSubmitted?.Invoke(this, value)" — samma säkra
            // mönster som DataReceived i TeensyConnection: fråga
            // "finns det någon som lyssnar?" innan vi faktiskt utlöser
            // händelsen, så ett anrop utan någon prenumerant inte
            // kraschar.
            ValueSubmitted?.Invoke(this, value);
        }

        /// <summary>
        /// Öppnar NumericKeypadForm som en MODAL dialog — "ShowDialog()"
        /// (inte bara "Show()") pausar resten av programmet tills
        /// dialogen stängs, exakt som AGO:s egen FormNumeric öppnas.
        /// Given att dialogen redan validerat internt (kan bara stängas
        /// med OK om värdet är giltigt) skickar vi direkt vidare utan
        /// att kräva ett andra klick på vår egen Set-knapp också —
        /// matchar hur AGO:s egen FormNumeric fungerar: dialogens OK
        /// ÄR den slutgiltiga bekräftelsen.
        /// </summary>
        private void OpenKeypad()
        {
            // Se KalmanTuningControl.OpenKeypadFor() för identisk
            // kommentar om varför — samma konsekventa
            // komma-till-punkt-hantering som NumericValidator.
            double currentValue = double.TryParse(
                _txtNewValue.Text.Replace(',', '.'), System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out double v) ? v : _min;

            using (var keypad = new NumericKeypadForm(_min, _max, currentValue))
            {
                if (keypad.ShowDialog() == DialogResult.OK)
                {
                    _txtNewValue.Text = keypad.ReturnValue.ToString("0.####");
                    _lblError.Text = "";
                    ValueSubmitted?.Invoke(this, keypad.ReturnValue);
                }
            }
        }
    }
}
