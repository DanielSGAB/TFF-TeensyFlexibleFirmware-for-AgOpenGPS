using System;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Windows.Forms;
using TeensyTool.Protocol;

namespace TeensyTool.App
{
    /// <summary>
    /// Full återuppbyggnad av alla fem flikar, matchande Python
    /// Teensy Tool v0.3.8 innehållsmässigt — Kalmanfiltret medvetet
    /// avvaktat, byggs som ett eget, senare steg (en ny kontrolltyp,
    /// tre kommaseparerade tal i ETT kommando, förtjänar egen omsorg).
    ///
    /// Två paneler flyttade från sina ursprungliga, felaktiga platser
    /// i föregående version: Heading Alpha hör hemma i Operation (inte
    /// Receiver Configuration), och WAS/U-turn-panelen hör hemma i
    /// Operation (inte Vehicle Configuration) — bekräftat genom att
    /// läsa igenom Python-källkoden rad för rad, inte gissat.
    /// </summary>
    public class MainForm : Form
    {
        private TeensyConnection _connection;
        private readonly DiagnosticLogger _logger = new DiagnosticLogger();
        // Spårar tidpunkter för LogHeartbeat()'s tre-tillstånds-logik —
        // motsvarar Pythons last_rx/last_successful_apply.
        private DateTime? _lastRawPacketTime;
        private DateTime? _lastSuccessfulUpdateTime;
        private bool _wasSignalLost = false;
        // Kommer ihåg senast valda mapp under sessionen — null tills
        // användaren väljer en (eller om de alltid avbryter dialogen,
        // vilket faller tillbaka till .exe-mappen varje gång).
        private string _lastChosenLogDirectory = null;

        // --- Operation ---
        private Label _lblSatsMaster;
        private Label _lblSatsSlave;
        private PercentageSplitBar _splitBar;
        private Label _lblSol;
        private Label _lblHprSats;
        private Label _lblHprAge;
        private Label _lblOffset;
        private NumericSettingControl _numHeadingAlpha;
        private NumericSettingControl _numRollAlpha;
        private NumericSettingControl _numSatsFull;
        private Label _lblWasLeft;
        private Label _lblWasRight;
        private Label _lblSteerAngle;
        private NumericSettingControl _numUTurnStrength;
        private NumericSettingControl _numDualHold;
        private NumericSettingControl _numDualRamp;
        private CheckBox _cbLogfile;
        private Label _lblLogfileStatus;
        private Label _lblRawPackets;
        private Label _lblLastRawSentence;
        private TextBox _txtCommand;
        private Button _btnSend;

        // --- Receiver Configuration ---
        private RadioButton _rbUm982;
        private RadioButton _rbSingleImu;
        private RadioButton _rbDual;
        private CheckBox _cbGnssPassthrough;
        // _rbDualAntennaType borttagen — se BuildReceiverConfigurationTab()
        // egen kommentar om varför "Dual Antenna Receiver Type"-gruppen
        // togs bort helt (aldrig kopplad till firmware, mest förvirrande).
        private RadioButton[] _rbDualSingleType;    // SETRECEIVERTYPE (F9P/UM980 implemented)

        // --- IMU ---
        private RadioButton _rbImuBno08x;
        private RadioButton _rbImuTm171;
        private RadioButton _rbImuNone;
        private Label _lblDualRollLive;
        private Label _lblImuRollLive;
        private NumericSettingControl _numRollZero;
        private CheckBox _cbAutoRollAdjust;
        private Label _lblAutoRollStatus;
        private NumericSettingControl _numRollAutoDeadband;
        private NumericSettingControl _numRollAutoAlpha;
        private NumericSettingControl _numImuAxis;
        private CheckBox _cbRollInvert;

        // --- Vehicle Configuration ---
        private RadioButton[] _rbBrand;              // 9 options: 0-7 + BRAND_NONE(8)
        private RadioButton _rbMotorPwm;
        private RadioButton _rbMotorKeya;
        private RadioButton _rbWasNormal;
        private RadioButton _rbWasKeya;

        // --- Keya Auto-Zero Tuning (v0.3.10) ---
        private Label _lblAzZeroStatus;
        private NumericSettingControl _numAzSpeedMin;
        private NumericSettingControl _numAzYawRateMax;
        private NumericSettingControl _numAzGpsHdgMax;
        private NumericSettingControl _numAzTimeSlow;
        private NumericSettingControl _numAzTimeFast;
        private NumericSettingControl _numAzSpeedSlow;
        private NumericSettingControl _numAzSpeedFast;
        private NumericSettingControl _numAzUseBno;
        private NumericSettingControl _numAzUseGps;
        private NumericSettingControl _numAzBeta;

        // --- Board Configuration ---
        private RadioButton[] _rbSlot1;
        private RadioButton[] _rbSlot2;

        public MainForm()
        {
            Text = "TeensyTool v0.3.18 (matching TFF firmware version)";
            // 620, inte bara "precis räcker" (bredaste panelen,
            // Steering Brand, slutar på 440px) — Windows egna
            // fönsterramar och särskilt DPI-skalning (mycket vanligt
            // på moderna skärmar, 125%/150%) äter en påtaglig del av
            // det totala Width-värdet innan klientytan där
            // kontrollerna faktiskt ligger ens börjar. Satt generöst
            // efter ett verkligt fältrapporterat problem (fönstret
            // krävde manuell breddning varje gång), inte bara den
            // teoretiska minimibredden.
            Width = 620;
            Height = 560;

            // Sätts EN gång, här — WinForms-kontroller som inte fått
            // en egen, uttrycklig Font-egenskap ÄRVER automatiskt sin
            // förälders font, hela vägen ner genom kontrollträdet. Det
            // är därför den här enda raden räcker för att ändra
            // typsnitt/storlek/fetstil på praktiskt taget ALLA
            // etiketter, radioknappar, kryssrutor och textfält i hela
            // programmet, utan att röra dem en och en.
            //
            // Värdena (Tahoma, fetstil som standard) är inte gissade —
            // extraherade direkt ur AgOpenGPS/AgIO:s egen, färdiga
            // Designer.cs-kod, där "Bold" genomgående förekommer
            // OFTARE än "Regular" i nästan varje storleksklass, och
            // Tahoma är den ENDA fontfamilj som någonsin används,
            // över 126 genomsökta formulärfiler.
            Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Bold);

            var tabs = new TabControl { Dock = DockStyle.Fill };
            tabs.TabPages.Add(BuildReceiverConfigurationTab());
            tabs.TabPages.Add(BuildImuTab());
            tabs.TabPages.Add(BuildVehicleConfigurationTab());
            tabs.TabPages.Add(BuildBoardConfigurationTab());
            tabs.TabPages.Add(BuildOperationTab());
            Controls.Add(tabs);

            // Ljus, neutral flikbakgrund — WhiteSmoke var den vanligast
            // förekommande, faktiskt satta bakgrundsfärgen i hela
            // AGO-kodbasen (efter "Transparent", som bara betyder "ärv
            // förälderns färg", inte ett eget val).
            foreach (TabPage page in tabs.TabPages)
                page.BackColor = System.Drawing.Color.WhiteSmoke;

            // FlatStyle ärvs INTE automatiskt som Font gör — måste
            // sättas per knapp. Görs här, EN gång, genom att gå igenom
            // HELA det redan byggda kontrollträdet rekursivt — så ingen
            // enskild knapp riskerar att missas eller behöva sin egen,
            // manuellt upprepade rad. FlatStyle.Flat var det enskilt
            // mest dominerande valet i hela AGO-kodbasen (924 mot 37
            // "System" och 33 "Popup" tillsammans).
            ApplyFlatButtonStyleRecursively(this);

            StartConnection();
        }

        /// <summary>
        /// Går igenom en kontroll och ALLA dess barn, rekursivt (en
        /// GroupBox/Panel/TabPage kan i sin tur innehålla fler
        /// kontroller, som i sin tur kan innehålla ännu fler) — och
        /// sätter FlatStyle.Flat på varje Button den hittar på vägen.
        /// Garanterat heltäckande, till skillnad från att manuellt lista
        /// upp varje knappvariabel för sig (lätt att missa en i en så
        /// här stor, växande kodbas).
        /// </summary>
        private void ApplyFlatButtonStyleRecursively(Control root)
        {
            foreach (Control child in root.Controls)
            {
                if (child is Button button)
                {
                    button.FlatStyle = FlatStyle.Flat;
                }
                // Rekursivt anrop — går vidare in i child:s EGNA barn,
                // oavsett vilken typ av kontroll child råkar vara.
                ApplyFlatButtonStyleRecursively(child);
            }
        }

        // =================================================================
        // Operation tab
        // =================================================================
        private TabPage BuildOperationTab()
        {
            // "AutoScroll = true": Python's tab used a scrollable frame
            // (_make_scrollable_tab) since this tab genuinely has more
            // content than fits one screen — WinForms' TabPage has the
            // same built-in scrolling capability via this one property,
            // no separate scrollable-container class needed.
            var page = new TabPage("Operation") { AutoScroll = true };
            int y = 10;

            // Flyttad till TOPPEN — den ger redan en rikare, mer exakt
            // bild av det aktuella läget än den gamla "Mode: DUAL/
            // FALLBACK"-texten någonsin gjorde (kontinuerlig procent,
            // inte bara ett binärt namn). Kör man PANDA/Single+IMU
            // landar baren naturligt på 100% Single+IMU; ren Dual utan
            // IMU landar på 100% DUAL — därför tas "Mode:"-etiketten
            // bort helt nedan, den blev överflödig.
            _splitBar = new PercentageSplitBar("Single+IMU", "DUAL") { Left = 20, Top = y };
            page.Controls.Add(_splitBar); y += 75;

            // Master/Slave nu SIDA VID SIDA på samma rad, istället för
            // staplade — samma horisontella yta, mindre vertikalt
            // utrymme totalt.
            _lblSatsMaster = new Label { Text = "Master sats (GGA): --", Left = 20, Top = y, Width = 200 };
            page.Controls.Add(_lblSatsMaster);
            _lblSatsSlave = new Label { Text = "Slave sats (GPGGAH): --", Left = 230, Top = y, Width = 200 };
            page.Controls.Add(_lblSatsSlave); y += 22;

            _lblSol = new Label { Text = "HPR solution: ---", Left = 20, Top = y, Width = 300 };
            page.Controls.Add(_lblSol); y += 22;

            _lblHprSats = new Label { Text = "HPR heading sats: --", Left = 20, Top = y, Width = 300 };
            page.Controls.Add(_lblHprSats); y += 22;

            _lblHprAge = new Label { Text = "Last HPR: --", Left = 20, Top = y, Width = 300 };
            page.Controls.Add(_lblHprAge); y += 22;

            _lblOffset = new Label { Text = "IMU → HPR heading offset: --- °", Left = 20, Top = y, Width = 300 };
            page.Controls.Add(_lblOffset); y += 30;

            _numHeadingAlpha = new NumericSettingControl("Heading Alpha (initial value)", 0.0, 1.0) { Left = 20, Top = y };
            _numHeadingAlpha.ValueSubmitted += (s, v) =>
                SendCommand("SETHEADINGALPHA:" + v.ToString("0.####", CultureInfo.InvariantCulture));
            page.Controls.Add(_numHeadingAlpha); y += 65;

            _numRollAlpha = new NumericSettingControl("Roll Alpha (initial value)", 0.0, 1.0) { Left = 20, Top = y };
            _numRollAlpha.ValueSubmitted += (s, v) =>
                SendCommand("SETROLLALPHA:" + v.ToString("0.####", CultureInfo.InvariantCulture));
            page.Controls.Add(_numRollAlpha); y += 65;

            _numSatsFull = new NumericSettingControl("Sats Full (HPR sats threshold)", 1, 40) { Left = 20, Top = y };
            _numSatsFull.ValueSubmitted += (s, v) =>
                SendCommand("SETSATSFULL:" + ((int)v));
            page.Controls.Add(_numSatsFull); y += 65;

            var uturnGroup = new GroupBox { Text = "U-Turn Dual Boost", Left = 20, Top = y, Width = 400, Height = 110, Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Regular) };
            _lblWasLeft = new Label { Text = "WAS left: --", Left = 15, Top = 20, Width = 150 };
            var btnSetLeft = new Button { Text = "Set current", Left = 170, Top = 17, Width = 100 };
            btnSetLeft.Click += (s, e) => SendCommand("SETWASCURRENT:L");
            _lblWasRight = new Label { Text = "WAS right: --", Left = 15, Top = 50, Width = 150 };
            var btnSetRight = new Button { Text = "Set current", Left = 170, Top = 47, Width = 100 };
            btnSetRight.Click += (s, e) => SendCommand("SETWASCURRENT:R");
            _lblSteerAngle = new Label { Text = "Steer angle: --", Left = 15, Top = 80, Width = 300 };
            uturnGroup.Controls.Add(_lblWasLeft);
            uturnGroup.Controls.Add(btnSetLeft);
            uturnGroup.Controls.Add(_lblWasRight);
            uturnGroup.Controls.Add(btnSetRight);
            uturnGroup.Controls.Add(_lblSteerAngle);
            page.Controls.Add(uturnGroup); y += 120;

            _numUTurnStrength = new NumericSettingControl("U-Turn Strength (1-100)", 1, 100) { Left = 20, Top = y };
            _numUTurnStrength.ValueSubmitted += (s, v) => SendCommand("SETUTURNSTRENGTH:" + ((int)v));
            page.Controls.Add(_numUTurnStrength); y += 65;

            // Explanatory diagram — given directly by the person
            // running this project (same image posted to the
            // AgOpenGPS forum). SizeMode.Zoom scales the original
            // 738x424 image DOWN to fit PictureBox's own Width/Height
            // while preserving aspect ratio — no need to pre-scale
            // the embedded file itself, unlike the Python version
            // (Tkinter's PhotoImage only supports coarse integer
            // subsample() scaling, WinForms handles this natively).
            var uturnDiagram = new PictureBox
            {
                Left = 20, Top = y, Width = 360, Height = 207,
                SizeMode = PictureBoxSizeMode.Zoom,
                Image = LoadEmbeddedImage("uturn_boost_diagram.png")
            };
            page.Controls.Add(uturnDiagram); y += 215;

            _numDualHold = new NumericSettingControl("Dual Reconnect: Stabilization [s]", 0, 30) { Left = 20, Top = y };
            _numDualHold.ValueSubmitted += (s, v) =>
                SendCommand("SETDUALHOLD:" + v.ToString("0.#", CultureInfo.InvariantCulture));
            page.Controls.Add(_numDualHold); y += 65;

            _numDualRamp = new NumericSettingControl("Dual Reconnect: Transition fusion [s]", 0, 30) { Left = 20, Top = y };
            _numDualRamp.ValueSubmitted += (s, v) =>
                SendCommand("SETDUALRAMP:" + v.ToString("0.#", CultureInfo.InvariantCulture));
            page.Controls.Add(_numDualRamp); y += 65;

            // Rent lokalt — skickar ALDRIG något till Teensyn, se
            // DiagnosticLogger.cs för hela infrastrukturen (12
            // hälsokontroller loggade som övergångar, periodisk
            // heartbeat, robust filsökväg).
            _cbLogfile = new CheckBox { Text = "Enable logging", Left = 20, Top = y, Width = 380 };
            _cbLogfile.Click += (s, e) =>
            {
                if (_cbLogfile.Checked)
                {
                    // FolderBrowserDialog är en FÄRDIG, inbyggd
                    // WinForms-dialogruta — ingen egen UI byggd för
                    // det här. Frågar direkt vid aktivering (inte via
                    // en separat "Browse"-knapp) — ett steg istället
                    // för två. Avbryts av användaren → loggning
                    // aktiveras ändå, faller tillbaka till .exe-mappen
                    // (samma beteende som innan den här ändringen),
                    // inte ett tvingat val.
                    using (var folderDialog = new FolderBrowserDialog
                    {
                        Description = "Choose folder to save the log file in (Cancel = same folder as this program)",
                        SelectedPath = _lastChosenLogDirectory ?? Path.GetDirectoryName(Application.ExecutablePath)
                    })
                    {
                        if (folderDialog.ShowDialog() == DialogResult.OK)
                            _lastChosenLogDirectory = folderDialog.SelectedPath;
                    }

                    var (success, error) = _logger.Start(_connection?.TeensyIpAddress, _lastChosenLogDirectory);
                    if (success)
                        // Hela, absoluta sökvägen — inte bara filnamnet
                        // som tidigare, given att "var hamnar filen?"
                        // var precis frågan som ledde fram till den här
                        // ändringen.
                        _lblLogfileStatus.Text = "Logging to " + _logger.LogFilePath;
                    else
                    {
                        _cbLogfile.Checked = false;
                        _lblLogfileStatus.Text = error;
                    }
                }
                else
                {
                    _logger.Stop();
                    _lblLogfileStatus.Text = "Logging off";
                }
            };
            page.Controls.Add(_cbLogfile); y += 22;
            _lblLogfileStatus = new Label { Text = "Logging off", Left = 20, Top = y, Width = 380 };
            page.Controls.Add(_lblLogfileStatus); y += 30;

            // Öppnar ett eget, fristående fönster (LogAnalyzerForm) —
            // ren läsning, ingen koppling till den PÅGÅENDE loggnings-
            // sessionen ovan. Fungerar även för en loggfil från en
            // TIDIGARE session, eller från Python-versionen av
            // verktyget (nästan identiskt textformat).
            var btnAnalyzeLog = new Button
            {
                Text = "Analyze Log File...", Left = 20, Top = y, Width = 200, Height = 28
            };
            btnAnalyzeLog.Click += (s, e) =>
            {
                string startDir = _lastChosenLogDirectory
                    ?? Path.GetDirectoryName(Application.ExecutablePath);
                using (var analyzerForm = new LogAnalyzerForm(startDir))
                    analyzerForm.ShowDialog(this);
            };
            page.Controls.Add(btnAnalyzeLog); y += 36;

            _lblRawPackets = new Label { Text = "Raw packets received: 0", Left = 20, Top = y, Width = 300 };
            page.Controls.Add(_lblRawPackets); y += 22;

            // TILLFÄLLIG felsökningsetikett — se LastRawSentence-
            // kommentaren i TeensyConnection.cs. Height=40 + AutoSize
            // istället för fast bredd, eftersom en hel $PDIAG-sats är
            // lång och annars skulle klippas av.
            _lblLastRawSentence = new Label
            {
                Text = "Last raw sentence: (none yet)", Left = 20, Top = y,
                Width = 420, Height = 40, AutoEllipsis = false
            };
            page.Controls.Add(_lblLastRawSentence); y += 45;

            _txtCommand = new TextBox { Left = 20, Top = y, Width = 250 };
            // Samma touch-vänliga mönster som sifferfälten — klick
            // öppnar en riktig, fullständig knappsats (bokstäver OCH
            // siffror behövs här, given kommandoformatet "SETXXX:värde").
            _txtCommand.Click += (s, e) =>
            {
                using (var keyboard = new TextKeyboardForm(_txtCommand.Text))
                {
                    if (keyboard.ShowDialog() == DialogResult.OK)
                        _txtCommand.Text = keyboard.ReturnValue;
                }
            };
            _btnSend = new Button { Text = "Send", Left = 280, Top = y - 2, Width = 100 };
            _btnSend.Click += (s, e) => SendCommand(_txtCommand.Text);
            page.Controls.Add(_txtCommand);
            page.Controls.Add(_btnSend);

            return page;
        }

        // =================================================================
        // Receiver Configuration tab
        // =================================================================
        private TabPage BuildReceiverConfigurationTab()
        {
            var page = new TabPage("Receiver Configuration") { AutoScroll = true };

            var group = new GroupBox { Text = "GNSS Mode", Left = 20, Top = 20, Width = 400, Height = 120, Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Regular) };
            _rbUm982 = new RadioButton { Text = "UM982 (dual, single receiver)", Left = 15, Top = 25, Width = 360 };
            _rbSingleImu = new RadioButton { Text = "Single + IMU", Left = 15, Top = 50, Width = 360 };
            _rbDual = new RadioButton { Text = "Dual (two receivers, F9P/UM980)", Left = 15, Top = 75, Width = 360 };
            _rbUm982.Click += (s, e) => SendCommand("SETGNSSMODE:1");
            _rbSingleImu.Click += (s, e) => SendCommand("SETGNSSMODE:2");
            _rbDual.Click += (s, e) => SendCommand("SETGNSSMODE:3");
            group.Controls.Add(_rbUm982);
            group.Controls.Add(_rbSingleImu);
            group.Controls.Add(_rbDual);
            page.Controls.Add(group);

            var singleTypeGroup = new GroupBox { Text = "Dual Single-Receiver Type (SETRECEIVERTYPE)", Left = 20, Top = 150, Width = 400, Height = 130, Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Regular) };
            string[] singleLabels = { "2 x u-blox F9P", "2 x UnicoreComm UM980 (not verified)",
                "2 x u-blox X20P (not implemented yet)", "2 x Septentrio Mosaic-X5 (not implemented yet)" };
            _rbDualSingleType = new RadioButton[4];
            for (int i = 0; i < 4; i++)
            {
                var rb = new RadioButton { Text = singleLabels[i], Left = 15, Top = 20 + i * 22, Width = 360 };
                int value = i;
                // Matches Python's send_receiver_type(): only 0 (F9P)
                // and 1 (UM980) are actually implemented — refuse to
                // send for the two placeholder options, same as
                // firmware's own SETRECEIVERTYPE range check.
                rb.Click += (s, e) =>
                {
                    if (value == 0 || value == 1)
                        SendCommand("SETRECEIVERTYPE:" + value);
                };
                singleTypeGroup.Controls.Add(rb);
                _rbDualSingleType[i] = rb;
            }
            page.Controls.Add(singleTypeGroup);

            _cbGnssPassthrough = new CheckBox
            {
                Text = "GNSS Passthrough (bypasses all TFF processing)",
                Left = 20, Top = 290, Width = 400
            };
            _cbGnssPassthrough.Click += (s, e) =>
                SendCommand("SETGNSSPASSTHROUGH:" + (_cbGnssPassthrough.Checked ? "1" : "0"));
            page.Controls.Add(_cbGnssPassthrough);

            var passthroughWarning = new Label
            {
                Text = "⚠ Requires Teensy restart to take effect",
                Left = 20, Top = 312, Width = 400, Height = 20,
                AutoSize = false,
                ForeColor = System.Drawing.Color.DarkOrange
            };
            page.Controls.Add(passthroughWarning);

            var kalmanHeader = new Label
            {
                Text = "KALMAN FILTER — smooths raw GNSS heading/roll before the IMU blend " +
                       "(all dual-capable sources share this)",
                // 420 -> 438: passthroughWarning ovanför slutar vid
                // y=432 (Top=412 + Height=20) — 420 gav en 12px
                // överlappning, denna etikettens text ritades rakt
                // genom/ovanpå varningstextens egen, ett genuint
                // Y-räknefel, inte bara en marginalfråga.
                Left = 20, Top = 338, Width = 420, Height = 30,
                ForeColor = System.Drawing.Color.Gray
            };
            page.Controls.Add(kalmanHeader);

            // Live toggles — SETFILTERHEADING/SETFILTERROLL take effect
            // immediately, no reboot needed. NOTE: filterRoll/
            // filterHeading are never sent back in PDIAG (confirmed by
            // reading firmware's own snprintf() argument list — they
            // only appear in the EEPROM.get/put calls, never in the
            // sendDiagnostics() format string), so these checkboxes
            // cannot be synced to the actual firmware state — matches
            // Python's own behaviour exactly (its filter_heading_var/
            // filter_roll_var are plain local tkinter variables, never
            // updated from received data either).
            var cbFilterHeading = new CheckBox { Text = "Filter heading", Left = 20, Top = 373, Width = 150 };
            var cbFilterRoll = new CheckBox { Text = "Filter roll", Left = 180, Top = 373, Width = 150 };
            cbFilterHeading.Click += (s, e) => SendCommand("SETFILTERHEADING:" + (cbFilterHeading.Checked ? "1" : "0"));
            cbFilterRoll.Click += (s, e) => SendCommand("SETFILTERROLL:" + (cbFilterRoll.Checked ? "1" : "0"));
            page.Controls.Add(cbFilterHeading);
            page.Controls.Add(cbFilterRoll);

            var kalmanWhenNote = new Label
            {
                Text = "When to try this: if Dual Roll (IMU tab) looks noisier than the physical " +
                       "tilt actually is — small, fast flickers rather than a real, slow drift. " +
                       "Start with just one toggle (heading or roll, not both) so you can tell " +
                       "what changed. Note: with ROLL_ALPHA/HEADING_ALPHA already weighted " +
                       "heavily toward the IMU (e.g. 0.8), raw GNSS noise is already attenuated " +
                       "in the blended output before this filter even runs — so if the blended " +
                       "PAOGI signal already looks smooth, this filter may have little visible " +
                       "effect regardless of its settings.",
                // Hela denna kedja (kalmanWhenNote genom kalmanWarning)
                // förskjuten +18px jämfört med tidigare — samma
                // förskjutning som cbFilterHeading/cbFilterRoll ovan
                // fick, för att bevara de redan snäva (5px eller 0px)
                // avstånden mellan varje efterföljande kontroll utan
                // att skapa NYA överlappningar längre ner i kedjan.
                Left = 20, Top = 398, Width = 420, Height = 90,
                ForeColor = System.Drawing.Color.Gray
            };
            page.Controls.Add(kalmanWhenNote);

            var kalmanTuningNote = new Label
            {
                Text = "Tuning (only matters once a toggle above is on): mea = how much you " +
                       "distrust the raw reading — higher smooths harder. est = starting " +
                       "uncertainty — usually leave at 1.0, it settles within a few seconds " +
                       "either way. q = how fast the filter is allowed to follow a real change " +
                       "— lower is smoother but laggier during genuine turns, higher reacts " +
                       "faster but filters less. Change one number at a time.",
                Left = 20, Top = 493, Width = 420, Height = 75,
                ForeColor = System.Drawing.Color.Gray
            };
            page.Controls.Add(kalmanTuningNote);

            // Defaults match firmware's own compiled-in starting values
            // (020_TFF.ino: rollMEA/rollEST/rollQ, headingMEA/
            // headingEST/headingQ) — shown here so the fields aren't
            // just empty placeholders before first use. UNLIKE every
            // other NumericSettingControl on this tab, these values
            // are NEVER updated after this — see KalmanTuningControl's
            // own class comment for why (no PDIAG round-trip exists
            // for these six values at all).
            var rollKalman = new KalmanTuningControl("Roll:", 1.0, 1.0, 0.01) { Left = 20, Top = 573 };
            rollKalman.ValuesSubmitted += (s, values) =>
            {
                string mea = values.mea.ToString("0.####", CultureInfo.InvariantCulture);
                string est = values.est.ToString("0.####", CultureInfo.InvariantCulture);
                string q = values.q.ToString("0.####", CultureInfo.InvariantCulture);
                SendCommand($"SETROLLKALMAN:{mea},{est},{q}");
            };
            page.Controls.Add(rollKalman);

            var headingKalman = new KalmanTuningControl("Heading:", 1.0, 1.0, 0.01) { Left = 20, Top = 633 };
            headingKalman.ValuesSubmitted += (s, values) =>
            {
                string mea = values.mea.ToString("0.####", CultureInfo.InvariantCulture);
                string est = values.est.ToString("0.####", CultureInfo.InvariantCulture);
                string q = values.q.ToString("0.####", CultureInfo.InvariantCulture);
                SendCommand($"SETHEADINGKALMAN:{mea},{est},{q}");
            };
            page.Controls.Add(headingKalman);

            var kalmanWarning = new Label
            {
                Text = "⚠ mea/est/q changes require a Teensy restart (filter object is only " +
                       "built once, at boot) — the two toggles above do not.",
                Left = 20, Top = 688, Width = 420, Height = 45,
                AutoSize = false,
                ForeColor = System.Drawing.Color.DarkOrange
            };
            page.Controls.Add(kalmanWarning);

            return page;
        }

        // =================================================================
        // IMU tab
        // =================================================================
        private TabPage BuildImuTab()
        {
            var page = new TabPage("IMU") { AutoScroll = true };

            var typeGroup = new GroupBox { Text = "IMU Type", Left = 20, Top = 20, Width = 400, Height = 100, Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Regular) };
            _rbImuBno08x = new RadioButton { Text = "BNO08x (I2C)", Left = 15, Top = 25, Width = 360 };
            _rbImuTm171 = new RadioButton { Text = "TM171 (UART, SYD Dynamics TransducerM)", Left = 15, Top = 50, Width = 360 };
            _rbImuNone = new RadioButton { Text = "No fallback function (dual-only, no IMU)", Left = 15, Top = 75, Width = 360 };
            _rbImuBno08x.Click += (s, e) => SendCommand("SETIMUTYPE:0");
            _rbImuTm171.Click += (s, e) => SendCommand("SETIMUTYPE:1");
            _rbImuNone.Click += (s, e) => SendCommand("SETIMUTYPE:2");
            typeGroup.Controls.Add(_rbImuBno08x);
            typeGroup.Controls.Add(_rbImuTm171);
            typeGroup.Controls.Add(_rbImuNone);
            page.Controls.Add(typeGroup);

            var imuTypeWarning = new Label
            {
                Text = "⚠ Requires Teensy restart to take effect (I2C/UART only init at boot)",
                Left = 20, Top = 122, Width = 400, Height = 32,
                AutoSize = false,
                ForeColor = System.Drawing.Color.DarkOrange
            };
            page.Controls.Add(imuTypeWarning);

            var compareGroup = new GroupBox { Text = "Dual Roll vs IMU Roll (live comparison)", Left = 20, Top = 160, Width = 400, Height = 80, Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Regular) };
            _lblDualRollLive = new Label { Text = "Dual Roll (HPR): --.-°", Left = 15, Top = 25, Width = 180 };
            _lblImuRollLive = new Label { Text = "IMU Roll: --.-°", Left = 200, Top = 25, Width = 180 };
            compareGroup.Controls.Add(_lblDualRollLive);
            compareGroup.Controls.Add(_lblImuRollLive);
            page.Controls.Add(compareGroup);

            _numRollZero = new NumericSettingControl("IMU Roll Offset (matches IMU to Dual)", -30, 30) { Left = 20, Top = 250 };
            _numRollZero.ValueSubmitted += (s, v) =>
                SendCommand("SETROLLZERO:" + v.ToString("0.##", CultureInfo.InvariantCulture));
            page.Controls.Add(_numRollZero);

            // Auto Roll Adjust (v0.3.17/v0.3.18) — continuously nudges
            // the roll offset above for slow sensor-mounting drift.
            // NOTE: unlike Python, this tab has no established "gray
            // out when Single+IMU" mechanism for _numRollZero either
            // (a known, pre-existing C#/Python gap) — this panel
            // deliberately matches that same, already-existing state
            // rather than introducing a NEW inconsistency where only
            // this panel gets gray-out logic but the older, related
            // Roll Offset panel right above it still doesn't.
            var autoRollGroup = new GroupBox
            {
                Text = "Auto Roll Adjust", Left = 20, Top = 325, Width = 420, Height = 295,
                Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Regular)
            };
            var autoRollExplain = new Label
            {
                Text = "Continuously nudges the roll offset above to correct for slow sensor "
                     + "drift over time — not a one-time calibration. Only active with a dual "
                     + "RTK-fixed signal.",
                Left = 10, Top = 20, Width = 400, Height = 45,
                ForeColor = System.Drawing.Color.Gray
            };
            autoRollGroup.Controls.Add(autoRollExplain);

            _cbAutoRollAdjust = new CheckBox { Text = "Auto Roll Adjust", Left = 10, Top = 70, Width = 200 };
            _cbAutoRollAdjust.Click += (s, e) =>
                SendCommand("SETAUTOROLLADJUST:" + (_cbAutoRollAdjust.Checked ? "1" : "0"));
            autoRollGroup.Controls.Add(_cbAutoRollAdjust);

            _lblAutoRollStatus = new Label { Text = "", Left = 10, Top = 95, Width = 300 };
            autoRollGroup.Controls.Add(_lblAutoRollStatus);

            // (lo, hi) match firmware's own constrain() ranges exactly
            // (SETROLLAUTODEADBAND/SETROLLAUTOALPHA handlers,
            // zHandlers.ino).
            _numRollAutoDeadband = new NumericSettingControl("Min angle diff to adjust [deg]", 0.02, 2.0) { Left = 10, Top = 120 };
            _numRollAutoDeadband.ValueSubmitted += (s, v) =>
                SendCommand("SETROLLAUTODEADBAND:" + v.ToString("0.###", CultureInfo.InvariantCulture));
            autoRollGroup.Controls.Add(_numRollAutoDeadband);

            _numRollAutoAlpha = new NumericSettingControl("Adjustment speed", 0.0001, 0.1) { Left = 10, Top = 190 };
            _numRollAutoAlpha.ValueSubmitted += (s, v) =>
                SendCommand("SETROLLAUTOALPHA:" + v.ToString("0.####", CultureInfo.InvariantCulture));
            autoRollGroup.Controls.Add(_numRollAutoAlpha);

            // Same "fills fields only, still requires Set per row"
            // philosophy as Keya Auto-Zero's own Load Defaults button
            // — no single "reset everything to firmware right now"
            // action without a review step.
            var btnRollAutoLoadDefaults = new Button
            {
                Text = "Load Defaults (fills fields only, still requires Set per row)",
                Left = 10, Top = 260, Width = 400, Height = 28
            };
            btnRollAutoLoadDefaults.Click += (s, e) =>
            {
                _numRollAutoDeadband.LoadDefault(0.2);
                _numRollAutoAlpha.LoadDefault(0.0005);
            };
            autoRollGroup.Controls.Add(btnRollAutoLoadDefaults);

            page.Controls.Add(autoRollGroup);

            _numImuAxis = new NumericSettingControl("Roll Axis (0=X 1=Y 2=Z)", 0, 2) { Left = 20, Top = 630 };
            _numImuAxis.ValueSubmitted += (s, v) => SendCommand("SETIMUAXIS:" + ((int)v));
            page.Controls.Add(_numImuAxis);

            _cbRollInvert = new CheckBox { Text = "Invert Roll", Left = 20, Top = 705, Width = 200 };
            // Matches Python's send_roll_invert() exactly: unchecked
            // = -1 (X forward, normal mounting), checked = 1 (inverted)
            // — deliberately NOT 0/1, firmware expects a signed value.
            _cbRollInvert.Click += (s, e) =>
                SendCommand("SETROLLINVERT:" + (_cbRollInvert.Checked ? "1" : "-1"));
            page.Controls.Add(_cbRollInvert);

            return page;
        }

        // =================================================================
        // Vehicle Configuration tab
        // =================================================================
        private TabPage BuildVehicleConfigurationTab()
        {
            var page = new TabPage("Vehicle Configuration") { AutoScroll = true };

            var brandGroup = new GroupBox { Text = "Steering Brand", Left = 20, Top = 20, Width = 420, Height = 190, Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Regular) };
            // (number, label) pairs — BRAND_NONE=8 first, matching
            // Python's own BRAND_OPTIONS list order exactly.
            (int, string)[] brandOptions =
            {
                (8, "No CAN — classic PWM/relay"),
                (0, "Claas"),
                (1, "Valtra / Massey Ferguson"),
                (2, "Case IH / New Holland"),
                (3, "Fendt"),
                (4, "JCB"),
                (5, "FendtOne"),
                (6, "Lindner"),
                (7, "AgOpenGPS — PWM/external valve (CAN active in background)"),
            };
            _rbBrand = new RadioButton[9];
            for (int i = 0; i < brandOptions.Length; i++)
            {
                int col = i % 2, row = i / 2;
                var (num, label) = brandOptions[i];
                var rb = new RadioButton
                {
                    Text = num + " — " + label,
                    Left = 15 + col * 205, Top = 25 + row * 25, Width = 195
                };
                int value = num;
                rb.Click += (s, e) => SendCommand("SETBRAND:" + value);
                brandGroup.Controls.Add(rb);
                _rbBrand[i] = rb;
            }
            page.Controls.Add(brandGroup);

            var keyaGroup = new GroupBox { Text = "Motor/Valve Drive & WAS Source", Left = 20, Top = 220, Width = 420, Height = 110, Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Regular) };

            // KRITISKT: Motor Drive-knapparna och WAS Source-knapparna
            // ligger i VARSIN Panel, inte direkt i samma GroupBox. Två
            // radioknappar i SAMMA behållare blir automatiskt EN
            // gemensam, ömsesidigt uteslutande grupp i WinForms — utan
            // den här uppdelningen skulle klick på "PWM" (Motor Drive)
            // av misstag avmarkera "Normal" (WAS Source) också, trots
            // att de är två helt orelaterade inställningar. Hittat via
            // ett verkligt, reproducerbart symptom: PWM gick inte att
            // välja alls, Normal blinkade av/på i en självförstärkande
            // loop (varje synkronisering av den ena avmarkerade den
            // andra, som sedan synkades tillbaka, som avmarkerade den
            // första igen). Panel har ingen egen synlig ram/rubrik —
            // bara en osynlig behållare, så den yttre GroupBoxens
            // gemensamma rubrik ("Motor/Valve Drive & WAS Source")
            // bevaras visuellt oförändrad.
            var motorPanel = new Panel { Left = 10, Top = 20, Width = 195, Height = 80 };
            _rbMotorPwm = new RadioButton { Text = "PWM (default)", Left = 5, Top = 5, Width = 185 };
            _rbMotorKeya = new RadioButton { Text = "Keya (CAN)", Left = 5, Top = 30, Width = 185 };
            // Deliberately no value 1 — matches firmware's own
            // MOTOR_DRIVE_PWM=0 / MOTOR_DRIVE_KEYA=2 comment.
            _rbMotorPwm.Click += (s, e) => SendCommand("SETMOTORDRIVETYPE:0");
            _rbMotorKeya.Click += (s, e) => SendCommand("SETMOTORDRIVETYPE:2");
            motorPanel.Controls.Add(_rbMotorPwm);
            motorPanel.Controls.Add(_rbMotorKeya);

            var wasPanel = new Panel { Left = 210, Top = 20, Width = 195, Height = 80 };
            _rbWasNormal = new RadioButton { Text = "Normal (default)", Left = 5, Top = 5, Width = 185 };
            _rbWasKeya = new RadioButton { Text = "Keya encoder", Left = 5, Top = 30, Width = 185 };
            _rbWasNormal.Click += (s, e) => SendCommand("SETWASSOURCE:0");
            _rbWasKeya.Click += (s, e) => SendCommand("SETWASSOURCE:1");
            wasPanel.Controls.Add(_rbWasNormal);
            wasPanel.Controls.Add(_rbWasKeya);

            keyaGroup.Controls.Add(motorPanel);
            keyaGroup.Controls.Add(wasPanel);
            page.Controls.Add(keyaGroup);

            // =============================================================
            // Keya Auto-Zero tuning (v0.3.10) — only meaningful when
            // WAS Source above is set to "Keya encoder"; shown
            // regardless (not conditionally hidden), matching the
            // Python version's same reasoning — these are persistent
            // EEPROM values worth seeing/adjusting even while a
            // different WasSource is momentarily selected.
            // =============================================================
            var azGroup = new GroupBox
            {
                Text = "Keya Auto-Zero Tuning", Left = 20, Top = 340, Width = 420, Height = 40,
                Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Regular)
            };
            var azExplain = new Label
            {
                Text = "Only relevant when WAS Source = Keya encoder. Establishes and " +
                       "continuously corrects the encoder's zero-point using GPS/IMU heading " +
                       "while driving straight — no guidance is possible until the first zero " +
                       "is established.",
                Left = 10, Top = 15, Width = 400, Height = 45,
                ForeColor = System.Drawing.Color.Gray
            };
            azGroup.Height = 65;
            azGroup.Controls.Add(azExplain);
            page.Controls.Add(azGroup);

            _lblAzZeroStatus = new Label
            {
                // AutoSize=false + explicit Height — samma fix som de
                // andra varningsetiketterna (Kalman/IMU/Passthrough),
                // missad här första gången eftersom sökningen då var
                // specifik för "Requires Teensy restart"-texterna.
                // Texten kan bli lång ("... guidance blocked until the
                // vehicle drives straight for a bit") och klipptes i
                // botten utan detta.
                Text = "Zero established: --", Left = 20, Top = 410, Width = 420, Height = 32,
                AutoSize = false,
                ForeColor = System.Drawing.Color.DarkOrange
            };
            page.Controls.Add(_lblAzZeroStatus);

            int azY = 448;
            // (lo, hi) match firmware's own constrain() ranges exactly
            // (azSet*() functions, zKeyaAutoZero.ino) — not
            // independently chosen, so a value this UI accepts is
            // guaranteed to also be accepted, unclamped, by firmware.
            _numAzSpeedMin = new NumericSettingControl("Min speed [km/h]", 0.1, 20.0) { Left = 20, Top = azY };
            _numAzSpeedMin.ValueSubmitted += (s, v) => SendCommand("SETAZSPEEDMIN:" + v.ToString("0.##", CultureInfo.InvariantCulture));
            page.Controls.Add(_numAzSpeedMin); azY += 65;

            _numAzYawRateMax = new NumericSettingControl("Max yaw rate [deg/s]", 0.01, 10.0) { Left = 20, Top = azY };
            _numAzYawRateMax.ValueSubmitted += (s, v) => SendCommand("SETAZYAWRATEMAX:" + v.ToString("0.##", CultureInfo.InvariantCulture));
            page.Controls.Add(_numAzYawRateMax); azY += 65;

            _numAzGpsHdgMax = new NumericSettingControl("Max GPS heading change [deg]", 0.01, 10.0) { Left = 20, Top = azY };
            _numAzGpsHdgMax.ValueSubmitted += (s, v) => SendCommand("SETAZGPSHDGMAX:" + v.ToString("0.##", CultureInfo.InvariantCulture));
            page.Controls.Add(_numAzGpsHdgMax); azY += 65;

            _numAzTimeSlow = new NumericSettingControl("Stable time @ low speed [ms]", 100, 5000) { Left = 20, Top = azY };
            _numAzTimeSlow.ValueSubmitted += (s, v) => SendCommand("SETAZTIMESLOW:" + ((int)v));
            page.Controls.Add(_numAzTimeSlow); azY += 65;

            _numAzTimeFast = new NumericSettingControl("Stable time @ high speed [ms]", 100, 5000) { Left = 20, Top = azY };
            _numAzTimeFast.ValueSubmitted += (s, v) => SendCommand("SETAZTIMEFAST:" + ((int)v));
            page.Controls.Add(_numAzTimeFast); azY += 65;

            _numAzSpeedSlow = new NumericSettingControl("\"Low speed\" threshold [km/h]", 0.1, 30.0) { Left = 20, Top = azY };
            _numAzSpeedSlow.ValueSubmitted += (s, v) => SendCommand("SETAZSPEEDSLOW:" + v.ToString("0.##", CultureInfo.InvariantCulture));
            page.Controls.Add(_numAzSpeedSlow); azY += 65;

            _numAzSpeedFast = new NumericSettingControl("\"High speed\" threshold [km/h]", 0.1, 30.0) { Left = 20, Top = azY };
            _numAzSpeedFast.ValueSubmitted += (s, v) => SendCommand("SETAZSPEEDFAST:" + v.ToString("0.##", CultureInfo.InvariantCulture));
            page.Controls.Add(_numAzSpeedFast); azY += 65;

            _numAzUseBno = new NumericSettingControl("Use BNO yaw rate (0/1)", 0, 1) { Left = 20, Top = azY };
            _numAzUseBno.ValueSubmitted += (s, v) => SendCommand("SETAZUSEBNO:" + ((int)v));
            page.Controls.Add(_numAzUseBno); azY += 65;

            _numAzUseGps = new NumericSettingControl("Use GPS heading (0/1)", 0, 1) { Left = 20, Top = azY };
            _numAzUseGps.ValueSubmitted += (s, v) => SendCommand("SETAZUSEGPS:" + ((int)v));
            page.Controls.Add(_numAzUseGps); azY += 65;

            _numAzBeta = new NumericSettingControl("Correction gain (beta)", 0.001, 1.0) { Left = 20, Top = azY };
            _numAzBeta.ValueSubmitted += (s, v) => SendCommand("SETAZBETA:" + v.ToString("0.###", CultureInfo.InvariantCulture));
            page.Controls.Add(_numAzBeta); azY += 65;

            // Fyller ENDAST fälten (azParams' egna kompileringstids-
            // defaultvärden, zKeyaAutoZero.ino) — skickar INGET till
            // firmware. Kräver fortfarande ett explicit Set-klick per
            // rad, exakt som varje annan ändring — en enda "återställ
            // allt direkt"-knapp utan granskningssteg avsiktligt
            // undveks.
            var btnAzLoadDefaults = new Button
            {
                Text = "Load Defaults (fills fields only, still requires Set per row)",
                Left = 20, Top = azY, Width = 380, Height = 30
            };
            btnAzLoadDefaults.Click += (s, e) =>
            {
                _numAzSpeedMin.LoadDefault(2.5);
                _numAzYawRateMax.LoadDefault(0.3);
                _numAzGpsHdgMax.LoadDefault(0.3);
                _numAzTimeSlow.LoadDefault(500);
                _numAzTimeFast.LoadDefault(200);
                _numAzSpeedSlow.LoadDefault(3.0);
                _numAzSpeedFast.LoadDefault(12.0);
                _numAzUseBno.LoadDefault(1);
                _numAzUseGps.LoadDefault(1);
                _numAzBeta.LoadDefault(0.3);
            };
            page.Controls.Add(btnAzLoadDefaults); azY += 40;

            return page;
        }

        // =================================================================
        // Board Configuration tab
        // =================================================================
        private TabPage BuildBoardConfigurationTab()
        {
            var page = new TabPage("Board Configuration") { AutoScroll = true };

            var warning = new Label
            {
                Text = "Warning: the two slots must end up different, and at least one\n" +
                       "must be Master or Slave — not enforced live, only validated\n" +
                       "(and reset to AIO-default if still invalid) at next reboot.",
                Left = 20, Top = 20, Width = 400, Height = 40,
                ForeColor = System.Drawing.Color.DarkOrange
            };
            page.Controls.Add(warning);

            // NOTE: the two slots' option ORDER genuinely differs in
            // Python — Slot 1 lists Master/TM171/Slave/Empty (Master
            // first, the AIO default for this slot), Slot 2 lists
            // Slave/TM171/Master/Empty (Slave first, ITS default) —
            // not the same order repeated. Matched exactly here, not
            // assumed identical.
            var slot1Group = BuildSlotGroupBox(
                "Slot 1 (Serial7)", "SETBOARDSLOT1:",
                new[] { (0, "Master GNSS (default)"), (2, "TM171"), (1, "Slave GNSS"), (3, "Empty") },
                left: 20, top: 65, out _rbSlot1);
            var slot2Group = BuildSlotGroupBox(
                "Slot 2 (Serial2)", "SETBOARDSLOT2:",
                new[] { (1, "Slave GNSS (default)"), (2, "TM171"), (0, "Master GNSS"), (3, "Empty") },
                left: 220, top: 65, out _rbSlot2);

            page.Controls.Add(slot1Group);
            page.Controls.Add(slot2Group);

            return page;
        }

        /// <summary>
        /// "(int value, string label)[]" lets each slot specify its
        /// OWN option order (see the ordering note in
        /// BuildBoardConfigurationTab() above) while still sharing one
        /// piece of layout code — a slightly more flexible version of
        /// the same array-based reuse pattern as before.
        /// </summary>
        private GroupBox BuildSlotGroupBox(
            string title, string commandPrefix, (int value, string label)[] options,
            int left, int top, out RadioButton[] radioButtons)
        {
            var group = new GroupBox { Text = title, Left = left, Top = top, Width = 190, Height = 140, Font = new System.Drawing.Font("Tahoma", 9.75F, System.Drawing.FontStyle.Regular) };
            radioButtons = new RadioButton[options.Length];

            for (int i = 0; i < options.Length; i++)
            {
                var rb = new RadioButton
                {
                    Text = options[i].label, Left = 15, Top = 20 + i * 25, Width = 160
                };
                int value = options[i].value;
                rb.Click += (s, e) => SendCommand(commandPrefix + value);
                group.Controls.Add(rb);
                radioButtons[i] = rb;
            }

            return group;
        }

        // =================================================================
        // Connection lifecycle
        // =================================================================
        private void StartConnection()
        {
            try
            {
                _connection = new TeensyConnection(listenPort: 5555, sendPort: 5556);
            }
            catch (Exception ex)
            {
                MessageBox.Show(
                    "Could not start network reception:\n\n" + ex.Message +
                    "\n\nIs the Python version of Teensy Tool perhaps running at the same time, " +
                    "or another program already using port 5555?",
                    "Startup error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            _connection.DataReceived += Connection_DataReceived;
            _connection.ErrorOccurred += (s, message) =>
                MessageBox.Show(message, "Network error", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            FormClosing += (s, e) => _connection.Dispose();
            _connection.Start();

            var statusTimer = new Timer { Interval = 500 };
            statusTimer.Tick += (s, e) =>
            {
                _lblRawPackets.Text = "Raw packets received: " + _connection.RawPacketsReceived;
                _lblLastRawSentence.Text = "Last raw sentence: " + (_connection.LastRawSentence ?? "(none yet)");

                // Signal-lost-detektering, samma 10-sekunders tröskel
                // som Python — sätter flaggan LogHeartbeat() OCH
                // UpdateLabels() (för "signal restored"-loggningen)
                // båda läser.
                bool signalStale = !_lastRawPacketTime.HasValue
                    || (DateTime.Now - _lastRawPacketTime.Value).TotalSeconds > 10;
                if (signalStale && !_wasSignalLost && _lastRawPacketTime.HasValue)
                {
                    _logger.LogLine("COMMUNICATION ERROR: no $PDIAG received for >10s (signal lost)");
                    _wasSignalLost = true;
                }
            };
            statusTimer.Start();

            // Periodisk heartbeat — var 30:e sekund, matchande Python
            // exakt. Egen, separat Timer (inte återanvänd statusTimer
            // ovan) eftersom intervallen genuint skiljer sig (500ms
            // kontra 30s) — att blanda in en räknare i statusTimer
            // hade varit onödigt krångligare än bara en till Timer.
            var heartbeatTimer = new Timer { Interval = 30000 };
            heartbeatTimer.Tick += (s, e) =>
                _logger.LogHeartbeat(_lastRawPacketTime, _lastSuccessfulUpdateTime);
            heartbeatTimer.Start();

            // Säkerställer loggfilen stängs korrekt (inte bara
            // övergiven mitt i skrivning) om fönstret stängs medan
            // loggning fortfarande är aktiv — matchar Pythons egen
            // _on_close()-hantering.
            FormClosing += (s, e) => _logger.Stop();
        }

        /// <summary>
        /// Fires on TeensyConnection's own background receive thread,
        /// NOT the UI thread — the InvokeRequired/Invoke dance here is
        /// the standard WinForms pattern for safely marshalling back
        /// to the UI thread before touching any control, since
        /// updating a Label/TextBox directly from a background thread
        /// would either silently corrupt the UI or throw, depending
        /// on timing. UpdateLabels() itself always runs on the UI
        /// thread by the time it's called, either directly (already
        /// on it) or via this Invoke.
        /// </summary>
        private void Connection_DataReceived(object sender, PDiagData data)
        {
            if (InvokeRequired)
            {
                Invoke(new Action(() => UpdateLabels(data)));
                return;
            }
            UpdateLabels(data);
        }

        /// <summary>
        /// The single place every received PDIAG packet's values get
        /// pushed out to every control across all five tabs — called
        /// once per packet (~every 2s), always on the UI thread (see
        /// Connection_DataReceived above). Each field checked
        /// individually for null before use (most PDiagData
        /// properties are nullable) rather than one big null-check
        /// up front, since older firmware missing a specific new
        /// field should still update everything it DOES have, not
        /// skip the whole update.
        /// </summary>
        private void UpdateLabels(PDiagData data)
        {
            _lastRawPacketTime = DateTime.Now;
            _lastSuccessfulUpdateTime = DateTime.Now;

            // Loggar VARJE mottaget paket, samma "DATA: {rådictionary}"-
            // stil som Python — bara ToString() på hela PDiagData-
            // objektet ger inte en läsbar, likvärdig sammanfattning
            // (C# skriver ut typnamnet, inte fältvärdena), så här
            // byggs en explicit, kompakt sammanfattning istället.
            _logger.LogLine($"DATA: Mode={data.Mode} SatsM={data.SatsMaster} SatsS={data.SatsSlave} "
                + $"HprSats={data.HprSats} Sol={data.SolQuality} HeadingAlpha={data.HeadingAlpha:0.0000} "
                + $"RollValid={data.RollValid}");

            if (_wasSignalLost)
            {
                _logger.LogLine("COMMUNICATION: signal restored");
                _wasSignalLost = false;
            }

            // Diagnostiska hälsotransitioner — matchar Pythons egen
            // lista över ~12 kontroller exakt. == 1 hanterar korrekt
            // ett saknat fält (äldre firmware utan denna funktion) som
            // "inget problem", inte ett falskt larm.
            _logger.LogHealthTransition("can_k",       data.CanKBusTimeout == 1,    "CAN K-Bus timeout (no message received)");
            _logger.LogHealthTransition("can_iso",     data.CanIsoBusTimeout == 1,  "CAN ISO-Bus timeout (no message received)");
            _logger.LogHealthTransition("can_v",       data.CanVBusTimeout == 1,    "CAN V-Bus timeout (no message received)");
            _logger.LogHealthTransition("can_content", data.CanContentImplausible == 1, "CAN content implausible (steeringValveReady out of sane range)");
            _logger.LogHealthTransition("gnss_wd",     data.GnssWatchdog == 1,      "GNSS serial watchdog timeout (no byte received from receiver)");
            _logger.LogHealthTransition("hpr_wd",      data.HprTimeout == 1,        "HPR watchdog timeout (GNSS bytes arriving, but no valid $GPHPR sentence completed — only meaningful in UM982 mode)");
            _logger.LogHealthTransition("rtk",         data.RtkTimeout == 1,        "RTK radio timeout (no RTCM byte received)");
            _logger.LogHealthTransition("was",         data.WasImplausible == 1,    "WAS signal implausible (out of calibrated range or sudden jump)");
            _logger.LogHealthTransition("eth_link",    data.EthernetLinkUp == 0,    "Ethernet link down");

            // IMU-watchdog — hoppas över helt när ImuType==2 (IMU_NONE),
            // matchande Python: ingen IMU förväntas där, så
            // ImuHealthy/UseImu==false är normalt, inte ett problem.
            if (data.ImuType != null && data.ImuType != 2)
            {
                _logger.LogHealthTransition("imu_given_up", data.UseImu == 0,
                    "IMU watchdog gave up permanently (exceeded max retries) — running dual-only for the rest of this session");
                if (data.UseImu == 1)
                {
                    _logger.LogHealthTransition("imu_unhealthy", data.ImuHealthy == 0,
                        "IMU watchdog unhealthy (retrying)");
                }
                // else: UseIMU==0 — redan täckt av imu_given_up ovan,
                // loggas medvetet INTE separat här (samma resonemang
                // som Python: skulle annars ge en missvisande
                // "RESOLVED" när IMU:n i själva verket gett upp, inte
                // återhämtat sig).
            }

            // Keya-fel — bara meningsfullt när MotorDriveType==2 (Keya).
            if (data.MotorDriveType == 2)
            {
                _logger.LogHealthTransition("keya_fault", data.KeyaFaultActive == 1,
                    "Keya motor fault active (see motor's own LED blink code for specifics)");
            }

            _lblSatsMaster.Text = "Master sats (GGA): " + data.SatsMaster;
            _lblSatsSlave.Text = "Slave sats (GPGGAH): " + data.SatsSlave;

            int imuPct = (int)Math.Round(data.HeadingAlpha * 100.0);
            // DualPercent är "hur mycket åt Dual-änden", medan imuPct
            // ovan är "hur mycket åt IMU-änden" — de två är varandras
            // motsats (summan alltid 100), därav 100-imuPct här.
            _splitBar.DualPercent = 100 - imuPct;

            // -1 är vårt etablerade "inte tillämpligt i det här läget"-
            // sentinel (Single+IMU: båda fälten; Dual F9P: bara
            // hprSats, eftersom RELPOSNED saknar satellitantal men har
            // en egen kvalitetssignal, carrSoln, som solQuality FÅR
            // ett riktigt värde ifrån där) — se firmwarens egna
            // kommentarer i zzGNSS_SingleIMU.ino/zzGNSS_DualF9P.ino.
            // Kontrollerar EXAKT -1, inte "<=0" — solQuality=0 är ett
            // riktigt, giltigt värde ("ingen fix, men verkligen mätt"),
            // inte samma sak som "aldrig tillämpligt".
            _lblSol.Text = "HPR solution: " + (data.SolQuality == -1 ? "N/A" : data.SolQuality.ToString());
            _lblHprSats.Text = "HPR heading sats: " + (data.HprSats == -1 ? "N/A" : data.HprSats.ToString());

            if (data.HprAgeSeconds == null) _lblHprAge.Text = "Last HPR: --";
            else if (data.HprAgeSeconds < 0) _lblHprAge.Text = "Last HPR: never";
            else _lblHprAge.Text = $"Last HPR: {data.HprAgeSeconds:0.0}s ago";

            _lblOffset.Text = "IMU → HPR heading offset: " + data.HeadingOffsetDeg.ToString("+0.0;-0.0") + " °";

            _numHeadingAlpha.UpdateCurrentValue(data.InitialHeadingAlpha);
            _numRollAlpha.UpdateCurrentValue(data.InitialRollAlpha);
            _numSatsFull.UpdateCurrentValue(data.SatsFullThreshold);

            _lblWasLeft.Text = "WAS left: " + data.WasLeftDeg.ToString("0.0") + "°";
            _lblWasRight.Text = "WAS right: " + data.WasRightDeg.ToString("0.0") + "°";
            _lblSteerAngle.Text = "Steer angle: " + data.SteerAngleActualDeg.ToString("0.0") + "°";
            _numUTurnStrength.UpdateCurrentValue(data.UTurnStrength);
            _numDualHold.UpdateCurrentValue(data.DualHoldSeconds);
            _numDualRamp.UpdateCurrentValue(data.DualRampSeconds);

            SyncGnssModeRadioButtons(data.GnssMode);
            SyncGnssPassthroughCheckbox(data.GnssPassthrough);
            SyncRadioArrayByValue(_rbSlot1, new[] { 0, 2, 1, 3 }, data.BoardSlot1);
            SyncRadioArrayByValue(_rbSlot2, new[] { 1, 2, 0, 3 }, data.BoardSlot2);
            SyncImuTypeRadioButtons(data.ImuType);

            // Dual Roll vs IMU Roll (IMU tab). DualRollRaw är BARA
            // meningsfull när RollValid==1 (en aktuell dual-lösning
            // faktiskt finns just nu) — firmware nollställer ALDRIG
            // DualRollRaw när ingen giltig lösning finns, den behåller
            // sitt gamla, inaktuella värde (se PDiagData.cs' egen
            // kommentar) — utan den här kontrollen visades ett
            // inaktuellt, missvisande tal som om det vore aktuellt.
            // ImuRollRaw visas ALLTID (om inte null) — giltig oavsett
            // om en dual-lösning finns, matchande Pythons egen logik
            // exakt (samma asymmetri: bara Dual Roll är villkorad).
            if (data.DualRollRaw != null)
            {
                if (data.RollValid == 1)
                    _lblDualRollLive.Text = $"Dual Roll (HPR): {data.DualRollRaw:0.0}°";
                else
                    _lblDualRollLive.Text = "Dual Roll (HPR): --.-°";
            }
            if (data.ImuRollRaw != null) _lblImuRollLive.Text = $"IMU Roll: {data.ImuRollRaw:0.0}°";
            _numRollZero.UpdateCurrentValue(data.RollZeroOffsetDeg);
            _numImuAxis.UpdateCurrentValue(data.ImuAxis);

            // Auto Roll Adjust — checkbox synced unconditionally (a
            // momentary click, not an ongoing edit). SyncEntryOnce()
            // used for the two entry fields, matching the established,
            // one-time-sync pattern (see NumericSettingControl.cs' own
            // comment on why — avoids overwriting active typing).
            if (data.AutoRollAdjust != null)
            {
                _cbAutoRollAdjust.Checked = (data.AutoRollAdjust == 1);
                _lblAutoRollStatus.Text = data.AutoRollAdjust == 1
                    ? "Active — correcting live" : "Off — manual control";
            }
            if (data.RollAutoDeadband != null)
            {
                _numRollAutoDeadband.UpdateCurrentValue(data.RollAutoDeadband.Value);
                _numRollAutoDeadband.SyncEntryOnce(data.RollAutoDeadband.Value);
            }
            if (data.RollAutoAlpha != null)
            {
                _numRollAutoAlpha.UpdateCurrentValue(data.RollAutoAlpha.Value);
                _numRollAutoAlpha.SyncEntryOnce(data.RollAutoAlpha.Value);
            }
            // rollInvert: -1 (unchecked) or 1 (checked) — see
            // BuildImuTab()'s own comment on _cbRollInvert.
            _cbRollInvert.Checked = (data.RollInvert == 1);

            SyncRadioArrayByValue(_rbBrand, new[] { 8, 0, 1, 2, 3, 4, 5, 6, 7 }, data.Brand);
            if (data.MotorDriveType != null)
            {
                if (data.MotorDriveType == 0) _rbMotorPwm.Checked = true;
                else if (data.MotorDriveType == 2) _rbMotorKeya.Checked = true;
            }
            if (data.WasSource != null)
            {
                if (data.WasSource == 0) _rbWasNormal.Checked = true;
                else if (data.WasSource == 1) _rbWasKeya.Checked = true;
            }

            // Keya auto-zero — all nullable, so older firmware without
            // these fields (pre-v0.3.10) just leaves everything at its
            // "--" placeholder rather than throwing, matching how
            // every other nullable PDIAG field already behaves here.
            if (data.AzZeroDone != null)
            {
                if (data.AzZeroDone == 1)
                {
                    string offsetText = data.AzZeroDeg != null ? $" (offset {data.AzZeroDeg:+0.00;-0.00}°)" : "";
                    _lblAzZeroStatus.Text = "Zero established: YES" + offsetText;
                    _lblAzZeroStatus.ForeColor = System.Drawing.Color.Green;
                }
                else
                {
                    _lblAzZeroStatus.Text = "Zero established: NO — guidance blocked until the "
                        + "vehicle drives straight for a bit";
                    _lblAzZeroStatus.ForeColor = System.Drawing.Color.Red;
                }
            }
            if (data.AzSpeedMin   != null) { _numAzSpeedMin.UpdateCurrentValue(data.AzSpeedMin.Value);     _numAzSpeedMin.SyncEntryOnce(data.AzSpeedMin.Value); }
            if (data.AzYawRateMax != null) { _numAzYawRateMax.UpdateCurrentValue(data.AzYawRateMax.Value); _numAzYawRateMax.SyncEntryOnce(data.AzYawRateMax.Value); }
            if (data.AzGpsHdgMax  != null) { _numAzGpsHdgMax.UpdateCurrentValue(data.AzGpsHdgMax.Value);   _numAzGpsHdgMax.SyncEntryOnce(data.AzGpsHdgMax.Value); }
            if (data.AzTimeSlowMs != null) { _numAzTimeSlow.UpdateCurrentValue(data.AzTimeSlowMs.Value);   _numAzTimeSlow.SyncEntryOnce(data.AzTimeSlowMs.Value); }
            if (data.AzTimeFastMs != null) { _numAzTimeFast.UpdateCurrentValue(data.AzTimeFastMs.Value);   _numAzTimeFast.SyncEntryOnce(data.AzTimeFastMs.Value); }
            if (data.AzSpeedSlow  != null) { _numAzSpeedSlow.UpdateCurrentValue(data.AzSpeedSlow.Value);   _numAzSpeedSlow.SyncEntryOnce(data.AzSpeedSlow.Value); }
            if (data.AzSpeedFast  != null) { _numAzSpeedFast.UpdateCurrentValue(data.AzSpeedFast.Value);   _numAzSpeedFast.SyncEntryOnce(data.AzSpeedFast.Value); }
            if (data.AzUseBno     != null) { _numAzUseBno.UpdateCurrentValue(data.AzUseBno.Value);         _numAzUseBno.SyncEntryOnce(data.AzUseBno.Value); }
            if (data.AzUseGps     != null) { _numAzUseGps.UpdateCurrentValue(data.AzUseGps.Value);         _numAzUseGps.SyncEntryOnce(data.AzUseGps.Value); }
            if (data.AzBeta       != null) { _numAzBeta.UpdateCurrentValue(data.AzBeta.Value);             _numAzBeta.SyncEntryOnce(data.AzBeta.Value); }
        }

        /// <summary>
        /// Searches for which array element actually HOLDS the given
        /// firmware value, rather than assuming array index equals
        /// firmware value directly — needed because Board
        /// Configuration's two slots (and the Brand panel) each have
        /// their own, non-sequential option order (see
        /// BuildBoardConfigurationTab()'s ordering note).
        /// </summary>
        private void SyncRadioArrayByValue(RadioButton[] buttons, int[] valuesInOrder, int? firmwareValue)
        {
            if (firmwareValue == null) return;
            for (int i = 0; i < valuesInOrder.Length && i < buttons.Length; i++)
            {
                if (valuesInOrder[i] == firmwareValue.Value)
                {
                    buttons[i].Checked = true;
                    return;
                }
            }
        }

        /// <summary>
        /// Three near-identical radio-button/checkbox sync helpers —
        /// nullable firmware value in, matching control checked/
        /// selected if a value was actually received (null means
        /// older firmware without this field, or the sentence just
        /// hasn't arrived yet — left untouched either way, not reset
        /// to some default). Kept as separate small methods rather
        /// than one generic helper since each maps a genuinely
        /// different value range onto a genuinely different set of
        /// controls (checkbox vs. two different radio-button groups).
        /// </summary>
        private void SyncGnssPassthroughCheckbox(int? gnssPassthrough)
        {
            if (gnssPassthrough == null) return;
            _cbGnssPassthrough.Checked = (gnssPassthrough.Value == 1);
        }

        private void SyncImuTypeRadioButtons(int? imuType)
        {
            if (imuType == null) return;
            switch (imuType.Value)
            {
                case 0: _rbImuBno08x.Checked = true; break;
                case 1: _rbImuTm171.Checked = true; break;
                case 2: _rbImuNone.Checked = true; break;
            }
        }

        private void SyncGnssModeRadioButtons(int? gnssMode)
        {
            if (gnssMode == null) return;
            switch (gnssMode.Value)
            {
                case 1: _rbUm982.Checked = true; break;
                case 2: _rbSingleImu.Checked = true; break;
                case 3: _rbDual.Checked = true; break;
            }
        }

        /// <summary>
        /// Läser en bild inbäddad via EmbeddedResource (se .csproj-
        /// filens egen kommentar om varför den vägen valdes framför
        /// .resx). Fullt kvalificerat resursnamn = standard-
        /// namnrymden ("TeensyTool.App", projektfilens eget namn,
        /// ingen egen RootNamespace satt) + mappstruktur med punkter
        /// istället för snedstreck + filnamn.
        ///
        /// Fångar fel istället för att låta ett saknat/felstavat
        /// resursnamn krascha HELA programmet vid start — en enda
        /// förklaringsbild är inte värd att riskera det för. Om något
        /// går fel returneras null; PictureBox visar då bara en tom
        /// yta, inget kraschar.
        /// </summary>
        private Image LoadEmbeddedImage(string fileName)
        {
            try
            {
                string resourceName = "TeensyTool.App.Resources." + fileName;
                using (Stream stream = Assembly.GetExecutingAssembly().GetManifestResourceStream(resourceName))
                {
                    if (stream == null) return null;
                    return Image.FromStream(stream);
                }
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// The single choke point every SETxxx command in this file
        /// passes through before reaching TeensyConnection — guards
        /// against sending anything at all before a connection exists
        /// (_connection still null, e.g. if StartConnection() failed
        /// at startup) or an empty/whitespace-only command (a raw
        /// text-box Send with nothing typed).
        /// </summary>
        private void SendCommand(string command)
        {
            if (_connection == null) return;
            if (string.IsNullOrWhiteSpace(command)) return;
            _connection.SendCommand(command);
        }
    }
}
