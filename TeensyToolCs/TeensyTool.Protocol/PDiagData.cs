namespace TeensyTool.Protocol
{
    /// <summary>
    /// Ett enda tolkat $PDIAG-meddelande. Motsvarar den Python-dict vi
    /// byggde upp i parse_pdiag() — men i C# använder vi en riktig
    /// klass med NAMNGIVNA, TYPADE egenskaper istället för en generisk
    /// nyckel-värde-samling.
    ///
    /// Varför det är bättre här: i Python skrev vi d.get('sats_m') —
    /// om vi stavat fel ("sats_mm") hade Python bara tyst gett oss
    /// None, och vi hade kanske aldrig märkt det förrän i produktion.
    /// Här ger C#:s KOMPILATOR ett fel direkt om vi skriver
    /// data.SatsMmm — samma sorts skydd som vårt eget "räkna aldrig
    /// för hand"-skript gav oss för fältindex, fast nu inbyggt i
    /// själva språket, för fältNAMN.
    ///
    /// "int?" (med frågetecken) betyder "nullable int" — ett heltal
    /// SOM ÄVEN kan vara "inget värde alls" (null). Det är C#:s sätt
    /// att uttrycka exakt samma sak som Pythons "kan vara ett tal
    /// eller None" — vi använder det för fält som bara är meningsfulla
    /// under vissa förutsättningar (t.ex. Brand är null om firmware är
    /// så gammal att den aldrig skickade det fältet alls).
    /// </summary>
    public class PDiagData
    {
        // De första fälten fanns redan i den allra första PDIAG-
        // versionen (v0.1) — de får ALDRIG vara null, eftersom även
        // den äldsta tänkbara firmware skickar dem. Matchar Python-
        // sidans mönster där dessa fick konkreta default-värden
        // (0, 0.0, etc.) snarare än None.
        public string Mode { get; set; } = "";
        public int SatsMaster { get; set; }
        public int SatsSlave { get; set; }
        public int HprSats { get; set; }
        public int SolQuality { get; set; }
        public float HeadingOffsetDeg { get; set; }
        public float HeadingAlpha { get; set; }
        public float RollAlpha { get; set; }
        public float InitialHeadingAlpha { get; set; }
        public float InitialRollAlpha { get; set; }
        public int DualPercent { get; set; }
        public int SatsFullThreshold { get; set; }
        public float RollZeroOffsetDeg { get; set; }
        public float DualHoldSeconds { get; set; }
        public float DualRampSeconds { get; set; }
        public int ImuAxis { get; set; }
        public int RollInvert { get; set; }
        public float WasLeftDeg { get; set; }
        public float WasRightDeg { get; set; }
        public int UTurnStrength { get; set; }
        public float SteerAngleActualDeg { get; set; }

        // Fälten nedan tillkom senare i TFF:s historia (Brand v0.6,
        // resten under TFF-utvecklingen). Nullable ("?") av exakt det
        // skälet — samma "None mot äldre firmware"-resonemang som i
        // Python-kommentarerna.
        public int? Brand { get; set; }
        public int? GnssMode { get; set; }
        public float? DualRollRaw { get; set; }
        public float? ImuRollRaw { get; set; }
        // Index 26 — firmware sätter INTE DualRollRaw till null/NaN när
        // ingen giltig dual-lösning finns just nu, den behåller sitt
        // gamla, inaktuella värde ("stale/last-known", firmwarens egen
        // kommentar) — den här separata flaggan är det ENDA sättet att
        // veta om värdet ovan faktiskt är aktuellt. Ignoreras felaktigt
        // fram tills nu — se MainForm.cs för den korrigerade
        // användningen.
        public int?   RollValid { get; set; }
        public int? ImuType { get; set; }
        public int? BoardSlot1 { get; set; }
        public int? BoardSlot2 { get; set; }
        public int? GnssPassthrough { get; set; }
        public int? CanKBusTimeout { get; set; }
        public int? CanIsoBusTimeout { get; set; }
        public int? CanVBusTimeout { get; set; }
        public int? CanContentImplausible { get; set; }
        public int? GnssWatchdog { get; set; }
        public int? RtkTimeout { get; set; }
        public int? WasImplausible { get; set; }
        public int? EthernetLinkUp { get; set; }
        public int? ResetCause { get; set; }
        public int? ImuHealthy { get; set; }
        public int? ImuRetryCount { get; set; }
        public int? UseImu { get; set; }
        public int? MotorDriveType { get; set; }
        public int? WasSource { get; set; }
        public int? KeyaDetected { get; set; }
        public int? KeyaFaultActive { get; set; }
        public int? HprTimeout { get; set; }

        // v0.3.7, det senaste tillskottet — den "levande"
        // felsökningsåldern vi byggde tillsammans i förra steget.
        public float? HprAgeSeconds { get; set; }

        // v0.3.10 — Keya-as-WAS auto-zero (zKeyaAutoZero.ino). All
        // nullable, matching HprAgeSeconds' own pattern above — older
        // firmware without these fields simply gives null here, not a
        // parse failure for the whole sentence.
        public int?   AzZeroDone   { get; set; }
        public float? AzZeroDeg    { get; set; }
        public float? AzSpeedMin   { get; set; }
        public float? AzYawRateMax { get; set; }
        public float? AzGpsHdgMax  { get; set; }
        public int?   AzTimeSlowMs { get; set; }
        public int?   AzTimeFastMs { get; set; }
        public float? AzSpeedSlow  { get; set; }
        public float? AzSpeedFast  { get; set; }
        public int?   AzUseBno     { get; set; }
        public int?   AzUseGps     { get; set; }
        public float? AzBeta       { get; set; }
        // v0.3.17/v0.3.18 — Auto Roll Adjust. The existing
        // RollZeroOffsetDeg field (index 15/16 area, already parsed
        // above) now reports the EFFECTIVE, combined value
        // (rollZeroOffset+rollAutoCorrection) directly from firmware
        // — no separate "combined" property needed, the existing
        // Current display already shows the right number.
        public int?   AutoRollAdjust   { get; set; }
        public float? RollAutoDeadband { get; set; }
        public float? RollAutoAlpha    { get; set; }
    }
}
