using System;
using System.Globalization;

namespace TeensyTool.Protocol
{
    /// <summary>
    /// Tolkar en rå $PDIAG-textrad från Teensyn till ett PDiagData-
    /// objekt. Motsvarar Pythons fristående funktion parse_pdiag().
    /// </summary>
    public static class PDiagParser
    {
        /// <summary>
        /// "static class" — precis som en Python-modulnivåfunktion,
        /// inte en metod på ett objekt. Vi behöver aldrig skapa en
        /// "PDiagParser"-instans (new PDiagParser()) för att använda
        /// den här metoden, exakt som vi aldrig gjorde
        /// "ny instans av modulen" för att anropa parse_pdiag(sats) i
        /// Python.
        ///
        /// "TryParse"-mönstret: C#:s standardkonvention för "försök
        /// göra det här, tala om ifall det gick" — du känner redan
        /// igen den från t.ex. int.TryParse(text, out int resultat) i
        /// .NET:s eget bibliotek. "out PDiagData result" betyder: om
        /// metoden lyckas (returnerar true), har "result"-parametern
        /// fyllts i med det färdiga objektet av anroparen. Det är C#:s
        /// motsvarighet till Pythons "returnera antingen ett riktigt
        /// objekt ELLER None" — bara uttryckt med två separata delar
        /// (ett bool-returvärde + en out-parameter) istället för ett
        /// enda värde som kan vara antingen/eller.
        /// </summary>
        public static bool TryParse(string sentence, out PDiagData result)
        {
            result = null;
            sentence = sentence.Trim();

            // Motsvarar Pythons re.match(r'^\$(.+)\*([0-9A-Fa-f]{2})$', ...)
            // C# har inget lika kort inbyggt "matcha och plocka ut
            // grupper"-uttryck utan att importera System.Text.
            // RegularExpressions separat, så här görs det istället med
            // enkel strängsökning — samma logik, bara utskriven för
            // hand.
            if (!sentence.StartsWith("$") || sentence.Length < 4)
                return false;

            int starIndex = sentence.LastIndexOf('*');
            if (starIndex < 0 || starIndex != sentence.Length - 3)
                return false;

            string body = sentence.Substring(1, starIndex - 1);
            string checksumHex = sentence.Substring(starIndex + 1, 2);

            // Kontrollsumman — bokstavligen samma XOR-loop som i både
            // Python och den ursprungliga C++-firmwaren. Den här typen
            // av liten, delad beräkning är exakt det som är värt att
            // hålla IDENTISK mellan alla tre språken, snarare än att
            // "förbättra" den olika på varje ställe.
            int checksum = 0;
            foreach (char c in body)
                checksum ^= c;

            if (!int.TryParse(checksumHex, NumberStyles.HexNumber,
                               CultureInfo.InvariantCulture, out int expectedChecksum))
                return false;

            if (checksum != expectedChecksum)
                return false;

            string[] parts = body.Split(',');
            if (parts.Length < 7 || parts[0] != "PDIAG")
                return false;

            // Motsvarande vår _f()-hjälpfunktion i Python — men här är
            // den en lokal funktion (en funktion definierad INUTI en
            // annan metod, bara synlig här) istället för en fristående
            // en. "local function" är ett C#-koncept som inte riktigt
            // fanns i vår Python-kod på samma sätt, men gör exakt
            // samma jobb: försök konvertera fältet på given position,
            // ge tillbaka null om det saknas ELLER om konverteringen
            // misslyckas — ALDRIG kasta ett fel som stoppar resten av
            // tolkningen. Det här är precis den fält-för-fält-
            // motståndskraft som var v0.3.2-fixen i Python, byggd in
            // från BÖRJAN här istället för tillagd efteråt.
            float? TryFloat(int index)
            {
                if (index >= parts.Length) return null;
                return float.TryParse(parts[index], NumberStyles.Float,
                                       CultureInfo.InvariantCulture, out float v) ? v : (float?)null;
            }

            int? TryInt(int index)
            {
                if (index >= parts.Length) return null;
                return int.TryParse(parts[index], out int v) ? v : (int?)null;
            }

            var data = new PDiagData
            {
                // De ovillkorliga fälten — samma "garanterat finns,
                // ge ett konkret default om konverteringen ändå
                // misslyckas" som i Python.
                Mode                 = parts.Length > 1 ? parts[1] : "",
                SatsMaster           = TryInt(2)  ?? 0,
                SatsSlave            = TryInt(3)  ?? 0,
                HprSats              = TryInt(4)  ?? 0,
                SolQuality           = TryInt(5)  ?? 0,
                HeadingOffsetDeg     = TryFloat(6)  ?? 0f,
                HeadingAlpha         = TryFloat(7)  ?? 0f,
                RollAlpha            = TryFloat(8)  ?? 0f,
                InitialHeadingAlpha  = TryFloat(9)  ?? 0f,
                InitialRollAlpha     = TryFloat(10) ?? 0f,
                DualPercent          = TryInt(11)   ?? 100,
                SatsFullThreshold    = TryInt(12)   ?? 7,
                RollZeroOffsetDeg    = TryFloat(13) ?? 0f,
                DualHoldSeconds      = TryFloat(14) ?? 3.0f,
                DualRampSeconds      = TryFloat(15) ?? 5.0f,
                ImuAxis              = TryInt(16)   ?? 1,
                RollInvert           = TryInt(17)   ?? -1,
                WasLeftDeg           = TryFloat(18) ?? -45.0f,
                WasRightDeg          = TryFloat(19) ?? 45.0f,
                UTurnStrength        = TryInt(20)   ?? 1,
                SteerAngleActualDeg  = TryFloat(21) ?? 0f,

                // De valfria fälten (nullable) — helt enkelt "null om
                // det inte fanns eller inte gick att tolka", exakt
                // samma "None"-standardvärde för alla dessa i Python.
                Brand                = TryInt(22),
                GnssMode             = TryInt(23),
                DualRollRaw          = TryFloat(24),
                ImuRollRaw           = TryFloat(25),
                RollValid            = TryInt(26),
                ImuType              = TryInt(27),
                BoardSlot1           = TryInt(28),
                BoardSlot2           = TryInt(29),
                GnssPassthrough      = TryInt(30),
                CanKBusTimeout       = TryInt(31),
                CanIsoBusTimeout     = TryInt(32),
                CanVBusTimeout       = TryInt(33),
                CanContentImplausible = TryInt(34),
                GnssWatchdog         = TryInt(35),
                RtkTimeout           = TryInt(36),
                WasImplausible       = TryInt(37),
                EthernetLinkUp       = TryInt(38),
                ResetCause           = TryInt(39),
                ImuHealthy           = TryInt(40),
                ImuRetryCount        = TryInt(41),
                UseImu               = TryInt(42),
                MotorDriveType       = TryInt(43),
                WasSource            = TryInt(44),
                KeyaDetected         = TryInt(45),
                KeyaFaultActive      = TryInt(46),
                HprTimeout           = TryInt(47),
                HprAgeSeconds        = TryFloat(48),
                AzZeroDone           = TryInt(49),
                AzZeroDeg            = TryFloat(50),
                AzSpeedMin           = TryFloat(51),
                AzYawRateMax         = TryFloat(52),
                AzGpsHdgMax          = TryFloat(53),
                AzTimeSlowMs         = TryInt(54),
                AzTimeFastMs         = TryInt(55),
                AzSpeedSlow          = TryFloat(56),
                AzSpeedFast          = TryFloat(57),
                AzUseBno             = TryInt(58),
                AzUseGps             = TryInt(59),
                AzBeta               = TryFloat(60),
                AutoRollAdjust       = TryInt(61),
                RollAutoDeadband     = TryFloat(62),
                RollAutoAlpha        = TryFloat(63),
            };

            result = data;
            return true;
        }
    }
}
