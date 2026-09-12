using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;

namespace TeensyTool.App
{
    /// <summary>
    /// Hela diagnosloggnings-infrastrukturen, som en egen klass —
    /// bruten ut från MainForm.cs (som redan är mycket stor) snarare
    /// än att klämma in ~150 rader loggningslogik där. En trogen,
    /// fullständig portning av Python-versionens motsvarande
    /// funktion (send_logfile_toggle/_log_line/_log_health_transition/
    /// _log_heartbeat), inte bara den enkla "status only"-platshållare
    /// C#-porten tidigare hade.
    ///
    /// Rent lokal — skickar ALDRIG något till Teensyn, bara skriver
    /// till en textfil på den här datorn, matchande Pythons egen,
    /// redan etablerade princip.
    /// </summary>
    public class DiagnosticLogger
    {
        private StreamWriter _logFile;
        private readonly Dictionary<string, bool> _healthState = new Dictionary<string, bool>();
        private string _teensyIp;

        public bool IsLogging => _logFile != null;
        public string LogFilePath { get; private set; }

        /// <summary>
        /// Öppnar loggfilen — samma robusta, absoluta sökväg-lösning
        /// som Python fick efter ett verkligt, fältrapporterat problem
        /// (Errno 13/Permission denied): skriver INTE till en relativ
        /// sökväg (som skulle bero på Windows egen "aktuella
        /// arbetskatalog", oförutsägbar om verktyget körs från en
        /// tillfällig temp-mapp skapad av en e-postklient/dokumentvisare
        /// vid extrahering av en bilaga) — utan mot EXE-filens EGEN,
        /// absoluta mapp. Returnerar (success, errorMessage) istället
        /// för att kasta — anroparen (MainForm) visar felet i UI:t
        /// snarare än att låta ett loggningsfel krascha resten av
        /// verktyget, matchande Pythons egen "nice-to-have, not
        /// something that should take the whole tool down"-princip.
        /// </summary>
        /// <summary>
        /// targetDirectory är valfri — null (standard) ger exakt samma
        /// beteende som tidigare (.exe-filens egen mapp). Given via
        /// MainForm, som frågar användaren med WinForms egen, färdiga
        /// FolderBrowserDialog innan Start() anropas, istället för att
        /// bygga en egen dialogruta här.
        /// </summary>
        public (bool success, string error) Start(string teensyIp, string targetDirectory = null)
        {
            _teensyIp = teensyIp;
            string dir = targetDirectory ?? Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);
            string fileName = $"tff_log_{DateTime.Now:yyyyMMdd_HHmmss}.txt";
            string fullPath = Path.Combine(dir ?? ".", fileName);

            try
            {
                _logFile = new StreamWriter(fullPath, append: false) { AutoFlush = true };
                LogFilePath = fullPath;
                _healthState.Clear();
                LogLine($"=== Teensy Tool logging started (Teensy IP: {teensyIp ?? "unknown"}) ===");
                return (true, null);
            }
            catch (Exception e)
            {
                _logFile = null;
                LogFilePath = null;
                // Samma tydligare, mer hjälpsamma felmeddelande som
                // Python-versionen — nämner den TROLIGA, verkliga
                // orsaken (körd från en icke-skrivbar/tillfällig plats)
                // istället för bara det råa undantagsmeddelandet.
                return (false, $"Could not open log file (folder not writable — try running "
                    + $"the .exe from Downloads/Documents first): {e.Message}");
            }
        }

        /// <summary>
        /// Stänger loggfilen — matchar Pythons "=== Logging stopped ==="-
        /// avslutningsrad, och Pythons egen felsäkra .close()-hantering
        /// (ett fel vid stängning ska inte krascha verktyget).
        /// </summary>
        public void Stop()
        {
            if (_logFile == null) return;
            LogLine("=== Logging stopped ===");
            try { _logFile.Close(); } catch { /* samma "nice-to-have" -princip som Start() */ }
            _logFile = null;
            LogFilePath = null;
        }

        /// <summary>
        /// Grundläggande, tidsstämplad radskrivning — gör ingenting
        /// alls om loggning är avstängd (samma no-op-mönster som
        /// Python), och låter ALDRIG ett skrivfel (disk full,
        /// behörighet borttagen mitt i sessionen, etc.) krascha resten
        /// av verktyget.
        /// </summary>
        public void LogLine(string text)
        {
            if (_logFile == null) return;
            try
            {
                string ts = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff");
                _logFile.WriteLine($"{ts}  {text}");
            }
            catch { /* samma felsäkra princip som Start()/Stop() */ }
        }

        /// <summary>
        /// Delad av alla ~12 hälsokontroller (CAN-timeouts, GNSS/RTK-
        /// watchdogs, WAS-rimlighet, Ethernet-länk, IMU-watchdog, Keya-
        /// fel) istället för ett nästan identiskt block per kontroll —
        /// samma återanvändningsprincip som _health_transition i
        /// Python. Loggar EXAKT en gång när ett problem BÖRJAR och en
        /// gång när det UPPHÖR — aldrig varje enskild pollning medan
        /// tillståndet är oförändrat. _healthState fungerar också som
        /// källan för "vilka problem är aktiva just nu" som
        /// LogHeartbeat() nedan använder — en enda sanningskälla för
        /// båda, inte separat tillståndsspårning på två ställen.
        /// </summary>
        public void LogHealthTransition(string name, bool isProblem, string problemText)
        {
            bool wasProblem = _healthState.TryGetValue(name, out bool prev) && prev;
            if (isProblem && !wasProblem)
                LogLine($"COMMUNICATION ERROR: {problemText}");
            else if (wasProblem && !isProblem)
                LogLine($"RESOLVED: {problemText}");
            _healthState[name] = isProblem;
        }

        /// <summary>
        /// Periodisk (var 30:e sekund, anropad från en Timer i
        /// MainForm) "är allt faktiskt bra"-sammanfattning — samma
        /// tre-tillstånds-logik som Python: ingen signal alls, signal
        /// men ingen bearbetning (fångar det verkliga fältfallet där
        /// UDP-mottagning fungerade men _apply()/UpdateLabels() själv
        /// hade slutat köra), eller genuint frisk. Att skanna en lång
        /// logg efter FRÅNVARON av fel är mycket svårare än att se en
        /// tydlig, återkommande bekräftelse — därför denna rad finns,
        /// inte bara felloggning.
        /// </summary>
        public void LogHeartbeat(DateTime? lastRawPacketTime, DateTime? lastSuccessfulUpdateTime)
        {
            if (_logFile == null) return;

            bool hasRecentSignal = lastRawPacketTime.HasValue
                && (DateTime.Now - lastRawPacketTime.Value).TotalSeconds <= 10;

            if (!hasRecentSignal)
            {
                // Ingen signal alls ännu — "signal lost"-övergången
                // (loggad separat, i MainForm) täcker redan det
                // fallet, inget att lägga till här ovanpå.
                return;
            }

            bool updateStale = !lastSuccessfulUpdateTime.HasValue
                || (DateTime.Now - lastSuccessfulUpdateTime.Value).TotalSeconds > 10;

            if (updateStale)
            {
                LogLine("STATUS: receiving data but NOT processing it — "
                    + "UpdateLabels() appears to have stopped (try restarting Teensy Tool)");
                return;
            }

            var active = new List<string>();
            foreach (var kvp in _healthState)
                if (kvp.Value) active.Add(kvp.Key);

            if (active.Count > 0)
                LogLine($"STATUS: ongoing issue(s) — {string.Join(", ", active)}");
            else
                LogLine("STATUS OK — signal alive, no active issues");
        }
    }
}
