using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;

namespace TeensyTool.App
{
    /// <summary>
    /// Läser en loggfil skriven av DiagnosticLogger (eller Pythonverk-
    /// tygets motsvarande, nästan identiskt formaterade logg — samma
    /// "COMMUNICATION ERROR:"/"RESOLVED:"/"STATUS..."-mönster fungerar
    /// för båda) och bygger en KORT, mänskligt läsbar sammanfattning —
    /// INTE bara en rå visning av loggens rader. Byggd specifikt för
    /// någon som inte redan vet vilka mönster att leta efter (given
    /// att VI själva bestämt exakt vad som skrivs, kan vi matcha det
    /// exakt, i motsats till att gissa/tolka en godtycklig textfil).
    ///
    /// Egen, separat klass (inte inbakad i DiagnosticLogger, som bara
    /// SKRIVER loggen, aldrig LÄSER en) — samma "en klass, ett tydligt
    /// ansvar"-princip vi redan följt genomgående.
    /// </summary>
    public static class LogAnalyzer
    {
        private static readonly Regex TimestampPattern =
            new Regex(@"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\s+(.*)$");

        /// <summary>
        /// Huvudmetoden — läser filen, bygger sammanfattningen som en
        /// enda, färdig sträng att visa direkt i UI:t. Kastar aldrig
        /// vidare ett filfel till anroparen okontrollerat — returnerar
        /// istället ett tydligt felmeddelande som text, matchande
        /// samma "ett loggningsproblem ska aldrig krascha verktyget"-
        /// princip som DiagnosticLogger själv följer.
        /// </summary>
        public static string Analyze(string filePath)
        {
            string[] lines;
            try
            {
                lines = File.ReadAllLines(filePath);
            }
            catch (Exception e)
            {
                return $"Could not read the log file:\n{e.Message}";
            }

            if (lines.Length == 0)
                return "The file is empty — nothing to analyze.";

            DateTime? sessionStart = null;
            DateTime? sessionEnd = null;
            DateTime? lastTimestamp = null;
            bool endedCleanly = false;
            int dataLineCount = 0;

            // problemText -> (antal gånger startat, total varaktighet,
            // pågår just nu sedan när)
            var issues = new Dictionary<string, (int count, TimeSpan totalDuration, DateTime? startedAt)>();
            DateTime? signalLostAt = null;
            int signalLostCount = 0;
            TimeSpan signalLostTotalDuration = TimeSpan.Zero;
            var resetEvents = new List<string>();
            var unparseableCount = 0;
            var threadErrors = new List<string>();

            foreach (string rawLine in lines)
            {
                var m = TimestampPattern.Match(rawLine);
                if (!m.Success) continue;   // rad utan giltig tidsstämpel — hoppa över, inte krascha

                DateTime ts = DateTime.Parse(m.Groups[1].Value);
                string text = m.Groups[2].Value;
                lastTimestamp = ts;

                if (text.StartsWith("=== Teensy Tool logging started"))
                {
                    sessionStart = ts;
                }
                else if (text.StartsWith("=== Logging stopped") || text.StartsWith("=== Teensy Tool closing"))
                {
                    sessionEnd = ts;
                    endedCleanly = true;
                }
                else if (text.StartsWith("DATA:"))
                {
                    dataLineCount++;
                }
                else if (text.Contains("no $PDIAG received") && text.StartsWith("COMMUNICATION ERROR:"))
                {
                    // Signalförlust hanteras SEPARAT från de vanliga
                    // COMMUNICATION ERROR:/RESOLVED:-paren — se denna
                    // klass egen header-kommentar för varför.
                    signalLostAt = ts;
                    signalLostCount++;
                }
                else if (text == "COMMUNICATION: signal restored")
                {
                    if (signalLostAt.HasValue)
                    {
                        signalLostTotalDuration += (ts - signalLostAt.Value);
                        signalLostAt = null;
                    }
                }
                else if (text.StartsWith("COMMUNICATION ERROR: "))
                {
                    string problemText = text.Substring("COMMUNICATION ERROR: ".Length);
                    var entry = issues.TryGetValue(problemText, out var existing)
                        ? existing : (count: 0, totalDuration: TimeSpan.Zero, startedAt: (DateTime?)null);
                    entry.count++;
                    entry.startedAt = ts;
                    issues[problemText] = entry;
                }
                else if (text.StartsWith("RESOLVED: "))
                {
                    string problemText = text.Substring("RESOLVED: ".Length);
                    if (issues.TryGetValue(problemText, out var entry) && entry.startedAt.HasValue)
                    {
                        entry.totalDuration += (ts - entry.startedAt.Value);
                        entry.startedAt = null;
                        issues[problemText] = entry;
                    }
                }
                else if (text.StartsWith("TEENSY RESET CAUSE:"))
                {
                    resetEvents.Add(text.Substring("TEENSY RESET CAUSE:".Length).Trim());
                }
                else if (text.StartsWith("UNPARSEABLE PACKET"))
                {
                    unparseableCount++;
                }
                else if (text.StartsWith("RX THREAD ERROR:"))
                {
                    threadErrors.Add(text.Substring("RX THREAD ERROR:".Length).Trim());
                }
            }

            return BuildSummary(sessionStart, sessionEnd, lastTimestamp, endedCleanly, dataLineCount,
                issues, signalLostCount, signalLostTotalDuration, signalLostAt.HasValue,
                resetEvents, unparseableCount, threadErrors);
        }

        /// <summary>
        /// Bygger själva den lästa texten — separerad från Analyze()
        /// ovan så själva PARSNINGEN och själva TEXTFORMATERINGEN inte
        /// blandas i en enda, lång metod.
        /// </summary>
        private static string BuildSummary(
            DateTime? sessionStart, DateTime? sessionEnd, DateTime? lastTimestamp, bool endedCleanly,
            int dataLineCount, Dictionary<string, (int count, TimeSpan totalDuration, DateTime? startedAt)> issues,
            int signalLostCount, TimeSpan signalLostTotalDuration, bool signalStillLost,
            List<string> resetEvents, int unparseableCount, List<string> threadErrors)
        {
            var sb = new StringBuilder();

            // --- Sessionsöversikt ---
            sb.AppendLine("SESSION OVERVIEW");
            sb.AppendLine("----------------");
            if (sessionStart.HasValue && lastTimestamp.HasValue)
            {
                TimeSpan duration = lastTimestamp.Value - sessionStart.Value;
                sb.AppendLine($"Started: {sessionStart.Value:yyyy-MM-dd HH:mm:ss}");
                sb.AppendLine($"Duration: {FormatDuration(duration)}");
                sb.AppendLine($"Data packets logged: {dataLineCount}");
            }
            else
            {
                sb.AppendLine("Could not find a clear session start — this may not be a Teensy Tool log file.");
            }

            if (!endedCleanly)
            {
                // Ingen "Logging stopped"/"closing"-rad hittades — filen
                // slutar bara mitt i. Värt att flagga tydligt: kan
                // betyda verktyget kraschade, eller att datorn stängdes
                // av/tappade ström, INTE nödvändigtvis ett Teensy-
                // relaterat problem alls — men värt att veta om.
                sb.AppendLine();
                sb.AppendLine("⚠ This log does NOT end with a clean \"logging stopped\" marker.");
                sb.AppendLine("  The tool may have crashed, or the computer lost power/closed unexpectedly.");
            }
            sb.AppendLine();

            // --- Signalförlust ---
            sb.AppendLine("SIGNAL (network connection to Teensy)");
            sb.AppendLine("--------------------------------------");
            if (signalLostCount == 0)
            {
                sb.AppendLine("No signal loss detected — connection was stable throughout.");
            }
            else
            {
                sb.AppendLine($"Signal was lost {signalLostCount} time(s), "
                    + $"totaling {FormatDuration(signalLostTotalDuration)} without a response.");
                if (signalStillLost)
                    sb.AppendLine("⚠ Signal was STILL lost when logging ended — connection never recovered.");
            }
            sb.AppendLine();

            // --- Problemområden ---
            sb.AppendLine("PROBLEM AREAS FOUND");
            sb.AppendLine("--------------------");
            if (issues.Count == 0)
            {
                sb.AppendLine("None — no communication errors, watchdog timeouts, or faults were logged.");
            }
            else
            {
                // Sorterat efter total varaktighet, längst först — de
                // mest betydande problemen syns direkt, inte begravda
                // längst ner i en alfabetisk lista.
                foreach (var kv in issues.OrderByDescending(k => k.Value.totalDuration))
                {
                    string stillOngoing = kv.Value.startedAt.HasValue ? " (STILL ACTIVE when logging ended)" : "";
                    sb.AppendLine($"• {kv.Key}");
                    sb.AppendLine($"    Occurred {kv.Value.count} time(s), "
                        + $"totaling {FormatDuration(kv.Value.totalDuration)}{stillOngoing}");
                }
            }
            sb.AppendLine();

            // --- Övrigt ---
            bool hasExtras = resetEvents.Count > 0 || unparseableCount > 0 || threadErrors.Count > 0;
            if (hasExtras)
            {
                sb.AppendLine("OTHER NOTEWORTHY EVENTS");
                sb.AppendLine("------------------------");
                if (resetEvents.Count > 0)
                    sb.AppendLine($"Teensy restarted {resetEvents.Count} time(s): {string.Join(", ", resetEvents.Distinct())}");
                if (unparseableCount > 0)
                    sb.AppendLine($"{unparseableCount} malformed/unparseable packet(s) received — usually harmless network noise, not a concern unless very frequent.");
                if (threadErrors.Count > 0)
                    sb.AppendLine($"{threadErrors.Count} internal receive-thread error(s) — worth reporting if this recurs.");
                sb.AppendLine();
            }

            // --- Slutsats ---
            sb.AppendLine("OVERALL");
            sb.AppendLine("-------");
            bool anythingWrong = issues.Count > 0 || signalLostCount > 0 || !endedCleanly
                || resetEvents.Count > 0 || threadErrors.Count > 0;
            sb.AppendLine(anythingWrong
                ? "Issues were found — see the sections above for specifics."
                : "No problems detected in this log — everything looks healthy.");

            return sb.ToString();
        }

        private static string FormatDuration(TimeSpan span)
        {
            if (span.TotalHours >= 1) return $"{(int)span.TotalHours}h {span.Minutes}m {span.Seconds}s";
            if (span.TotalMinutes >= 1) return $"{(int)span.TotalMinutes}m {span.Seconds}s";
            return $"{span.Seconds}s";
        }
    }
}
