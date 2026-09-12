using System.Globalization;

namespace TeensyTool.Protocol
{
    /// <summary>
    /// Ren valideringslogik för numeriska inställningar — inget UI
    /// alls här, medvetet. Konceptet (min/max-gräns, tydligt fel om
    /// utanför intervallet) är lånat från AGO:s egen FormNumeric.cs,
    /// men skrivet om helt utan beroende av någon specifik UI-form —
    /// exakt därför att vi VET att UI:t kring den här logiken kommer
    /// bytas ut senare (från vår enkla textruta till en riktig
    /// pekskärms-knappsats som i AGO), medan själva valideringsregeln
    /// ("är talet inom tillåtet intervall?") förblir densamma oavsett.
    /// </summary>
    public static class NumericValidator
    {
        /// <summary>
        /// "out double value, out string error" — två separata
        /// out-parametrar istället för en enda PDiagData-liknande
        /// klass, eftersom resultatet här bara är EN sak (ett tal)
        /// snarare än många fält — enklare att bara ha två separata
        /// utdata: "här är talet" ELLER "här är varför det inte gick".
        /// </summary>
        public static bool TryValidate(
            string input, double min, double max,
            out double value, out string error)
        {
            value = 0;
            error = null;

            if (string.IsNullOrWhiteSpace(input))
            {
                error = "No value entered.";
                return false;
            }

            // Ersätter komma med punkt INNAN parsning — svensk (och
            // stor del av europeisk) standard är komma som
            // decimaltecken, men InvariantCulture nedan kräver alltid
            // punkt (avsiktligt, se kommentaren där) — utan den här
            // raden avvisades ett fullt giltigt, naturligt skrivet
            // tal som "2,5" med "not a valid number", ett verkligt,
            // fältrapporterat problem. Säkert att göra rakt av: en
            // enkel textersättning, inget smart kultur-baserat
            // tolkningsförsök, eftersom NumberStyles.Float (nedan)
            // ALDRIG tillåter tusentalsavgränsare — kommatecknet kan
            // aldrig betyda något ANNAT än decimaltecken här, ingen
            // risk att "1,234" skulle feltolkas som ettusen tvåhundra
            // trettiofyra.
            // Originaltexten sparas separat — annars skulle ett
            // FORTFARANDE ogiltigt tal (t.ex. "abc,5") visa den redan
            // komma-till-punkt-konverterade versionen i felmeddelandet
            // ("abc.5 is not a valid number"), inte vad användaren
            // faktiskt skrev. Liten detalj, men värd att göra rätt.
            string originalInput = input;
            input = input.Replace(',', '.');

            // CultureInfo.InvariantCulture — avgörande detalj: utan
            // den skulle "0.5" tolkas olika beroende på vilket
            // språk/region datorn är inställd på (vissa länder
            // använder komma som decimaltecken istället för punkt).
            // Vi vill att "0.5" alltid betyder samma sak, oavsett
            // vems dator programmet körs på — samma anledning som vi
            // använde CultureInfo.InvariantCulture i PDiagParser för
            // att tolka inkommande satser.
            if (!double.TryParse(input, NumberStyles.Float,
                                  CultureInfo.InvariantCulture, out double parsed))
            {
                error = $"'{originalInput}' is not a valid number.";
                return false;
            }

            if (parsed < min)
            {
                error = $"Value must be at least {min}.";
                return false;
            }

            if (parsed > max)
            {
                error = $"Value must be at most {max}.";
                return false;
            }

            value = parsed;
            return true;
        }
    }
}
