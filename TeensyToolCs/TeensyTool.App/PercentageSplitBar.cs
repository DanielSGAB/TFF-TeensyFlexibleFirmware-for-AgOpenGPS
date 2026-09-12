using System;
using System.Drawing;
using System.Windows.Forms;

namespace TeensyTool.App
{
    /// <summary>
    /// En helt egen, RITAD kontroll — till skillnad från allt annat vi
    /// byggt hittills (Label, Button, RadioButton, som alla är
    /// färdiga, inbyggda WinForms-kontroller). Här ritar vi själva,
    /// för hand, via OnPaint().
    ///
    /// Medvetet UTAN färgkodning (grönt/gult/rött) — inget läge är
    /// "fel", baren visar bara var i spannet blandningen befinner sig
    /// just nu, rent informativt. Visas ALLTID, även vid ren
    /// Single+IMU (0%) eller ren Dual (100%) — det blir bara en tydlig
    /// visning av vilket tillstånd som råder just då, ingen anledning
    /// att dölja den i något läge.
    ///
    /// Procenttexten ovanför baren är LEVANDE (visar den faktiska,
    /// aktuella fördelningen), men står på en FAST position (alltid
    /// centrerad) — en mellanväg mellan två tidigare, avfärdade
    /// varianter: dels en helt fast "50/50"-referenstext (blev
    /// missvisande så fort fördelningen genuint skiljde sig från
    /// 50/50), dels en tidigare version där texten även FLYTTADE i
    /// sidled efter fyllningsgränsen.
    /// </summary>
    public class PercentageSplitBar : UserControl
    {
        private int _dualPercent = 50;   // 0 = helt Single+IMU, 100 = helt Dual
        private readonly string _leftLabel;
        private readonly string _rightLabel;

        /// <summary>
        /// "public int DualPercent { get; set; }" hade räckt om vi
        /// bara ville LAGRA värdet — men vi vill att kontrollen RITAR
        /// OM SIG SJÄLV varje gång värdet ändras. "Invalidate()" är
        /// WinForms egen, inbyggda signal "något har ändrats, rita om
        /// mig" — den anropar automatiskt OnPaint() igen åt oss.
        /// </summary>
        public int DualPercent
        {
            get => _dualPercent;
            set
            {
                _dualPercent = Math.Max(0, Math.Min(100, value));
                Invalidate();
            }
        }

        public PercentageSplitBar(string leftLabel, string rightLabel)
        {
            _leftLabel = leftLabel;
            _rightLabel = rightLabel;
            Width = 420;
            // 55 -> 70: 25 (barTop) + 14 (barHeight) + 4 (marginal) +
            // en hel textrad för "Single+IMU"/"DUAL"-etiketterna under
            // baren (ca 20px med detta teckensnitt) rymdes INTE inom
            // 55px — etiketterna klipptes tyst i botten, WinForms
            // ritar aldrig utanför en kontrolls egna gränser. Den
            // faktiska huvudorsaken till att etiketterna "inte fick
            // plats" i praktiken, inte bara en marginalfråga i sidled.
            Height = 70;

            // Dubbelbuffring — förhindrar synligt flimmer varje gång
            // kontrollen ritas om (vanligt problem med egen-ritade
            // kontroller om man inte slår på det här explicit).
            DoubleBuffered = true;
        }

        /// <summary>
        /// Anropas AUTOMATISKT av WinForms varje gång kontrollen
        /// behöver ritas om — vid start, vid storleksändring, och
        /// varje gång vi själva anropar Invalidate() (se DualPercent
        /// ovan). Vi anropar ALDRIG den här metoden direkt själva.
        /// </summary>
        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;

            const int barTop = 25;
            const int barHeight = 14;
            int barLeft = 0;
            int barWidth = Width;

            // Bakgrundsspåret — en ljusgrå, tom rektangel som
            // representerar HELA skalan (0-100%).
            using (var trackBrush = new SolidBrush(Color.Gainsboro))
            {
                g.FillRectangle(trackBrush, barLeft, barTop, barWidth, barHeight);
            }

            // Den fyllda delen — hur långt in mot "Dual" (höger sida)
            // blandningen befinner sig just nu. Medvetet EN enda,
            // neutral grön ton hela vägen — INTE en färg som byter
            // (gul/röd) beroende på var på skalan vi är, eftersom
            // inget läge är "fel" eller "sämre" än ett annat.
            int fillWidth = (int)(barWidth * (_dualPercent / 100.0));
            using (var fillBrush = new SolidBrush(Color.MediumSeaGreen))
            {
                g.FillRectangle(fillBrush, barLeft, barTop, fillWidth, barHeight);
            }

            // Levande procenttal — den FAKTISKA, aktuella fördelningen,
            // inte ett fast referensmärke. Bytt tillbaka från en fast
            // "50/50"-text efter direkt återkoppling: en text som ALDRIG
            // ändras, samtidigt som fyllningen tydligt visar ett helt
            // annat läge (t.ex. 85% fyllt men texten fortfarande säger
            // "50/50"), blev mer förvirrande än hjälpsamt i praktiken.
            // Fast POSITION (alltid centrerad ovanför baren), men
            // levande INNEHÅLL — en mellanväg mellan "helt fast" och
            // den ursprungliga versionen som även lät texten FLYTTA
            // sig i sidled efter fyllningsgränsen.
            string splitText = $"{100 - _dualPercent}% / {_dualPercent}%";
            SizeF splitSize = g.MeasureString(splitText, Font);
            float splitX = (barWidth - splitSize.Width) / 2f;
            g.DrawString(splitText, Font, Brushes.Black, splitX, 2);

            // Ändetiketterna ("Single+IMU" / "DUAL") — fasta, ritas
            // aldrig om sin position, bara sin egen text. 2px marginal
            // på båda sidor (inte exakt x=0/barWidth) — annars trycks
            // texten helt mot kontrollens egen vänster-/högerkant, med
            // risk att se klämd/avskuren ut beroende på vilken
            // container kontrollen själv sitter i.
            g.DrawString(_leftLabel, Font, Brushes.Black, 2, barTop + barHeight + 4);
            SizeF rightSize = g.MeasureString(_rightLabel, Font);
            g.DrawString(_rightLabel, Font, Brushes.Black, barWidth - rightSize.Width - 2, barTop + barHeight + 4);

            // De separata "100%"-referensmarkeringarna togs bort — när
            // baren ändå visar de LEVANDE procenttalen ovan blir en
            // fast "100% i varje ände"-påminnelse överflödig; talen
            // ovan säger redan allt som behövs.
        }
    }
}
