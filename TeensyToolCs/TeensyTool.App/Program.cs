using System;
using System.Windows.Forms;

namespace TeensyTool.App
{
    internal static class Program
    {
        // [STAThread] — ett obligatoriskt attribut för alla WinForms-
        // program. STA står för "Single-Threaded Apartment", ett
        // gammalt Windows COM-koncept som WinForms fortfarande bygger
        // på under huven. Du behöver inte förstå VARFÖR i detalj — bara
        // veta att den här raden alltid ska finnas precis ovanför
        // Main() i alla WinForms-program, annars kan vissa
        // dialogrutor/urklipp-funktioner bete sig konstigt.
        [STAThread]
        static void Main()
        {
            // Standardmönstret för att starta ett WinForms-program —
            // samma tre rader i praktiskt taget varje sådant program
            // som någonsin skrivits, inklusive AGO:s egen Program.cs.
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MainForm());
        }
    }
}
