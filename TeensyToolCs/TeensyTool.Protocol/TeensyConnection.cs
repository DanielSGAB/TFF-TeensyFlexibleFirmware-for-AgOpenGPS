using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace TeensyTool.Protocol
{
    /// <summary>
    /// Kapslar in UDP-kommunikationen med Teensyn — mottagning av
    /// $PDIAG och sändning av SETxxx-kommandon. Motsvarar, tillsammans,
    /// det som i Python låg utspritt i _start_udp()/_rx_thread()/
    /// _send_cmd().
    ///
    /// "public class TeensyConnection : IDisposable" — IDisposable är
    /// ett inbyggt C#-gränssnitt (interface) som säger "den här klassen
    /// äger något (här: en nätverkssocket) som MÅSTE städas upp
    /// uttryckligen, inte bara lämnas åt skräpsamlaren". Motsvarar
    /// varför vi i Python var noga med att stänga sockeln vid
    /// WM_DELETE_WINDOW — bara att C# gör kontraktet EXPLICIT och
    /// kompilatorkontrollerat via IDisposable/Dispose(), istället för
    /// att bara vara en konvention vi själva kom ihåg att följa.
    /// </summary>
    public class TeensyConnection : IDisposable
    {
        // "event EventHandler<T> NamnHändelse" — C#:s inbyggda
        // publish/subscribe-mönster. Vem som helst (t.ex. ett
        // framtida Forms-fönster) kan skriva
        // "connection.DataReceived += MinMetod;" för att bli
        // notifierad varje gång ny data tolkats — utan att
        // TeensyConnection behöver veta något alls om VEM som lyssnar.
        // Motsvarar konceptuellt att vi i Python lade nytt data i
        // self.pending_data och lät _poll() (en helt annan, oberoende
        // körd bit kod) plocka upp det senare — men här sker
        // "leveransen" direkt, händelsestyrt, istället för att någon
        // måste fråga "finns det något nytt?" var 500:e millisekund.
        //
        // (OBS: inget "?" efter EventHandler<PDiagData> här — det
        // hade varit samma "nullable reference types"-syntax som gav
        // oss ett kompileringsfel i PDiagParser.cs tidigare, eftersom
        // net48 utan explicit språkversionsinställning kör C# 7.3.
        // "DataReceived?.Invoke(...)" längre ner i filen ser
        // förvirrande likt ut, men ÄR en annan, mycket äldre och helt
        // säker funktion — se den kodkommentaren för skillnaden.)
        public event EventHandler<PDiagData> DataReceived;

        // NY: en synlig "något gick fel"-händelse, skild från
        // DataReceived. Lades till efter att vi insåg att den
        // ursprungliga "catch (Exception) { }" (nedan i
        // ReceiveLoopAsync) var EXAKT samma sorts blinda fläck som
        // Python-hjärtslagsbuggen vi jagade tidigare i den här
        // sessionen — genuina, viktiga fel (t.ex. att porten redan är
        // upptagen) skulle försvinna helt tyst, med noll synlig
        // information om VARFÖR ingenting fungerar.
        public event EventHandler<string> ErrorOccurred;

        // DateTime? (nullable, samma "?" -syntax som int?/float? vi
        // redan använde i PDiagData) — motsvarar exakt self.last_rx
        // i Python: null/None tills första paketet någonsin kommit.
        public DateTime? LastReceivedUtc { get; private set; }
        public string TeensyIpAddress { get; private set; }

        // NY: räknar VARJE mottaget UDP-paket, oavsett om det gick
        // att tolka som en giltig $PDIAG-sats eller inte. Skild från
        // LastReceivedUtc (som bara uppdateras vid LYCKAD tolkning) —
        // ger oss möjlighet att skilja "inget når nätverksnivån
        // överhuvudtaget" (den här räknaren förblir 0) från "något
        // kommer in men misslyckas tolkas av någon anledning" (den
        // här räknaren ökar, men LastReceivedUtc gör det inte).
        public int RawPacketsReceived { get; private set; }

        // TILLFÄLLIG felsökningsegenskap — visar exakt vad som senast
        // mottogs, oavsett om tolkning lyckades. Byggd specifikt för
        // att undersöka varför SteerAngleActualDeg verkar stå still i
        // UI:t trots att andra fält uppdateras — ger oss direkt insyn
        // i rådata istället för att fortsätta gissa. Kan tas bort igen
        // när felet är löst, eller byggas ut till en riktig loggfil.
        public string LastRawSentence { get; private set; }

        private readonly UdpClient _receiveClient;
        private readonly UdpClient _sendClient;
        private readonly int _sendPort;

        // CancellationTokenSource — C#:s inbyggda "be en pågående
        // asynkron uppgift att avsluta snällt". Vi skapar en här vid
        // Start(), och anropar _cts.Cancel() vid Stop()/Dispose() för
        // att signalera till mottagningsloopen att den ska sluta.
        // Loopen kollar själv, i sin egen takt, om avbrott begärts —
        // ingen tvingar fram ett abrupt stopp mitt i något.
        private CancellationTokenSource _cts;
        private Task _receiveTask;

        public TeensyConnection(int listenPort, int sendPort)
        {
            _sendPort = sendPort;

            // "new UdpClient(listenPort)" binder direkt till porten,
            // motsvarande vår Pythons socket.bind(('', LISTEN_PORT)) —
            // fast UdpClient gömmer undan flera lägre nivå-detaljer
            // (skapa socket, sätta SO_REUSEADDR, etc.) som vi
            // hanterade för hand i Python.
            //
            // Omslutet i try/catch HÄR specifikt (inte bara nere i
            // mottagningsloopen) — det HÄR är exakt platsen där ett
            // fel skulle kastas om porten redan är upptagen av något
            // annat program (t.ex. om Python-versionen av verktyget
            // fortfarande kör samtidigt) — och innan den här fixen
            // skulle ett sådant fel bara krascha hela konstruktionen
            // med ingen förklaring alls synlig i ett vanligt kört
            // Forms-program.
            try
            {
                _receiveClient = new UdpClient(listenPort);
            }
            catch (Exception ex)
            {
                ErrorOccurred?.Invoke(this,
                    $"Could not listen on port {listenPort}: {ex.Message}");
                throw;
            }

            // Sändningsklienten binder INTE till en specifik port (0
            // = "vilken ledig port som helst") — motsvarar att vår
            // Python-kod skapade sock_tx utan att binda den alls,
            // eftersom den bara SKICKAR, aldrig tar emot på en känd
            // port.
            _sendClient = new UdpClient(0);
        }

        public void Start()
        {
            _cts = new CancellationTokenSource();

            // Task.Run(async lambda) — INTE Task.Factory.StartNew,
            // trots att AgDiags egen UdpCommunication.cs råkar
            // använda den äldre formen (och gav oss idén från
            // början). Skillnaden: Task.Factory.StartNew med en
            // "async"-lambda ger dig en Task<Task> ("en task inuti en
            // task") — ett känt, dokumenterat C#-fallgropsmönster,
            // eftersom den YTTRE tasken anses "klar" så fort lambdan
            // returnerar den INRE tasken (praktiskt taget omedelbart,
            // vid metodens första "await"-punkt), inte när hela
            // mottagningsloopen faktiskt avslutas. Task.Run "packar
            // upp" (unwrappar) det här automatiskt korrekt åt oss,
            // vilket är exakt vad vi vill ha. Den saknar visserligen
            // TaskCreationOptions.LongRunning-optimeringen — men den
            // är mindre viktig här än den skulle vara för en RIKTIGT
            // blockerande, synkron loop, eftersom vår loop (tack vare
            // "await") faktiskt lämnar ifrån sig tråden medan den
            // väntar på nästa paket, istället för att uppta den hela
            // tiden.
            _receiveTask = Task.Run(() => ReceiveLoopAsync(_cts.Token));
        }

        /// <summary>
        /// Själva mottagningsloopen. "async Task" (inte bara "void")
        /// är C#:s sätt att markera "den här metoden gör asynkront
        /// arbete och kan 'pausas' vid await-punkter utan att blockera
        /// tråden den kör på". "await" är ungefär "vänta här på det
        /// här resultatet, men lämna tillbaka kontrollen medan vi
        /// väntar, istället för att bara sitta och blockera" — inte
        /// helt olikt varför Pythons egen trådmodell lät _rx_thread()
        /// köra samtidigt som huvudtråden, fast uttryckt helt
        /// annorlunda under huven.
        /// </summary>
        private async Task ReceiveLoopAsync(CancellationToken token)
        {
            // Motsvarar Pythons "while True:" — men här kollar loopen
            // uttryckligen "har någon bett mig sluta?" varje varv,
            // istället för att bara köra för evigt tills processen
            // dödas utifrån.
            while (!token.IsCancellationRequested)
            {
                try
                {
                    // ReceiveAsync väntar (utan att blockera tråden)
                    // tills ETT UDP-paket anländer — motsvarar
                    // sock_rx.recvfrom(512) i Python, fast utan att vi
                    // själva behöver ange en buffertstorlek; UdpClient
                    // hanterar det åt oss.
                    UdpReceiveResult receiveResult = await _receiveClient.ReceiveAsync();

                    // Ökas HÄR, direkt vid mottagning, INNAN
                    // tolkningsförsöket — det är exakt den distinktion
                    // som ger oss diagnostisk upplösning: "något kom
                    // in på nätverksnivå" är en annan fråga än "det
                    // gick att tolka".
                    RawPacketsReceived++;

                    string senderIp = receiveResult.RemoteEndPoint.Address.ToString();
                    string sentence = Encoding.ASCII.GetString(receiveResult.Buffer).Trim();
                    LastRawSentence = sentence;

                    // Den faktiska tolknings-/uppdateringslogiken är
                    // medvetet utbruten till en EGEN, liten metod
                    // (ProcessIncomingSentence nedan) — just för att
                    // den ska gå att testa isolerat, utan en riktig
                    // socket, precis som vi ville.
                    ProcessIncomingSentence(sentence, senderIp);
                }
                catch (ObjectDisposedException)
                {
                    // Kastas av UdpClient när sockeln stängs medan
                    // ReceiveAsync fortfarande väntar — det HÄNDER
                    // normalt vid avstängning (Stop()/Dispose()), inte
                    // ett verkligt fel. Motsvarar varför vår Python-
                    // kod fångade socket.timeout tyst, utan att
                    // logga det som ett problem.
                    break;
                }
                catch (Exception)
                {
                    // Motsvarar Pythons brett "except Exception: pass"
                    // i _rx_thread() — ETT trasigt/oväntat paket ska
                    // aldrig döda hela mottagningsloopen. Samma
                    // motståndskraftsprincip vi redan byggt in i
                    // PDiagParser, fast på nätverksnivå istället för
                    // fältnivå.
                }
            }
        }

        /// <summary>
        /// Den testbara kärnan — tar emot rå text + avsändarens IP,
        /// försöker tolka den, och utlöser händelsen om det lyckas.
        /// Innehåller INGEN nätverkskod alls, vilket är precis vad som
        /// gör att vi kan testa den utan en riktig socket.
        /// </summary>
        internal void ProcessIncomingSentence(string sentence, string senderIp)
        {
            if (!PDiagParser.TryParse(sentence, out PDiagData data))
                return;

            LastReceivedUtc = DateTime.UtcNow;
            TeensyIpAddress = senderIp;

            // "DataReceived?.Invoke(this, data)" — frågetecknet här är
            // C#:s säkra sätt att fråga "finns det NÅGON som
            // prenumererar på den här händelsen just nu?" innan man
            // faktiskt utlöser den. Om ingen lyssnar (t.ex. i ett rent
            // enhetstest utan något UI kopplat) skulle ett vanligt
            // anrop annars krascha.
            DataReceived?.Invoke(this, data);
        }

        public void SendCommand(string command)
        {
            byte[] bytes = Encoding.ASCII.GetBytes(command);

            // Skickar till samma adress vi senast HÖRDE FRÅN Teensyn
            // (TeensyIpAddress) — motsvarar precis samma logik som
            // Pythons _send_cmd(), som skickade till self.teensy_ip.
            if (TeensyIpAddress != null)
            {
                _sendClient.Send(bytes, bytes.Length, TeensyIpAddress, _sendPort);
            }
        }

        /// <summary>
        /// Cancels the background receive loop (ReceiveLoopAsync,
        /// further up this file) and closes the UDP socket it's
        /// reading from — the two together are what actually make the
        /// loop's next ReceiveAsync() call throw/return rather than
        /// block forever. Called from Dispose() below, and directly
        /// from MainForm's own FormClosing handler, so the background
        /// task doesn't keep running after the window is gone.
        /// </summary>
        public void Stop()
        {
            _cts?.Cancel();
            _receiveClient.Close();
        }

        // Dispose() är den metod IDisposable-kontraktet kräver att vi
        // implementerar — "här är hur du städar upp efter mig".
        // Anropas antingen manuellt, eller automatiskt om klassen
        // används inuti ett C# "using"-block (ett annat, separat
        // begrepp vi inte behöver gå in på än).
        public void Dispose()
        {
            Stop();
            _receiveClient.Dispose();
            _sendClient.Dispose();
            _cts?.Dispose();
        }
    }
}
