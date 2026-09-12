/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

/// <summary>
/// Brings up the Teensy 4.1's built-in Ethernet, using a STATIC IP
/// (from EEPROM's networkAddress — never DHCP), and opens the four
/// separate UDP sockets this whole firmware depends on:
///   - Eth_udpPAOGI: sends $PAOGI/PANDA position sentences out to AGO
///   - Eth_udpNtrip: receives RTK correction data forwarded FROM AGO
///     (see zUdpNtrip.ino)
///   - Eth_udpAutoSteer: the bidirectional PGN 254/253/252 socket —
///     receives steer commands, sends steer state back
///   - Eth_udpMonitor: Teensy Tool's own SETxxx commands in, $PDIAG
///     diagnostics out (see receiveMonitorCommands()/sendDiagnostics()
///     in zHandlers.ino)
/// The last IP octet is set automatically based on Autosteer_running
/// — 126 for a full steering module, 120 for a GPS-only module — a
/// convention AGO's own network scanning relies on to tell the two
/// module types apart.
///
/// Deliberately proceeds even if no Ethernet cable is plugged in
/// (LinkOFF) — only bails out early if the Ethernet HARDWARE itself
/// isn't detected at all (EthernetNoHardware), which would mean a
/// genuinely absent/broken chip, not just an unplugged cable that
/// might get connected moments later.
/// </summary>
void EthernetStart()
{
#ifdef ARDUINO_TEENSY41
  // start the Ethernet connection:
  Serial.println("Initializing ethernet with static IP address");

  // try to congifure using IP:
  Ethernet.begin(mac,0);          // Start Ethernet with IP 0.0.0.0

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    Serial.println("Ethernet shield was not found. GPS via USB only.");

    return;
  }

  if (Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Ethernet cable is not connected - Who cares we will start ethernet anyway.");
  }

//grab the ip from EEPROM
  Eth_myip[0] = networkAddress.ipOne;
  Eth_myip[1] = networkAddress.ipTwo;
  Eth_myip[2] = networkAddress.ipThree;

  if (Autosteer_running) 
  {
    Eth_myip[3] = 126;  //126 is steer module, with or without GPS
  }
  else
  {
    Eth_myip[3] = 120;  //120 is GPS only module
  }

  Ethernet.setLocalIP(Eth_myip);  // Change IP address to IP set by user
  Serial.println("\r\nEthernet status OK");
  Serial.print("IP set Manually: ");
  Serial.println(Ethernet.localIP());

  Ethernet_running = true;

  Eth_ipDestination[0] = Eth_myip[0];
  Eth_ipDestination[1] = Eth_myip[1];
  Eth_ipDestination[2] = Eth_myip[2];
  Eth_ipDestination[3] = 255;

  Serial.print("\r\nEthernet IP of module: "); Serial.println(Ethernet.localIP());
  Serial.print("Ethernet sending to IP: "); Serial.println(Eth_ipDestination);
  Serial.print("All data sending to port: "); Serial.println(portDestination);

  // init UPD Port sending to AOG
  if (Eth_udpPAOGI.begin(portMy))
  {
    Serial.print("Ethernet GPS UDP sending from port: ");
    Serial.println(portMy);
  }

  // init UPD Port getting NTRIP from AOG
  if (Eth_udpNtrip.begin(AOGNtripPort)) // AOGNtripPort
  {
    Serial.print("Ethernet NTRIP UDP listening to port: ");
    Serial.println(AOGNtripPort);
  }

  // init UPD Port getting AutoSteer data from AOG
  if (Eth_udpAutoSteer.begin(AOGAutoSteerPort)) // AOGAutoSteerPortipPort
  {
    Serial.print("Ethernet AutoSteer UDP listening to & send from port: ");
    Serial.println(AOGAutoSteerPort);
  }

  // UDP monitor: receives commands from Teensy Tool (SETHEADINGALPHA,
  // SETROLLALPHA, SETBRAND, etc — see receiveMonitorCommands() in
  // zHandlers.ino for the full list), sends $PDIAG diagnostics back
  if (Eth_udpMonitor.begin(portMonitorIn))
  {
    Serial.print("Ethernet Monitor UDP listening on port: ");
    Serial.println(portMonitorIn);
  }
 #endif
}
