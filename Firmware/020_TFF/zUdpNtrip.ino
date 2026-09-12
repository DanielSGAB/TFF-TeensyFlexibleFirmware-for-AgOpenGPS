/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

/// <summary>
/// Forwards RTK correction data — AGO's own NTRIP client fetches it
/// from a caster and relays it over UDP to this module, which just
/// passes the raw bytes straight through to the GNSS receiver's own
/// serial port unmodified. TFF itself never parses or understands
/// NTRIP data at all, purely a pass-through. Called every loop()
/// iteration; the Ethernet_running check up front avoids calling
/// parsePacket() on a socket that was never successfully opened
/// (EthernetStart() failed or hasn't run yet), which would otherwise
/// block.
/// </summary>
void udpNtrip()
{
#ifdef ARDUINO_TEENSY41
  // When ethernet is not running, return directly. parsePacket() will block when we don't
  if (!Ethernet_running)
  {
    return;
  }

  unsigned int packetLength = Eth_udpNtrip.parsePacket();
  
  if (packetLength > 0)
  {
    Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLength);
    SerialGPS->write(Eth_NTRIP_packetBuffer, packetLength);
  }
#endif
}
