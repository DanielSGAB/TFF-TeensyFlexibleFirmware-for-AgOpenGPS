// =====================================================================
// zGnssPassthrough.ino — raw byte forwarding, GnssPassthrough mode
// =====================================================================
// Ported from Chris Kinal's original UM982 firmware
// (AOG_Teensy_UM982.ino, the "udpPassthrough" branch in loop()) —
// confirmed against that source directly, not reconstructed from
// memory. Logic kept as close to the original as reasonably possible;
// the only real changes are renaming the buffer/state variables to
// avoid a genuine collision with this project's own msgBuf/msgBufLen/
// gotDollar (used by GGAH_Handler_Raw() in zzGNSS_UM982.ino for an
// unrelated purpose — the two would never actually run at the same
// time, since GnssPassthrough replaces the normal GNSS dispatch
// entirely rather than running alongside it, but sharing a name for
// two unrelated purposes anywhere in this codebase risks confusing a
// future reader even when it's not a real runtime conflict), and
// reading from SerialGPS (which already respects Board Configuration —
// see resolveBoardSlots() in the main .ino) rather than a hardcoded
// port the way the original did.
//
// Does NOT parse or recognise any particular sentence type — see the
// GnssPassthrough declaration comment in the main .ino for why that's
// deliberate. This file has no idea what $KSXT is; it just moves
// complete "$...\r\n" lines from SerialGPS to AGO, unexamined.
// =====================================================================

bool ptGotDollar = false;
bool ptGotCR     = false;
bool ptGotLF     = false;
char ptMsgBuf[254];
int  ptMsgBufLen = 0;

/// <summary>
/// Same position-role port as every other GNSS mode, respecting
/// whatever Board Configuration resolved it to — see
/// resolveBoardSlots() in the main .ino. No parser attached (unlike
/// every other GNSS source): passthrough mode never touches
/// NMEAParser at all, on purpose — the whole point of this mode is
/// to forward the receiver's raw bytes unmodified, bypassing every
/// bit of TFF's own heading/roll/quality processing entirely.
/// </summary>
void gnssPassthrough_setup()
{
    SerialGPS->begin(baudGPS);
    SerialGPS->addMemoryForRead (GPSrxbuffer, serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);
}

// Called every loop() iteration when GnssPassthrough is true — see the
// dispatch in loop() in the main .ino, which calls this INSTEAD OF the
// normal GNSS source update()/BuildNmea() pipeline, not alongside it.
void gnssPassthrough_update()
{
    if (SerialGPS->available())
    {
        char incoming = SerialGPS->read();
        switch (incoming)
        {
            case '$':
                ptMsgBuf[ptMsgBufLen] = incoming;
                ptMsgBufLen++;
                ptGotDollar = true;
                break;
            case '\r':
                ptMsgBuf[ptMsgBufLen] = incoming;
                ptMsgBufLen++;
                ptGotCR = true;
                ptGotDollar = false;
                break;
            case '\n':
                ptMsgBuf[ptMsgBufLen] = incoming;
                ptMsgBufLen++;
                ptGotLF = true;
                ptGotDollar = false;
                break;
            default:
                if (ptGotDollar && ptMsgBufLen < (int)sizeof(ptMsgBuf) - 1)
                {
                    ptMsgBuf[ptMsgBufLen] = incoming;
                    ptMsgBufLen++;
                }
                break;
        }

        if (ptGotCR && ptGotLF)
        {
            if (sendUSB) SerialAOG.write(ptMsgBuf, ptMsgBufLen);

            if (Ethernet_running)
            {
                Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
                Eth_udpPAOGI.write(ptMsgBuf, ptMsgBufLen);
                Eth_udpPAOGI.endPacket();
            }

            ptGotCR     = false;
            ptGotLF     = false;
            ptGotDollar = false;
            memset(ptMsgBuf, 0, sizeof(ptMsgBuf));
            ptMsgBufLen = 0;

            digitalWrite(GGAReceivedLED, blink ? HIGH : LOW);
            blink = !blink;
            digitalWrite(GPSGREEN_LED, HIGH);   // same "data is flowing" meaning as every other source's green LED
        }
    }
}
