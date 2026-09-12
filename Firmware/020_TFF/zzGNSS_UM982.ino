// =====================================================================
// zzGNSS_UM982.ino — GNSS source: UM982 (integrated dual-antenna, HPR)
// =====================================================================
// Part of TFF (Teensy Flexible Firmware). Selected when
// GnssMode == MODE_UM982 (Receiver Configuration → "Dual antenna
// receivers" → UnicoreComm UM982).
//
// This is the SAME logic that shipped in UM982 Fallback v0.7 — moved into its own
// file, not rewritten. HPR_Handler() and GGAH_Handler_Raw() are
// verbatim (only relocated); um982_setup()/um982_update() are new thin
// wrappers around what used to be inline code in setup()/loop().
//
// Contract with the shared fusion engine (zHandlers.ino): this file
// writes into the SAME global variables the engine already consumed
// before TFF existed — heading, rollDual, solQuality, hprSats,
// dualReadyGGA, dualReadyRelPos — rather than a new struct. This was a
// deliberate simplification made when TFF was built: imuHandler(),
// updateAlphaControl(), and BuildNmea() needed zero internal changes to
// become receiver-agnostic, because every GNSS source (this file,
// zzGNSS_SingleIMU.ino, zzGNSS_DualF9P.ino, zzGNSS_DualUM980.ino) just
// populates the same variables HPR_Handler always has. Lower risk than
// introducing a new GnssFusionInput struct and rewriting the engine's
// internals to read from it — see TFF architecture reference for the
// full reasoning, and note this is a conscious deviation from that
// document's original §3 struct proposal.
// =====================================================================

void um982_setup()
{
    // UM982 sends everything (GGA, VTG, HPR, GGAH) multiplexed on one
    // serial port — no separate heading-role receiver/port needed,
    // unlike the dual-single-receiver sources.
    SerialGPS->begin(baudGPS);
    SerialGPS->addMemoryForRead (GPSrxbuffer, serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);
}

// Called every loop() iteration when GnssMode == MODE_UM982. Reads
// UM982's serial stream, feeds it to the shared NMEAParser (which
// dispatches to GGA_Handler/VTG_Handler/HPR_Handler automatically —
// those stay registered once in setup(), not here), and separately
// watches the raw byte stream for "GGAH," so GGAH_Handler_Raw() can be
// called directly — NMEAParser's 5-character token limit truncates
// "G-GGAH" to "G-GGA" and would otherwise collide with the real GGA
// handler, so GGAH bypasses the parser entirely.
void um982_update()
{
    while (SerialGPS->available())
    {
        char c = SerialGPS->read();
        parser << c;

        if (c == '$')
        {
            msgBufLen = 0;
            gotDollar = true;
        }
        if (gotDollar && msgBufLen < (int)sizeof(msgBuf) - 1)
        {
            msgBuf[msgBufLen++] = c;
            if (c == '\n')
            {
                msgBuf[msgBufLen] = '\0';
                if (strstr(msgBuf, "GGAH,") != nullptr)
                    GGAH_Handler_Raw(msgBuf, msgBufLen);
                msgBufLen = 0;
                gotDollar = false;
            }
        }
    }
}


// =====================================================================
// HPR_Handler
// =====================================================================
// Parses the $GPHPR dual-antenna heading sentence.
//
// When solQuality >= HPR_QUALITY_MIN (RTK fixed):
//   1. Apply optional Kalman filter (see filterHeading / filterRoll).
//   2. Update headingOffset (BNO-to-HPR EMA, used in fallback).
//   3. Call imuHandler() to select and format heading/roll for PAOGI.
//   4. Set dualReadyRelPos so PAOGI sends on the next GGA.
//
// When solQuality < HPR_QUALITY_MIN:
//   dualReadyRelPos stays false; the BNO fallback path in loop() handles it.
// =====================================================================
void HPR_Handler()
{
    // Set first, unconditionally — this handler only ever runs when
    // the NMEAParser library has already confirmed a complete,
    // checksum-valid $GPHPR sentence (that's the whole point of it
    // being a registered handler, not a raw byte check) — see
    // lastHprMsgTime's own declaration comment in the main .ino for
    // why this is deliberately a stronger guarantee than
    // lastGnssByteTime's "some byte is available" check.
    lastHprMsgTime = millis();

    parser.getArg(1, umHeading);
    heading = atof(umHeading);

    // Roll field selection — see hprRollIsWords2 in User Settings.
    // Official Table 7-42: words[2]=Pitch, words[3]=Roll.
    // Chris Kinal's code reads words[2], suggesting CONFIG HEADING OFFSET 90
    // swaps the axes in HPR output. Test empirically to confirm.
    bool haveRoll = hprRollIsWords2 ? parser.getArg(2, umRoll)
                                    : parser.getArg(3, umRoll);
    if (haveRoll)
    {
        rollDual = atof(umRoll);
        digitalWrite(GPSGREEN_LED, HIGH);      // solid = roll present
    }
    else
    {
        digitalWrite(GPSGREEN_LED, blink ? HIGH : LOW);  // flash = heading only
    }

    parser.getArg(4, solQuality);
    digitalWrite(GPSRED_LED, LOW);  // HPR arriving → red LED off

    // words[5] = satellites used in HPR heading solution
    // Stored separately — do not confuse with per-antenna KSXT satellite counts
    char satBuf[32] = {};  // sized with defensive margin — see the GGA
                            // fields comment in zHandlers.ino for why
                            // (getArg()'s unbounded strcpy())
    if (parser.getArg(5, satBuf))
        hprSats = atoi(satBuf);

    // ImuType == IMU_NONE ("No fallback function") always enters this
    // block and always ends up setting dualReadyRelPos = true below,
    // regardless of solQuality — PAOGI must always be sent from
    // whatever the dual source reports in that mode, matching
    // traditional non-fallback dual-only behaviour, never silently
    // withheld during a quality dip the way the IMU-fallback path
    // would otherwise wait out.
    if (solQuality >= HPR_QUALITY_MIN || ImuType == IMU_NONE)
    {
        // Kalman filter — now routed through applyHeadingKalman()
        // (zHandlers.ino) instead of calling headingFilter directly,
        // to fix a 0°/360° wraparound bug found when this filter was
        // generalised from UM982-only to all three dual-receiver
        // sources — see that function's header comment for the full
        // explanation. rollDual needs no such wrapper (roll never
        // wraps), so it still calls rollFilter directly.
        if (filterHeading)
            heading  = applyHeadingKalman(heading);
        if (filterRoll && haveRoll)
            rollDual = rollFilter->updateEstimate(rollDual);

        if (useIMU)
            updateHeadingOffset(heading);

        dualReadyRelPos = true;   // signal loop() to call imuHandler + BuildNmea
    }
}


// =====================================================================
// GGAH_Handler_Raw
// =====================================================================
// Parses a $GNGGAH / $GPGGAH sentence from a raw character buffer.
// Called from um982_update() when "GGAH," is detected in the serial
// stream, bypassing NMEAParser which cannot distinguish GGAH from GGA
// due to its 5-character token limit ("G-GGAH" truncates to "G-GGA").
//
// Field layout (Table 7-30, same structure as GGA, slave antenna):
//   field 0 = sentence ID ($GNGGAH)
//   field 1 = UTC
//   field 2 = lat (empty when disconnected)
//   field 3 = NS
//   field 4 = lon (empty when disconnected)
//   field 5 = EW
//   field 6 = qual     0=no fix 4=RTKfix 5=RTKfloat
//   field 7 = #sats    slave sats in solution (0/"00" when disconnected)
// =====================================================================
void GGAH_Handler_Raw(const char* buf, int len)
{
    ggahReceived = true;

    int  fieldIdx = 0;
    int  start    = 0;
    char field[32] = {};

    for (int i = 0; i <= len; i++)
    {
        char c = buf[i];
        if (c == ',' || c == '*' || c == '\0' || c == '\r' || c == '\n')
        {
            int flen = min(i - start, (int)sizeof(field) - 1);
            strncpy(field, buf + start, flen);
            field[flen] = '\0';
            start = i + 1;

            if (fieldIdx == 6)
                slaveFixQuality = atoi(field);
            if (fieldIdx == 7)
            {
                satsSlave = atoi(field);
                break;
            }
            fieldIdx++;
        }
    }
}
