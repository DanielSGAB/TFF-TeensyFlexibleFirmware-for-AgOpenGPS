// =====================================================================
// zzGNSS_DualUM980.ino — GNSS source: 2x UnicoreComm UM980, MODE HEADING2
// =====================================================================
// Part of TFF (Teensy Flexible Firmware). Selected when
// GnssMode == MODE_DUAL and DualReceiverType == RX_UM980 (Receiver
// Configuration → "Dual single antenna receiver" → "2 x UnicoreComm
// UM980").
//
// STATUS: field layout confirmed directly against Unicore's official
// N4 Products Commands and Logs Reference Manual (Table 7-119,
// UNIHEADING2 Message Structure, section 7.3.49) — this is NOT a guess.
// However it has NEVER been run against real UM980 hardware. Treat as
// "should work" rather than "known working" until verified in the
// field — unlike zzGNSS_UM982.ino, which is proven UM982 Fallback v0.7 code.
//
// Hardware pattern: same master/rover roles as the F9P pair (see
// zzGNSS_DualF9P.ino) — position-role UM980 on SerialGPS sends GGA+VTG
// and streams RTCM directly to the heading-role UM980 module-to-module
// (not through the Teensy); heading-role UM980 (MODE HEADING2,
// configured with "UNIHEADING2A ONCHANGED") streams UNIHEADING2 on
// SerialGPSHeading.
//
// Why UNIHEADING2 and not $GPTHS2: $GPTHS2 (the NMEA-text heading
// sentence UM980 also supports in this mode) was checked first and
// found to carry only a heading value and a FIXED mode character "T" —
// no quality/satellite information at all, so it can't drive
// updateAlphaControl(). UNIHEADING2 (Unicore's own ASCII log, not
// standard NMEA) carries everything needed: pos type (quality),
// #solnSVs (satellite count), heading, AND pitch — so $GPTHS2 is not
// parsed here at all; it would be redundant.
//
// UNIHEADING2 field layout (confirmed, ASCII form), after the leading
// standard Unicore header fields and the ';' separator:
//   0  sol_stat        (e.g. SOL_COMPUTED — not currently checked)
//   1  pos_type        (Table 0-4 enum, see mapping below)
//   2  length          (baseline length — unit not explicitly stated
//                        in the manual; assumed metres per standard
//                        Unicore convention, NOT yet verified — treat
//                        rawBaselineLength below with suspicion until
//                        confirmed against real hardware output)
//   3  heading          (0-360°)
//   4  pitch            (±90°) — used as our "roll" equivalent, see below
//   5  Reserved
//   6  hdgstddev        (standard deviation of heading — not currently
//                        used, but available for a future refinement;
//                        UM982's HPR has no equivalent at all)
//   7  ptchstddev
//   8  Master stn ID    (quoted string, e.g. "201" — ignored here)
//   9  #SVs             (satellites tracked)
//   10 #solnSVs         (satellites USED in solution — this is what
//                        maps to hprSats, same role as HPR words[5])
//   11 #obs
//   12 #multi
//   13 Reserved
//   14 ext sol stat
//   15 Galileo&BDS3 sig mask
//   16 GPS/GLONASS/BDS-2 sig mask
// =====================================================================

// pos_type enum values (Table 0-4 in the manual) actually checked here.
// Only the narrow-lane fixed solution is treated as "fixed" quality —
// L1_INT (48) and WIDE_INT (49) are intermediate fixed-type solutions
// that exist in the enum but are deliberately NOT mapped to "fixed"
// here, a conservative simplification: only the best-quality solution
// counts as equivalent to HPR's solQuality=4 threshold.
#define UM980_POSTYPE_NARROW_FLOAT 34
#define UM980_POSTYPE_NARROW_INT   50

static char um980HeadingLineBuf[220];
static int  um980HeadingLineLen = 0;

// Parses one complete UNIHEADING2A ASCII line (already collected by
// dualUM980_update() below, '\0'-terminated). Splits on ';' to skip the
// standard Unicore preamble, then walks comma-separated fields by the
// confirmed index table above.
static void uniHeading2Decode(char* line)
{
    char* semi = strchr(line, ';');
    if (semi == nullptr) return;   // malformed / not a data line
    char* p = semi + 1;

    char* fields[17] = { nullptr };
    int   fieldCount = 0;
    char* tok = strtok(p, ",*");
    while (tok != nullptr && fieldCount < 17)
    {
        fields[fieldCount++] = tok;
        tok = strtok(nullptr, ",*");
    }
    if (fieldCount < 11) return;   // not enough fields to be useful

    int   posType  = atoi(fields[1]);
    float hdg      = atof(fields[3]);
    float pitchVal = atof(fields[4]);
    int   solnSVs  = atoi(fields[10]);

    heading  = (double)hdg;
    rollDual = (double)pitchVal;   // see file header — UM980 pair has no
                                    // true "roll" field; pitch is used
                                    // as the closest available analogue.
                                    // NOT yet verified this is the right
                                    // physical quantity for AOG's roll —
                                    // flagged for field verification.

    if (posType == UM980_POSTYPE_NARROW_INT)
        solQuality = 4;   // fixed — matches HPR_QUALITY_MIN threshold
    else if (posType == UM980_POSTYPE_NARROW_FLOAT)
        solQuality = 5;   // float
    else
        solQuality = 0;

    hprSats = solnSVs;   // #solnSVs — same role as HPR words[5]

    // GREEN/RED LED pattern — see zzGNSS_DualF9P.ino for the rationale;
    // identical logic here for a consistent feel across sources.
    if (solQuality >= HPR_QUALITY_MIN)
        digitalWrite(GPSGREEN_LED, HIGH);
    else
        digitalWrite(GPSGREEN_LED, blink ? HIGH : LOW);
    digitalWrite(GPSRED_LED, LOW);

    // ImuType == IMU_NONE always enters this block — see the identical
    // note in zzGNSS_UM982.ino for the full rationale.
    if (solQuality >= HPR_QUALITY_MIN || ImuType == IMU_NONE)
    {
        // Kalman filter — see zzGNSS_DualF9P.ino for the full rationale;
        // identical pattern here, same shared headingFilter/rollFilter
        // instances and filterHeading/filterRoll toggles.
        // Kalman filter — routed through applyHeadingKalman()
        // (zHandlers.ino) — see zzGNSS_DualF9P.ino for the full
        // wraparound-safety rationale; identical here.
        if (filterHeading)
            heading  = applyHeadingKalman(heading);
        if (filterRoll)
            rollDual = rollFilter->updateEstimate(rollDual);

        if (useIMU)
            updateHeadingOffset(heading);
        dualReadyRelPos = true;
    }
}

// See this file's own header comment above for the full hardware
// picture and field-layout provenance. Setup follows the same
// headingReceiverConfigured guard as dualF9P_setup()
// (zzGNSS_DualF9P.ino) — same v0.3.8 UART-collision fix, same
// Board Configuration dependency, applies identically here.
void dualUM980_setup()
{
    SerialGPS->begin(baudGPS);
    SerialGPS->addMemoryForRead (GPSrxbuffer, serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

    // Guarded by headingReceiverConfigured — see the identical comment
    // in zzGNSS_DualF9P.ino's dualF9P_setup() for the full story (a
    // real field-reported hang, not a theoretical concern).
    if (headingReceiverConfigured)
    {
        SerialGPSHeading->begin(baudGPS);
        SerialGPSHeading->addMemoryForRead (GPSHeadingRxBuffer, serial_buffer_size);
        SerialGPSHeading->addMemoryForWrite(GPSHeadingTxBuffer, serial_buffer_size);
    }
    else
    {
        Serial.println(F("WARNING: Dual mode selected, but no Board Configuration "
                          "slot is set to Slave — heading receiver NOT initialised. "
                          "Set one slot to Slave in Board Configuration and reboot."));
    }

    um980HeadingLineLen = 0;
}

// Called every loop() iteration when GnssMode==MODE_DUAL and
// DualReceiverType==RX_UM980. Two streams: standard NMEA GGA/VTG for
// position, and UM980's own UNIHEADING2A ASCII sentence (starts with
// '#', not NMEA-standard '$', so hand-parsed rather than routed
// through the shared NMEAParser) for heading/roll.
void dualUM980_update()
{
    // Position stream: standard GGA/VTG, shared handlers.
    while (SerialGPS->available())
        parser << SerialGPS->read();

    // Heading stream: UNIHEADING2A is ASCII text, one line per update
    // (ONCHANGED trigger) — accumulate into a line buffer and decode on
    // newline, same simple approach as GGAH_Handler_Raw() in
    // zzGNSS_UM982.ino, rather than routing through NMEAParser (which
    // expects standard NMEA '$'-prefixed sentences; UNIHEADING2 starts
    // with '#' and has its own header format).
    //
    // Guarded by headingReceiverConfigured — same reason as setup()
    // above.
    if (headingReceiverConfigured)
    {
    while (SerialGPSHeading->available())
    {
        char c = SerialGPSHeading->read();
        if (um980HeadingLineLen < (int)sizeof(um980HeadingLineBuf) - 1)
            um980HeadingLineBuf[um980HeadingLineLen++] = c;

        if (c == '\n')
        {
            um980HeadingLineBuf[um980HeadingLineLen] = '\0';
            if (um980HeadingLineBuf[0] == '#')   // Unicore ASCII log prefix
                uniHeading2Decode(um980HeadingLineBuf);
            um980HeadingLineLen = 0;
        }
    }
    }
}
