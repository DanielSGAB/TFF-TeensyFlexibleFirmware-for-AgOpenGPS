// =====================================================================
// zzGNSS_DualF9P.ino — GNSS source: 2x u-blox F9P, moving-base heading
// =====================================================================
// Part of TFF (Teensy Flexible Firmware). Selected when
// GnssMode == MODE_DUAL and DualReceiverType == RX_F9P (Receiver
// Configuration → "Dual single antenna receiver" → "2 x u-blox F9P").
//
// STATUS: heading and roll are now confirmed against the user's own
// working firmware (zRelPos.ino / relPosDecode()) — this is a direct
// port of confirmed logic, not an inference from protocol docs anymore.
// Byte offsets, the heading mounting correction, and the roll formula
// (including its sign) all now match that source exactly. Still not
// run on this specific hardware/board combination by us — the source
// logic is confirmed, the end-to-end build/flash/field test is not.
//
// Hardware pattern (confirmed against AgOpenGPS's own official dual-F9P
// documentation): RIGHT/master F9P = "Position" role — sends GGA+VTG on
// SerialGPS, and streams RTCM correction directly to the LEFT F9P over
// a separate module-to-module UART link (NOT through the Teensy).
// LEFT/slave F9P = "Heading" role — computes UBX-NAV-RELPOSNED (moving
// base relative to the position unit) and streams it on
// SerialGPSHeading. See TFF architecture reference §5 for the full
// derivation of why moving-base RTK was chosen over independent
// GGA-vector heading.
//
// STILL OPEN:
//   HEADING SATELLITE COUNT: RELPOSNED doesn't carry a satellite count
//      for the heading solution the way UM982's HPR words[5] or
//      UM980's UNIHEADING2 #solnSVs do — the confirmed source doesn't
//      use one either (it only uses carrSoln, not a satellite count,
//      for its quality decision). A real count would need UBX-NAV-PVT
//      parsed alongside this message — not implemented here. hprSats
//      is held at -1 ("not applicable"), which safely disables
//      updateAlphaControl()'s satellite-based degradation term for
//      this source (see the `if (hprSats > 0)` guard in zHandlers.ino)
//      without corrupting it — the quality-based term (from carrSoln)
//      still works normally on its own.
// =====================================================================

// UBX-NAV-RELPOSNED (message class 0x01, ID 0x3C, version 1) is fixed
// at 64 payload bytes. Frame: B5 62 | class | id | len_lo len_hi |
// payload[len] | CK_A | CK_B (8-bit Fletcher checksum over class..payload).
#define UBX_RELPOSNED_CLASS 0x01
#define UBX_RELPOSNED_ID    0x3C
#define UBX_RELPOSNED_LEN   64

// Antenna mounting correction, confirmed from the user's own
// zRelPos.ino: antennas are mounted transversely (same convention as
// UM982's "CONFIG HEADING OFFSET 90.0"), so raw relPosHeading points
// along the antenna baseline, not the direction of travel — this
// constant corrects for that. Units: decidegrees (0.1°), so 900 = 90°.
// This was completely missing from the earlier version of this file.
static const double headingcorr = 900.0;

enum UbxState { UBX_SYNC1, UBX_SYNC2, UBX_CLASS, UBX_ID, UBX_LEN1, UBX_LEN2,
                UBX_PAYLOAD, UBX_CKA, UBX_CKB };
static UbxState ubxState = UBX_SYNC1;
static uint8_t  ubxClass, ubxId;
static uint16_t ubxLen, ubxPayloadIdx;
static uint8_t  ubxPayload[UBX_RELPOSNED_LEN];
static uint8_t  ubxCkA, ubxCkB;        // received checksum
static uint8_t  ubxCkACalc, ubxCkBCalc; // running Fletcher checksum

static void ubxChecksumAdd(uint8_t b)
{
    ubxCkACalc += b;
    ubxCkBCalc += ubxCkACalc;
}

static void ubxResetParser()
{
    ubxState   = UBX_SYNC1;
    ubxCkACalc = 0;
    ubxCkBCalc = 0;
}

// Reads little-endian signed/unsigned ints out of ubxPayload at a byte
// offset — RELPOSNED's field layout is fixed, see offsets used below.
static int32_t ubxI4(int off)
{
    return (int32_t)((uint32_t)ubxPayload[off] | ((uint32_t)ubxPayload[off+1] << 8) |
                      ((uint32_t)ubxPayload[off+2] << 16) | ((uint32_t)ubxPayload[off+3] << 24));
}
static uint32_t ubxU4(int off)
{
    return (uint32_t)ubxPayload[off] | ((uint32_t)ubxPayload[off+1] << 8) |
           ((uint32_t)ubxPayload[off+2] << 16) | ((uint32_t)ubxPayload[off+3] << 24);
}

// Called once a complete, checksum-valid RELPOSNED payload has been
// received. Extracts heading + solution quality and writes them into
// the same shared variables HPR_Handler uses (heading, solQuality,
// hprSats, dualReadyRelPos) — see zzGNSS_UM982.ino's file header for why
// every GNSS source shares this contract instead of a new struct.
static void relPosNedDecode()
{
    // Field offsets confirmed against the user's own zRelPos.ino
    // (relPosDecode()) — this function now matches that source, not
    // just the public UBX protocol spec. Differences from the original
    // are called out explicitly below; everything else is a direct
    // port.

    // relPosHeading: offset 24, I4, raw units 1e-5 degrees.
    // headingcorr applies the antenna mounting correction — confirmed
    // from the original: antennas are mounted transversely (same
    // convention as UM982's own "CONFIG HEADING OFFSET 90.0"), so a
    // 90° correction is needed to point heading along the direction of
    // travel rather than along the antenna baseline. This was MISSING
    // from our earlier version — a real fix, not just a refinement.
    int32_t rawHeading = ubxI4(24);
    double headingDecideg = (double)rawHeading * 0.0001;  // -> decidegrees (0.1°)
    headingDecideg += headingcorr;                         // += 900 (90°)
    while (headingDecideg >= 3600.0) headingDecideg -= 3600.0;
    while (headingDecideg <    0.0) headingDecideg += 3600.0;
    heading = headingDecideg * 0.1;                        // -> degrees

    // relPosD (offset 16) and relPosLength/"baseline" (offset 20): I4,
    // both in cm, refined with their high-precision extension bytes
    // (relPosHPD at offset 34, relPosHPLength at offset 35 — signed,
    // 0.01 cm units). The earlier version used the plain cm integers
    // only; this adds the same HP refinement the confirmed source uses.
    int32_t relPosD_i      = ubxI4(16);
    int32_t relPosLength_i = ubxI4(20);
    double  relPosHPD      = (double)(int8_t)ubxPayload[34] * 0.01;
    double  relPosHPLength = (double)(int8_t)ubxPayload[35] * 0.01;
    double  relPosD_cm     = (double)relPosD_i      + relPosHPD;
    double  baseline_cm    = (double)relPosLength_i + relPosHPLength;

    // flags: offset 60, U4 bitfield
    uint32_t flags = ubxU4(60);
    bool gnssFixOK   = flags & 0x01;
    bool diffSoln    = flags & 0x02;
    bool relPosValid = flags & 0x04;
    uint8_t carrSoln = (flags >> 3) & 0x03;   // 0=none 1=float 2=fixed

    // Map onto the same 0/4/5 scale HPR's solQuality already uses (see
    // zzGNSS_DualUM980.ino for the same pattern applied to UM980's pos
    // type). This graduated signal is a deliberate TFF design choice,
    // different from the confirmed original's simpler binary logic
    // (which only ever fully trusted carrSoln==2/fixed and otherwise
    // just decayed the previous roll estimate) — our shared
    // updateAlphaControl() needs a graduated quality value to work at
    // all, so this divergence is kept. The roll formula itself and the
    // float-quality decay behaviour below ARE taken directly from the
    // confirmed original, though.
    if (!gnssFixOK || !diffSoln || !relPosValid)
    {
        solQuality = 0;
    }
    else if (carrSoln == 2 && baseline_cm > 1.0)
    {
        solQuality = 4;   // fixed
        // baseline_cm > 1.0 above is the confirmed original's own
        // guard (matches "if (carrSoln == 2 && baseline > 1)" in the
        // reference source) — missed when this formula was first
        // ported (v0.1 only had a plain == 0.0 divide-by-zero guard,
        // added in v0.2). A baseline near-but-not-exactly zero (e.g. a
        // transient/unstable reading right at the moment RTK first
        // locks to fixed) would previously divide by a tiny, non-zero
        // denominator and produce an absurd, ratio-clamped-to-±1 roll
        // value that looked like a plausible reading rather than being
        // rejected outright — the >1cm floor catches that case
        // upstream of the division ever happening, instead of trying
        // to detect a bad RESULT after the fact. The old == 0.0 guard
        // is gone — no longer reachable now that the whole block
        // requires baseline_cm > 1.0 before computing ratio at all.
        double ratio = constrain(relPosD_cm / baseline_cm, -1.0, 1.0);  // guard asin() domain
        // Confirmed formula, including the negative sign — the earlier
        // version was missing it, which would have given roll the
        // wrong direction.
        rollDual = -asin(ratio) * (180.0 / M_PI);
    }
    else
    {
        // Reached either for genuine float/none, OR for carrSoln == 2
        // with an untrustworthy (<=1cm) baseline — treated the same
        // way as float/none rather than as a trustworthy fixed
        // solution, since the baseline itself can't be trusted yet
        // regardless of what the carrier-solution flag claims.
        solQuality = (carrSoln == 1) ? 5 : 0;   // float : none
        // Confirmed behaviour for float/none: decay the previous roll
        // estimate toward zero rather than compute a fresh value from
        // a less-trusted relPosD — the original does not recompute
        // roll at all in this branch.
        rollDual *= 0.9;
    }

    hprSats = -1;   // not available from this message — see file header

    // GREEN LED: ON = dual solution good, flashing = present but below
    // quality threshold. RED LED: LOW/off while heading data is
    // actively arriving. Same pattern as zzGNSS_UM982.ino's HPR_Handler,
    // for a consistent feel across sources.
    if (solQuality >= HPR_QUALITY_MIN)
        digitalWrite(GPSGREEN_LED, HIGH);
    else
        digitalWrite(GPSGREEN_LED, blink ? HIGH : LOW);
    digitalWrite(GPSRED_LED, LOW);

    // ImuType == IMU_NONE always enters this block — see the identical
    // note in zzGNSS_UM982.ino for the full rationale.
    if (solQuality >= HPR_QUALITY_MIN || ImuType == IMU_NONE)
    {
        // Kalman filter — ported from zzGNSS_UM982.ino's HPR_Handler(),
        // same shared headingFilter/rollFilter instances and same
        // filterHeading/filterRoll toggles. Generalised here per an
        // explicit decision to make filtering a source-independent,
        // user-selectable option rather than a UM982-only behaviour —
        // it was originally only wired into the UM982 path. Applied at
        // the same point in the pipeline (post-computation, pre-
        // updateHeadingOffset()) for a consistent feel across sources.
        // Kalman filter — routed through applyHeadingKalman()
        // (zHandlers.ino), not headingFilter directly, to avoid a
        // 0°/360° wraparound bug — see that function's header comment.
        // rollDual needs no such wrapper (roll never wraps).
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
// picture (which F9P plays which role, what STILL OPEN issues exist).
// This function specifically: opens both UARTs (position role on
// SerialGPS, heading role on SerialGPSHeading) — guarded by
// headingReceiverConfigured (see resolveBoardSlots(), main .ino),
// since a Board Configuration with no slot set to Slave would
// otherwise leave SerialGPSHeading pointing at the SAME physical UART
// SerialGPS just claimed — a real, previously field-hit hang (v0.3.8).
void dualF9P_setup()
{
    // Position-role receiver — same port every mode uses.
    SerialGPS->begin(baudGPS);
    SerialGPS->addMemoryForRead (GPSrxbuffer, serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

    // Heading-role receiver — see SerialGPSHeading comment in the main
    // .ino for the caveat about which physical UART your board's slave
    // header actually uses.
    //
    // Guarded by headingReceiverConfigured (set in resolveBoardSlots(),
    // main .ino) — found necessary after a real hang: without this
    // check, a Board Configuration with no slot set to Slave leaves
    // SerialGPSHeading pointing at the SAME physical UART SerialGPS
    // just initialised above, and calling .begin()/.addMemoryForRead()
    // a second time on that same object is a genuine low-level UART
    // driver collision — not a controlled error, an actual hang.
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

    ubxResetParser();
}

// Called every loop() iteration when GnssMode==MODE_DUAL and
// DualReceiverType==RX_F9P. Two independent streams handled here:
// standard NMEA GGA/VTG on SerialGPS (through the shared NMEAParser,
// same as every other mode), and raw UBX-NAV-RELPOSNED binary on
// SerialGPSHeading (hand-rolled state machine below — NOT NMEAParser,
// since RELPOSNED is a binary UBX message, not NMEA text at all).
void dualF9P_update()
{
    // Position stream: standard GGA/VTG, shared handlers, identical to
    // every other mode.
    while (SerialGPS->available())
        parser << SerialGPS->read();

    // Heading stream: raw UBX binary, hand-rolled state machine (NOT
    // NMEAParser — RELPOSNED is not NMEA text).
    //
    // Guarded by headingReceiverConfigured, same reason as in
    // dualF9P_setup() above — if it's false, SerialGPSHeading was
    // never actually .begin()'d as a separate receiver at all (it may
    // still share SerialGPS's own UART), so reading from it here
    // would just be reading GGA/VTG bytes intended for the position
    // parser through the wrong code path, not a genuine heading
    // stream.
    if (headingReceiverConfigured)
    {
    while (SerialGPSHeading->available())
    {
        uint8_t b = SerialGPSHeading->read();

        switch (ubxState)
        {
            case UBX_SYNC1:
                if (b == 0xB5) ubxState = UBX_SYNC2;
                break;
            case UBX_SYNC2:
                ubxState = (b == 0x62) ? UBX_CLASS : UBX_SYNC1;
                break;
            case UBX_CLASS:
                ubxClass = b;
                ubxCkACalc = ubxCkBCalc = 0;
                ubxChecksumAdd(b);
                ubxState = UBX_ID;
                break;
            case UBX_ID:
                ubxId = b;
                ubxChecksumAdd(b);
                ubxState = UBX_LEN1;
                break;
            case UBX_LEN1:
                ubxLen = b;
                ubxChecksumAdd(b);
                ubxState = UBX_LEN2;
                break;
            case UBX_LEN2:
                ubxLen |= ((uint16_t)b << 8);
                ubxChecksumAdd(b);
                ubxPayloadIdx = 0;
                // Only RELPOSNED (class/id/len all match) is handled —
                // anything else is skipped byte-by-byte back to sync
                // rather than buffered, since ubxPayload is sized for
                // RELPOSNED only.
                if (ubxClass == UBX_RELPOSNED_CLASS && ubxId == UBX_RELPOSNED_ID
                    && ubxLen == UBX_RELPOSNED_LEN)
                    ubxState = UBX_PAYLOAD;
                else if (ubxLen == 0)
                    ubxState = UBX_CKA;
                else
                    ubxResetParser();  // unknown/unsized message — resync
                break;
            case UBX_PAYLOAD:
                ubxPayload[ubxPayloadIdx++] = b;
                ubxChecksumAdd(b);
                if (ubxPayloadIdx >= ubxLen)
                    ubxState = UBX_CKA;
                break;
            case UBX_CKA:
                ubxCkA = b;
                ubxState = UBX_CKB;
                break;
            case UBX_CKB:
                ubxCkB = b;
                if (ubxCkA == ubxCkACalc && ubxCkB == ubxCkBCalc)
                    relPosNedDecode();
                // else: checksum failed, silently drop — next byte
                // starts a fresh sync search.
                ubxResetParser();
                break;
        }
    }
    }

    // ROLL_ALPHA behaves normally here — no override needed. The roll
    // formula in relPosNedDecode() (including its sign) is now a
    // confirmed port of the user's own working zRelPos.ino, not an
    // inference, so there's no expected sign ambiguity to guard
    // against the way there was in the earlier version of this file.
    //
    // If field testing somehow still shows the sign is wrong: unlike
    // BNO roll, rollInvert (Teensy Tool → IMU tab) does NOT apply here
    // — per imuHandler()'s existing documented convention, dual roll
    // (HPR/RELPOSNED/UNIHEADING2) is always the trusted reference and
    // is never adjusted by rollInvert or rollZeroOffset; only BNO roll
    // is calibrated to match it, not the other way round. A sign flip
    // would mean negating the result in relPosNedDecode() above — a
    // code change, not a runtime calibration setting.
}
