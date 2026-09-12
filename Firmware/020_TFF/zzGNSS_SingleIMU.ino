// =====================================================================
// zzGNSS_SingleIMU.ino — GNSS source: single antenna + IMU (PANDA)
// =====================================================================
// Part of TFF (Teensy Flexible Firmware). Selected when
// GnssMode == MODE_SINGLE_IMU (Receiver Configuration → "Single antenna
// system, single+IMU"). Works with any receiver that outputs standard
// $GxGGA + $GxVTG — no vendor-specific sentence needed, since this mode
// never touches a dual-antenna heading source at all.
//
// STATUS: fully implemented, not yet hardware-tested against real
// firmware/hardware — unlike zzGNSS_UM982.ino (which is UM982 Fallback v0.7's proven
// code, just relocated), this is new code written directly from the
// TFF architecture reference, so treat it as unverified until it's
// actually been run.
//
// Design (confirmed decisions from TFF architecture reference §2):
//   - HEADING_ALPHA and ROLL_ALPHA are pinned to 1.0 — always pure BNO.
//     There is no dual signal in this mode, so no blend ratio applies.
//     updateAlphaControl() (which would otherwise adjust these based on
//     HPR-style quality/satellite signals) must never be called while
//     this mode is active — see the dispatch guard in loop() in the
//     main .ino.
//   - solQuality and hprSats are held at -1 (our "not applicable"
//     convention — see GnssFusionInput discussion in the architecture
//     reference §3): there is no baseline being measured at all in
//     this mode, not just a poor one, and -1 keeps that distinction
//     explicit rather than looking like a bad HPR reading.
//   - Because solQuality (-1) is always < HPR_QUALITY_MIN, GGA_Handler's
//     existing COG-based calibration branch (originally UM982 Fallback v0.7's
//     *extended fallback* mechanism — see updateHeadingOffsetFromCOG()
//     in zHandlers.ino) fires continuously here, exactly as intended:
//     it becomes this mode's PRIMARY heading-offset calibration
//     mechanism, not a rarely-used reserve. No new code was needed for
//     that — it already worked this way by coincidence of the existing
//     threshold check, confirmed when this file was written.
//   - rollZeroOffset calibration still applies (BNO-to-itself, no dual
//     signal needed) — but AGO does its own IMU zeroing for PANDA (its
//     "Zero IMU" workflow), so Teensy Tool grays out rollZeroOffset and
//     Roll Invert controls in this mode (IMU tab) to avoid the two
//     calibration paths fighting each other. Roll axis selection stays
//     enabled — it's a physical mounting fact, not a per-mode setting.
// =====================================================================

void singleImu_setup()
{
    // Same position-role port as every other mode — no second serial
    // port needed here, since there's only ever one receiver in this
    // mode.
    SerialGPS->begin(baudGPS);
    SerialGPS->addMemoryForRead (GPSrxbuffer, serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

    // Fixed, not runtime-adjustable in this mode — see file header.
    HEADING_ALPHA = 1.0f;
    ROLL_ALPHA    = 1.0f;
    solQuality    = -1;
    hprSats       = -1;
}

// Called every loop() iteration when GnssMode == MODE_SINGLE_IMU. Only
// GGA_Handler/VTG_Handler ever fire here (no HPR sentence exists on
// this wire) — both are the same shared handlers UM982 mode uses, in
// zHandlers.ino, completely unchanged. No GGAH detection needed either
// (single receiver, no slave antenna concept at all), so this is
// simpler than um982_update() — just feed bytes to the parser.
void singleImu_update()
{
    while (SerialGPS->available())
        parser << SerialGPS->read();

    // GREEN LED stays off — there is never a dual solution to report in
    // this mode, unlike the other three sources. RED LED follows the
    // official meaning from the AIO firmware header ("ON = GPS fix with
    // IMU, flashing = no IMU or dual"): solid once both GGA data and
    // the BNO are present and healthy, flashing otherwise.
    digitalWrite(GPSGREEN_LED, LOW);
    if (useIMU && GGA_Available)
        digitalWrite(GPSRED_LED, HIGH);
    else
        digitalWrite(GPSRED_LED, blink ? HIGH : LOW);

    // Re-assert every call, cheap and idempotent — guards against any
    // code path that might otherwise nudge these away from this mode's
    // fixed values (belt-and-braces; nothing currently does, but this
    // mode's entire premise depends on these staying fixed).
    HEADING_ALPHA = 1.0f;
    ROLL_ALPHA    = 1.0f;
    solQuality    = -1;
    hprSats       = -1;
}
