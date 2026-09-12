// =====================================================================
// zIMU_TM171.ino — IMU source: SYD Dynamics TransducerM (TM171)
// =====================================================================
// Part of TFF. Selected when ImuType == IMU_TM171 (Teensy Tool → IMU
// tab, radio button at the top of the tab). Alternative to BNO08x
// (zHandlers.ino's readBNO()) — writes into the SAME shared variables
// (yaw, roll, pitch, correctionHeading, lastImuDataTime, imuHealthy)
// that readBNO() already writes, so the entire fusion engine downstream
// (imuHandler(), updateAlphaControl(), the alpha-blend, COG-based
// heading offset calibration) needs ZERO changes to work with either
// IMU — confirmed by mapping all 15 existing useIMU call sites
// across the codebase before writing this file: only three of them
// (boot-time detection, the per-loop poll dispatch, and
// checkImuWatchdog()'s recovery action) needed to become IMU-type-
// aware; everything else already only asks "is an IMU active", not
// "which one".
//
// PROTOCOL STATUS: confirmed against real, working, field-tested
// source code — the "TM171.ino" tab from the AgOpenGPS community's
// own Teensy 4.1 dual-IMU-WAS project (MarekJarczewwski, itself
// crediting "Paulius a.k.a. babtai RTK" as the original author) — not
// a guess, not reverse-engineered from a datasheet. Frame format,
// function codes, and the exact CRC16 table are a direct, unmodified
// port. See TFF architecture reference for the full research trail
// (SYD Dynamics' own protocol manual was found to exist but was not
// directly accessible; this confirmed community implementation was
// used instead once located).
//
// Frame format (binary, 115200 baud):
//   byte 0-1   header: 0xAA 0x55
//   byte 2     payload length N
//   byte 3     function code (35 = RPY output, 22 = status/QoS)
//   byte 4-10  (unused here — sequence/timestamp fields, not needed)
//   byte 11-14 pitch, float32, byte-for-byte (union, native endianness)
//   byte 15-18 roll,  float32
//   byte 19-22 yaw,   float32
//   last 2 bytes      CRC16 (standard MODBUS algorithm, table-based)
//   total frame size = N + 5
//
// UNVERIFIED (flagged honestly, not guessed at): TM171's exact axis/
// sign convention for roll and yaw relative to physical mounting
// orientation has not been bench-tested against our own sign
// convention (yaw: 0-360°, clockwise-positive per readBNO()'s
// `-yawRad` negation; roll: positive = right lean). If field testing
// shows either is backwards, fix it at the two marked lines below
// (search "SIGN CONVENTION") rather than via rollInvert — rollInvert
// is applied uniformly to whichever IMU is active and flipping it here
// instead would make BNO08x's sign wrong to fix TM171's.
// =====================================================================

// TM171 uses the default hardware serial buffer only — not because
// Serial5 lacks the capability (it doesn't; every Teensy 4.1 hardware
// serial port, including this one, is a HardwareSerialIMXRT instance
// under the hood and could get the same addMemoryForRead/Write
// treatment SerialGPS/SerialGPSHeading already have in the main .ino —
// this was checked and confirmed, not assumed). It's simply not done
// here, deliberately: at 115200 baud a 27-byte RPY frame takes ~2.3ms
// to arrive, TM171_update() is called every loop() iteration (same as
// readBNO()'s dispatch), so overflow risk already looks low — and with
// no field reports of a TM171 memory/data problem and no local TM171
// hardware to verify a change against, adding the extra buffer isn't
// worth doing blind. If a real symptom ever shows up (dropped/garbled
// TM171 frames specifically), this is the first place to look —
// SerialImuTM171 would need to change from HardwareSerial* to
// HardwareSerialIMXRT*, plus a TM171rxbuffer/addMemoryForRead call in
// TM171_setup(), matching the existing GNSS-port pattern exactly.
uint8_t TM171lastData = 0;   // ms since last valid packet — mirrors the
                              // "TM171lastData = 0;" reset in the
                              // confirmed reference source, kept for
                              // parity even though our own watchdog
                              // uses lastImuDataTime instead.

enum TM171ParseState { TM171_WAIT_HEADER_1, TM171_WAIT_HEADER_2, TM171_WAIT_LENGTH, TM171_WAIT_PAYLOAD };
static TM171ParseState tm171State   = TM171_WAIT_HEADER_1;
static uint8_t         tm171PktLen  = 0;
static uint8_t         tm171PayIdx  = 0;
static uint8_t         tm171Data[64];

union TM171Onion { uint8_t fBytes[sizeof(float)]; float fValue; };

// MODBUS CRC16 — verbatim table from the confirmed reference source.
static uint16_t TM171_CRC16(const unsigned char *buf, unsigned int len)
{
    static const uint16_t table[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
    };
    uint8_t  tor = 0;
    uint16_t crc = 0xFFFF;
    buf += 2;   // skip the two 0xAA 0x55 sync bytes — not covered by CRC
    len -= 2;
    while (len--)
    {
        tor = (*buf++) ^ (uint8_t)crc;
        crc >>= 8;
        crc ^= table[tor];
    }
    return crc;
}

static bool TM171_GoodCRC(uint8_t data[], uint8_t length)
{
    uint16_t ck = TM171_CRC16(data, (unsigned int)(length - 2));
    return ck == (uint16_t)data[length - 2] + ((uint16_t)data[length - 1] << 8);
}

/// <summary>
/// Starts the TM171's UART at its fixed, factory baud rate (115200 —
/// not configurable, unlike GNSS baud rates, since this is the SYD
/// Dynamics TransducerM's own hardcoded default). Called once at
/// boot when ImuType==IMU_TM171, and again by checkImuWatchdog()
/// after a detected timeout, as the re-begin() half of that
/// function's recovery attempt.
/// </summary>
void TM171_setup()
{
    SerialImuTM171->begin(115200);
}

// Called by checkImuWatchdog() (zHandlers.ino) after a TM171 timeout,
// alongside re-issuing begin() on the UART — clears any partial frame
// that was mid-flight when data stopped arriving, so the parser starts
// clean rather than potentially waiting forever for the remainder of a
// frame that will never arrive.
void TM171_resetParser()
{
    tm171State  = TM171_WAIT_HEADER_1;
    tm171PktLen = 0;
    tm171PayIdx = 0;
}

// Called every loop() iteration when ImuType == IMU_TM171 — same
// dispatch point readBNO() uses when ImuType == IMU_BNO08X (see the
// main .ino's loop()). Parses the byte stream; on a valid, CRC-checked
// RPY frame, writes yaw/roll/pitch/correctionHeading and refreshes the
// shared watchdog timestamp — all identical in meaning to what
// readBNO() writes, so imuHandler() and everything downstream needs no
// awareness of which IMU produced them.
void TM171_update()
{
    while (SerialImuTM171->available())
    {
        uint8_t b = SerialImuTM171->read();

        switch (tm171State)
        {
            case TM171_WAIT_HEADER_1:
                if (b == 0xAA) { tm171Data[0] = b; tm171State = TM171_WAIT_HEADER_2; }
                break;

            case TM171_WAIT_HEADER_2:
                if (b == 0x55) { tm171Data[1] = b; tm171State = TM171_WAIT_LENGTH; }
                else            tm171State = TM171_WAIT_HEADER_1;
                break;

            case TM171_WAIT_LENGTH:
                tm171PktLen = b;
                tm171Data[2] = b;
                tm171PayIdx = 3;
                tm171State = TM171_WAIT_PAYLOAD;
                if ((uint16_t)tm171PktLen + 5u > (uint16_t)sizeof(tm171Data))
                    tm171State = TM171_WAIT_HEADER_1;   // implausible length, resync
                break;

            case TM171_WAIT_PAYLOAD:
                tm171Data[tm171PayIdx++] = b;
                if (tm171PayIdx >= tm171PktLen + 5)
                {
                    // Complete frame — validate and decode inline
                    // (matches confirmed reference structure; not
                    // pulled into a separate function since it's only
                    // ever called from this one spot).
                    if (TM171_GoodCRC(tm171Data, tm171Data[2] + 5))
                    {
                        TM171lastData = 0;
                        uint8_t functionCode = tm171Data[3];

                        if (functionCode == 35)   // RPY Output
                        {
                            TM171Onion pitchV, rollV, yawV;
                            pitchV.fBytes[0] = tm171Data[11]; pitchV.fBytes[1] = tm171Data[12];
                            pitchV.fBytes[2] = tm171Data[13]; pitchV.fBytes[3] = tm171Data[14];
                            rollV.fBytes[0]  = tm171Data[15]; rollV.fBytes[1]  = tm171Data[16];
                            rollV.fBytes[2]  = tm171Data[17]; rollV.fBytes[3]  = tm171Data[18];
                            yawV.fBytes[0]   = tm171Data[19]; yawV.fBytes[1]   = tm171Data[20];
                            yawV.fBytes[2]   = tm171Data[21]; yawV.fBytes[3]   = tm171Data[22];

                            // --- SIGN CONVENTION (unverified — see file header) ---
                            pitch = pitchV.fValue;
                            roll  = rollV.fValue * (float)rollInvert;   // same rollInvert BNO08x uses

                            yaw = yawV.fValue;
                            while (yaw <    0.0f) yaw += 360.0f;
                            while (yaw >= 360.0f) yaw -= 360.0f;
                            correctionHeading = (double)(-yawV.fValue * (float)(M_PI / 180.0));
                            // --- end SIGN CONVENTION ---

                            lastImuDataTime = millis();   // shared watchdog timestamp
                            imuHealthy      = true;       // shared watchdog health flag
                        }
                        // functionCode 22 (status/QoS) intentionally not
                        // decoded yet — see IMU tab TODO in the TFF
                        // architecture reference for surfacing qos as a
                        // PDIAG diagnostic field in a future pass.
                    }
                    tm171State = TM171_WAIT_HEADER_1;
                }
                break;
        }
    }
}
