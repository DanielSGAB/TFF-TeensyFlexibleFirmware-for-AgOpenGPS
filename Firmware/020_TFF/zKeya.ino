// =====================================================================
// zKeya.ino — Keya CAN steering motor support
// =====================================================================
// HARDWARE NOTE: AIO v4.x boards have only one physical CAN channel on
// the main connector — Keya (hardcoded onto V_Bus, see CAN_setup() in
// zCAN_All_Brands.ino) and a CAN-ready tractor Brand cannot coexist on
// standard v4.x hardware without extra wiring. See the full explanation
// at the MOTOR_DRIVE_KEYA declaration, 020_TFF.ino.
//
// Protocol verified against Keya's own official manual ("ELECTRIC
// STEERING MOTOR User manual V2.4", KY170DD01005-08G, JINAN KEYA
// ELECTRON SCIENCE AND TECHNOLOGY CO., LTD — fetched in full from
// bsg-i.nbxc.com/upload/669/442/pdf/0d6c506ea3ebf97650f00dc39d.pdf at
// the time this was written) — not just community source alone, which turned out to genuinely disagree with itself:
// one community repo (zKeya.ino from AIO-UM982AGRIC-Keya) used a
// ±9995..9998 speed scale with the value in bytes 6-7, while two
// others (KeyaCANBUS.ino from AIO_Keya_WasKeyaFiltre, and zCANBUS.ino
// from KeyaBNO-WAS) used a ±1000ish scale with the value in bytes 4-5.
// The manual confirms the second group: "Speed: -1000~+1000
// corresponds to negative rated speed~rated speed", value in
// DATA_L(H) DATA_L(L) = bytes 4-5. A field report independently
// flagged the "5A firmware" (the specific repo with the wrong scale,
// which also silently auto-writes a 5A current-limit to the motor's
// own EEPROM on first detect) as unreliable in practice — consistent
// with it also having gotten the protocol scale wrong. That automatic
// EEPROM-write behaviour is deliberately NOT replicated here for the
// same reason; if current-limit configuration is ever added, it
// should be an explicit, installer-triggered action, not a silent
// automatic write at every boot.
//
// What the byte 6-7 "direction flag" in community code actually is:
// not a separate flag at all — SPEED_L(H) SPEED_L(L) SPEED_H(H)
// SPEED_H(L) is a plain 32-bit two's-complement signed integer sent
// as two 16-bit words, low word first. For any speed magnitude within
// the actual ±1000 range, the high word is naturally 0x0000 (positive)
// or 0xFFFF (negative, sign-extended) — which is what community code
// interpreted as a "direction flag", and which happens to work
// correctly for this range, but the manual's own explanation (full
// 32-bit two's complement) is what's actually implemented below.
// =====================================================================

// --- CAN IDs (motor node address 1, the factory default) ---
// ID of sending data:   0x0600000 + node address (hex)
// ID of returned data:  0x0580000 + node address (hex)
// ID of heartbeat data: 0x0700000 + node address (hex)
// If a Keya motor is ever configured to a different node address
// (parameter 0018 in the motor's own config, via Keya's own RS232
// config software — TFF has no part in setting this), these three
// constants would need updating to match. Not made configurable here
// since every known AOG/TFF-relevant reference used the factory
// default address of 1.
#define KEYA_CAN_ID_SEND      0x06000001
#define KEYA_CAN_ID_RESPONSE  0x05800001
#define KEYA_CAN_ID_HEARTBEAT 0x07000001

// --- Command bytes (bytes 0-3 of an 8-byte extended CAN frame) ---
// Confirmed directly from the manual's own worked examples (section
// 4.5.2/4.5.3), not just inferred from community code.
const uint8_t keyaEnableCmd[4]  = {0x23, 0x0D, 0x20, 0x01};
const uint8_t keyaDisableCmd[4] = {0x23, 0x0C, 0x20, 0x01};
const uint8_t keyaSpeedCmd[4]   = {0x23, 0x00, 0x20, 0x01};

// --- Fault bits (heartbeat bytes 6-7, manual section 5.1) ---
// Byte 7 (low byte of the 16-bit Control_Close/error code):
#define KEYA_FAULT_DISABLED             0x01
#define KEYA_FAULT_OVERVOLTAGE          0x02
#define KEYA_FAULT_HARDWARE_PROTECTION  0x04
#define KEYA_FAULT_EEPROM               0x08
#define KEYA_FAULT_UNDERVOLTAGE         0x10
// bit 0x20 = N/A (manual leaves this bit unlabelled)
#define KEYA_FAULT_OVERCURRENT          0x40
#define KEYA_FAULT_MODE_FAILURE         0x80
// Byte 6 (high byte):
#define KEYA_FAULT_LESS_PHASE           0x0100
#define KEYA_FAULT_MOTOR_STALL          0x0200
// bit 0x0400 = Reserved
#define KEYA_FAULT_HALL_FAILURE         0x0800
#define KEYA_FAULT_CURRENT_SENSING      0x1000
#define KEYA_FAULT_232_DISCONNECTED     0x2000
#define KEYA_FAULT_CAN_DISCONNECTED     0x4000
#define KEYA_FAULT_MOTOR_STALLED        0x8000

bool     keyaDetected      = false;   // set true on first heartbeat seen
uint16_t keyaFaultBits     = 0;       // raw Control_Close bits, latest heartbeat
bool     keyaFaultActive   = false;   // keyaFaultBits != 0 — diagnostic only,
                                        // reported via PDIAG (see zHandlers.ino),
                                        // never itself disables autosteer here;
                                        // that decision is made once, explicitly,
                                        // where intendToSteer is evaluated
                                        // (c00_Autosteer.ino), matching how
                                        // every other TFF diagnostic flag works.
int16_t  keyaMotorCurrent  = 0;       // heartbeat bytes 4-5, signed, raw units
int16_t  keyaMotorSpeed    = 0;       // heartbeat bytes 2-3, signed, raw units
int32_t  keyaEncoderAccum  = 0;       // accumulated angle position — see
                                        // keyaReceive() for the wrap-handling
                                        // logic (heartbeat's own angle field
                                        // is only 16-bit and wraps at 65535)
uint16_t keyaLastRawAngle  = 0;
bool     keyaFirstAngle    = true;    // guards the very first wrap-delta
                                        // calculation, which would otherwise
                                        // compare against an arbitrary zero

elapsedMillis keyaSendTimer;          // watchdog compliance — manual: "the
                                        // watchdog detects the line-off period
                                        // of 1000ms (speed command is sent
                                        // continuously, the interval must not
                                        // exceed 1000ms)". TFF's own send rate
                                        // (driven from loop(), far faster than
                                        // 1Hz) comfortably satisfies this on
                                        // its own — this timer exists so a
                                        // stalled/blocked loop() iteration
                                        // doesn't silently let the interval
                                        // lapse without at least being
                                        // tracked/knowable, not to slow down
                                        // the send rate.
#define KEYA_SEND_INTERVAL_MS 200      // well under the 1000ms watchdog limit,
                                        // matches the general cadence other
                                        // CAN sends in this project already use

void keyaSetup()
{
    // V_Bus itself is already begin()'d and baud-configured in
    // CAN_setup() (zCAN_All_Brands.ino) — 250000 baud, matching Keya's
    // own factory-default CAN baud rate (parameter 0021, factory
    // setting 250k) exactly, no conflict. This function only adds the
    // Keya-specific FIFO filter (see CAN_setup()'s own comment on why
    // filter index 5 was chosen) and sends nothing yet — the motor
    // stays disabled until autosteer actually engages.
}

void keyaSendSpeed(int16_t pwmValue, bool enable)
{
    // pwmValue is TFF's existing PID output range (see calcSteeringPID(),
    // c01_AutosteerPID.ino) — same -255..255-ish range already used for
    // the Cytron/IBT2 PWM paths, kept deliberately unscaled/unclamped
    // coming into this function so it stays a drop-in alternative
    // output path in motorDrive(), not a special case that needs its
    // own PID tuning.
    //
    // Manual: "Speed: -1000~+1000 corresponds to negative rated
    // speed~rated speed". Scaling -255..255 to that range: ×3.9 (255 ×
    // 3.9 ≈ 994.5) — matches the scale already confirmed correct
    // against two independent community implementations AND the
    // manual itself, not the "5A firmware" repo's ×39.2-ish
    // (-9995..9998) scale.
    int32_t speed32 = (int32_t)((float)pwmValue * 3.9f);
    if (speed32 >  1000) speed32 =  1000;
    if (speed32 < -1000) speed32 = -1000;

    CAN_message_t msg;
    msg.id = KEYA_CAN_ID_SEND;
    msg.flags.extended = true;
    msg.len = 8;
    memcpy(msg.buf, keyaSpeedCmd, 4);

    // Manual's own worked example: -50 (rated 100) -> FE 0C FF FF.
    // Plain 32-bit two's complement, low word first, each word
    // high-byte-first — NOT a separate "direction flag" the way
    // community code comments described it (see file header comment).
    int16_t lowWord  = (int16_t)(speed32 & 0xFFFF);
    int16_t highWord = (int16_t)(speed32 < 0 ? -1 : 0);
    msg.buf[4] = highByte(lowWord);
    msg.buf[5] = lowByte(lowWord);
    msg.buf[6] = highByte(highWord);
    msg.buf[7] = lowByte(highWord);

    V_Bus.write(msg);

    if (enable)
    {
        CAN_message_t en;
        en.id = KEYA_CAN_ID_SEND;
        en.flags.extended = true;
        en.len = 8;
        memcpy(en.buf, keyaEnableCmd, 4);
        en.buf[4] = en.buf[5] = en.buf[6] = en.buf[7] = 0x00;
        V_Bus.write(en);
    }
    else
    {
        CAN_message_t dis;
        dis.id = KEYA_CAN_ID_SEND;
        dis.flags.extended = true;
        dis.len = 8;
        memcpy(dis.buf, keyaDisableCmd, 4);
        dis.buf[4] = dis.buf[5] = dis.buf[6] = dis.buf[7] = 0x00;
        V_Bus.write(dis);
    }

    keyaSendTimer = 0;
}

/// <summary>
/// Drains every pending CAN message on V_Bus this call, keeping only
/// Keya's own heartbeat (KEYA_CAN_ID_HEARTBEAT) — everything else is
/// silently discarded. Extracts four things from each heartbeat: the
/// cumulative steering angle (accumulated into keyaEncoderAccum, with
/// 16-bit wrap-around handling — see the detailed comment inline
/// below for the manual-verified byte order and wrap logic),
/// instantaneous motor speed, motor current, and the raw fault-code
/// bits (keyaFaultActive is just "was any fault bit set at all",
/// diagnostic only — the specific bits aren't decoded further here).
/// Called every loop() iteration from autosteerLoop() (c00_Autosteer.ino).
/// </summary>
void keyaReceive()
{
    CAN_message_t msg;
    while (V_Bus.read(msg))
    {
        if (msg.id != KEYA_CAN_ID_HEARTBEAT) continue;

        keyaDetected = true;

        // Manual: heartbeat data is high-byte-first (opposite of the
        // command bytes above, which are low-byte-first) — confirmed
        // by the manual's own worked example: "01 68 00 14 00 00 40
        // 01" -> cumulative angle 0x0168, error code 0x4001. Byte 0 is
        // the HIGH byte of the angle here, not the low byte.
        uint16_t rawAngle = ((uint16_t)msg.buf[0] << 8) | msg.buf[1];

        // The heartbeat's own angle field is only 16-bit and wraps at
        // 65535 (manual: "when angle value reaches 0xffff=65535,
        // automatically clear and recount") — this accumulates across
        // wraps into a signed 32-bit running total, same fundamental
        // problem (and same fix shape) as GNSS heading wrap-handling
        // elsewhere in this project, just a different physical
        // quantity. Guarded on keyaFirstAngle so the very first
        // heartbeat ever received doesn't compare against an
        // arbitrary zero and register a huge, spurious jump.
        if (!keyaFirstAngle)
        {
            int32_t delta = (int32_t)rawAngle - (int32_t)keyaLastRawAngle;
            // Half-range wrap detection — if the raw delta looks
            // implausibly large in either direction, assume a wrap
            // occurred and correct for it, rather than accepting a
            // huge single-step jump.
            if      (delta >  32768) delta -= 65536;
            else if (delta < -32768) delta += 65536;
            keyaEncoderAccum += delta;
        }
        keyaLastRawAngle = rawAngle;
        keyaFirstAngle   = false;

        keyaMotorSpeed   = (int16_t)(((uint16_t)msg.buf[2] << 8) | msg.buf[3]);
        keyaMotorCurrent = (int16_t)(((uint16_t)msg.buf[4] << 8) | msg.buf[5]);
        keyaFaultBits    = ((uint16_t)msg.buf[6] << 8) | msg.buf[7];
        keyaFaultActive  = (keyaFaultBits != 0);
    }

    // Watchdog compliance note (see KEYA_SEND_INTERVAL_MS above) — the
    // actual periodic re-send happens from motorDrive() every loop()
    // iteration while steering is engaged, which already runs far
    // faster than the 1000ms limit; nothing additional needed here.
}

// Encoder-as-WAS: converts the accumulated encoder position into a
// steer angle, reusing steerSettings.steerSensorCounts — the same
// calibration value already used for physical-WAS installations
// (confirmed against community source: steerAngleActual =
// steeringPosition / steerSensorCounts, identical formula), so an
// installer switching WasSource between WAS_SOURCE_NORMAL and
// WAS_SOURCE_KEYA on the same vehicle doesn't need a second, separate
// counts-per-degree calibration. AckermanFix is deliberately NOT
// applied here — see this function's own body comment for why.
float keyaGetSteerAngle()
{
    // Deliberately does NOT apply AckermanFix here — the shared line
    // right after this function's call site in c00_Autosteer.ino
    // ("Ackerman fix", applied uniformly to steerAngleActual regardless
    // of which WAS source branch set it) already does, for all three
    // sources alike. Applying it here too would double it specifically
    // for the Keya path — caught and fixed before this was ever the
    // final version, not left as a subtle per-source inconsistency.
    //
    // keyaZeroTicks subtracted here — see zKeyaAutoZero.ino for how
    // that reference point is established and continuously corrected.
    // Before this, "0 degrees" meant "wherever the wheels happened to
    // be at the very first Keya heartbeat this boot" — an arbitrary,
    // never-calibrated reference. The watchdog gate in
    // c00_Autosteer.ino already prevents guidance from starting at
    // all until keyaZeroTicks has a real, established value (see
    // azGetWasZeroDone() there) — so by the time this line's result is
    // ever actually used to steer, the subtraction below is
    // meaningful, not just "minus zero".
    //
    // CAUGHT AT COMPILE TIME (v0.3.10 shipped with this broken):
    // azGetZeroTicksRaw() used here, NOT direct keyaZeroTicks access —
    // this file compiles BEFORE zKeyaAutoZero.ino alphabetically
    // ("zKeya" < "zKeyaAutoZero"), so the variable itself isn't
    // visible here yet in the merged sketch, only a function call
    // (which gets an automatic forward declaration regardless of file
    // order) works. The same reason every SETAZxxx command and PDIAG
    // getter already goes through a function in zKeyaAutoZero.ino
    // rather than touching azParams/wasZeroDone/keyaZeroTicks
    // directly — just missed for this one call site originally.
    return (float)(keyaEncoderAccum - azGetZeroTicksRaw()) / steerSettings.steerSensorCounts;
}
