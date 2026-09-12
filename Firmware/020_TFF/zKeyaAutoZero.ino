// =============================================================
// zKeyaAutoZero.ino — Automatic zero-point establishment and
// continuous drift correction for "Keya encoder as WAS"
// (WasSource == WAS_SOURCE_KEYA).
//
// PROVENANCE: this is a faithful port of the auto-zero algorithm
// from Flo's AIO_Keya_WasKeyaFiltre (github.com/Flodu81/
// AIO_ECU_Keya_WasKeyaFiltre), shared directly by the person running
// this project after a genuine field report ("motor just spins
// non-stop") with Keya-as-WAS on our own, un-corrected
// keyaGetSteerAngle() — see that function's own comment in zKeya.ino
// for the root cause this fixes (a purely incremental encoder with
// NO zero-point reference at all, previously just assuming wherever
// the wheels happened to be at the very first heartbeat was "zero").
//
// WHY THIS APPROACH, NOT JUST "require wheels straight at boot":
// A one-time, manual "wheels must be straight when powered on"
// requirement (which is what the SIMPLER community repo,
// 87yj/AgOpen_Keya_VirtualWAS, relies on, and explicitly warns "has
// not and should not be used on real machinery" as a result) is
// fragile — one missed check and the whole session's steering
// reference is wrong from the start, with no way to recover except a
// power cycle. Flo's approach instead establishes the zero
// automatically the first time the vehicle drives straight for a
// bit, and CONTINUES correcting for drift throughout the session
// using GPS/IMU heading as ground truth — matching what an operator
// intuitively does anyway ("drive straight, and the guidance
// straightens itself out").
//
// SAFETY GATING (ported faithfully, not simplified away): no
// guidance is possible until the first zero has been established —
// see the watchdogTimer coupling in c00_Autosteer.ino's main loop,
// directly analogous to the original's own
// "if (!wasZeroDone) watchdogTimer = WATCHDOG_FORCE_VALUE;".
//
// WHAT WAS DELIBERATELY CHANGED FROM THE ORIGINAL, AND WHY:
//   - Gated on WasSource == WAS_SOURCE_KEYA (a setting that already,
//     correctly means "Keya is being used as the WAS") — NOT the
//     original's steerConfig.IsDanfoss, which is a real, unrelated
//     Danfoss proportional-valve setting the original repurposed as
//     a convenient existing toggle. Using our own, already-correct
//     WasSource flag avoids hijacking an unrelated setting.
//   - Default parameter values use the ORIGINAL repo's own instance
//     defaults (speedMin=2.5, yawRateMax=0.3, gpsHdgMax=0.3,
//     beta=0.3), not its "reset to default" menu values
//     (yawRateMax=0.8, gpsHdgMax=1.0, beta=0.05) — the repo's own
//     code has these two disagreeing with each other (a genuine
//     inconsistency in the source, not something we introduced). The
//     instance defaults are the STRICTER of the two (a LOWER
//     yawRateMax/gpsHdgMax requires straighter driving before a
//     correction is accepted) — chosen deliberately as the safer
//     starting point for an active steering calibration, since it's
//     easier to loosen later than to have started too permissive.
//   - No serial menu (that was for the original's own standalone
//     testing setup) — parameters are, for now, compile-time
//     constants below; a later step could expose them via
//     SETXXX-style UDP commands the same way every other TFF setting
//     already works, matching the established pattern rather than
//     inventing a new one.
//   - Debug prints kept as Serial.println() (matching this project's
//     own established convention — see e.g. HPR_Handler,
//     resolveBoardSlots — of using Serial for verbose diagnostic
//     narration), not removed, since they were genuinely useful for
//     the original author's own field debugging and cost nothing
//     when nobody's watching the serial monitor.
// =============================================================

struct AutoZeroParams
{
    float    speedMin;      // km/h — below this, never attempt a correction
    float    yawRateMax;    // deg/s (BNO) — below this, driving counts as "straight"
    float    gpsHdgMax;     // deg — GPS heading variation below this counts as "straight"
    uint32_t timeSlowMs;    // required stable duration at/below speedSlow
    uint32_t timeFastMs;    // required stable duration at/above speedFast
    float    speedSlow;     // km/h
    float    speedFast;     // km/h
    uint8_t  useBno;        // 1 = require yaw-rate condition
    uint8_t  useGps;        // 1 = require GPS heading condition
    float    beta;          // soft-correction gain while guidance is active (0-1)
    // CAUGHT DURING THE PARAMETER-EXPOSURE WORK (v0.3.9 shipped
    // without this field at all — a genuine gap this fixes before
    // EEPROM persistence is added): the original repo's own struct
    // has an "ident" field for exactly this reason ("bumpe pour
    // forcer reinit EEPROM avec nouveaux champs" — its own comment).
    // Without it, a unit already running v0.3.9 (which never wrote
    // anything to this struct's EEPROM address at all) upgrading to
    // this version would read UNWRITTEN, effectively random EEPROM
    // bytes into every field below the moment loadAlphasFromEEPROM()
    // runs — this ident guards specifically against that, independent
    // of the broader ALPHA_EEPROM_MAGIC check (see
    // AUTOZERO_PARAMS_IDENT below for the actual comparison).
    uint32_t ident;
};

// See this file's own header comment for why these specific values
// (the original repo's OWN instance defaults, not its "reset"
// button's values, which disagree with each other) were chosen.
AutoZeroParams azParams =
{
    2.5f,   // speedMin
    0.3f,   // yawRateMax
    0.3f,   // gpsHdgMax
    500,    // timeSlowMs
    200,    // timeFastMs
    3.0f,   // speedSlow
    12.0f,  // speedFast
    1,      // useBno
    1,      // useGps
    0.3f,   // beta
    0xA202, // ident — same value as the original repo's own; not a
            // meaningful compatibility signal to preserve (we don't
            // read THEIR EEPROM), just kept identical rather than
            // picking an arbitrary different constant for no reason
};

// --- Zero-point state — flyttad hit, FÖRE alla funktioner i den här
// filen, medvetet efter ett verkligt kompileringsfel (v0.3.11 kunde
// inte kompilera): C++ kräver att en variabel är TEXTUELLT deklarerad
// FÖRE den används, ÄVEN inom SAMMA fil — till skillnad från
// FUNKTIONER (som Arduino automatiskt framåtdeklarerar, oavsett var i
// filen eller i vilken fil de definieras). azGetWasZeroDone()/
// azGetZeroTicksDegrees()/azGetZeroTicksRaw() längre ner i den här
// filen läser de här variablerna — de MÅSTE alltså stå textuellt
// FÖRE de funktionerna, inte bara "någonstans i samma fil". Redan
// korrekt löst för ANDRA filer (zHandlers.ino, zKeya.ino, som
// kompileras FÖRE den här filen) via getter-funktioner — men det
// LÖSER inte ordningen INOM den här filen själv, vilket är en helt
// separat regel.
int32_t keyaZeroTicks = 0;    // the "zero" reference point, in raw encoder ticks
bool    wasZeroDone   = false; // false until the FIRST zero has ever been established this boot
float   azCorrAccum   = 0.0f; // sub-tick accumulator for the soft, gradual correction path

// EEPROM address defined in zHandlers.ino (AUTOZERO_PARAMS_EEPROM_ADDR)
// — declared there, used here. This file's own load/save functions
// exist SPECIFICALLY because zHandlers.ino (where the shared
// loadAlphasFromEEPROM()/saveAlphasToEEPROM() live) compiles BEFORE
// this file alphabetically ("zHandlers" < "zKeyaAutoZero") — the
// AutoZeroParams type/azParams variable wouldn't be visible yet at
// that point in the merged sketch. A plain FUNCTION call works
// regardless of file order (Arduino auto-generates forward
// declarations for functions, but not for variables/types), so
// zHandlers.ino's existing functions just call these two, rather than
// touching azParams directly themselves.
#define AUTOZERO_PARAMS_IDENT 0xA202

void loadAutoZeroParamsFromEEPROM()
{
    AutoZeroParams saved;
    EEPROM.get(AUTOZERO_PARAMS_EEPROM_ADDR, saved);
    if (saved.ident == AUTOZERO_PARAMS_IDENT)
    {
        azParams = saved;
        // Defensive constrain() on every field, matching this
        // project's own established pattern elsewhere (e.g.
        // loadAlphasFromEEPROM()) — protects against a corrupted or
        // partially-written EEPROM region even when the ident happens
        // to match by chance.
        azParams.speedMin   = constrain(azParams.speedMin,   0.1f,  20.0f);
        azParams.yawRateMax = constrain(azParams.yawRateMax, 0.01f, 10.0f);
        azParams.gpsHdgMax  = constrain(azParams.gpsHdgMax,  0.01f, 10.0f);
        azParams.timeSlowMs = constrain(azParams.timeSlowMs, (uint32_t)100, (uint32_t)5000);
        azParams.timeFastMs = constrain(azParams.timeFastMs, (uint32_t)100, (uint32_t)5000);
        azParams.speedSlow  = constrain(azParams.speedSlow,  0.1f, 30.0f);
        azParams.speedFast  = constrain(azParams.speedFast,  0.1f, 30.0f);
        azParams.useBno     = (azParams.useBno != 0) ? 1 : 0;
        azParams.useGps     = (azParams.useGps != 0) ? 1 : 0;
        azParams.beta       = constrain(azParams.beta, 0.001f, 1.0f);
    }
    // else: EEPROM never written by this version before (or a v0.3.9
    // unit that predates this struct's EEPROM address entirely) —
    // azParams simply keeps the compile-time defaults set above,
    // exactly like every other never-yet-saved TFF setting.
}

void saveAutoZeroParamsToEEPROM()
{
    azParams.ident = AUTOZERO_PARAMS_IDENT;
    EEPROM.put(AUTOZERO_PARAMS_EEPROM_ADDR, azParams);
}

// One small setter per parameter, called from zHandlers.ino's
// SETxxx command handlers — same compilation-order reason as
// load/save above (azParams isn't visible yet at that point in the
// merged sketch). Each does its own constrain() and then saves, same
// pattern every other TFF setting already follows.
void azSetSpeedMin(float v)   { azParams.speedMin   = constrain(v, 0.1f,  20.0f); saveAutoZeroParamsToEEPROM(); }
void azSetYawRateMax(float v) { azParams.yawRateMax = constrain(v, 0.01f, 10.0f); saveAutoZeroParamsToEEPROM(); }
void azSetGpsHdgMax(float v)  { azParams.gpsHdgMax  = constrain(v, 0.01f, 10.0f); saveAutoZeroParamsToEEPROM(); }
void azSetTimeSlowMs(int v)   { azParams.timeSlowMs = constrain(v, 100, 5000);    saveAutoZeroParamsToEEPROM(); }
void azSetTimeFastMs(int v)   { azParams.timeFastMs = constrain(v, 100, 5000);    saveAutoZeroParamsToEEPROM(); }
void azSetSpeedSlow(float v)  { azParams.speedSlow  = constrain(v, 0.1f, 30.0f);  saveAutoZeroParamsToEEPROM(); }
void azSetSpeedFast(float v)  { azParams.speedFast  = constrain(v, 0.1f, 30.0f);  saveAutoZeroParamsToEEPROM(); }
void azSetUseBno(int v)       { azParams.useBno     = (v != 0) ? 1 : 0;           saveAutoZeroParamsToEEPROM(); }
void azSetUseGps(int v)       { azParams.useGps     = (v != 0) ? 1 : 0;           saveAutoZeroParamsToEEPROM(); }
void azSetBeta(float v)       { azParams.beta       = constrain(v, 0.001f, 1.0f); saveAutoZeroParamsToEEPROM(); }

// Getters for PDIAG reporting (zHandlers.ino, sendDiagnostics()) —
// same compilation-order reason as the setters above: a plain
// function CALL works regardless of file order, direct field access
// wouldn't. Called directly as snprintf() arguments there.
float azGetSpeedMin()      { return azParams.speedMin; }
float azGetYawRateMax()    { return azParams.yawRateMax; }
float azGetGpsHdgMax()     { return azParams.gpsHdgMax; }
int   azGetTimeSlowMs()    { return (int)azParams.timeSlowMs; }
int   azGetTimeFastMs()    { return (int)azParams.timeFastMs; }
float azGetSpeedSlow()     { return azParams.speedSlow; }
float azGetSpeedFast()     { return azParams.speedFast; }
int   azGetUseBno()        { return azParams.useBno; }
int   azGetUseGps()        { return azParams.useGps; }
float azGetBeta()          { return azParams.beta; }
int   azGetWasZeroDone()   { return wasZeroDone ? 1 : 0; }
// Reported in DEGREES, not raw ticks — raw encoder ticks mean nothing
// to someone reading Teensy Tool; the same steerSensorCounts scale
// keyaGetSteerAngle() itself already uses makes this directly
// comparable to steerAngleActual, which IS already shown.
float azGetZeroTicksDegrees() { return (float)keyaZeroTicks / steerSettings.steerSensorCounts; }

// RAW ticks, NOT degrees — needed by keyaGetSteerAngle() (zKeya.ino)
// specifically, which does its OWN division by steerSensorCounts
// AFTER subtracting this from keyaEncoderAccum. Added because
// zKeya.ino ALSO compiles before this file alphabetically ("zKeya" <
// "zKeyaAutoZero") — the exact same compilation-order reason every
// other getter/setter in this file already exists for, just missed
// for this one specific call site when keyaGetSteerAngle() was first
// written (a genuine bug, caught by the compiler at build time, not
// by review — see zKeya.ino's own updated comment).
int32_t azGetZeroTicksRaw() { return keyaZeroTicks; }

// Fixed thresholds tightening the straightness requirement as the
// steered angle approaches zero DURING active guidance — see this
// file's header comment; a small error near-centre is proportionally
// more disruptive than the same error at a large steering angle.
static const float AZ_NEAR_ZERO_DEG    = 2.0f;
static const float AZ_NEAR_ZERO_FACTOR = 0.3f;

static const float EMA_GPS_ALPHA = 0.1f;

// EMA-filtered GPS course-over-ground, in TENTHS of a degree (matches
// this project's own established convention elsewhere, e.g.
// headingOffset, for representing an angle as an integer-friendly
// x10 value) — built on top of vtgHeading, which zHandlers.ino's own
// VTG_Handler() already parses and stores; this file only adds the
// smoothing layer on top, touching nothing already working.
float emaGpsHdg = 0.0f;

/// <summary>
/// Called once per main loop iteration, from c00_Autosteer.ino,
/// AFTER steerAngleActual has been computed for this iteration (the
/// correction logic below reads it) and only when
/// WasSource == WAS_SOURCE_KEYA (checked by the caller, not
/// duplicated here — see this file's header comment on gating).
/// </summary>
void keyaAutoZeroUpdate()
{
    // --- EMA-smoothed GPS heading, updated every call regardless of
    // whether a correction is currently being attempted — mirrors
    // the original's own placement of this update outside the
    // straightness-check block. ---
    static bool emaGpsInit = false;
    float rawHdg = atof(vtgHeading) * 10.0f;
    if (!emaGpsInit)
    {
        emaGpsHdg = rawHdg;
        emaGpsInit = true;
    }
    else
    {
        float diff = rawHdg - emaGpsHdg;
        if (diff > 1800.0f)  diff -= 3600.0f;
        if (diff < -1800.0f) diff += 3600.0f;
        emaGpsHdg += EMA_GPS_ALPHA * diff;
        if (emaGpsHdg < 0.0f)    emaGpsHdg += 3600.0f;
        if (emaGpsHdg >= 3600.0f) emaGpsHdg -= 3600.0f;
    }

    static float    azLastYaw    = 0.0f;
    static uint32_t azLastTime   = 0;
    static bool     azYawInit    = false;
    static float    azLastGpsHdg = 0.0f;
    static bool     azGpsInit    = false;
    static int64_t  azAccum      = 0;
    static uint32_t azCount      = 0;
    static uint32_t azCooldown   = 0;
    static uint32_t dbgLastPrint = 0;
    static uint32_t stableStart  = 0;

    uint32_t nowMs = millis();
    bool guidanceActive = (watchdogTimer < WATCHDOG_THRESHOLD);

    // --- Yaw rate from BNO (deg/s), derived from our own already-
    // maintained yaw variable — no new IMU sensor report needed. ---
    float yawRate = 0.0f;
    if (!azYawInit)
    {
        azLastYaw  = yaw;
        azLastTime = nowMs;
        azYawInit  = true;
    }
    else
    {
        float dt = (nowMs - azLastTime) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        float dYaw = yaw - azLastYaw;
        if (dYaw >  180.0f) dYaw -= 360.0f;
        if (dYaw < -180.0f) dYaw += 360.0f;
        yawRate    = fabsf(dYaw) / dt;
        azLastYaw  = yaw;
        azLastTime = nowMs;
    }

    // --- GPS heading rate, from the emaGpsHdg computed above
    // (stored as x10 degrees, converted back to plain degrees here). ---
    float gpsHdgDeg  = emaGpsHdg / 10.0f;
    float gpsHdgRate = 0.0f;
    if (!azGpsInit)
    {
        azLastGpsHdg = gpsHdgDeg;
        azGpsInit    = true;
    }
    else
    {
        float dHdg = gpsHdgDeg - azLastGpsHdg;
        if (dHdg >  180.0f) dHdg -= 360.0f;
        if (dHdg < -180.0f) dHdg += 360.0f;
        gpsHdgRate   = fabsf(dHdg);
        azLastGpsHdg = gpsHdgDeg;
    }

    // --- Thresholds tighten automatically as steerAngleActual
    // approaches zero, but ONLY while guidance is actively steering
    // (no reason to be extra strict while the operator is just
    // driving manually with the motor disengaged). ---
    float adaptFactor = 1.0f;
    if (guidanceActive)
    {
        float absAngle = fabsf(steerAngleActual);
        if (absAngle < AZ_NEAR_ZERO_DEG)
        {
            float ratio = absAngle / AZ_NEAR_ZERO_DEG;
            adaptFactor = AZ_NEAR_ZERO_FACTOR + ratio * (1.0f - AZ_NEAR_ZERO_FACTOR);
        }
    }

    float yawRateMaxNow = azParams.yawRateMax * adaptFactor;
    float gpsHdgMaxNow  = azParams.gpsHdgMax  * adaptFactor;
    bool  gpsOk         = (gpsHdgRate < gpsHdgMaxNow);

    // --- Required stable duration interpolates between timeSlowMs
    // and timeFastMs depending on current speed — driving fast needs
    // less time to trust a correction than crawling slowly. ---
    float azTimeMsF;
    if      (gpsSpeed <= azParams.speedSlow) azTimeMsF = (float)azParams.timeSlowMs;
    else if (gpsSpeed >= azParams.speedFast) azTimeMsF = (float)azParams.timeFastMs;
    else
    {
        float t = (gpsSpeed - azParams.speedSlow) / (azParams.speedFast - azParams.speedSlow);
        azTimeMsF = (float)azParams.timeSlowMs + t * ((float)azParams.timeFastMs - (float)azParams.timeSlowMs);
    }
    azTimeMsF = constrain(azTimeMsF, 200.0f, 5000.0f);
    uint32_t azTimeMs = (uint32_t)azTimeMsF;

    bool speedOk    = (gpsSpeed > azParams.speedMin);
    bool straightOk = (!azParams.useBno) || (yawRate < yawRateMaxNow);
    bool gpsCapOk   = (!azParams.useGps) || gpsOk;
    bool cooldownOk = (nowMs - azCooldown > 2000);

    if (stableStart > 0 && (nowMs - dbgLastPrint > 5000))
    {
        dbgLastPrint = nowMs;
        Serial.print(guidanceActive ? F("[AZ-PRECISE] ") : F("[AZ-FAST] "));
        Serial.print(F("stable ")); Serial.print(nowMs - stableStart);
        Serial.print(F("/")); Serial.print(azTimeMs); Serial.print(F("ms"));
        Serial.print(F(" spd=")); Serial.print(gpsSpeed, 1);
        Serial.print(F(" straight=")); Serial.print(straightOk ? F("OK") : F("NOK"));
        Serial.print(F(" gps=")); Serial.print(gpsCapOk ? F("OK") : F("NOK"));
        Serial.print(F(" angle=")); Serial.println(steerAngleActual, 2);
    }

    if (speedOk && straightOk && gpsCapOk && cooldownOk)
    {
        if (stableStart == 0)
        {
            stableStart = nowMs;
            azAccum     = 0;
            azCount     = 0;
        }

        azAccum += (int64_t)keyaEncoderAccum;
        azCount++;

        if ((nowMs - stableStart) > azTimeMs && azCount > 0)
        {
            int32_t meanTicks = (int32_t)(azAccum / (int64_t)azCount);

            if (!wasZeroDone)
            {
                // First zero this boot — see the watchdog coupling in
                // c00_Autosteer.ino for why guidance was blocked
                // until this moment.
                keyaZeroTicks = meanTicks;
                wasZeroDone   = true;
                Serial.print(F("[AZ] First zero established, ticks="));
                Serial.println(keyaZeroTicks);
            }
            else if (!guidanceActive)
            {
                // Not actively steering right now — safe to jump
                // straight to the new value, no risk of a visible,
                // jerky correction mid-guidance.
                keyaZeroTicks = meanTicks;
                Serial.print(F("[AZ-FAST] Jump to ticks="));
                Serial.println(keyaZeroTicks);
            }
            else
            {
                // Actively steering — soft, sub-tick correction only,
                // so the operator never sees a sudden jump in the
                // steered angle while guidance is engaged.
                //
                // CAUGHT DURING PORTING, FIXED BEFORE FIRST USE: the
                // original multiplies by keyaTicksPerDeg (its own
                // ticks-per-degree scale) here — steerAngleActual is
                // in DEGREES, but azCorrAccum/keyaZeroTicks are in
                // raw ENCODER TICKS, so the degree value must be
                // scaled up before accumulating, or the correction
                // would silently run at the wrong rate (off by
                // whatever steerSensorCounts happens to be — could be
                // wildly too slow or too fast depending on the
                // specific encoder's resolution). We don't have
                // keyaTicksPerDeg as a separate variable — our own
                // keyaGetSteerAngle() (zKeya.ino) already uses
                // steerSettings.steerSensorCounts for exactly this
                // same ticks-per-degree role, so that's the correct,
                // consistent scale to use here too.
                float corrSign = steerConfig.InvertWAS ? -1.0f : 1.0f;
                azCorrAccum += corrSign * azParams.beta * steerAngleActual * steerSettings.steerSensorCounts;
                int32_t corrInt = (int32_t)azCorrAccum;
                if (corrInt != 0)
                {
                    keyaZeroTicks += corrInt;
                    azCorrAccum   -= (float)corrInt;
                }
            }

            stableStart = 0;
            azCooldown  = nowMs;
        }
    }
    else
    {
        stableStart = 0;
    }
}
