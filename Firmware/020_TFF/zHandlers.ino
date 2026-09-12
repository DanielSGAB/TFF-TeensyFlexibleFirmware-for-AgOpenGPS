// zHandlers.ino — TFF v0.3.7. Heavily extended from its UM982 Fallback
// v0.7 origin across several versions since (Board Configuration,
// GNSS Passthrough, the full diagnostic health-monitoring system,
// Keya CAN steering motor support — all their EEPROM/command/PDIAG
// handling lives in this file) — "inherited unchanged" stopped being
// an accurate description before this comment was last corrected.
// PDIAG alone grew from 27 to 46 fields across that history.
/* =====================================================================
 * zHandlers.ino — NMEA handlers, BNO08x reader, PAOGI builder
 * Part of UM982 Fallback Firmware
 * GNSS/IMU fallback logic only — CAN steering lives in
 * zCAN_All_Brands.ino and is entirely independent of this file.
 * =====================================================================
 *
 *   GGA_Handler()          parses $G?GGA  — position
 *   VTG_Handler()          parses $G?VTG  — speed and course
 *   HPR_Handler()          parses $GPHPR  — dual heading and roll
 *   GGAH_Handler_Raw()     parses $GPGGAH — slave position/sats (bypasses
 *                          NMEAParser's 5-char token limit via strstr)
 *   readBNO()              reads BNO08x quaternion → yaw/roll/pitch
 *   updateHeadingOffset()  EMA: tracks HPR-to-BNO heading difference
 *   updateHeadingOffsetFromCOG()  EMA offset calibration from GPS course
 *                          over ground, used during extended fallback
 *   updateAlphaControl()   quality/satellite-driven HEADING_ALPHA and
 *                          ROLL_ALPHA blend control, plus dual
 *                          reconnect HOLD/RAMP sequencing
 *   imuHandler()           selects heading/roll source, formats strings
 *   BuildNmea()             assembles and transmits $PAOGI
 *   CalculateChecksum()    NMEA XOR checksum
 *   checkImuWatchdog()     non-blocking BNO08x restart on I2C hang
 *   sendDiagnostics()      sends $PDIAG to Teensy Tool every 2 s
 *                          (includes current CAN Brand — see below)
 *   receiveMonitorCommands() receives SETHEADINGALPHA, SETROLLALPHA,
 *                          SETSATSFULL, SETROLLZERO, SETDUALHOLD,
 *                          SETDUALRAMP, SETIMUAXIS, SETROLLINVERT,
 *                          SETWASCURRENT, SETUTURNSTRENGTH, and
 *                          SETBRAND (selects CAN steering brand,
 *                          0-7 or 8=BRAND_NONE — see
 *                          zCAN_All_Brands.ino for the CAN side)
 *   loadAlphasFromEEPROM() / saveAlphasToEEPROM()  persist all of the
 *                          above, including Brand, across power cycles
 * =====================================================================
 */

const char* asciiHex = "0123456789ABCDEF";

#include "zNMEAParser.h"

// -----------------------------------------------------------------------
// GGA fields  ($GPGGA / $GNGGA)
// -----------------------------------------------------------------------
// Sizes below are generous defensive margin, not tight-fit-to-spec —
// see the comment at getArg() usage further down for why: the shared
// NMEA parser library (zNMEAParser.h, third-party, kept unmodified)
// fills these via a raw strcpy() with NO bounds checking against the
// destination at all. A receiver sending an unexpectedly wide field
// (malformed sentence, firmware quirk, anything deviating from the
// normal/expected format) could overflow a tightly-sized buffer with
// zero protection. Can't fix the root cause without touching the
// third-party library — sized generously instead, low-cost insurance
// against a real but lower-probability risk than the two buffer bugs
// already found and fixed (which were guaranteed to occur under
// realistic conditions, not just theoretically possible under
// malformed input).
char fixTime[32];    // [0]  UTC hhmmss.ss
char latitude[32];   // [1]  DDMM.mmmmm
char latNS[32];      // [2]  N or S
char longitude[32];  // [3]  DDDMM.mmmmm
char lonEW[32];       // [4]  E or W
char fixQuality[32]; // [5]  0=none 1=GPS 2=DGPS 4=RTK 5=Float
char numSats[32];    // [6]  satellites tracked
char HDOP[32];       // [7]  horizontal dilution of precision
char altitude[32];   // [8]  altitude above MSL (m)
char ageDGPS[32];    // [12] seconds since last DGPS update

// -----------------------------------------------------------------------
// VTG fields  ($GPVTG / $GNVTG)
// -----------------------------------------------------------------------
char vtgHeading[32]; // [0]  true course over ground (degrees)
char speedKnots[32] = "0.000"; // [4]  speed over ground (knots) → PAOGI word[11]

// -----------------------------------------------------------------------
// HPR fields  ($GPHPR)
// Official Unicore format (Table 7-42, N4 Reference Manual R1.4):
//   $GPHPR,UTC,Heading,Pitch,Roll,QF,Sat,Age,StnID*CS
//   [1] Heading    dual-antenna azimuth, 0-360° (4dp)
//   [2] Pitch      fore-aft tilt, -90 to +90° (4dp)
//   [3] Roll       lateral tilt, -90 to +90° (4dp)
//
// Note: with CONFIG HEADING OFFSET 90, UM982 swaps pitch/roll axes —
// words[2] carries lateral tilt. Confirmed by field test. See hprRollIsWords2.
//   [4] QF         solution quality:
//                    0=invalid  1=single  2=DGPS
//                    4=RTK fixed (best)  5=RTK float
//                    Note: 5 (float) is WORSE precision than 4 (fixed)
//   [5] Sat        number of satellites used in heading solution
//   [6] Age        age of differential data (seconds)
//   [7] StnID      base station ID
// -----------------------------------------------------------------------
char umHeading[32];  // HPR words[1] — sized with defensive margin, see
                      // GGA fields comment above for why (getArg()'s
                      // unbounded strcpy())
char umRoll[32];     // HPR words[3] (Roll field — lateral tilt)

// Satellite counts
// satsMaster: from GPGGA getArg(6) — master antenna satellites in solution
// satsSlave:  from GPGGAH getArg(6) — slave antenna satellites in solution
//             Empty field (antenna disconnected) treated as 0
int satsMaster = 0;
int satsSlave  = 0;

// Slave fix quality from GPGGAH getArg(5)
// 0=no fix, 1=single, 4=RTK fix, 5=RTK float
int slaveFixQuality = 0;

// Flag: set true on first GPGGAH to activate satellite-based degradation.
bool ggahReceived = false;

// BNO roll zero offset — compensates for BNO mounting angle in fallback.
// Set via Teensy Tool, saved to EEPROM.
// Applied in fallback mode and in rollDiff stability check.
float rollZeroOffset = 0.0f;

// --- Auto Roll Adjust (v0.3.17) — continuously nudges rollZeroOffset
// to correct for slow BNO mounting-angle drift over time, ONLY when a
// trustworthy dual reference (RTK-fixed) is actually available. NOT a
// replacement for the "real" tilt calibration AGO's own "Zero IMU"
// workflow handles — this only corrects the OFFSET BETWEEN Dual and
// IMU roll, never the vehicle's actual, physical lean.
//
// autoRollAdjust/rollAutoDeadband/rollAutoAlpha: EEPROM-persisted
// settings (user-configured, rarely change — safe to persist).
// rollAutoCorrection: DELIBERATELY RAM-ONLY, never written to EEPROM
// — this is the continuously-updating live correction itself, not a
// setting. Persisting every single adjustment would wear out EEPROM
// cells (typically ~100k write endurance per cell) for something
// that updates at up to 10Hz while active. Folded into rollZeroOffset
// (ONE EEPROM write) only when auto-adjust is switched off — see
// SETAUTOROLLADJUST handler.
bool  autoRollAdjust    = false;
float rollAutoDeadband  = 0.2f;   // deg — differences below this are treated as noise, not drift
// 0.0005, not an earlier 0.02 — that value was picked by loose
// analogy to updateHeadingOffset()'s alpha, WITHOUT actually deriving
// it from this function's own 10Hz update rate (HPR arrives at 10Hz,
// same as GGA, in every dual-capable GnssMode). At 10Hz, alpha=0.02
// reaches 95% correction of a persistent difference in only ~15
// seconds — barely distinguishable from noise, not the "slow drift
// over time" this feature is meant to chase. This value instead
// targets ~10 minutes to 95% correction (t_95 ≈ 3*dt/alpha at 10Hz),
// a target explicitly chosen to be un-mistakably slower than any
// short-term noise pattern, while still self-correcting within a
// single field day rather than needing hours.
float rollAutoAlpha     = 0.0005f;
float rollAutoCorrection = 0.0f;  // RAM-only — the live, accumulating correction itself

// Settling timer — solQuality can read RTK-fixed for a moment right
// at boot before the dual solution has actually converged, sometimes
// showing a wildly wrong roll (tens of degrees) during that window.
// Tracks how long solQuality==4 has been CONTINUOUSLY true; auto-
// adjust only acts once that streak exceeds ROLL_AUTO_SETTLE_MS.
// 0 = "not currently in a good streak" (sentinel, reset whenever
// solQuality != 4).
uint32_t rollAutoSettleStartMs = 0;
const uint32_t ROLL_AUTO_SETTLE_MS = 12000;  // ~12s — midpoint of the 10-15s discussed

// IMU axis and roll invert — set in Teensy Tool to match physical mounting.
// These are applied to raw BNO quaternion data BEFORE PAOGI, so they work
// identically in dual and fallback regardless of AGO SteerConfig settings.
// imuAxis:    0=X, 1=Y, 2=Z  (which axis gives lateral tilt = roll)
// rollInvert: 1=normal, -1=inverted (flip roll sign)
// rollDual from HPR is also adjusted by rollInvert for consistency.
int   imuAxis    = 0;    // 0=X, 1=Y, 2=Z
int   rollInvert = -1;   // -1=normal X-forward (unchecked), 1=inverted (checked in monitor)

// WAS (wheel angle sensor) calibration for u-turn dual boost
// wasLeft/wasRight: steerAngleActual at full left/right lock (degrees)
// uTurnStrength: 1-10, how much dual is boosted at max steer angle
// 1 = no effect, 10 = alpha reduced to 0.1 at max steer
float wasLeft      = -45.0f;  // default, set via monitor
float wasRight     =  45.0f;  // default, set via monitor
int   uTurnStrength =  1;      // 1 = off

// HPR words[5] = satellites used in heading solution
int hprSats = 0;

// steeringPosition from c00_Autosteer.ino — used for u-turn dual boost
extern int16_t steeringPosition;
extern float   steerAngleActual;
extern float   steerSettings_steerSensorCounts;

// COG-based headingOffset calibration (fallback mode)
// Previous GGA position for COG calculation
double prevLatRad  = 0.0;   // radians
double prevLonRad  = 0.0;   // radians
bool   prevPosValid = false; // true after first valid GGA in fallback

// Minimum speed (km/h) for COG calibration to be trusted
// Below this threshold COG is too noisy to use
#define COG_MIN_SPEED_KMH  1.0f

// EMA alpha for COG-based headingOffset update.
// Slower than HPR (0.02) because COG is a noisier reference.
// 0.005 at 10Hz -> ~200 updates -> ~20s convergence.
#define COG_OFFSET_ALPHA  0.005

// -----------------------------------------------------------------------
// PAOGI output fields  (filled by imuHandler, used by BuildNmea)
// -----------------------------------------------------------------------
char imuHeading[32]; // degrees decimal, e.g. "285.37" — filled by our
                      // own dtostrf(outHeading, 6, 2, imuHeading), NOT
                      // getArg(), so not vulnerable to the same
                      // unbounded-strcpy() risk as the GGA/VTG/HPR
                      // fields above. Different, lower-risk mechanism
                      // still worth the same generous margin though:
                      // dtostrf()'s width parameter (6) is only a
                      // MINIMUM pad width, not a hard cap — if
                      // outHeading/outRoll/outPitch ever ended up
                      // genuinely out of their normal range (an
                      // upstream calculation bug, not expected in
                      // normal operation), dtostrf() would still write
                      // the full number, exceeding that width.
char imuRoll[32];    // degrees decimal, e.g. "-1.24"
char imuPitch[32];   // degrees decimal
char imuYawRate[32]; // deg/s, always "0" (not measured) — genuinely
                      // fixed content here, but sized to match the
                      // others for consistency rather than left as an
                      // outlier.

char nmea[180];      // assembled NMEA sentence buffer
                      // Sized with real margin, not assumed safe: a
                      // rigorous field-by-field worst-case calculation
                      // (every strcat() source's own declared buffer
                      // size, summed) gives up to 146 characters —
                      // the previous 120-byte size was 26 bytes short.
                      // Found via a direct comparison against an
                      // earlier saved snapshot after a report of
                      // "position reaches AGO but roll doesn't" — this
                      // buffer is built with strcat(), which has NO
                      // bounds checking at all (unlike snprintf()
                      // elsewhere in this project), so overflowing it
                      // isn't a safe truncation, it's undefined
                      // behaviour writing past the array into whatever
                      // memory sits after it. imuRoll is appended 13th
                      // of 16 fields (see BuildNmea()) — exactly late
                      // enough in the sequence that an overflow
                      // starting around there would explain position
                      // fields (appended earlier) arriving intact
                      // while roll (and anything after it) doesn't.


// =====================================================================
// GGA_Handler
// =====================================================================
// Parses position fields from $GPGGA / $GNGGA and sets dualReadyGGA.
// PAOGI is sent when both dualReadyGGA and dualReadyRelPos are true.
// =====================================================================
void GGA_Handler()
{
    parser.getArg(0,  fixTime);
    parser.getArg(1,  latitude);
    parser.getArg(2,  latNS);
    parser.getArg(3,  longitude);
    parser.getArg(4,  lonEW);
    parser.getArg(5,  fixQuality);
    parser.getArg(6,  numSats);
    parser.getArg(7,  HDOP);
    parser.getArg(8,  altitude);
    parser.getArg(12, ageDGPS);

    digitalWrite(GGAReceivedLED, blink ? HIGH : LOW);
    blink = !blink;

    GGA_Available = true;
    dualReadyGGA  = true;
    gpsReadyTime  = systick_millis_count;

    // Read master satellite count from GGA words[6]
    satsMaster = atoi(numSats);

    // COG-based headingOffset calibration (fallback mode only)
    // Convert NMEA lat/lon strings to radians for COG calculation
    if (solQuality < HPR_QUALITY_MIN && useIMU)
    {
        // Parse NMEA latitude: DDMM.mmmmm
        double latVal = atof(latitude);
        int    latDeg = (int)(latVal / 100.0);
        double latMin = latVal - latDeg * 100.0;
        double latDD  = latDeg + latMin / 60.0;
        if (latNS[0] == 'S') latDD = -latDD;

        // Parse NMEA longitude: DDDMM.mmmmm
        double lonVal = atof(longitude);
        int    lonDeg = (int)(lonVal / 100.0);
        double lonMin = lonVal - lonDeg * 100.0;
        double lonDD  = lonDeg + lonMin / 60.0;
        if (lonEW[0] == 'W') lonDD = -lonDD;

        // Only calibrate if moving fast enough for COG to be reliable
        float speedKmh = atof(speedKnots) * 1.852f;
        if (speedKmh >= COG_MIN_SPEED_KMH && latDD != 0.0 && lonDD != 0.0)
        {
            double latRad = latDD * (M_PI / 180.0);
            double lonRad = lonDD * (M_PI / 180.0);
            updateHeadingOffsetFromCOG(latRad, lonRad);
        }
        else
        {
            // Not moving or no position — reset so next valid pos starts fresh
            prevPosValid = false;
        }
    }
    else
    {
        // In dual mode: reset COG state so it's fresh when fallback starts
        prevPosValid = false;
    }
}


// =====================================================================
// VTG_Handler
// =====================================================================
// Extracts speed (knots) → PAOGI word[11].
// Course-over-ground (vtgHeading) is stored but not placed in PAOGI;
// PAOGI word[12] carries HPR or BNO heading instead.
// =====================================================================
void VTG_Handler()
{
    parser.getArg(0, vtgHeading);
    parser.getArg(4, speedKnots);
}




// =====================================================================
// readBNO
// =====================================================================
// Reads one GameRotationVector quaternion from BNO08x at ~50 Hz.
// GameRotationVector uses accelerometer + gyro (no magnetometer):
//   roll and pitch are gravity-referenced and accurate
//   yaw is relative to BNO startup orientation (not absolute north)
//
// headingOffset (maintained by updateHeadingOffset) converts the
// relative yaw to an approximate absolute azimuth for fallback use.
//
// Euler angles from quaternion (q = w + xi + yj + zk):
//   yaw   = atan2(2(wz + xy), 1 - 2(y² + z²))
//   pitch = asin(2(wy - zx))
//   roll  = atan2(2(wx + yz), 1 - 2(x² + y²))
//
// IsUseY_Axis (set in AGO SteerConfig) swaps roll and pitch axes so
// the user can match the BNO orientation without reflashing.
// =====================================================================
void readBNO()
{
    if (!bno08x.dataAvailable()) return;

    lastImuDataTime = millis();
    imuHealthy      = true;

    float dqx, dqy, dqz, dqw, dacr;
    uint8_t dac;
    bno08x.getQuat(dqx, dqy, dqz, dqw, dacr, dac);

    float norm = sqrtf(dqw*dqw + dqx*dqx + dqy*dqy + dqz*dqz);
    if (norm < 0.0001f) return;
    dqw /= norm; dqx /= norm; dqy /= norm; dqz /= norm;

    float ysqr = dqy * dqy;

    // Yaw
    float t3 = 2.0f * (dqw * dqz + dqx * dqy);
    float t4 = 1.0f - 2.0f * (ysqr + dqz * dqz);
    float yawRad = atan2f(t3, t4);
    correctionHeading = (double)(-yawRad);  // radians, for updateHeadingOffset

    yaw = -yawRad * (180.0f / M_PI);
    if (yaw < 0.0f)    yaw += 360.0f;
    if (yaw >= 360.0f) yaw -= 360.0f;

    // Pitch and roll (before axis swap)
    float t2 = 2.0f * (dqw * dqy - dqz * dqx);
    t2 = constrain(t2, -1.0f, 1.0f);
    float pitchRaw = asinf(t2) * (180.0f / M_PI);

    float t0 = 2.0f * (dqw * dqx + dqy * dqz);
    float t1 = 1.0f - 2.0f * (dqx * dqx + ysqr);
    float rollRaw = atan2f(t0, t1) * (180.0f / M_PI);

    // Axis selection: use Teensy Tool settings, not AGO SteerConfig.
    // AGO SteerConfig axis/invert only applies to PANDA format — we use PAOGI.
    switch (imuAxis)
    {
        case 1:  roll = pitchRaw; pitch = rollRaw;  break;  // Y-axis = roll
        case 2:  roll = dqz * (180.0f / M_PI); pitch = pitchRaw; break; // Z (approx)
        default: roll = rollRaw;  pitch = pitchRaw; break;  // X-axis = roll
    }
    roll *= (float)rollInvert;
}


// =====================================================================
// updateHeadingOffset
// =====================================================================
// EMA tracking of the difference between HPR heading and raw BNO yaw.
// Called every HPR_Handler call while solQuality >= HPR_QUALITY_MIN.
//
// Flip protection uses a rate gate:
//   headingRate = EMA of |HPR change| per second (10Hz updates = /0.1s)
//   If instantaneous HPR change >> filtered rate → likely flip → reject.
//   EMA smoothing prevents single noisy samples from causing false rejects.
//
// Physical limit: a tractor doing a hard 90° turn takes ~1s → ~90°/s max.
// A 180° flip appears as ~1800°/s (180° in one 0.1s update interval).
// Threshold at 200°/s gives comfortable margin for real turns.
// =====================================================================
static double prevHprHeading  = -1.0;   // previous HPR heading for rate calc
static double headingRateEMA  =  0.0;   // EMA of heading rate (°/s)
#define HPR_RATE_EMA_ALPHA    0.3        // EMA alpha for rate filter
#define HPR_RATE_MAX_DEG_S    200.0      // max plausible rate (°/s)
#define HPR_RATE_SPIKE_MULT   5.0        // spike must exceed EMA × this

// Second, separate flip check — used only in imuHandler()'s alpha=0
// branch. Correcting a stale comment found during review: this is NOT
// a rate check the way the three constants above are (it doesn't
// compare two consecutive HPR samples 0.1s apart) — it compares a
// single live HPR reading against the CURRENTLY EMA-TRACKED BNO+offset
// estimate, which is itself continuously converging toward HPR
// whenever quality is good. During legitimate driving, including a
// hard turn at the ~90°/s physical max established for
// HPR_RATE_MAX_DEG_S above, the EMA's own convergence lag can plausibly
// account for several degrees of momentary disagreement between the
// two — applying the other check's rate×0.1s logic here (≈20°) would
// be too tight and risk rejecting genuine fast-turn data. This
// threshold is deliberately generous instead: it exists only to reject
// overtly implausible single readings (flip-magnitude, not turn-
// magnitude), leaving smaller anomalies to the rate-based check above.
#define HPR_ABS_DIFF_FLIP_DEG 150.0f

// =====================================================================
// applyHeadingKalman() — wraparound-safe wrapper around the shared
// headingFilter (SimpleKalmanFilter) instance.
// =====================================================================
// SimpleKalmanFilter is a plain scalar filter with no notion that
// heading is circular (0°/360° are the same point). Calling
// headingFilter->updateEstimate(heading) directly — as an earlier
// version of this code did — breaks badly every time heading crosses
// due north: e.g. 359° followed by 1° looks like a -358° jump to a
// scalar filter, which it would try to track/smooth as if it were a
// genuine, almost-full-circle turn, producing a visible glitch right
// at that heading. This was found (not yet triggered in practice,
// since filterHeading defaults to off) before the Kalman filter was
// generalised from UM982-only to all three dual-receiver sources
// (zzGNSS_UM982.ino, zzGNSS_DualF9P.ino, zzGNSS_DualUM980.ino) — fixed
// once, centrally, here, rather than three times.
//
// Fix: unwrap the new raw measurement to be within ±180° of the last
// filtered output before handing it to the filter, then wrap the
// filter's result back into [0,360) before returning it. This makes
// the filter always see a small, continuous-looking step near a
// heading crossing 360°/0°, instead of an apparent near-full-circle
// jump. roll (see rollFilter, used directly in each GNSS source file)
// does not need this — roll is bounded to [-90,+90] and never wraps.
static double lastFilteredHeading = -1.0;  // [0,360), seeds itself on first call

double applyHeadingKalman(double rawHeadingDeg)
{
    if (lastFilteredHeading < 0.0)
        lastFilteredHeading = rawHeadingDeg;  // first call: no prior reference yet

    double diff = rawHeadingDeg - lastFilteredHeading;
    while (diff >  180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    double unwrapped = lastFilteredHeading + diff;   // near lastFilteredHeading, not necessarily in [0,360)

    double filtered = (double)headingFilter->updateEstimate((float)unwrapped);

    while (filtered >= 360.0) filtered -= 360.0;
    while (filtered <    0.0) filtered += 360.0;

    lastFilteredHeading = filtered;
    return filtered;
}

/// <summary>
/// Keeps headingOffset (the correction added to the IMU's own,
/// arbitrary yaw to align it with true/GPS heading) tracking the
/// latest HPR-derived heading, in two stages:
///   1. Rate gate: rejects this update entirely if the implied
///      heading change is implausibly fast — either exceeding an
///      absolute cap (HPR_RATE_MAX_DEG_S) or spiking well above the
///      recent, EMA-smoothed typical rate (HPR_RATE_SPIKE_MULT), the
///      kind of thing a bad GNSS fix or a momentary RTK glitch would
///      produce, not real vehicle motion. Skipped entirely on the
///      very first call (prevHprHeading still at its -1 sentinel —
///      nothing to compare a rate against yet).
///   2. Dual-alpha offset update: the actual correction step, once
///      the update passes the gate above. Uses a FASTER alpha (0.2)
///      when the current offset is far from correct (>10°, e.g. right
///      after boot) so the tractor doesn't drive with a wildly wrong
///      heading for long, and a SLOWER alpha (0.1) once already close,
///      for a steadier, less jittery steady-state offset.
/// </summary>
void updateHeadingOffset(double hprHeading)
{
    // --- Rate gate: reject implausible heading jumps ---
    if (prevHprHeading >= 0.0)
    {
        double delta = hprHeading - prevHprHeading;
        while (delta >  180.0) delta -= 360.0;
        while (delta < -180.0) delta += 360.0;

        double instantRate = fabs(delta) * 10.0;  // °/s at 10Hz

        // Update EMA of heading rate
        headingRateEMA = HPR_RATE_EMA_ALPHA * instantRate
                       + (1.0 - HPR_RATE_EMA_ALPHA) * headingRateEMA;

        // Reject if rate exceeds absolute max AND is a spike vs filtered rate
        bool absoluteExceeded = instantRate > HPR_RATE_MAX_DEG_S;
        bool spikeDetected    = instantRate > HPR_RATE_SPIKE_MULT * headingRateEMA
                                && headingRateEMA > 5.0;  // only if we have history
        if (absoluteExceeded || spikeDetected)
        {
            prevHprHeading = hprHeading;  // update prev so next calc is from here
            return;  // reject this update
        }
    }
    prevHprHeading = hprHeading;

    // --- Normal offset update ---
    double diff = hprHeading - (yaw + headingOffset);
    while (diff >  180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;

    // Dual alpha: fast convergence when far, slow tracking when close
    double alpha = (fabs(diff) > 10.0) ? 0.2 : 0.1;
    headingOffset += alpha * diff;

    if (headingOffset >  360.0) headingOffset -= 360.0;
    if (headingOffset < -360.0) headingOffset += 360.0;
}


// =====================================================================
// updateHeadingOffsetFromCOG
// =====================================================================
// Calibrates headingOffset from GPS course-over-ground (COG) when in
// fallback mode (no dual heading available).
//
// COG is the direction of travel over ground — when reversing, COG is
// heading+180°. To avoid contaminating headingOffset during reversing,
// COG is compared against current estimated heading. If COG differs
// by more than 90°, the tractor is likely reversing — skip update.
//
// Uses slower EMA alpha (COG_OFFSET_ALPHA) than HPR calibration
// because COG is noisier, especially at low speed or during turns.
//
// STATUS: effectively dead code in practice, deliberately left that
// way — see the full reasoning in the comment directly above the
// `dist2 < 1e-12` guard a few lines into the function body below.
// =====================================================================
void updateHeadingOffsetFromCOG(double latRad, double lonRad)
{
    if (!prevPosValid)
    {
        prevLatRad   = latRad;
        prevLonRad   = lonRad;
        prevPosValid = true;
        return;
    }

    double dLat = latRad - prevLatRad;
    double dLon = lonRad - prevLonRad;

    // =====================================================================
    // KNOWN, DELIBERATELY UNFIXED: this threshold requires ~230 km/h of
    // implied speed between two consecutive 10Hz GGA fixes to pass (a
    // tractor never gets remotely close) — found during review to be
    // roughly 50,000x stricter than it needs to be, given the separate,
    // already-correct speed gate at the call site (COG_MIN_SPEED_KMH,
    // GGA_Handler() — real measured VTG speed, not reconstructed from
    // position deltas). In practice this means the code below this
    // point — including the cogDiff > 90° reverse-detection a few lines
    // down — has been effectively dead/unreached since v0.5.
    //
    // Deliberately left as-is, not "fixed", after working through the
    // reasoning with the person running this firmware in the field:
    //   - 20 hours of real v0.5 field use with this exact code, this
    //     exact threshold, produced zero flips and zero incorrect
    //     forward/back detection.
    //   - That's not a coincidence: heading (chassis orientation) stays
    //     physically correct through reversing on its own — the chassis
    //     hasn't rotated, only the direction of travel has — so AGO's
    //     map/vehicle icon doesn't need any active reverse-detection to
    //     render correctly. Position alone already shows the vehicle
    //     moving backward along an unchanged heading arrow.
    //   - The two flip protections that ARE independent of this
    //     function and DO run unconditionally (the rate-gate in
    //     updateHeadingOffset() above, and the HPR_ABS_DIFF_FLIP_DEG
    //     check in imuHandler()) are what actually protect against a
    //     genuine flip. Neither depends on COG or this threshold at all.
    //   - If this threshold were fixed and COG calibration became live,
    //     a NEW, currently-unsolved risk would open up: jockeying
    //     (rapid forward/reverse maneuvering in a tight yard, typically
    //     in exactly the poor-GNSS environments — near buildings/trees —
    //     where this fallback path is most likely to be active at all)
    //     can produce a single noisy, low-speed COG reading that lands
    //     inside the 90° "looks forward-ish" window by chance, corrupting
    //     headingOffset even though it doesn't represent a real
    //     direction. Solving that would need something beyond the
    //     current single-reading check (e.g. a cooldown/consistency
    //     requirement across several readings) before it would be safe
    //     to activate — not implemented, because there was no evidence
    //     the inactive code was causing a problem worth that added
    //     complexity.
    // =====================================================================
    double dist2 = dLat*dLat + dLon*dLon;
    if (dist2 < 1e-12)
    {
        prevLatRad = latRad;
        prevLonRad = lonRad;
        return;
    }

    // Forward azimuth (COG), 0=N, 90=E, 180=S, 270=W
    double cosLat = cos((prevLatRad + latRad) * 0.5);
    double cogRad = atan2(dLon * cosLat, dLat);
    double cogDeg = cogRad * (180.0 / M_PI);
    if (cogDeg < 0.0)    cogDeg += 360.0;
    if (cogDeg >= 360.0) cogDeg -= 360.0;

    // Reverse detection: COG should match heading within ~90°.
    // If difference > 90°, tractor is likely reversing — skip update.
    double currentHeading = yaw + headingOffset;
    while (currentHeading >= 360.0) currentHeading -= 360.0;
    while (currentHeading <    0.0) currentHeading += 360.0;

    double cogDiff = cogDeg - currentHeading;
    while (cogDiff >  180.0) cogDiff -= 360.0;
    while (cogDiff < -180.0) cogDiff += 360.0;

    if (fabs(cogDiff) > 90.0)
    {
        // Likely reversing — do not update headingOffset from COG
        prevLatRad = latRad;
        prevLonRad = lonRad;
        return;
    }

    headingOffset += COG_OFFSET_ALPHA * cogDiff;

    if (headingOffset >  360.0) headingOffset -= 360.0;
    if (headingOffset < -360.0) headingOffset += 360.0;

    prevLatRad = latRad;
    prevLonRad = lonRad;
}


// =====================================================================
// imuHandler
// =====================================================================
// Selects heading and roll source, applies fusion if configured, and
// formats the results as decimal strings for BuildNmea().
//
// DUAL mode  (solQuality >= HPR_QUALITY_MIN):
//   roll    = HPR words[2]  (dual is more accurate for roll)
//   heading = HPR words[1]  (pure dual)
//             OR blend: (1-alpha)*HPR + alpha*(BNO+offset)  if HEADING_ALPHA>0
//
// FALLBACK mode  (solQuality < HPR_QUALITY_MIN):
//   roll    = BNO roll   (axis and zero set by user in AGO SteerConfig)
//   heading = BNO yaw + headingOffset
//
// Fusion blending uses the shortest arc to avoid 0/360° wrap errors.
// =====================================================================
/// <summary>
/// Continuously nudges rollAutoCorrection (RAM-only) to close the gap
/// between the trusted dual roll reading and the IMU's own,
/// offset-corrected roll — chasing slow sensor-mounting drift over
/// time, NOT a one-shot calibration. Called every cycle from
/// imuHandler()'s own DUAL-mode branch, with the same rollDual/
/// rollImu values that branch already computed.
///
/// All conditions below must hold for an adjustment to actually
/// happen this cycle:
///   1. autoRollAdjust must be on (user's own checkbox state)
///   2. solQuality == 4 — STRICT RTK-fixed only, deliberately
///      stricter than the existing RollValid/HPR_QUALITY_MIN
///      threshold (which also accepts float=5, less precise — not
///      good enough grounds to nudge a persisted-adjacent value)
///   3. Settling timer: solQuality==4 must have held CONTINUOUSLY
///      for ROLL_AUTO_SETTLE_MS — protects against the dual solution
///      showing a wildly wrong roll (tens of degrees) for a few
///      seconds right after acquiring a fix, before it's actually
///      converged
///   4. |rollDual - rollImu| must exceed rollAutoDeadband — smaller
///      differences are treated as ordinary sensor noise, not real
///      drift; adjusting on every tiny fluctuation would just chase
///      noise instead of correcting genuine long-term drift
/// </summary>
void updateAutoRollAdjust(float rollDualDeg, float rollImuDeg)
{
    if (!autoRollAdjust) { rollAutoSettleStartMs = 0; return; }

    // CAUGHT ON REVIEW: this function is called from inside
    // imuHandler()'s "if (ImuType == IMU_NONE || solQuality >=
    // HPR_QUALITY_MIN)" branch — that condition ALSO admits
    // ImuType==IMU_NONE regardless of solQuality, so without this
    // explicit check here, a genuinely solQuality==4 reading with NO
    // IMU configured at all would still reach this function and start
    // "correcting" rollImuDeg — which in that case is derived from
    // whatever stale/meaningless roll value exists with no IMU
    // actually feeding it. The spec's own "ImuType != IMU_NONE"
    // condition was never actually implemented anywhere until this
    // line — the surrounding branch does NOT provide that guard, a
    // genuine gap found on review, not by compiling.
    if (ImuType == IMU_NONE) { rollAutoSettleStartMs = 0; return; }

    if (solQuality != 4)
    {
        rollAutoSettleStartMs = 0;  // streak broken, reset
        return;
    }

    // CAUGHT ON REVIEW: solQuality is only ever reset to 0 by
    // zzGNSS_DualF9P.ino/zzGNSS_DualUM980.ino's own UBX-derived
    // solution-lost handling — UM982's own HPR_Handler (zzGNSS_UM982.ino)
    // never resets it at all when HPR sentences stop arriving. Without
    // this check, a stale solQuality==4 left over from BEFORE the dual
    // signal was lost could still satisfy the check above in UM982
    // mode specifically, even while genuinely no fresh HPR data is
    // coming in — hprTimeout (already computed elsewhere, watchdog on
    // time since the last HPR sentence) is the correct, independent
    // signal that data is actually current, not just a plausible-
    // looking leftover number.
    if (hprTimeout)
    {
        rollAutoSettleStartMs = 0;
        return;
    }

    // First cycle of a fresh RTK-fixed streak — start the clock, don't
    // adjust yet (0 is the sentinel for "no streak in progress").
    if (rollAutoSettleStartMs == 0)
    {
        rollAutoSettleStartMs = millis();
        return;
    }

    if (millis() - rollAutoSettleStartMs < ROLL_AUTO_SETTLE_MS)
        return;  // still within the post-acquisition settling window

    float diff = rollDualDeg - rollImuDeg;
    if (fabsf(diff) <= rollAutoDeadband)
        return;  // within noise tolerance, no adjustment needed

    rollAutoCorrection += rollAutoAlpha * diff;
    // Same overall bound as the manual SETROLLZERO command
    // (-30..30°) applied to the COMBINED value, not rollAutoCorrection
    // in isolation — rollZeroOffset itself could already be close to
    // either bound, so this clamps the sum, then backs rollAutoCorrection
    // off to whatever's left of that combined bound.
    float combined = constrain(rollZeroOffset + rollAutoCorrection, -30.0f, 30.0f);
    rollAutoCorrection = combined - rollZeroOffset;
}


void imuHandler()
{
    float outHeading, outRoll, outPitch;

    // ImuType == IMU_NONE ("No fallback function") always takes this
    // branch, regardless of actual solQuality — deliberately. This
    // branch already degrades safely to pure-dual output when
    // useIMU is false (see the useIMU checks a few lines down,
    // unchanged) — the *other* branch (the true fallback, below) does
    // not have — and doesn't need — that guard, since it unconditionally
    // reads yaw/roll/pitch assuming an IMU exists. Rerouting IMU_NONE
    // here rather than adding a guard to the fallback branch keeps that
    // branch exactly as it was, only reachable at all when an IMU is
    // actually configured (BNO08x or TM171).
    if (ImuType == IMU_NONE || solQuality >= HPR_QUALITY_MIN)
    {
        // Roll fusion: blend BNO roll into HPR roll
        // No wraparound handling needed (roll stays within ±180°)
        // HPR roll is the reference — not adjusted by rollInvert or rollZeroOffset.
        // AGO SteerConfig "Zero IMU" handles the dual antenna mounting offset.
        // rollInvert and rollZeroOffset only adjust BNO roll to match HPR.

        // BNO roll has rollInvert applied in readBNO(), subtract offset to match HPR.
        // rollZeroOffset + rollAutoCorrection: the manually-set baseline
        // plus the live, RAM-only auto-adjust correction (0.0 if auto-
        // adjust is off) — see the variable declarations further up
        // this file for the full Auto Roll Adjust design.
        float rollImu = roll - (rollZeroOffset + rollAutoCorrection);

        // Expose both raw components (pre-blend) for Teensy Tool's IMU
        // tab "Dual Roll vs IMU Roll" comparison panel — lets the user
        // tune rollZeroOffset by eye until the two match, without
        // touching ROLL_ALPHA or looking anything up in AgOpenGPS.
        // See TFF_ROLL_COMPARE fields in sendDiagnostics().
        rawDualRoll      = (float)rollDual;
        rawImuRoll       = rollImu;
        rawDualRollValid = true;

        // Auto Roll Adjust — called here, not in a separate loop()
        // location, so it only ever runs alongside a genuinely fresh
        // rollDual/rollImu pair (this whole block only executes when a
        // dual-capable GnssMode is active and useIMU-eligible ImuType
        // is configured — the other two required conditions from the
        // spec). solQuality itself, plus the internal deadband/settle-
        // timer checks, are evaluated inside the function.
        updateAutoRollAdjust((float)rollDual, rollImu);

        if (useIMU && ROLL_ALPHA > 0.0f)
            outRoll = (1.0f - ROLL_ALPHA) * (float)rollDual + ROLL_ALPHA * rollImu;
        else
            outRoll = (float)rollDual;

        outPitch = 0.0f;

        if (useIMU && HEADING_ALPHA > 0.0f)
        {
            float imuAbs = (float)(yaw + headingOffset);
            while (imuAbs >= 360.0f) imuAbs -= 360.0f;
            while (imuAbs <    0.0f) imuAbs += 360.0f;

            float diff = imuAbs - (float)heading;
            while (diff >  180.0f) diff -= 360.0f;
            while (diff < -180.0f) diff += 360.0f;

            outHeading = (float)heading + HEADING_ALPHA * diff;
            while (outHeading >= 360.0f) outHeading -= 360.0f;
            while (outHeading <    0.0f) outHeading += 360.0f;
        }
        else if (useIMU)
        {
            // HEADING_ALPHA = 0: pure HPR, but protect against flip.
            // See HPR_ABS_DIFF_FLIP_DEG above for why this uses a flat
            // absolute-difference threshold rather than the rate-based
            // gate updateHeadingOffset() uses — different check,
            // different purpose, deliberately different shape.
            float imuAbs = (float)(yaw + headingOffset);
            while (imuAbs >= 360.0f) imuAbs -= 360.0f;
            while (imuAbs <    0.0f) imuAbs += 360.0f;

            float diff = (float)heading - imuAbs;
            while (diff >  180.0f) diff -= 360.0f;
            while (diff < -180.0f) diff += 360.0f;

            // |diff| > HPR_ABS_DIFF_FLIP_DEG assume flip — use BNO+offset
            if (fabsf(diff) > HPR_ABS_DIFF_FLIP_DEG)
                outHeading = imuAbs;
            else
                outHeading = (float)heading;
        }
        else
        {
            // useIMU == false — no IMU at all (ImuType == IMU_NONE,
            // or the BNO/TM171 watchdog gave up after repeated
            // failures mid-session). Pure dual heading, no flip
            // protection possible here — that check exists specifically
            // to sanity-check HPR against BNO, and with no IMU present
            // there is nothing to check it against, so trusting the
            // dual source directly is the correct (and only) option,
            // matching traditional non-fallback dual-only behaviour.
            //
            // This branch was MISSING before — outHeading was left
            // unassigned (stack garbage) whenever useIMU was false
            // and solQuality was still >= HPR_QUALITY_MIN, a real,
            // pre-existing bug reachable any time the watchdog disabled
            // useIMU mid-session, not something newly introduced by
            // adding IMU_NONE — just newly guaranteed to be hit on every
            // single cycle in that mode instead of only occasionally.
            outHeading = (float)heading;
        }
    }
    else  // fallback
    {
        float imuAbs = (float)(yaw + headingOffset);
        while (imuAbs >= 360.0f) imuAbs -= 360.0f;
        while (imuAbs <    0.0f) imuAbs += 360.0f;

        outHeading = imuAbs;
        // rollAutoCorrection included here too, even though
        // updateAutoRollAdjust() is never CALLED from this branch (it
        // only runs in the DUAL branch above, which requires
        // solQuality==4) — an already-accumulated correction from a
        // previous dual-signal period is still a valid mounting-
        // offset correction, and doesn't suddenly stop being true just
        // because the dual signal temporarily dropped out. Including
        // it here avoids a visible jump in roll if the system
        // transitions from DUAL to FALLBACK mid-session.
        outRoll    = roll - (rollZeroOffset + rollAutoCorrection);  // BNO roll with offset correction
        outPitch   = pitch;

        // No dual solution right now (either genuinely degraded, or
        // GnssMode == MODE_SINGLE_IMU where a dual solution never
        // exists at all) — rawDualRoll is stale/last-known, flag it
        // invalid so Teensy Tool can show "--" instead of a misleading
        // frozen number.
        rawImuRoll       = outRoll;
        rawDualRollValid = false;
    }

    dtostrf(outHeading, 6, 2, imuHeading);
    dtostrf(outRoll,    6, 2, imuRoll);
    dtostrf(outPitch,   6, 2, imuPitch);
    strcpy(imuYawRate, "0");
}


// =====================================================================
// BuildNmea
// =====================================================================
// Assembles $PAOGI (dual-capable modes) or $PANDA (MODE_SINGLE_IMU)
// from parsed GGA/VTG fields and imuHandler strings. Sends via USB
// serial (if sendUSB) and UDP (if Ethernet_running).
//
// Fields 1-11 (time..speed) are IDENTICAL between PAOGI and PANDA —
// only the sentence name and the source of fields 12-15 differ (dual
// heading/roll vs pure-IMU heading/roll, both already selected upstream
// by imuHandler() into imuHeading/imuRoll/imuPitch/imuYawRate). This
// restores PANDA support that existed in Chris Kinal's original
// firmware (a single `makeOGI` bool) but was silently dropped when the
// fallback engine was built on top of it — see TFF architecture
// reference for the full history. Restoring it is this one branch;
// nothing else in this function changes.
//
// $PAOGI,time,lat,NS,lon,EW,fix,sats,hdop,alt,age,knots,hdg,roll,pitch,yawrate*CS
// $PANDA,time,lat,NS,lon,EW,fix,sats,hdop,alt,age,knots,hdg,roll,pitch,yawrate*CS
// word:  [1]  [2]  [3][4] [5] [6] [7] [8]  [9] [10] [11] [12] [13] [14]  [15]
// =====================================================================
void BuildNmea()
{
    if (GnssMode == MODE_SINGLE_IMU) strcpy(nmea, "$PANDA,");
    else                              strcpy(nmea, "$PAOGI,");
    strcat(nmea, fixTime);    strcat(nmea, ",");
    strcat(nmea, latitude);   strcat(nmea, ",");
    strcat(nmea, latNS);      strcat(nmea, ",");
    strcat(nmea, longitude);  strcat(nmea, ",");
    strcat(nmea, lonEW);      strcat(nmea, ",");
    strcat(nmea, fixQuality); strcat(nmea, ",");
    strcat(nmea, numSats);    strcat(nmea, ",");
    strcat(nmea, HDOP);       strcat(nmea, ",");
    strcat(nmea, altitude);   strcat(nmea, ",");
    strcat(nmea, ageDGPS);    strcat(nmea, ",");
    strcat(nmea, speedKnots); strcat(nmea, ",");
    strcat(nmea, imuHeading); strcat(nmea, ",");
    strcat(nmea, imuRoll);    strcat(nmea, ",");
    strcat(nmea, imuPitch);   strcat(nmea, ",");
    strcat(nmea, imuYawRate);
    strcat(nmea, "*");
    CalculateChecksum();
    strcat(nmea, "\r\n");

    if (sendUSB) SerialAOG.print(nmea);

    if (Ethernet_running)
    {
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write(nmea, strlen(nmea));
        Eth_udpPAOGI.endPacket();
    }
}


// =====================================================================
// CalculateChecksum
// =====================================================================
// NMEA XOR checksum of characters between '$' and '*', appended as
// two uppercase hex digits to the nmea buffer.
// =====================================================================
void CalculateChecksum()
{
    uint8_t sum = 0;
    for (int i = 1; nmea[i] != '*' && nmea[i] != '\0'; i++)
        sum ^= (uint8_t)nmea[i];

    char hex[3] = { asciiHex[sum >> 4], asciiHex[sum & 0x0F], '\0' };
    strcat(nmea, hex);
}


// =====================================================================
// checkImuWatchdog
// =====================================================================
// Called from loop() every iteration. Detects a frozen IMU (no data
// for IMU_TIMEOUT_MS ms) and attempts a non-blocking soft restart.
// Function name kept as-is (predates TM171 support) — the timeout
// detection below is already IMU-agnostic (just checks a shared
// timestamp), only the actual recovery action branches on ImuType.
//
// BNO08x: tries bno08x.softReset() + enableGameRotationVector() — an
// SHTP reset over I2C (~100 ms including internal delays). If the I2C
// bus itself is unresponsive, falls back to full begin().
//
// TM171: no I2C bus to probe — a UART timeout more plausibly means the
// module lost power/got unplugged than a protocol-level hang, so
// recovery is just re-issuing begin() on the UART and resetting our
// own frame parser state machine (zIMU_TM171.ino) back to a known
// state, in case a partial frame was mid-flight when data stopped.
//
// After IMU_MAX_RETRIES failures, sets useIMU = false and the
// system continues in dual-only mode for the rest of the session —
// this part is identical for either IMU type.
// =====================================================================
// =====================================================================
// updateDiagnostics — the periodic (non-WAS) half of the health
// monitoring described in the block comment above setup() in the main
// .ino. WAS plausibility is checked inline in c00_Autosteer.ino instead
// (right where steerAngleActual is finalised each iteration) since it
// naturally belongs there; everything else — CAN bus timeouts, GNSS/
// RTK serial liveness, and the CAN content sanity check — is gathered
// here in one place instead of scattered across the many individual
// CAN message-parsing sites, since these are all simple "how long
// since we last saw something" comparisons that don't need to live
// next to where the data itself is parsed. Called once per loop()
// iteration. PURELY DIAGNOSTIC — see that same block comment for the
// scope this was deliberately kept within.
// =====================================================================
void updateDiagnostics()
{
    uint32_t now = millis();

    if (Brand != BRAND_NONE)   // CAN not even in use otherwise — a
                                 // "timeout" would be meaningless noise
    {
        // != 0 guard: matches the same pattern gnssWatchdogTimeout/
        // rtkRadioTimeout already use below, and for the same reason —
        // found missing here during a later audit pass, not present
        // from the start. lastXBusMsgTime starts at 0 (never yet
        // received), and without this guard, "now - 0" already
        // exceeds CAN_MSG_TIMEOUT_MS a few seconds after boot
        // regardless of whether the bus is actually healthy — a false
        // "timeout" for however long it genuinely takes the first real
        // message to arrive after boot, not a real communication
        // problem. Without the guard this could show up as a
        // spurious COMMUNICATION ERROR in the log every single boot.
        canKBusTimeout   = (lastKBusMsgTime   != 0) && ((now - lastKBusMsgTime)   > CAN_MSG_TIMEOUT_MS);
        canISOBusTimeout = (lastISOBusMsgTime != 0) && ((now - lastISOBusMsgTime) > CAN_MSG_TIMEOUT_MS);
        canVBusTimeout   = (lastVBusMsgTime   != 0) && ((now - lastVBusMsgTime)   > CAN_MSG_TIMEOUT_MS);

        // CAN content plausibility — deliberately a wide, generous
        // sanity bound rather than an exact enumeration of every
        // legitimate per-brand value (0, 16, 20, 80 are the ones
        // confirmed in use across the brand-specific parsing sites in
        // zCAN_All_Brands.ino) — enumerating every brand's exact valid
        // set precisely would need more source verification than was
        // available here, so this catches genuinely implausible
        // garbage (a corrupted byte far outside anything ever
        // legitimately sent) without risking false positives from a
        // brand-specific value this wasn't checked against.
        canContentImplausible = (steeringValveReady > 200);
    }
    else
    {
        canKBusTimeout = canISOBusTimeout = canVBusTimeout = canContentImplausible = false;
    }

    gnssWatchdogTimeout = (lastGnssByteTime != 0) && ((now - lastGnssByteTime) > GNSS_WATCHDOG_TIMEOUT_MS);
    rtkRadioTimeout     = (lastRtkByteTime  != 0) && ((now - lastRtkByteTime)  > RTK_RADIO_TIMEOUT_MS);
    hprTimeout          = (lastHprMsgTime   != 0) && ((now - lastHprMsgTime)   > HPR_WATCHDOG_TIMEOUT_MS);
}


/// <summary>
/// Detects a silently-stopped IMU (no data for IMU_TIMEOUT_MS) and
/// attempts to recover it — checked every loop() iteration, but does
/// nothing until data has genuinely gone quiet AND enough time has
/// passed since the last restart attempt (IMU_RESTART_DELAY_MS,
/// avoiding a tight restart-fail-restart loop hammering the bus).
/// Gives up permanently after IMU_MAX_RETRIES failed attempts,
/// falling back to useIMU=false (dual-only heading/roll from then on
/// — see updateAlphaControl()) rather than retrying forever against
/// hardware that's genuinely gone.
///
/// Handles the two IMU types differently, matching how each was
/// originally brought up: TM171 just needs its UART re-begin() and
/// parser state cleared (a simple, stateless restart); BNO08x first
/// PROBES the I2C bus with a bare address transmission before
/// attempting a full begin() + re-enabling the sensor report — a
/// probe-first approach that distinguishes "I2C bus itself is wedged"
/// from "sensor stopped reporting but the bus is fine", which would
/// otherwise need different recovery strategies but are handled with
/// the same begin() call either way here.
/// </summary>
void checkImuWatchdog()
{
    uint32_t now = millis();
    if (now - lastImuDataTime < IMU_TIMEOUT_MS) return;
    if (now - imuRestartTime  < IMU_RESTART_DELAY_MS) return;

    if (IMU_MAX_RETRIES > 0 && imuRetryCount >= IMU_MAX_RETRIES)
    {
        useIMU = false;
        Serial.print(F("IMU: gave up after "));
        Serial.print(IMU_MAX_RETRIES);
        Serial.println(F(" attempts. Dual-only mode."));
        return;
    }

    imuRetryCount++;
    imuRestartTime = now;

    if (ImuType == IMU_TM171)
    {
        Serial.print(F("TM171: timeout — restart attempt "));
        Serial.println(imuRetryCount);
        TM171_setup();          // re-begin() the UART
        TM171_resetParser();    // clear any partial frame — see zIMU_TM171.ino
        lastImuDataTime = millis();
        imuHealthy      = false;
        imuRetryCount   = 0;
        Serial.println(F("  TM171 re-init OK"));
        return;
    }

    Serial.print(F("BNO08x: timeout — restart attempt "));
    Serial.println(imuRetryCount);

    ImuWire.beginTransmission(bno08xAddress);
    uint8_t error = ImuWire.endTransmission();

    if (error != 0)
    {
        // I2C bus unresponsive — try full re-initialisation
        Serial.println(F("  I2C probe failed, trying begin()..."));
        if (bno08x.begin(bno08xAddress, ImuWire))
        {
            ImuWire.setClock(IMU_I2C_CLOCK);
            bno08x.enableGameRotationVector(REPORT_INTERVAL);
            lastImuDataTime = millis();
            imuHealthy      = false;
            imuRetryCount   = 0;
            Serial.println(F("  begin() OK"));
        }
        else { Serial.println(F("  begin() failed — will retry")); }
        return;
    }

    // I2C alive — soft reset via SHTP protocol (~100 ms)
    bno08x.softReset();
    bno08x.enableGameRotationVector(REPORT_INTERVAL);
    lastImuDataTime = millis();
    imuHealthy      = false;
    imuRetryCount   = 0;
    Serial.println(F("  softReset() OK"));
}

// =====================================================================
// readResetCause — decodes the SRC_SRSR register to report why the
// Teensy last started: power-on, watchdog-triggered, or software-
// triggered. Confirmed against the i.MX RT1062 reference manual (via
// the PJRC Teensy forum thread on reading reset cause on Teensy 4.1 —
// SRC_SRSR bit names, not guessed) rather than assumed. Called once,
// early in setup(), before anything else could plausibly touch this
// register. The "SRC_SRSR = SRC_SRSR;" line is the confirmed idiom for
// clearing the register after reading it (write-1-to-clear semantics),
// so the NEXT reset's cause isn't confused by bits left set from this
// one.
//
// Purely informational — reported once via PDIAG at boot and never
// touched again; distinguishing "the board rebooted because someone
// power-cycled it" from "the board rebooted because the watchdog fired
// after a hang" is exactly the kind of thing that's invisible from
// Teensy Tool's side otherwise (both look identical: a signal loss
// followed by recovery), but meaningfully different for troubleshooting.
// =====================================================================
void readResetCause()
{
    uint32_t srsr = SRC_SRSR;
    SRC_SRSR = SRC_SRSR;   // confirmed clear-on-read idiom, not a no-op

    if (srsr & SRC_SRSR_WDOG_RST_B)              resetCause = 2;  // watchdog
    else if (srsr & SRC_SRSR_LOCKUP_SYSRESETREQ) resetCause = 3;  // software
    else if (srsr & SRC_SRSR_IPP_RESET_B)        resetCause = 1;  // power-on
    else                                          resetCause = 0;  // unknown/other

    Serial.print(F("Reset cause = "));
    Serial.println(resetCause);
}


// =====================================================================
// sendDiagnostics
// =====================================================================
// Broadcasts $PDIAG to UDP port 5555 every 2 s, for Teensy Tool.
//
// $PDIAG,MODE,SATS_M,SATS_S,HPR_SATS,SOL,HDG_OFF,H_ALPHA,R_ALPHA,
//        INIT_H,INIT_R,DUAL_PCT,SATS_FULL,ROLL_ZERO,DUAL_HOLD_S,
//        DUAL_RAMP_S,IMU_AXIS,ROLL_INVERT,WAS_LEFT,WAS_RIGHT,
//        UTURN_STRENGTH,STEER_ACTUAL,BRAND,GNSS_MODE,DUAL_ROLL_RAW,
//        IMU_ROLL_RAW,ROLL_VALID*CS
//
//   Field          Description
//   -------------  ---------------------------------------------------
//   MODE           "DUAL" or "FALLBACK" (based on solQuality)
//   SATS_M         master antenna satellite count (from GGA)
//   SATS_S         slave antenna satellite count (from GPGGAH — only
//                  meaningful in GnssMode==MODE_UM982; stays at its
//                  last/default value in the other modes, since they
//                  don't use GGAH at all — harmless unused placeholder
//                  rather than something actively misleading)
//   HPR_SATS       satellites used in the heading solution, -1 if not
//                  applicable to the current source (see GnssMode)
//   SOL            heading solution quality (4=fixed, 5=float, 0=none)
//   HDG_OFF        current BNO-to-dual heading offset, degrees
//   H_ALPHA        current HEADING_ALPHA (auto-adjusted, 0=pure dual,
//                  1=pure BNO)
//   R_ALPHA        current ROLL_ALPHA (auto-adjusted)
//   INIT_H         initial/floor heading alpha (stored in EEPROM)
//   INIT_R         initial/floor roll alpha (stored in EEPROM)
//   DUAL_PCT       (1-H_ALPHA)*100 — 100=full dual, 0=full BNO
//   SATS_FULL      heading-sats value considered "full" dual confidence
//   ROLL_ZERO      BNO roll offset (degrees), matches BNO to dual roll
//   DUAL_HOLD_S    reconnect HOLD phase duration, seconds
//   DUAL_RAMP_S    reconnect RAMP phase duration, seconds
//   IMU_AXIS       which BNO axis is used for roll (0=X, 1=Y, 2=Z)
//   ROLL_INVERT    BNO roll sign (1 or -1)
//   WAS_LEFT       WAS left end-stop angle, degrees
//   WAS_RIGHT      WAS right end-stop angle, degrees
//   UTURN_STRENGTH U-turn dual boost strength (1=off .. 100=max)
//   STEER_ACTUAL   current steering angle from PID (degrees)
//   BRAND          CAN steering brand: 0-7 = brand, 8 = BRAND_NONE
//                  (no CAN, classic PWM/relay — see zCAN_All_Brands.ino)
//   GNSS_MODE      1=UM982, 2=Single+IMU, 3=Dual single receivers (TFF)
//   DUAL_ROLL_RAW  pure dual-sourced roll (pre-blend), degrees — see
//                  Teensy Tool's IMU tab "Dual Roll vs IMU Roll" panel
//   IMU_ROLL_RAW   pure BNO-sourced roll (pre-blend, post rollZeroOffset)
//   ROLL_VALID     1 if DUAL_ROLL_RAW is current/trustworthy right now,
//                  0 if it's stale (no current dual solution — always 0
//                  in GnssMode==MODE_SINGLE_IMU, since a dual solution
//                  never exists there at all)
//
// Fields 23-26 (GNSS_MODE onward) were appended on top of the original
// UM982 Fallback v0.7 22-field layout rather than restructuring it into
// the "core + mode-tagged tail" design sketched in the TFF architecture
// reference — lower risk, since Teensy Tool's existing Operation-tab
// parsing of fields 1-22 needed no changes at all. A full core+tail
// redesign remains a possible future cleanup, not done here.
// =====================================================================
void sendDiagnostics()
{
    const char* mode = (solQuality >= HPR_QUALITY_MIN) ? "DUAL" : "FALLBACK";
    int dualPct = (int)((1.0f - HEADING_ALPHA) * 100.0f + 0.5f);

    // Raw age (seconds since last successfully-parsed HPR sentence),
    // separate from the hprTimeout boolean above — added specifically
    // so Teensy Tool can show a live, continuously-updating "last HPR:
    // 0.3s ago" during active troubleshooting, without needing to wait
    // for the (deliberately generous, field-tuned) 8s hprTimeout
    // threshold to actually trip. -1.0 sentinel for "never received
    // at all this boot" (lastHprMsgTime == 0), rather than sending a
    // huge, meaningless "millis() since epoch 0" value.
    float hprAgeSeconds = (lastHprMsgTime == 0)
        ? -1.0f
        : (millis() - lastHprMsgTime) / 1000.0f;

    char buf[400];
    // Buffer sizing re-checked HERE, before adding the 12 new
    // Keya-auto-zero fields below (wasZeroDone, the zero-point in
    // degrees, and all ten azParams values) — the block comment
    // directly below this one already documents ONE past instance of
    // this exact buffer needing a resize after fields kept being
    // added without the size being revisited; deliberately not
    // repeating that mistake a second time. The 12 new fields add
    // roughly 60 characters of worst-case width on top of the
    // previous 242-character calculation (2 for wasZeroDone, ~8 for
    // the zero-point degrees value, ~6 each for the seven float
    // azParams fields, ~5 each for the two integer time fields, 2
    // each for the two boolean use-flags) — comfortably inside 400
    // with real headroom, not the bare minimum.
    // Buffer sizing check (re-verified, not just assumed, after
    // discovering this needed fixing): with all 46 fields now present,
    // a rigorous worst-case-width calculation (every %s/%.2f/%.4f/%.1f/
    // %d field at its own maximum realistic width, computed in code
    // rather than hand-estimated) gives up to 242 characters — already
    // exceeding the previous 240-byte size by 2 bytes in the absolute
    // worst case, even though typical real values stay near ~160 and
    // rarely trigger it. Found via a direct comparison against an
    // earlier saved snapshot (27 fields, same 240-byte buf, safely
    // sufficient then) that revealed buf's size was never revisited as
    // fields kept being added across several later sessions — sized
    // here with real headroom instead, not the bare minimum that
    // happens to survive today's field count.
    snprintf(buf, sizeof(buf),
             "$PDIAG,%s,%d,%d,%d,%d,%.2f,%.4f,%.4f,%.4f,%.4f,%d,%d,%.2f,%.1f,%.1f,%d,%d,%.1f,%.1f,%d,%.1f,%d,%d,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%d,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%d,%d,%.3f,%d,%.3f,%.4f",
             mode, satsMaster, satsSlave, hprSats, solQuality,
             (float)headingOffset,
             HEADING_ALPHA, ROLL_ALPHA,
             initialHeadingAlpha, initialRollAlpha,
             // rollZeroOffset + rollAutoCorrection, not rollZeroOffset
             // alone — the EFFECTIVE, currently-applied offset,
             // matching exactly what Teensy Tool's "Current" display
             // should show regardless of whether the value came from
             // manual SETROLLZERO or the live auto-adjust correction.
             // When auto-adjust is off, rollAutoCorrection is always
             // 0.0, so this is identical to the old, raw rollZeroOffset
             // value in every case that mattered before this feature
             // existed — no behaviour change for anyone not using it.
             dualPct, satsSlaveFull, (rollZeroOffset + rollAutoCorrection),
             (float)dualHoldUpdates / 10.0f,
             (float)dualRampUpdates / 10.0f,
             imuAxis, rollInvert,
             wasLeft, wasRight,
             uTurnStrength,
             steerAngleActual,
             Brand,
             GnssMode,
             rawDualRoll,
             rawImuRoll,
             rawDualRollValid ? 1 : 0,
             ImuType,
             BoardSlot1, BoardSlot2,
             GnssPassthrough ? 1 : 0,
             // --- Purely diagnostic health fields, added together —
             // see updateDiagnostics() (this file) and the
             // EthernetLinkUp/resetCause declaration comments in the
             // main .ino for what each one means and why it exists.
             // "Nothing is controlled, only listened to" — none of
             // these change firmware behaviour, only what gets
             // reported.
             canKBusTimeout      ? 1 : 0,
             canISOBusTimeout    ? 1 : 0,
             canVBusTimeout      ? 1 : 0,
             canContentImplausible ? 1 : 0,
             gnssWatchdogTimeout ? 1 : 0,
             rtkRadioTimeout     ? 1 : 0,
             wasImplausible      ? 1 : 0,
             EthernetLinkUp      ? 1 : 0,
             resetCause,
             // IMU watchdog status — genuinely missed in the first
             // pass of this same health-monitoring addition, caught
             // afterward: imuHealthy/imuRetryCount already existed
             // (checkImuWatchdog(), predates this session entirely)
             // but were never reported anywhere before now. useIMU is
             // included alongside them specifically because it's what
             // lets Teensy Tool tell apart three states that would
             // otherwise all look identical as "imuHealthy == false":
             // actively retrying (useIMU still true), permanently
             // given up after IMU_MAX_RETRIES (useIMU false — a more
             // severe, no-longer-recovering state), and IMU_NONE mode
             // where no IMU was ever expected at all (ImuType == 2,
             // already reported above — imuHealthy is meaningless
             // there and should not be treated as a problem).
             imuHealthy    ? 1 : 0,
             imuRetryCount,
             useIMU        ? 1 : 0,
             // Keya CAN steering motor status — purely diagnostic,
             // "nothing is controlled, only listened to" like every
             // other health field above. Only meaningful when
             // MotorDriveType == MOTOR_DRIVE_KEYA; Teensy Tool should
             // check that field before treating keyaDetected/
             // keyaFaultActive as significant (a non-Keya installation
             // will just show 0/0/0 here, not an error).
             MotorDriveType,
             WasSource,
             keyaDetected    ? 1 : 0,
             keyaFaultActive ? 1 : 0,
             hprTimeout      ? 1 : 0,
             hprAgeSeconds,
             // Keya-as-WAS auto-zero — wasZeroDone first (the single
             // most important field: whether guidance is even
             // possible right now, see the watchdog gate in
             // c00_Autosteer.ino), then the current zero-point in
             // degrees, then all ten tunable parameters, in the same
             // order the SETAZxxx commands above list them.
             azGetWasZeroDone(),
             azGetZeroTicksDegrees(),
             azGetSpeedMin(),
             azGetYawRateMax(),
             azGetGpsHdgMax(),
             azGetTimeSlowMs(),
             azGetTimeFastMs(),
             azGetSpeedSlow(),
             azGetSpeedFast(),
             azGetUseBno(),
             azGetUseGps(),
             azGetBeta(),
             autoRollAdjust ? 1 : 0, rollAutoDeadband, rollAutoAlpha);

    uint8_t cs = 0;
    for (int i = 1; buf[i] != '\0'; i++) cs ^= (uint8_t)buf[i];

    // sentence's size is now RELATIVE to buf's own size (sizeof(buf) +
    // 20), not a separate, hardcoded number — this is the SECOND time
    // this exact class of bug has occurred: sentence was bumped
    // 220->240 earlier in this project's history when buf first grew,
    // then buf grew again (280->400, v0.3.10, adding the Keya
    // auto-zero PDIAG fields) without anyone remembering sentence
    // needed to grow again in step — caught this time by GCC's own
    // -Wformat-truncation warning, not by review. A hardcoded number
    // here will keep silently falling out of sync every time buf
    // grows for a good reason (more PDIAG fields); tying sentence's
    // size directly to sizeof(buf) removes the possibility of this
    // happening a third time — the +20 covers the "*XX\r\n\0" suffix
    // (up to 6 bytes) plus real headroom on top, not just the bare
    // minimum. snprintf() itself was never actually vulnerable to a
    // real memory overflow either time (it always truncates safely at
    // sizeof(sentence)) — the real risk both times was silent,
    // undersized-output truncation of the diagnostic sentence itself.
    char sentence[sizeof(buf) + 20];
    snprintf(sentence, sizeof(sentence), "%s*%02X\r\n", buf, cs);

    Eth_udpMonitor.beginPacket(Eth_ipDestination, portMonitorOut);
    Eth_udpMonitor.write(sentence, strlen(sentence));
    Eth_udpMonitor.endPacket();

}


// =====================================================================
// receiveMonitorCommands
// =====================================================================
// Reads UDP packets on port 5556. Handles plain-text commands from
// Teensy Tool. All numeric values are clamped to sane ranges and
// saved to EEPROM immediately; every command's effect is confirmed in
// the next $PDIAG transmission (see sendDiagnostics() above).
//
//   SETHEADINGALPHA:x.xxxx   initialHeadingAlpha, floor for HEADING_ALPHA
//   SETROLLALPHA:x.xxxx      initialRollAlpha, floor for ROLL_ALPHA
//   SETSATSFULL:N            hprSats value considered "full" dual signal
//   SETROLLZERO:x.xx         BNO roll offset (degrees)
//   SETDUALHOLD:x.x          reconnect HOLD phase duration (seconds)
//   SETDUALRAMP:x.x          reconnect RAMP phase duration (seconds)
//   SETIMUAXIS:N             which BNO axis drives roll (0=X, 1=Y, 2=Z)
//   SETROLLINVERT:N          BNO roll sign (1 or -1)
//   SETWASCURRENT:L / :R     capture current WAS angle as left/right
//                            end-stop (wasLeft / wasRight)
//   SETUTURNSTRENGTH:N       U-turn dual boost strength (1=off..100=max)
//   SETBRAND:N               CAN steering brand (0-7), or 8=BRAND_NONE
//                            for classic PWM/relay steering with zero
//                            CAN bus activity — see zCAN_All_Brands.ino
//                            and BRAND_NONE in the main .ino. Reboot
//                            required after changing this for CAN
//                            filters to take effect.
//   SETGNSSMODE:N             GNSS source (TFF): 1=UM982, 2=Single+IMU,
//                            3=Dual single receivers. Reboot required.
//   SETRECEIVERTYPE:N         Dual receiver pair (only used when
//                            GnssMode==3): 0=F9P, 1=UM980. Reboot
//                            required.
//   SETMOTORDRIVETYPE:N       0=PWM (Cytron/IBT2 signal choice made by
//                            AGO's own steerConfig.CytronDriver,
//                            unaffected by this setting), 2=Keya (CAN
//                            steering motor). No value 1. Reboot required.
//   SETWASSOURCE:N            0=Normal (physical WAS or CAN-brand valve
//                            feedback), 1=Keya encoder. Reboot required.
//   SETGNSSPASSTHROUGH:N      0/1. When 1, bypasses GnssMode and every
//                            fusion feature entirely — raw byte
//                            forwarding of whatever the receiver
//                            sends (e.g. $KSXT, self-fused by the
//                            receiver), restored from Chris Kinal's
//                            original "udpPassthrough". Reboot
//                            required.
//   SETIMUTYPE:N              Physical IMU (TFF): 0=BNO08x, 1=TM171,
//                            2=IMU_NONE ("No fallback function" —
//                            traditional dual-only, no IMU blend/
//                            fallback at all; PAOGI always sent from
//                            whatever the dual source reports). Reboot
//                            required.
//   SETBOARDSLOT1:N           Board Configuration, physical slot 1:
//                            0=Master, 1=Slave, 2=TM171, 3=Empty.
//                            Reboot required.
//   SETBOARDSLOT2:N           Board Configuration, physical slot 2:
//                            same values as SETBOARDSLOT1. The two
//                            slots must end up different, and at
//                            least one must be Master or Slave — not
//                            enforced live (swapping needs two
//                            commands), but validated and reset to
//                            AIO-default (Slot1=Master, Slot2=Slave)
//                            at the next boot if still invalid then.
//                            Reboot required.
//   SETFILTERROLL:N           Kalman filter for roll, 0/1. Live, no reboot.
//   SETFILTERHEADING:N        Kalman filter for heading, 0/1. Live, no reboot.
//   SETROLLKALMAN:mea,est,q   Roll Kalman tuning (3 floats). Reboot required.
//   SETHEADINGKALMAN:mea,est,q  Heading Kalman tuning. Reboot required.
// =====================================================================
void receiveMonitorCommands()
{
    int len = Eth_udpMonitor.parsePacket();
    if (len <= 0) return;

    char buf[64] = {};
    int n = Eth_udpMonitor.read(buf, sizeof(buf) - 1);
    if (n <= 0) return;
    buf[n] = '\0';

    if (strncmp(buf, "SETIMUAXIS:", 11) == 0)
    {
        int v = atoi(buf + 11);
        imuAxis = constrain(v, 0, 2);
        saveAlphasToEEPROM();
    }
    else if (strncmp(buf, "SETROLLINVERT:", 14) == 0)
    {
        int v = atoi(buf + 14);
        rollInvert = (v < 0) ? -1 : 1;
        saveAlphasToEEPROM();
    }
    else if (strncmp(buf, "SETWASMAX:", 10) == 0)
    {
        wasRight = constrain(atof(buf + 10), 5.0f, 90.0f);
        saveAlphasToEEPROM();
    }
    else if (strncmp(buf, "SETWASMIN:", 10) == 0)
    {
        wasLeft = constrain(atof(buf + 10), -90.0f, -5.0f);
        saveAlphasToEEPROM();
    }
    else if (strncmp(buf, "SETUTURNSTRENGTH:", 17) == 0)
    {
        uTurnStrength = constrain(atoi(buf + 17), 1, 100);
        saveAlphasToEEPROM();
    }
    else if (strncmp(buf, "SETWASCURRENT:", 14) == 0)
    {
        // Read current steerAngleActual and store as left or right limit
        // buf = "SETWASCURRENT:L" or "SETWASCURRENT:R"
        if (buf[14] == 'L') wasLeft  = constrain(steerAngleActual, -90.0f, -1.0f);
        else                wasRight = constrain(steerAngleActual,   1.0f,  90.0f);
        saveAlphasToEEPROM();
    }
    else if (strncmp(buf, "SETDUALHOLD:", 12) == 0)
    {
        float s = atof(buf + 12);
        dualHoldUpdates = constrain((int)(s * 10.0f), 0, 300);
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: dualHoldUpdates = "));
        Serial.println(dualHoldUpdates);
    }
    else if (strncmp(buf, "SETDUALRAMP:", 12) == 0)
    {
        float s = atof(buf + 12);
        dualRampUpdates = constrain((int)(s * 10.0f), 0, 300);
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: dualRampUpdates = "));
        Serial.println(dualRampUpdates);
    }
    else if (strncmp(buf, "SETROLLZERO:", 12) == 0)
    {
        // Set BNO roll offset to the specified value (degrees).
        // Applied in fallback mode: outRoll = BNO_roll - rollZeroOffset
        // Set to current BNO roll value on flat ground to zero out mounting offset.
        float v = atof(buf + 12);
        rollZeroOffset = constrain(v, -30.0f, 30.0f);
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: rollZeroOffset = "));
        Serial.println(rollZeroOffset, 2);
    }
    else if (strncmp(buf, "SETAUTOROLLADJUST:", 18) == 0)
    {
        bool newValue = (atoi(buf + 18) == 1);
        // Only fold rollAutoCorrection into rollZeroOffset when
        // TRANSITIONING from active to inactive — if it was already
        // off, rollAutoCorrection is already 0 (nothing to merge),
        // and re-sending the same "off" value shouldn't trigger an
        // unnecessary EEPROM write.
        if (autoRollAdjust && !newValue)
        {
            rollZeroOffset = constrain(rollZeroOffset + rollAutoCorrection, -30.0f, 30.0f);
            rollAutoCorrection = 0.0f;
            rollAutoSettleStartMs = 0;  // reset the settle timer too, for a clean restart if re-enabled later
        }
        autoRollAdjust = newValue;
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: autoRollAdjust = "));
        Serial.println(autoRollAdjust ? 1 : 0);
    }
    else if (strncmp(buf, "SETROLLAUTODEADBAND:", 20) == 0)
    {
        float v = atof(buf + 20);
        rollAutoDeadband = constrain(v, 0.02f, 2.0f);
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: rollAutoDeadband = "));
        Serial.println(rollAutoDeadband, 3);
    }
    else if (strncmp(buf, "SETROLLAUTOALPHA:", 17) == 0)
    {
        float v = atof(buf + 17);
        rollAutoAlpha = constrain(v, 0.0001f, 0.1f);
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: rollAutoAlpha = "));
        Serial.println(rollAutoAlpha, 4);
    }
    else if (strncmp(buf, "SETSATSFULL:", 12) == 0)
    {
        int v = atoi(buf + 12);
        satsSlaveFull = constrain(v, 1, 40);
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: satsSlaveFull = "));
        Serial.println(satsSlaveFull);
    }
    else if (strncmp(buf, "SETHEADINGALPHA:", 16) == 0)
    {
        // Sets the initial (minimum) heading alpha and saves to EEPROM.
        // The actual HEADING_ALPHA will be >= this value depending on quality.
        float v = constrain(atof(buf + 16), 0.0f, 1.0f);
        initialHeadingAlpha = v;
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: initialHeadingAlpha = "));
        Serial.println(initialHeadingAlpha, 4);
    }
    else if (strncmp(buf, "SETROLLALPHA:", 13) == 0)
    {
        float v = constrain(atof(buf + 13), 0.0f, 1.0f);
        initialRollAlpha = v;
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: initialRollAlpha = "));
        Serial.println(initialRollAlpha, 4);
    }
    else if (strncmp(buf, "SETBRAND:", 9) == 0)
    {
        // Selects the CAN steering brand (0-7) or BRAND_NONE (8) for
        // classic PWM/relay steering with zero CAN bus activity.
        // See BRAND_NONE comment block in 010_UM982_Fallback_CAN.ino
        // for the full brand list and rationale.
        int v = atoi(buf + 9);
        if (v >= 0 && v <= BRAND_NONE)
        {
            Brand = (uint8_t)v;
            saveAlphasToEEPROM();
            Serial.print(F("Monitor: Brand = "));
            Serial.println(Brand);
            Serial.println(F("  NOTE: reboot Teensy for CAN bus filters to apply."));
        }
        else
        {
            Serial.print(F("Monitor: SETBRAND out of range: "));
            Serial.println(v);
        }
    }
    else if (strncmp(buf, "SETGNSSMODE:", 12) == 0)
    {
        // Selects the GNSS source (TFF): 1=MODE_UM982, 2=MODE_SINGLE_IMU,
        // 3=MODE_DUAL (paired with DualReceiverType below to pick F9P
        // vs UM980). Same "takes effect after reboot" rule as SETBRAND
        // and for the same reason — the chosen source's _setup() (serial
        // port init, UBX/UNIHEADING2 parser reset, etc.) only runs once,
        // in setup(), never again while running.
        int v = atoi(buf + 12);
        if (v >= MODE_UM982 && v <= MODE_DUAL)
        {
            GnssMode = (uint8_t)v;
            saveAlphasToEEPROM();
            Serial.print(F("Monitor: GnssMode = "));
            Serial.println(GnssMode);
            Serial.println(F("  NOTE: reboot Teensy for the new GNSS source to take effect."));
        }
        else
        {
            Serial.print(F("Monitor: SETGNSSMODE out of range: "));
            Serial.println(v);
        }
    }
    else if (strncmp(buf, "SETRECEIVERTYPE:", 16) == 0)
    {
        // Selects which receiver pair to use when GnssMode == MODE_DUAL:
        // 0 = RX_F9P (u-blox, UBX-NAV-RELPOSNED), 1 = RX_UM980
        // (UnicoreComm, UNIHEADING2). Meaningless in the other two
        // modes, but always accepted/stored regardless of current
        // GnssMode — simpler than rejecting it contextually, and it
        // just sits unused in EEPROM until MODE_DUAL is selected. Same
        // reboot requirement as SETGNSSMODE.
        int v = atoi(buf + 16);
        if (v == RX_F9P || v == RX_UM980)
        {
            DualReceiverType = (uint8_t)v;
            saveAlphasToEEPROM();
            Serial.print(F("Monitor: DualReceiverType = "));
            Serial.println(DualReceiverType);
            Serial.println(F("  NOTE: reboot Teensy for this to take effect."));
        }
        else
        {
            Serial.print(F("Monitor: SETRECEIVERTYPE out of range: "));
            Serial.println(v);
        }
    }
    else if (strncmp(buf, "SETMOTORDRIVETYPE:", 18) == 0)
    {
        // 0=PWM (Cytron/IBT2 signal choice made separately by AGO's own
        // steerConfig.CytronDriver, unaffected by this setting), 2=Keya
        // (CAN). Deliberately no value 1 — an earlier version had a
        // separate IBT2/Cytron split here that motorDrive() never
        // actually read, so it silently did nothing; removed rather
        // than kept as dead UI. Reboot required — motorDrive()'s branch
        // and CAN_setup()'s Keya filter are only evaluated once, at
        // boot (see 020_TFF.ino).
        int v = atoi(buf + 18);
        if (v == MOTOR_DRIVE_PWM || v == MOTOR_DRIVE_KEYA)
        {
            MotorDriveType = (uint8_t)v;
            saveAlphasToEEPROM();
            Serial.print(F("Monitor: MotorDriveType = "));
            Serial.println(v);
            Serial.println(F("  NOTE: reboot Teensy for this to take effect."));
        }
        else
        {
            Serial.print(F("Monitor: SETMOTORDRIVETYPE out of range: "));
            Serial.println(v);
        }
    }
    else if (strncmp(buf, "SETWASSOURCE:", 13) == 0)
    {
        // 0=Normal (physical WAS or CAN-brand valve feedback, today's
        // existing behaviour), 1=Keya encoder. Reboot required — same
        // reason as SETMOTORDRIVETYPE: the branch in the WAS
        // calculation (c00_Autosteer.ino) is only meaningful once
        // keyaSetup()/CAN_setup() have run at boot.
        int v = atoi(buf + 13);
        if (v >= WAS_SOURCE_NORMAL && v <= WAS_SOURCE_KEYA)
        {
            WasSource = (uint8_t)v;
            saveAlphasToEEPROM();
            Serial.print(F("Monitor: WasSource = "));
            Serial.println(v);
            Serial.println(F("  NOTE: reboot Teensy for this to take effect."));
        }
        else
        {
            Serial.print(F("Monitor: SETWASSOURCE out of range: "));
            Serial.println(v);
        }
    }
    else if (strncmp(buf, "SETGNSSPASSTHROUGH:", 19) == 0)
    {
        // On/off toggle — see the GnssPassthrough declaration comment
        // in the main .ino for the full rationale. Reboot required,
        // same reason as GnssMode: the setup()/loop() dispatch this
        // controls only runs once/every-iteration in the compiled
        // path chosen at boot, not re-evaluated live.
        GnssPassthrough = (atoi(buf + 19) != 0);
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: GnssPassthrough = "));
        Serial.println(GnssPassthrough);
        Serial.println(F("  NOTE: reboot Teensy for this to take effect."));
    }
    else if (strncmp(buf, "SETBOARDSLOT1:", 14) == 0 || strncmp(buf, "SETBOARDSLOT2:", 14) == 0)
    {
        // Board Configuration — which physical connector (Serial7
        // "master" GNSS position, Serial2 "slave" GNSS heading) each
        // slot actually is: 0=Master, 1=Slave, 2=TM171, 3=Empty. See
        // BoardSlot1/BoardSlot2 declaration in the main .ino for the
        // full rationale. Reboot required — same reason as
        // SETIMUTYPE: resolveBoardSlots() and the GNSS/IMU setup()
        // dispatch it feeds only run once, in the main setup().
        //
        // Deliberately does NOT force-correct an invalid combination
        // here the way loadAlphasFromEEPROM() does at boot — swapping
        // Master and Slave between the two slots genuinely requires
        // two separate commands (Teensy Tool sends one per radio
        // button change), and the state is briefly, legitimately
        // invalid between them (e.g. both temporarily "Slave" for one
        // instant). Rejecting or auto-correcting mid-sequence would
        // fight the user during an ordinary swap. Only range-checks
        // and saves each value individually; warns (doesn't block) if
        // the two slots don't yet form a valid combination, and full
        // validation/fallback-to-AIO-default still applies at the next
        // boot via loadAlphasFromEEPROM(), so a truly bad end state
        // can never persist past a reboot.
        bool isSlot1 = (buf[12] == '1');
        // Bug found and fixed here too, same audit pass as the
        // SETGNSSPASSTHROUGH/SETMOTORDRIVETYPE strncmp length fixes:
        // "SETBOARDSLOT1:" is 14 characters (indices 0-13) — the
        // digit distinguishing slot 1 from slot 2 is at index 12, the
        // colon is at index 13. buf[13] (the old code) was always the
        // colon, for BOTH SETBOARDSLOT1 and SETBOARDSLOT2, so
        // isSlot1 was always false — every SETBOARDSLOT1 command
        // silently mis-set BoardSlot2 instead, and BoardSlot1 could
        // never actually be set via Teensy Tool at all.
        int v = atoi(buf + 14);
        if (v >= SLOT_MASTER && v <= SLOT_EMPTY)
        {
            if (isSlot1) BoardSlot1 = (uint8_t)v; else BoardSlot2 = (uint8_t)v;
            saveAlphasToEEPROM();
            Serial.print(F("Monitor: BoardSlot"));
            Serial.print(isSlot1 ? 1 : 2);
            Serial.print(F(" = "));
            Serial.println(v);
            if (BoardSlot1 == BoardSlot2)
                Serial.println(F("  WARNING: both slots are the same — set the other slot too before rebooting."));
            else if (BoardSlot1 != SLOT_MASTER && BoardSlot1 != SLOT_SLAVE &&
                     BoardSlot2 != SLOT_MASTER && BoardSlot2 != SLOT_SLAVE)
                Serial.println(F("  WARNING: no Master or Slave GNSS role assigned to either slot."));
            Serial.println(F("  NOTE: reboot Teensy for this to take effect."));
        }
        else
        {
            Serial.print(F("Monitor: SETBOARDSLOT out of range: "));
            Serial.println(v);
        }
    }
    else if (strncmp(buf, "SETIMUTYPE:", 11) == 0)
    {
        // Selects which physical IMU is in use: 0 = IMU_BNO08X,
        // 1 = IMU_TM171 (zIMU_TM171.ino), 2 = IMU_NONE ("No fallback
        // function" — traditional dual-only). Reboot required — same
        // reason as SETGNSSMODE/SETRECEIVERTYPE, since the chosen
        // sensor's setup() (I2C scan vs UART begin() vs nothing at all)
        // only runs once, in the main setup().
        int v = atoi(buf + 11);
        if (v == IMU_BNO08X || v == IMU_TM171 || v == IMU_NONE)
        {
            ImuType = (uint8_t)v;
            saveAlphasToEEPROM();
            Serial.print(F("Monitor: ImuType = "));
            Serial.println(ImuType);
            Serial.println(F("  NOTE: reboot Teensy for this to take effect."));
        }
        else
        {
            Serial.print(F("Monitor: SETIMUTYPE out of range: "));
            Serial.println(v);
        }
    }
    else if (strncmp(buf, "SETFILTERROLL:", 14) == 0)
    {
        // Kalman filter on/off for roll — a plain boolean checked at
        // each call site (see zzGNSS_*.ino), so this takes effect
        // immediately, no reboot needed — unlike the tuning numbers
        // below, which are baked into the filter object at construction.
        filterRoll = (atoi(buf + 14) != 0);
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: filterRoll = "));
        Serial.println(filterRoll);
    }
    else if (strncmp(buf, "SETFILTERHEADING:", 17) == 0)
    {
        filterHeading = (atoi(buf + 17) != 0);
        saveAlphasToEEPROM();
        Serial.print(F("Monitor: filterHeading = "));
        Serial.println(filterHeading);
    }
    else if (strncmp(buf, "SETROLLKALMAN:", 14) == 0)
    {
        // Three comma-separated floats: mea,est,q — see the Kalman
        // filter comment block in the main .ino for what each means.
        // Baked into rollFilter at construction (main .ino) — reboot
        // required for a changed value to actually take effect, same
        // convention as GnssMode/Brand/ImuType.
        float mea, est, q;
        if (sscanf(buf + 14, "%f,%f,%f", &mea, &est, &q) == 3 &&
            mea > 0.0f && est > 0.0f && q > 0.0f)
        {
            rollMEA = mea; rollEST = est; rollQ = q;
            saveAlphasToEEPROM();
            Serial.print(F("Monitor: rollMEA/EST/Q = "));
            Serial.print(rollMEA, 4); Serial.print(F("/"));
            Serial.print(rollEST, 4); Serial.print(F("/"));
            Serial.println(rollQ, 4);
            Serial.println(F("  NOTE: reboot Teensy for this to take effect."));
        }
        else
        {
            Serial.println(F("Monitor: SETROLLKALMAN malformed or out of range"));
        }
    }
    else if (strncmp(buf, "SETHEADINGKALMAN:", 17) == 0)
    {
        float mea, est, q;
        if (sscanf(buf + 17, "%f,%f,%f", &mea, &est, &q) == 3 &&
            mea > 0.0f && est > 0.0f && q > 0.0f)
        {
            headingMEA = mea; headingEST = est; headingQ = q;
            saveAlphasToEEPROM();
            Serial.print(F("Monitor: headingMEA/EST/Q = "));
            Serial.print(headingMEA, 4); Serial.print(F("/"));
            Serial.print(headingEST, 4); Serial.print(F("/"));
            Serial.println(headingQ, 4);
            Serial.println(F("  NOTE: reboot Teensy for this to take effect."));
        }
        else
        {
            Serial.println(F("Monitor: SETHEADINGKALMAN malformed or out of range"));
        }
    }
    // ---------------------------------------------------------------
    // Keya-as-WAS auto-zero tuning (zKeyaAutoZero.ino, v0.3.10) — ten
    // separate commands, one per parameter, matching this project's
    // own established convention (one SETxxx per setting) rather than
    // combining them into a single multi-value command the way
    // SETROLLKALMAN/SETHEADINGKALMAN do — ten values in one command
    // would be far harder to validate individually and to show in a
    // UI than the Kalman filter's three. Each setter (azSet*, in
    // zKeyaAutoZero.ino) does its own constrain() and saves
    // immediately — no separate "reboot required" note, since
    // azParams is read fresh on every keyaAutoZeroUpdate() call, no
    // caching to invalidate.
    // ---------------------------------------------------------------
    else if (strncmp(buf, "SETAZSPEEDMIN:", 14) == 0)
    {
        azSetSpeedMin(atof(buf + 14));
        Serial.println(F("Monitor: az speedMin updated"));
    }
    else if (strncmp(buf, "SETAZYAWRATEMAX:", 16) == 0)
    {
        azSetYawRateMax(atof(buf + 16));
        Serial.println(F("Monitor: az yawRateMax updated"));
    }
    else if (strncmp(buf, "SETAZGPSHDGMAX:", 15) == 0)
    {
        azSetGpsHdgMax(atof(buf + 15));
        Serial.println(F("Monitor: az gpsHdgMax updated"));
    }
    else if (strncmp(buf, "SETAZTIMESLOW:", 14) == 0)
    {
        azSetTimeSlowMs(atoi(buf + 14));
        Serial.println(F("Monitor: az timeSlowMs updated"));
    }
    else if (strncmp(buf, "SETAZTIMEFAST:", 14) == 0)
    {
        azSetTimeFastMs(atoi(buf + 14));
        Serial.println(F("Monitor: az timeFastMs updated"));
    }
    else if (strncmp(buf, "SETAZSPEEDSLOW:", 15) == 0)
    {
        azSetSpeedSlow(atof(buf + 15));
        Serial.println(F("Monitor: az speedSlow updated"));
    }
    else if (strncmp(buf, "SETAZSPEEDFAST:", 15) == 0)
    {
        azSetSpeedFast(atof(buf + 15));
        Serial.println(F("Monitor: az speedFast updated"));
    }
    else if (strncmp(buf, "SETAZUSEBNO:", 12) == 0)
    {
        azSetUseBno(atoi(buf + 12));
        Serial.println(F("Monitor: az useBno updated"));
    }
    else if (strncmp(buf, "SETAZUSEGPS:", 12) == 0)
    {
        azSetUseGps(atoi(buf + 12));
        Serial.println(F("Monitor: az useGps updated"));
    }
    else if (strncmp(buf, "SETAZBETA:", 10) == 0)
    {
        azSetBeta(atof(buf + 10));
        Serial.println(F("Monitor: az beta updated"));
    }
}











// =====================================================================
// updateAlphaControl
// =====================================================================
// Called from loop() before the parser, so alpha is current when
// imuHandler() builds PAOGI this iteration.
//
// TARGET ALPHA is the highest of two independent calculations:
//
// 1. HPR QUALITY (from HPR solQuality):
//    solQuality < HPR_QUALITY_MIN → alpha = 1.0 immediately
//    solQuality >= HPR_QUALITY_MIN → alpha = initialAlpha (floor)
//
// 2. SATELLITE-BASED (from GPGGAH slave satellites in solution):
//    Linear interpolation from initialAlpha to 1.0 as satsSlave
//    drops from satsSlaveFull to 0.
//    satsTarget = initialAlpha + (1-initialAlpha) * (1 - sats/satsSlaveFull)
//
// Final target = max(statusTarget, satsTarget)
// This means either degradation signal alone can raise alpha,
// but neither can lower it below the other's value.
//
// User's initialAlpha always forms the floor — if initialAlpha > any
// computed target, initialAlpha wins.
//
// HEADING_ALPHA and ROLL_ALPHA are computed independently using their
// own initialHeadingAlpha / initialRollAlpha base values.
//
// ALPHA APPLICATION:
//   Increase (degradation): always immediate
//   Heading decrease (recovery): fast EMA (~1s at 10Hz)
//   Roll decrease (recovery): fast EMA (~1s) when fully healthy
// =====================================================================
void updateAlphaControl()
{
    // --- Compute targets independently for heading and roll ---

    float statusTargetH, statusTargetR;

    if (solQuality < HPR_QUALITY_MIN)
    {
        statusTargetH = 1.0f;
        statusTargetR = 1.0f;
    }
    else if (ggahReceived && satsSlave == 0)
    {
        // Slave antenna disconnected
        statusTargetH = initialHeadingAlpha;
        statusTargetR = initialRollAlpha;
    }
    else
    {
        // HPR quality OK — use initial alpha as status-based floor.
        // Alpha degradation is driven by satsSlave from GPGGAH (below).
        statusTargetH = initialHeadingAlpha;
        statusTargetR = initialRollAlpha;
    }

    // Satellite-based target: linear from initialAlpha to 1.0.
    // Uses hprSats (HPR words[5]) — satellites actually used in the
    // heading solution. This is the most direct quality indicator:
    // as fewer satellites contribute to heading, alpha increases.
    // No activation flag needed — hprSats starts at 0 and is only
    // updated when HPR sentences arrive with solQuality >= HPR_QUALITY_MIN.
    float satsTargetH = initialHeadingAlpha;
    float satsTargetR = initialRollAlpha;

    if (hprSats > 0)
    {
        float sats     = (float)min(hprSats, satsSlaveFull);
        float satsFrac = (satsSlaveFull > 0) ? sats / (float)satsSlaveFull : 1.0f;
        satsTargetH = initialHeadingAlpha
                      + (1.0f - initialHeadingAlpha) * (1.0f - satsFrac);
        satsTargetR = initialRollAlpha
                      + (1.0f - initialRollAlpha)    * (1.0f - satsFrac);
    }

    // Final target: highest of status and satellite signals
    float targetHeadingAlpha = max(statusTargetH, satsTargetH);
    float targetRollAlpha    = max(statusTargetR, satsTargetR);

    // Apply to actual alphas with asymmetric rate.
    //
    // DEGRADATION (alpha needs to increase): always immediate.
    //
    // RECOVERY (alpha needs to decrease):
    //
    // HEADING: jump directly when fully healthy. With CONFIG HEADING
    // FIXLENGTH + LENGTH, HPR heading is stable immediately after slave
    // reconnects (horizontal azimuth, no vertical convergence needed).
    //
    // ROLL: use stability detection instead of a fixed time constant.
    // HPR roll requires the vertical component of the RTK baseline to
    // converge — this is independent of FIXLENGTH and takes several
    // seconds after slave reconnect (vertical GPS accuracy is inherently
    // slower than horizontal). We check if HPR roll and BNO roll agree
    // fast EMA for smooth transition.
    // This is more correct than a fixed timer.

    // fullyHealthy: HPR solution quality is good.
    bool fullyHealthy = (solQuality >= HPR_QUALITY_MIN);

    // --- Dual reconnect stability timer ---
    //
    // When dual becomes available after fallback, HPR needs time to
    // stabilize before we trust it for steering. We:
    //   1. Hold alpha at 1.0 (pure BNO) for DUAL_STABLE_HOLD iterations (~3s)
    //   2. After hold: ramp alpha linearly to initialAlpha over
    //      DUAL_RAMP_UPDATES iterations (~5s)
    //
    // dualWasHealthy resets automatically when dual is lost (fullyHealthy=false),
    // which causes the hold+ramp sequence to restart on next reconnect.
    if (!fullyHealthy)
    {
        // Dual lost — reset all timers so next reconnect starts fresh.
        // Also reset rate gate history so it doesn't compare new HPR
        // against a stale pre-fallback value after rotation during fallback.
        dualWasHealthy = false;
        dualInHold     = false;
        dualInRamp     = false;
        prevHprHeading = -1.0;   // force rate gate to skip first update
        headingRateEMA =  0.0;   // clear rate history
    }
    else if (!dualWasHealthy)
    {
        // Dual just became available — start hold phase
        dualWasHealthy = true;
        dualInHold     = true;
        dualInRamp     = false;
        dualHoldTimer  = 0;
        targetHeadingAlpha = 1.0f;
        targetRollAlpha    = 1.0f;
    }
    else if (dualInHold)
    {
        // Hold phase: keep alpha at 1.0 until hold time elapsed
        uint32_t holdMs = (uint32_t)dualHoldUpdates * 100UL;  // iterations × 100ms
        targetHeadingAlpha = 1.0f;
        targetRollAlpha    = 1.0f;
        if (dualHoldTimer >= holdMs)
        {
            dualInHold    = false;
            dualInRamp    = true;
            dualRampTimer = 0;
        }
    }
    else if (dualInRamp)
    {
        // Ramp phase: linear interpolation from 1.0 to initialAlpha
        uint32_t rampMs = (uint32_t)dualRampUpdates * 100UL;
        float rampFrac  = min(1.0f, (float)dualRampTimer / (float)rampMs);
        targetHeadingAlpha = 1.0f + rampFrac * (initialHeadingAlpha - 1.0f);
        targetRollAlpha    = 1.0f + rampFrac * (initialRollAlpha    - 1.0f);
        if (dualRampTimer >= rampMs)
            dualInRamp = false;
    }
    // else: ramp complete — targetAlpha stays at initialAlpha

    // Heading alpha application
    if (HEADING_ALPHA < targetHeadingAlpha)
        HEADING_ALPHA = targetHeadingAlpha;   // degrade: immediate
    else
        HEADING_ALPHA += 0.5f * (targetHeadingAlpha - HEADING_ALPHA);  // fast follow

    // Roll alpha application
    if (ROLL_ALPHA < targetRollAlpha)
        ROLL_ALPHA = targetRollAlpha;         // degrade: immediate
    else
        ROLL_ALPHA += 0.5f * (targetRollAlpha - ROLL_ALPHA);  // fast follow

    // U-turn dual boost: polynomial modifier based on steer angle
    // f(x) = a(x-mid)^2 + 1, f(min)=f(max)=1/strength, f(mid)=1
    // Multiplies HEADING_ALPHA and ROLL_ALPHA → less alpha = more dual at full lock
    if (uTurnStrength > 1 && wasLeft < wasRight)
    {
        float mid      = (wasLeft + wasRight) * 0.5f;
        float halfSpan = wasRight - mid;
        float a        = (1.0f / (float)uTurnStrength - 1.0f) / (halfSpan * halfSpan);
        float x        = steerAngleActual - mid;
        float fx       = a * x * x + 1.0f;
        fx = constrain(fx, 1.0f / (float)uTurnStrength, 1.0f);
        HEADING_ALPHA *= fx;
        ROLL_ALPHA    *= fx;
    }

    // Clamp to valid range
    HEADING_ALPHA = constrain(HEADING_ALPHA, 0.0f, 1.0f);
    ROLL_ALPHA    = constrain(ROLL_ALPHA,    0.0f, 1.0f);
}


// =====================================================================
// =====================================================================
// EEPROM storage — persists alpha settings, IMU/roll calibration,
// dual-reconnect timing, U-turn boost, and CAN Brand across reboots.
// =====================================================================
// Uses the same EEPROM library as autosteer (no conflict — autosteer
// uses addresses 0-79, we start at ALPHA_EEPROM_ADDR = 100).
//
//   Addr 100: magic 0xA1B2C3D6 (detects uninitialised/stale-layout EEPROM)
//   Addr 104: initialHeadingAlpha (float, 4 bytes)
//   Addr 108: initialRollAlpha    (float, 4 bytes)
//   Addr 112: satsSlaveFull       (int,   4 bytes)
//   Addr 116: rollZeroOffset      (float, 4 bytes)
//   Addr 120: dualHoldUpdates     (int,   4 bytes)
//   Addr 124: dualRampUpdates     (int,   4 bytes)
//   Addr 128: imuAxis             (int,   4 bytes)
//   Addr 132: rollInvert          (int,   4 bytes)
//   Addr 136: wasLeft             (float, 4 bytes)
//   Addr 140: wasRight            (float, 4 bytes)
//   Addr 144: uTurnStrength       (int,   4 bytes)
//   Addr 148: Brand               (uint8_t, 1 byte) — CAN steering brand,
//             8 (BRAND_NONE) = classic PWM/relay, no CAN activity.
//             Added for the CAN-capable 010_UM982_Fallback_CAN variant;
//             harmlessly ignored by the CAN-less 000_UM982_Fallback.ino
//             build since that firmware never reads this address.
//
// Bump ALPHA_EEPROM_MAGIC whenever this layout changes, so old/stale
// EEPROM contents are detected and defaults are loaded instead of
// garbage.
// =====================================================================
#define ALPHA_EEPROM_ADDR  100
#define ALPHA_EEPROM_MAGIC 0xA1B2C3D6UL  // incremented when layout changed
#define BRAND_EEPROM_ADDR  (ALPHA_EEPROM_ADDR + 48)  // = 148
// TFF additions — GnssMode (1 byte) and DualReceiverType (1 byte),
// immediately after Brand. Same "harmlessly ignored by firmware that
// doesn't know about it" property as Brand had when it was added on
// top of the original alpha-only layout: a UM982 Fallback v0.7
// (CAN-only, no TFF) build would simply never read these two addresses.
#define GNSS_MODE_EEPROM_ADDR      (BRAND_EEPROM_ADDR + 1)  // = 149
#define DUAL_RX_TYPE_EEPROM_ADDR   (GNSS_MODE_EEPROM_ADDR + 1)  // = 150
// Further TFF additions — ImuType (1 byte) and the Kalman filter
// settings (2 bools + 6 floats = 26 bytes), immediately after
// DualReceiverType. Same "harmlessly ignored by older builds" property.
#define IMU_TYPE_EEPROM_ADDR       (DUAL_RX_TYPE_EEPROM_ADDR + 1)  // = 151
#define FILTER_ROLL_EEPROM_ADDR    (IMU_TYPE_EEPROM_ADDR + 1)      // = 152
#define FILTER_HEADING_EEPROM_ADDR (FILTER_ROLL_EEPROM_ADDR + 1)   // = 153
#define ROLL_MEA_EEPROM_ADDR       (FILTER_HEADING_EEPROM_ADDR + 1) // = 154
#define ROLL_EST_EEPROM_ADDR       (ROLL_MEA_EEPROM_ADDR + 4)      // = 158
#define ROLL_Q_EEPROM_ADDR         (ROLL_EST_EEPROM_ADDR + 4)      // = 162
#define HEADING_MEA_EEPROM_ADDR    (ROLL_Q_EEPROM_ADDR + 4)        // = 166
#define HEADING_EST_EEPROM_ADDR    (HEADING_MEA_EEPROM_ADDR + 4)   // = 170
#define HEADING_Q_EEPROM_ADDR      (HEADING_EST_EEPROM_ADDR + 4)   // = 174
// Board Configuration — which physical connector (Serial7 "master"
// GNSS position, Serial2 "slave" GNSS heading) each logical role is
// actually wired to. See BoardSlot1/BoardSlot2 declaration in the main
// .ino for the full rationale — this exists because TM171 can
// physically occupy either GNSS connector position instead of a real
// receiver (confirmed: same generic UART pins, no receiver-specific
// wiring), and different installations wire it differently.
#define BOARD_SLOT1_EEPROM_ADDR    (HEADING_Q_EEPROM_ADDR + 4)     // = 178
                                    // BUG FOUND AND FIXED: this was
                                    // previously "+ 1" instead of "+ 4"
                                    // — headingQ (the address right
                                    // before this one) is a float, 4
                                    // bytes, not 1. The old "+1" put
                                    // BoardSlot1/BoardSlot2/
                                    // GnssPassthrough INSIDE headingQ's
                                    // own 4-byte storage (its 2nd, 3rd,
                                    // and 4th bytes respectively) —
                                    // every routine save of headingQ
                                    // (part of the Kalman filter's own
                                    // periodic EEPROM writes) silently
                                    // overwrote GnssPassthrough with
                                    // one byte of that float's binary
                                    // representation, and vice versa.
                                    // Found via a full manual audit of
                                    // every EEPROM address computation
                                    // in this project after a user
                                    // report that GnssPassthrough kept
                                    // reverting to a nonsensical value
                                    // (60) and its checkbox couldn't
                                    // stay unchecked. Every other
                                    // Kalman-parameter-to-Kalman-
                                    // parameter transition above
                                    // correctly used "+ 4"; only this
                                    // one transition (Kalman -> the
                                    // newer, later-added settings) used
                                    // the wrong offset.
#define BOARD_SLOT2_EEPROM_ADDR    (BOARD_SLOT1_EEPROM_ADDR + 1)   // = 179
#define GNSS_PASSTHROUGH_EEPROM_ADDR (BOARD_SLOT2_EEPROM_ADDR + 1) // = 180
#define MOTOR_DRIVE_TYPE_EEPROM_ADDR (GNSS_PASSTHROUGH_EEPROM_ADDR + 1) // = 181
#define WAS_SOURCE_EEPROM_ADDR       (MOTOR_DRIVE_TYPE_EEPROM_ADDR + 1) // = 182
#define AUTOZERO_PARAMS_EEPROM_ADDR  (WAS_SOURCE_EEPROM_ADDR + 1)       // = 183, occupies 34 bytes (183-216)
#define AUTO_ROLL_ADJUST_EEPROM_ADDR (AUTOZERO_PARAMS_EEPROM_ADDR + 34) // = 217
#define ROLL_AUTO_DEADBAND_EEPROM_ADDR (AUTO_ROLL_ADJUST_EEPROM_ADDR + 1) // = 218, occupies 4 bytes (float)
#define ROLL_AUTO_ALPHA_EEPROM_ADDR    (ROLL_AUTO_DEADBAND_EEPROM_ADDR + 4) // = 222, occupies 4 bytes (float)
// next free address: 226

void loadAlphasFromEEPROM()
{
    uint32_t magic;
    EEPROM.get(ALPHA_EEPROM_ADDR, magic);

    if (magic == ALPHA_EEPROM_MAGIC)
    {
        EEPROM.get(ALPHA_EEPROM_ADDR + 4,  initialHeadingAlpha);
        EEPROM.get(ALPHA_EEPROM_ADDR + 8,  initialRollAlpha);
        EEPROM.get(ALPHA_EEPROM_ADDR + 12, satsSlaveFull);
        EEPROM.get(ALPHA_EEPROM_ADDR + 16, rollZeroOffset);
        EEPROM.get(ALPHA_EEPROM_ADDR + 20, dualHoldUpdates);
        EEPROM.get(ALPHA_EEPROM_ADDR + 24, dualRampUpdates);
        initialHeadingAlpha = constrain(initialHeadingAlpha, 0.0f, 1.0f);
        initialRollAlpha    = constrain(initialRollAlpha,    0.0f, 1.0f);
        satsSlaveFull       = constrain(satsSlaveFull, 1, 40);
        rollZeroOffset      = constrain(rollZeroOffset, -30.0f, 30.0f);
        dualHoldUpdates     = constrain(dualHoldUpdates, 0, 300);
        dualRampUpdates     = constrain(dualRampUpdates, 0, 300);
        EEPROM.get(ALPHA_EEPROM_ADDR + 28, imuAxis);
        EEPROM.get(ALPHA_EEPROM_ADDR + 32, rollInvert);
        EEPROM.get(ALPHA_EEPROM_ADDR + 36, wasLeft);
        EEPROM.get(ALPHA_EEPROM_ADDR + 40, wasRight);
        EEPROM.get(ALPHA_EEPROM_ADDR + 44, uTurnStrength);
        imuAxis       = constrain(imuAxis, 0, 2);
        rollInvert    = (rollInvert < 0) ? -1 : 1;
        wasLeft       = constrain(wasLeft,  -90.0f, -1.0f);
        wasRight      = constrain(wasRight,   1.0f,  90.0f);
        uTurnStrength = constrain(uTurnStrength, 1, 100);

        // Own load function in zKeyaAutoZero.ino — see that file's
        // own comment on why (compilation order: this file compiles
        // before zKeyaAutoZero.ino alphabetically, so the
        // AutoZeroParams type/azParams variable aren't visible here
        // yet — a function call works regardless).
        loadAutoZeroParamsFromEEPROM();

        // Auto Roll Adjust — three EEPROM-persisted settings; the
        // fourth, actively-drifting piece (rollAutoCorrection) is
        // deliberately RAM-only, never loaded/saved here — see this
        // file's own comment at the variable declarations for why.
        {
            uint8_t autoRollAdjustRaw;
            EEPROM.get(AUTO_ROLL_ADJUST_EEPROM_ADDR, autoRollAdjustRaw);
            autoRollAdjust = (autoRollAdjustRaw == 1);
            EEPROM.get(ROLL_AUTO_DEADBAND_EEPROM_ADDR, rollAutoDeadband);
            EEPROM.get(ROLL_AUTO_ALPHA_EEPROM_ADDR, rollAutoAlpha);
            rollAutoDeadband = constrain(rollAutoDeadband, 0.02f, 2.0f);
            rollAutoAlpha    = constrain(rollAutoAlpha, 0.0001f, 0.1f);
        }

        EEPROM.get(BRAND_EEPROM_ADDR, Brand);
        if (Brand > 8) Brand = BRAND_NONE;  // guard against garbage/uninitialised byte

        EEPROM.get(GNSS_MODE_EEPROM_ADDR, GnssMode);
        if (GnssMode < MODE_UM982 || GnssMode > MODE_DUAL) GnssMode = MODE_UM982;
        EEPROM.get(DUAL_RX_TYPE_EEPROM_ADDR, DualReceiverType);
        if (DualReceiverType > RX_UM980) DualReceiverType = RX_F9P;

        EEPROM.get(IMU_TYPE_EEPROM_ADDR, ImuType);
        if (ImuType > IMU_NONE) ImuType = IMU_BNO08X;

        // Board Configuration — load, then validate against the two
        // agreed rules (see BoardSlot1/BoardSlot2 in the main .ino for
        // the full rationale). Invalid combinations (corrupt EEPROM,
        // or a saved state from before this feature existed) reset to
        // the AIO-default rather than being left ambiguous.
        EEPROM.get(BOARD_SLOT1_EEPROM_ADDR, BoardSlot1);
        EEPROM.get(BOARD_SLOT2_EEPROM_ADDR, BoardSlot2);
        bool slotsRangeOk  = (BoardSlot1 <= SLOT_EMPTY) && (BoardSlot2 <= SLOT_EMPTY);
        bool slotsHaveGnss = slotsRangeOk &&
            ((BoardSlot1 == SLOT_MASTER || BoardSlot1 == SLOT_SLAVE) ||
             (BoardSlot2 == SLOT_MASTER || BoardSlot2 == SLOT_SLAVE));
        // Rule 1: the two slots must never be equal — this single
        // comparison already catches double-Master, double-Slave,
        // double-TM171, AND double-Empty, all at once, no separate
        // case needed for any of them.
        // Rule 2: at least one slot must be a real GNSS role.
        if (!slotsRangeOk || (BoardSlot1 == BoardSlot2) || !slotsHaveGnss)
        {
            BoardSlot1 = SLOT_MASTER;
            BoardSlot2 = SLOT_SLAVE;
        }

        EEPROM.get(GNSS_PASSTHROUGH_EEPROM_ADDR, GnssPassthrough);
        // Explicit normalize, not just trusted raw — added alongside
        // the BOARD_SLOT1_EEPROM_ADDR collision fix (see that define's
        // own comment): anyone upgrading from firmware with that bug
        // will have this address's PREVIOUS contents (a stray byte
        // from headingQ, or before this address existed at all)
        // freshly relocated here after reflashing, not a clean 0/1.
        // A raw bool read from garbage EEPROM is technically undefined
        // behaviour in C++ even though any later "? 1 : 0" ternary
        // would still normalize it before sending — cheap to make
        // this explicitly safe on load too, not just downstream.
        GnssPassthrough = (GnssPassthrough != false);

        EEPROM.get(MOTOR_DRIVE_TYPE_EEPROM_ADDR, MotorDriveType);
        if (MotorDriveType != MOTOR_DRIVE_PWM && MotorDriveType != MOTOR_DRIVE_KEYA)
            MotorDriveType = MOTOR_DRIVE_PWM;
        EEPROM.get(WAS_SOURCE_EEPROM_ADDR, WasSource);
        if (WasSource > WAS_SOURCE_KEYA) WasSource = WAS_SOURCE_NORMAL;
        EEPROM.get(FILTER_ROLL_EEPROM_ADDR,    filterRoll);
        EEPROM.get(FILTER_HEADING_EEPROM_ADDR, filterHeading);
        EEPROM.get(ROLL_MEA_EEPROM_ADDR,    rollMEA);
        EEPROM.get(ROLL_EST_EEPROM_ADDR,    rollEST);
        EEPROM.get(ROLL_Q_EEPROM_ADDR,      rollQ);
        EEPROM.get(HEADING_MEA_EEPROM_ADDR, headingMEA);
        EEPROM.get(HEADING_EST_EEPROM_ADDR, headingEST);
        EEPROM.get(HEADING_Q_EEPROM_ADDR,   headingQ);
        // Sanity-guard the floats against uninitialised/garbage EEPROM
        // (e.g. NaN, or a wildly out-of-range value from a byte pattern
        // that happens to decode as a huge float) — a bad Kalman
        // process-noise value could otherwise make the filter useless
        // or numerically unstable rather than just "not ideally tuned".
        if (!(rollMEA    > 0.0f && rollMEA    < 1000.0f)) rollMEA    = 1.0f;
        if (!(rollEST    > 0.0f && rollEST    < 1000.0f)) rollEST    = 1.0f;
        if (!(rollQ      > 0.0f && rollQ      < 1000.0f)) rollQ      = 0.01f;
        if (!(headingMEA > 0.0f && headingMEA < 1000.0f)) headingMEA = 1.0f;
        if (!(headingEST > 0.0f && headingEST < 1000.0f)) headingEST = 1.0f;
        if (!(headingQ   > 0.0f && headingQ   < 1000.0f)) headingQ   = 0.01f;

        Serial.print(F("EEPROM loaded: H="));
        Serial.print(initialHeadingAlpha, 4);
        Serial.print(F(" R="));
        Serial.print(initialRollAlpha, 4);
        Serial.print(F(" SatsFull="));
        Serial.print(satsSlaveFull);
        Serial.print(F(" RollZero="));
        Serial.print(rollZeroOffset, 2);
        Serial.print(F(" Brand="));
        Serial.print(Brand);
        Serial.print(F(" GnssMode="));
        Serial.print(GnssMode);
        Serial.print(F(" DualReceiverType="));
        Serial.println(DualReceiverType);
    }
    else
    {
        // First boot — use defaults
        initialHeadingAlpha = 0.0f;
        initialRollAlpha    = 0.0f;
        satsSlaveFull       = 7;
        rollZeroOffset      = 0.0f;
        dualHoldUpdates     = 30;
        dualRampUpdates     = 50;
        imuAxis             = 0;
        rollInvert          = -1;
        wasLeft             = -45.0f;
        wasRight            =  45.0f;
        uTurnStrength       = 1;   // 1=off, up to 100
        Brand               = BRAND_NONE;  // no CAN until deliberately configured
        GnssMode            = MODE_UM982;  // default: this project's proven mode
        DualReceiverType    = RX_F9P;
        ImuType             = IMU_BNO08X;  // default: this project's proven IMU
        filterRoll          = false;
        filterHeading       = false;
        rollMEA = rollEST = headingMEA = headingEST = 1.0f;
        rollQ   = headingQ = 0.01f;
        Serial.println(F("EEPROM uninitialised, using defaults"));
    }

    HEADING_ALPHA = initialHeadingAlpha;
    ROLL_ALPHA    = initialRollAlpha;
}

/// <summary>
/// Persists every setting loadAlphasFromEEPROM() (further up this
/// file) knows how to load — the write-side counterpart, called
/// after any SETxxx command changes one of these values. Writes the
/// ALPHA_EEPROM_MAGIC marker first, unconditionally — the same
/// "was this region ever actually written by us" signal
/// loadAlphasFromEEPROM() checks for before trusting anything else at
/// this address range, guarding against reading garbage from a
/// factory-fresh or previously-differently-used EEPROM.
/// </summary>
void saveAlphasToEEPROM()
{
    uint32_t magic = ALPHA_EEPROM_MAGIC;
    EEPROM.put(ALPHA_EEPROM_ADDR,      magic);
    EEPROM.put(ALPHA_EEPROM_ADDR + 4,  initialHeadingAlpha);
    EEPROM.put(ALPHA_EEPROM_ADDR + 8,  initialRollAlpha);
    EEPROM.put(ALPHA_EEPROM_ADDR + 12, satsSlaveFull);
    EEPROM.put(ALPHA_EEPROM_ADDR + 16, rollZeroOffset);
    EEPROM.put(ALPHA_EEPROM_ADDR + 20, dualHoldUpdates);
    EEPROM.put(ALPHA_EEPROM_ADDR + 24, dualRampUpdates);
    EEPROM.put(ALPHA_EEPROM_ADDR + 28, imuAxis);
    EEPROM.put(ALPHA_EEPROM_ADDR + 32, rollInvert);
    // Own save function in zKeyaAutoZero.ino — see
    // loadAutoZeroParamsFromEEPROM()'s comment above for why.
    saveAutoZeroParamsToEEPROM();
    EEPROM.put(ALPHA_EEPROM_ADDR + 36, wasLeft);
    EEPROM.put(ALPHA_EEPROM_ADDR + 40, wasRight);
    EEPROM.put(ALPHA_EEPROM_ADDR + 44, uTurnStrength);

    // Auto Roll Adjust — same three settings as loadAlphasFromEEPROM()
    // reads (see that function's own comment on why
    // rollAutoCorrection itself is deliberately excluded here).
    {
        uint8_t autoRollAdjustRaw = autoRollAdjust ? 1 : 0;
        EEPROM.put(AUTO_ROLL_ADJUST_EEPROM_ADDR, autoRollAdjustRaw);
        EEPROM.put(ROLL_AUTO_DEADBAND_EEPROM_ADDR, rollAutoDeadband);
        EEPROM.put(ROLL_AUTO_ALPHA_EEPROM_ADDR, rollAutoAlpha);
    }
    EEPROM.put(BRAND_EEPROM_ADDR,      Brand);
    EEPROM.put(GNSS_MODE_EEPROM_ADDR,    GnssMode);
    EEPROM.put(DUAL_RX_TYPE_EEPROM_ADDR, DualReceiverType);
    EEPROM.put(IMU_TYPE_EEPROM_ADDR,       ImuType);
    EEPROM.put(BOARD_SLOT1_EEPROM_ADDR,    BoardSlot1);
    EEPROM.put(BOARD_SLOT2_EEPROM_ADDR,    BoardSlot2);
    EEPROM.put(GNSS_PASSTHROUGH_EEPROM_ADDR, GnssPassthrough);
    EEPROM.put(MOTOR_DRIVE_TYPE_EEPROM_ADDR, MotorDriveType);
    EEPROM.put(WAS_SOURCE_EEPROM_ADDR, WasSource);
    EEPROM.put(FILTER_ROLL_EEPROM_ADDR,    filterRoll);
    EEPROM.put(FILTER_HEADING_EEPROM_ADDR, filterHeading);
    EEPROM.put(ROLL_MEA_EEPROM_ADDR,    rollMEA);
    EEPROM.put(ROLL_EST_EEPROM_ADDR,    rollEST);
    EEPROM.put(ROLL_Q_EEPROM_ADDR,      rollQ);
    EEPROM.put(HEADING_MEA_EEPROM_ADDR, headingMEA);
    EEPROM.put(HEADING_EST_EEPROM_ADDR, headingEST);
    EEPROM.put(HEADING_Q_EEPROM_ADDR,   headingQ);
    Serial.print(F("EEPROM saved: H="));
    Serial.print(initialHeadingAlpha, 4);
    Serial.print(F(" R="));
    Serial.print(initialRollAlpha, 4);
    Serial.print(F(" SatsFull="));
    Serial.print(satsSlaveFull);
    Serial.print(F(" RollZero="));
    Serial.print(rollZeroOffset, 2);
    Serial.print(F(" Brand="));
    Serial.print(Brand);
    Serial.print(F(" GnssMode="));
    Serial.print(GnssMode);
    Serial.print(F(" DualReceiverType="));
    Serial.println(DualReceiverType);
}

// =====================================================================
// errorHandler
// =====================================================================
// Called by NMEAParser on a malformed sentence. No action needed;
// the parser discards the bad sentence and continues automatically.
// =====================================================================
void errorHandler() {}
