/* =====================================================================
 * TFF (Teensy Flexible Firmware)  —  v0.3.20 for AgOpenGPS
 * Procedure by Daniel Nilsson, main coding by Claude AI
 * Teensy 4.1 AIO Board
 *
 * TFF is a new project with its own version history, starting at v0.1
 * — it is NOT a continuation of the UM982 Fallback Firmware's own
 * v0.1-v0.7 numbering. TFF was built by generalising that firmware's
 * GNSS/IMU fallback engine (alpha-blend, EEPROM persistence, BNO
 * watchdog, U-turn dual boost, COG-based heading offset calibration)
 * to work with multiple GNSS sources instead of UM982 only — see
 * "PROJECT LINEAGE" below for exactly what was carried over unchanged
 * versus newly built.
 *
 * LICENSE: GNU General Public License v3
 * =====================================================================
 *
 * PROJECT LINEAGE
 * ----------------
 * TFF v0.3.20 is built directly on top of, and reuses without modification:
 *
 *   UM982 Fallback Firmware v0.7 (own separate project, own version
 *   history — see that project's own files for its v0.1-v0.7 changelog)
 *     ├─ based on original firmware by Chris Kinal (2024), main branch
 *     ├─ v0.5: added IMU fallback capability (alpha-blend engine)
 *     ├─ v0.6: added optional CAN-bus steering output, adapted from
 *     │        AgOpenGPS-Official/Boards CANBUS firmware
 *     │        (MechanicTony/gunicsba branch)
 *     └─ v0.7: CAN WAS-source correction (Brand-aware physical vs CAN
 *              valve feedback) + V-Bus rate limiting (~50ms, matching
 *              AgOpenGPS's documented 40-80ms interval for CAN
 *              steering valve messages — was previously unthrottled)
 *
 * What TFF v0.3.20 changes relative to that v0.7 base:
 *   - The UM982-specific GNSS parsing (HPR_Handler, GGAH_Handler_Raw)
 *     was moved into its own file (zzGNSS_UM982.ino) unchanged, and
 *     three new GNSS sources were added alongside it as equal
 *     alternatives, selectable at runtime via GnssMode:
 *       zzGNSS_SingleIMU.ino  — single antenna + IMU, $PANDA output
 *       zzGNSS_DualF9P.ino    — 2x u-blox F9P, moving-base RTK
 *       zzGNSS_DualUM980.ino  — 2x UnicoreComm UM980, moving-base RTK
 *   - $PANDA output support was restored in BuildNmea() (it existed in
 *     Chris Kinal's original firmware but was dropped somewhere before
 *     UM982 Fallback v0.5 — see zHandlers.ino for the full history).
 *   - PDIAG gained a GNSS_MODE field plus a Dual-Roll/IMU-Roll live
 *     comparison pair, for Teensy Tool's IMU calibration tab.
 *   - v0.2: F9P roll calculation (zzGNSS_DualF9P.ino) gained a
 *     baseline > 1cm sanity guard, matching the official reference
 *     source's own condition (confirmed: "if (carrSoln == 2 &&
 *     baseline > 1)") — missed when the roll formula was originally
 *     ported. Without it, a baseline reading near-but-not-exactly zero
 *     (e.g. a transient/unstable reading right at the moment RTK
 *     first locks to fixed) could produce an absurd, maxed-out roll
 *     value via division by a tiny denominator, rather than being
 *     rejected as untrustworthy the way the reference implementation
 *     does.
 *   - v0.3: Board Configuration — BoardSlot1/BoardSlot2 let an
 *     installer say which physical connector (the "master" Serial7 or
 *     "slave" Serial2 GNSS position, confirmed against the official
 *     AIO v4 Firmware's own source, not just a schematic) has which
 *     role, since TM171 can physically occupy either GNSS position
 *     instead of a real receiver. See resolveBoardSlots() below.
 *   - v0.3: GNSS Passthrough — a content-agnostic raw byte forwarding
 *     mode, restored from Chris Kinal's original "udpPassthrough",
 *     for users who'd rather trust a receiver's own onboard fusion
 *     (e.g. UM982 outputting $KSXT) outright instead of any of TFF's
 *     fallback engine. See zGnssPassthrough.ino.
 *   - v0.3: diagnostic health monitoring — purely observational, never
 *     acts on anything (autosteer engagement decisions are unaffected
 *     by any of it): WAS plausibility (out-of-calibrated-range or
 *     sudden-jump detection), CAN message timeout per bus plus a
 *     content sanity check, GNSS/RTK serial watchdogs, live Ethernet
 *     link tracking (EthernetLinkUp — a real, previously-silent gap:
 *     Ethernet_running was only ever set once at boot, so a mid-
 *     session cable pull was never reflected anywhere), IMU watchdog
 *     status (imuHealthy/imuRetryCount/useIMU), and Teensy reset cause
 *     (read once at boot via SRC_SRSR). All reported through PDIAG,
 *     which grew from 27 to 46 fields across this and the following
 *     change. See updateDiagnostics() in zHandlers.ino.
 *   - v0.3: Keya CAN steering motor support (zKeya.ino) — protocol
 *     verified against Keya's own official manual (not just community
 *     source, which turned out to disagree with itself on speed
 *     scale/byte layout across different repos). MotorDriveType
 *     (IBT2/Cytron/Keya) and WasSource (physical WAS or CAN-brand
 *     valve feedback / Keya's own encoder) are new, independent
 *     TFF-own settings — deliberately NOT added to the shared
 *     steerConfig struct, since that struct's bit layout is a fixed
 *     protocol contract with AGO itself (synced via PGN 251/252).
 *     Encoder-as-WAS reuses steerSensorCounts, the same calibration
 *     value already used for physical WAS, confirmed against
 *     community source to use an identical formula.
 *   - v0.3.1: two buffer-size bugs found and fixed, both discovered
 *     through a user report of "position reaches AGO but roll
 *     doesn't", traced by rigorously recomputing worst-case field
 *     widths in code rather than assuming prior estimates still held
 *     as PDIAG kept growing across several sessions:
 *       - sendDiagnostics()'s buf[240]/sentence[256] (zHandlers.ino):
 *         worst-case PDIAG content had grown to 242 characters (46
 *         fields) — 2 bytes over buf's old size. snprintf() truncates
 *         safely, but a truncated sentence still corrupts the
 *         checksum and drops trailing fields. Resized to buf[280]/
 *         sentence[300].
 *       - BuildNmea()'s nmea[120] (zHandlers.ino): worst-case $PAOGI/
 *         $PANDA content is up to 146 characters — 26 bytes over, and
 *         built with strcat(), which has NO bounds checking at all
 *         (unlike snprintf() elsewhere in this project) — a genuine
 *         buffer overflow, not a safe truncation. imuRoll is
 *         appended 13th of 16 fields, late enough that an overflow
 *         starting around there explains position (appended earlier)
 *         arriving intact while roll doesn't. This exact 120-byte
 *         size was inherited unchanged all the way from the original
 *         v0.7 baseline — never triggered in that project's own 20-
 *         hour field test, most likely because the overflow only
 *         manifests when several fields simultaneously hit their
 *         maximum width (long coordinates, negative altitude,
 *         negative roll/IMU values together), which is location- and
 *         situation-dependent, not universal. Resized to nmea[180].
 *   - v0.3.3: a genuine EEPROM address collision found and fixed —
 *     BOARD_SLOT1_EEPROM_ADDR was computed as HEADING_Q_EEPROM_ADDR +
 *     1, when headingQ (the Kalman parameter immediately before it)
 *     is a float — 4 bytes, not 1. BoardSlot1/BoardSlot2/
 *     GnssPassthrough landed inside headingQ's own 4-byte storage
 *     (its 2nd, 3rd, and 4th bytes respectively): every routine save
 *     of headingQ (part of the Kalman filter's own periodic EEPROM
 *     writes) silently overwrote GnssPassthrough with one byte of
 *     that float's binary representation, and vice versa. Found via
 *     a full manual audit of every EEPROM address computation in this
 *     project, triggered by a user report that GnssPassthrough kept
 *     reading back as a nonsensical constant value and its Teensy
 *     Tool checkbox couldn't stay unchecked — bool(that value) was
 *     always truthy in the UI's own sync logic, forcing it back on
 *     every ~0.5s poll cycle regardless of what the user just clicked.
 *     Fixed at the source (HEADING_Q_EEPROM_ADDR + 4), which also
 *     correctly shifts BoardSlot1/BoardSlot2/GnssPassthrough/
 *     MotorDriveType/WasSource to the addresses their own comments
 *     already said they should be at (178-182) — those comments were
 *     right all along; only the actual code for the first of the five
 *     had the wrong offset. GnssPassthrough's own EEPROM load also
 *     gained an explicit bool normalization, since anyone upgrading
 *     from the buggy addressing will have this now-correct address's
 *     previous contents (a stray float byte, or pre-existing garbage)
 *     freshly relocated here after reflashing, not guaranteed to
 *     already be a clean 0/1.
 *   - v0.3.4: two commands' strncmp() length arguments were off by
 *   one, found by writing a script to check every strncmp(buf,
 *   "LITERAL", N) call's N against the literal's own actual length,
 *   rather than trusting each one had been counted correctly by eye:
 *     - SETGNSSPASSTHROUGH: used length 20; "SETGNSSPASSTHROUGH:" is
 *       19 characters.
 *     - SETMOTORDRIVETYPE: used length 19; "SETMOTORDRIVETYPE:" is
 *       18 characters.
 *   In both cases the extra length made strncmp() compare one byte
 *   past the colon — the literal's own null terminator — against
 *   whatever digit the actual command's value byte held (e.g. '0' or
 *   '2'), which can never match. The practical effect: neither
 *   command has ever actually been recognised by firmware since the
 *   day it was written — every SETGNSSPASSTHROUGH/SETMOTORDRIVETYPE
 *   sent from Teensy Tool silently fell through to "unrecognised
 *   command" with no visible error, and the corresponding EEPROM
 *   variable never changed from whatever it already held. Found
 *   after a user reported the GNSS Passthrough checkbox reverting
 *   immediately no matter how many times they unchecked it — not a
 *   delayed EEPROM-collision-style symptom (already ruled out by
 *   then), but instantaneous, which pointed at the command handler
 *   itself rather than anything happening later. The atoi() offset
 *   right after each strncmp() used the same wrong number, so both
 *   are fixed together (18/19 respectively) — correcting only the
 *   strncmp() length while leaving a mismatched atoi() offset would
 *   have "fixed" recognition while then parsing the value from the
 *   wrong starting character.
 *   - v0.3.5: a fourth command-parsing bug, same audit family as
 *   v0.3.4's two, found by extending the same systematic check to
 *   every direct buf[N] character comparison in this file (not just
 *   strncmp() lengths): SETBOARDSLOT1/SETBOARDSLOT2's shared handler
 *   read buf[13] to distinguish which of the two commands had been
 *   sent, expecting the '1' or '2' digit there — but "SETBOARDSLOT1:"
 *   is 14 characters (indices 0-13), so the digit is at index 12 and
 *   index 13 is the colon. buf[13] was the colon for BOTH commands,
 *   so the isSlot1 check was always false: every SETBOARDSLOT1 sent
 *   from Teensy Tool silently mis-set BoardSlot2 instead, and
 *   BoardSlot1 could never actually be changed via Teensy Tool at
 *   all. Fixed (buf[12]). A full search of every direct buf[N]
 *   comparison in this file found only one other instance
 *   (SETWASCURRENT's buf[14]), independently re-verified correct.
 *   - v0.3.6: a new, ADDED watchdog (hprTimeout) rather than a
 *   modification of gnssWatchdogTimeout — deliberately left that one
 *   alone (see its own declaration comment for why touching an
 *   already-working, coarser check carried its own risk). The gap:
 *   gnssWatchdogTimeout only confirms SOME byte arrived on the GNSS
 *   serial line, so a receiver correctly outputting GGA/VTG but never
 *   completing a valid $GPHPR sentence (wrong baud config on the
 *   dual-antenna link, HPR output not actually enabled on the
 *   receiver, etc.) would show that watchdog as healthy the entire
 *   time — exactly what happened during a real debugging session,
 *   where solQuality/satsMaster staying at 0 was the only visible
 *   symptom. hprTimeout is set from lastHprMsgTime, updated only
 *   inside HPR_Handler() itself, which — like the CAN watchdogs —
 *   only ever runs on a complete, checksum-valid sentence, not a raw
 *   byte. Deliberately loose (8s, looser than the 5s byte-level GNSS
 *   check): a brief real-world dropout is rarely even noticed while
 *   driving, and steering behaviour visibly degrading typically takes
 *   several seconds to become apparent — this is meant to catch a
 *   genuine, persistent loss, not flag every momentary hiccup. Only
 *   meaningful for GnssMode==MODE_UM982 (the only mode using
 *   HPR_Handler() at all); permanently timed-out in every other mode,
 *   same "gate interpretation, not computation" convention already
 *   used for the CAN timeouts. PDIAG grew from 46 to 47 fields.
 *   - v0.3.7: hprAgeSeconds added — a raw, continuously-updating
 *   "seconds since last successfully-parsed HPR" value, deliberately
 *   separate from hprTimeout's own (generous, field-tuned, 8s)
 *   boolean threshold. hprTimeout stays exactly as tuned for field
 *   use; this new field exists purely so Teensy Tool can show a live
 *   "last HPR: 0.3s ago" during active troubleshooting, updating
 *   every ~2s PDIAG cycle rather than only changing once an 8s
 *   threshold actually trips — raised directly by the person who'd
 *   just spent an entire session watching PDIAG fields for signs of
 *   life. -1.0 sentinel for "never received at all this boot" (rather
 *   than a meaningless huge "millis() since epoch 0" value). PDIAG
 *   grew from 47 to 48 fields.
 *   - v0.3.8: a genuine, field-reported hang fixed — Board
 *   Configuration set to Slot1=Empty, Slot2=Master in DUAL mode froze
 *   the Teensy solid (onboard LED stopped blinking, not a controlled
 *   error). Root cause: SerialGPSHeading only gets reassigned in
 *   resolveBoardSlots() if some slot is explicitly Slave — with
 *   neither slot Slave, it silently kept its file-scope default
 *   (&Serial2), the SAME physical UART SerialGPS had just been
 *   assigned via the Master rule. dualF9P_setup()/dualUM980_setup()
 *   then called .begin()/.addMemoryForRead() a SECOND time on that
 *   same already-initialised HardwareSerialIMXRT object with a
 *   different buffer — a genuine low-level UART driver collision, not
 *   a bug that could be caught by any of the validation already in
 *   place (the existing SETBOARDSLOT1/2 cross-check only warns about
 *   "both slots the same" and "neither is Master/Slave", neither of
 *   which describes this specific combination — Slot2=Master alone
 *   already satisfies "at least one Master/Slave", so this
 *   configuration was never flagged as invalid, yet still collided).
 *   Fixed with a new headingReceiverConfigured flag, set false in
 *   resolveBoardSlots() whenever neither slot is Slave, checked by
 *   every Dual-mode setup/update function before touching
 *   SerialGPSHeading at all — skips the receiver entirely with a
 *   logged warning instead of colliding. Found by building and
 *   bench-testing the new C# port's Board Configuration tab, then
 *   deliberately testing exactly the combination the original
 *   SETBOARDSLOT1 character-index bug (v0.3.5) had made impossible to
 *   reach cleanly before.
 *   - v0.3.9: Keya-as-WAS auto-zero, ported from Flo's
 *   AIO_Keya_WasKeyaFiltre (github.com/Flodu81/AIO_ECU_Keya_
 *   WasKeyaFiltre), shared directly by the person running this
 *   project after a genuine field report — "motor just spins
 *   non-stop" with WasSource=Keya on an un-corrected
 *   keyaGetSteerAngle(). Root cause: the Keya encoder is purely
 *   incremental, no absolute reference — the previous version simply
 *   treated wherever the wheels physically were at the very first
 *   heartbeat this boot as "zero", with no calibration step at all.
 *   New file zKeyaAutoZero.ino establishes the zero automatically the
 *   first time the vehicle drives straight for a bit (speed, BNO
 *   yaw-rate, and GPS-heading-stability conditions all required
 *   simultaneously), then continues correcting for drift throughout
 *   the session — a soft, sub-tick correction while guidance is
 *   actively engaged (so the operator never sees a sudden jump in
 *   the steered angle), a direct jump when it isn't. No guidance is
 *   possible until the first zero is established (same watchdogTimer
 *   mechanism already used elsewhere for lost-communication
 *   timeouts, in c00_Autosteer.ino) — this is the fix for the actual
 *   reported symptom, not just a diagnostic add-on. Gated on
 *   WasSource==WAS_SOURCE_KEYA specifically (unlike the original
 *   repo, which repurposes an unrelated Danfoss-valve setting as its
 *   toggle) — every other WasSource path completely unaffected. A
 *   genuine scaling bug caught and fixed during porting, before ever
 *   reaching this version: the soft-correction accumulator was
 *   missing a ticks-per-degree scale factor entirely (the original
 *   uses its own keyaTicksPerDeg; ours uses the already-existing
 *   steerSettings.steerSensorCounts for the same role) — without it,
 *   the correction would have silently run at an arbitrarily wrong
 *   rate.
 *   - v0.3.10: all ten Keya auto-zero tuning parameters (azParams,
 *   zKeyaAutoZero.ino) made adjustable via Teensy Tool — ten new
 *   SETAZxxx commands, one per parameter (matching this project's
 *   established one-setting-per-command convention rather than
 *   combining them like SETROLLKALMAN does), with EEPROM persistence
 *   and PDIAG reporting (wasZeroDone status, the current zero-point in
 *   degrees, and all ten parameter values — PDIAG grew from 48 to 60
 *   fields, buf[] resized 280->400 bytes with real headroom this
 *   time). A genuine gap caught before this: AutoZeroParams had no
 *   "ident" field at all when first ported in v0.3.9 — without one, a
 *   v0.3.9 unit upgrading to this version would read unwritten,
 *   effectively random EEPROM bytes into every field the moment
 *   loadAutoZeroParamsFromEEPROM() ran, since the broader
 *   ALPHA_EEPROM_MAGIC check that guards everything else was already
 *   satisfied by the earlier version and wouldn't have caught this.
 *   Fixed by adding the same "ident" field the original repo's own
 *   struct already had for exactly this reason, checked independently
 *   of ALPHA_EEPROM_MAGIC. A compilation-order constraint shaped this
 *   entire addition's structure: zHandlers.ino (where SETxxx commands
 *   and sendDiagnostics() live) compiles BEFORE zKeyaAutoZero.ino
 *   alphabetically, so azParams/wasZeroDone/keyaZeroTicks aren't
 *   visible there yet in the merged sketch — every new command and
 *   PDIAG field calls a plain setter/getter function in
 *   zKeyaAutoZero.ino instead of touching those variables directly
 *   (function calls, unlike variables, get automatic forward
 *   declarations from Arduino regardless of file order).
 *   - v0.3.11: two genuine build errors fixed — v0.3.10 never actually
 *   compiled. The SAME compilation-order rule v0.3.10's own comment
 *   above describes (zKeyaAutoZero.ino's variables not visible to
 *   earlier-compiling files) was correctly applied to zHandlers.ino,
 *   but missed for two other call sites: c00_Autosteer.ino's watchdog
 *   gate read wasZeroDone directly (fixed — reuses the already-
 *   existing azGetWasZeroDone() getter), and zKeya.ino's
 *   keyaGetSteerAngle() read keyaZeroTicks directly (fixed — new
 *   azGetZeroTicksRaw() getter added, returning the raw tick count
 *   rather than the already-existing azGetZeroTicksDegrees(), since
 *   keyaGetSteerAngle() does its own division after subtracting).
 *   Also includes the dynamic CAN filter index fix (zCAN_All_Brands.ino)
 *   — a real, field-found bug where a tester's own added Valtra
 *   armrest-engage filter used index 5, silently colliding with
 *   Keya's own, separately hardcoded index-5 reservation. Every
 *   setFIFOFilter() call in that file now uses a running
 *   "nextFilterIndex++" counter instead of a literal number, so no
 *   Brand block's filters can ever silently collide with Keya's
 *   again, regardless of how many filters a Brand grows to use
 *   (FlexCAN_T4's FIFO filter table has a hard limit of 8 slots total
 *   — confirmed via the library's own documentation, not something
 *   configurable away, only coordinated around correctly).
 *   - v0.3.12: a third genuine build error fixed — v0.3.11 still
 *   didn't compile. This one INSIDE zKeyaAutoZero.ino itself, not
 *   between files: azGetWasZeroDone()/azGetZeroTicksDegrees()/
 *   azGetZeroTicksRaw() were textually placed BEFORE the
 *   keyaZeroTicks/wasZeroDone variable declarations they read, in the
 *   same file. C++ requires a variable to be declared before use
 *   TEXTUALLY, even within a single file — unlike functions (which
 *   Arduino auto-forward-declares regardless of position or file) —
 *   a different rule than the cross-file compilation-order issue
 *   v0.3.11 fixed, easy to conflate but genuinely separate. Fixed by
 *   moving the three variable declarations to immediately after
 *   azParams, before every function in the file that reads them.
 *   - v0.3.13: documentation pass, no functional change — every
 *   function across all 15 .ino files now has a thorough header
 *   comment directly above its signature (previously several had none
 *   at all, or had their explanation only inline inside the body, or
 *   only at file-header level rather than per-function). Verified by
 *   scanning every function definition programmatically before and
 *   after, not just spot-checked.
 *   - v0.3.14: two genuine compiler warnings fixed (build now clean).
 *   sendDiagnostics()'s sentence[] buffer (zHandlers.ino) — which
 *   wraps buf[] with a checksum suffix — hadn't grown when buf[]
 *   itself grew 280->400 in v0.3.10 (Keya auto-zero PDIAG fields),
 *   flagged by GCC's own -Wformat-truncation: not an actual memory
 *   overflow (snprintf() always truncates safely), but a real risk of
 *   silently truncating the diagnostic sentence if buf[]'s actual
 *   content ever grew close to its declared size. This is the SECOND
 *   time this exact class of bug has occurred (sentence was bumped
 *   220->240 once already, earlier in this project's history) — fixed
 *   at the root this time by sizing sentence relative to buf's own
 *   size (sizeof(buf) + 20) rather than a separate hardcoded number,
 *   so it can never silently fall out of sync with buf[] again. The
 *   second warning (a -Wstringop-truncation on NMEAParser's internal
 *   strncpy() for the 5-character "G-GGA"/"G-VTG"/"G-HPR" tokens) was
 *   investigated and confirmed a harmless false positive, not fixed —
 *   the library's own mToken field is actually char[6] with an
 *   explicit null-terminator set on the line right after that
 *   strncpy(), which GCC's warning doesn't account for; documented
 *   inline rather than worked around, since nothing was actually
 *   broken.
 *   - v0.3.15: the v0.3.14 NMEAParser strncpy warning (three
 *   addHandler() calls, setup()) is now actually SUPPRESSED (a
 *   #pragma GCC diagnostic push/ignored "-Wstringop-truncation"/pop
 *   wrapped around just those three lines), not merely explained in a
 *   comment — documenting why a warning is safe doesn't stop GCC from
 *   still emitting it at every build; confirmed a build with genuinely
 *   zero warnings requires actually suppressing it. Scoped to only
 *   those three specific calls, not the whole file/project, so a
 *   genuinely different string-truncation bug appearing anywhere else
 *   in the future is still caught normally. -Wstringop-truncation
 *   confirmed as the correct flag name for this exact warning message
 *   via several independent, unrelated open-source projects hitting
 *   the identical GCC message and using the identical fix.
 *   - v0.3.16: v0.3.15's warning suppression DIDN'T actually work —
 *   confirmed by a real build still showing the identical warning
 *   verbatim. Root cause: addHandler() is a TEMPLATE method — the
 *   strncpy() the warning points at is textually written inside
 *   zNMEAParser.h's own template body, not at the parser.addHandler()
 *   call sites in this file, and GCC diagnostic pragmas apply based on
 *   the TEXTUAL location of code as the compiler processes it, not the
 *   "inlined from" call site a warning message displays — a push/pop
 *   wrapped around the call sites (v0.3.15) was therefore never going
 *   to have any effect. Moved instead to wrap the
 *   #include "zNMEAParser.h" directive itself, where the header's full
 *   content — including the template body — is textually inserted.
 *   STATUS: a well-reasoned hypothesis based on why the first attempt
 *   demonstrably failed, not yet confirmed by an actual build — this
 *   firmware cannot be compiled in the environment these fixes are
 *   written in at all; needs a real build to verify.
 *   - v0.3.17: Auto Roll Adjust — continuously nudges the roll offset
 *   between Dual and IMU to correct for slow sensor-mounting drift
 *   over time, ONLY the offset between the two (never the vehicle's
 *   actual physical lean, which AGO's own "Zero IMU" workflow still
 *   handles). New: autoRollAdjust/rollAutoDeadband/rollAutoAlpha
 *   (EEPROM-persisted settings) and rollAutoCorrection (deliberately
 *   RAM-only — the live, continuously-updating correction itself,
 *   never written to EEPROM to avoid wearing out cells at up to 10Hz).
 *   Folded into rollZeroOffset with a single EEPROM write only when
 *   auto-adjust is switched off (SETAUTOROLLADJUST:0), after which
 *   manual control resumes exactly where auto-adjust left off. Gated
 *   on solQuality==4 specifically (strict RTK-fixed only — stricter
 *   than the existing RollValid/HPR_QUALITY_MIN threshold, which also
 *   accepts float=5) held continuously for 12s (protects against the
 *   dual solution's own post-acquisition settling window, which can
 *   briefly show a wildly wrong roll), plus a configurable deadband
 *   (default 0.15°) to ignore ordinary sensor noise rather than
 *   chasing it. New PDIAG fields appended at the end (63 total now,
 *   up from 60) — the existing rollZeroOffset PDIAG field now reports
 *   rollZeroOffset+rollAutoCorrection (the effective, applied value)
 *   rather than the raw baseline alone, so Teensy Tool's existing
 *   "Current" display already shows the right number with no UI
 *   changes needed there — identical to the old value in every case
 *   that mattered before this feature existed, since
 *   rollAutoCorrection is always 0.0 when auto-adjust is off.
 *   - v0.3.18: two Auto Roll Adjust defaults corrected, both raised by
 *   the person running this project rather than found internally.
 *   rollAutoDeadband: 0.15 -> 0.2°. rollAutoAlpha: 0.02 -> 0.0005 — the
 *   old value was picked by loose analogy to updateHeadingOffset()'s
 *   own alpha, WITHOUT actually deriving it from this function's own
 *   10Hz update rate; at 10Hz, alpha=0.02 reaches 95% correction of a
 *   persistent difference in only ~15 seconds, barely distinguishable
 *   from noise — not the "slow drift over time" this feature is meant
 *   to chase. 0.0005 instead targets ~10 minutes to 95% correction
 *   (t_95 ≈ 3*dt/alpha at 10Hz), explicitly derived this time, not
 *   guessed. rollAutoAlpha's own allowed range also lowered
 *   (0.001-0.1 -> 0.0001-0.1) — the old minimum would have clamped
 *   the new default back up to 0.001 on load, undoing the point of
 *   lowering it.
 *   - v0.3.19: two genuine bugs found on a full variable-flow review
 *   of Auto Roll Adjust, neither caught by compiling (both are logic
 *   gaps, not syntax errors). (1) updateAutoRollAdjust() is called
 *   from inside imuHandler()'s "ImuType==IMU_NONE || solQuality>=
 *   HPR_QUALITY_MIN" branch — that condition ALSO admits
 *   ImuType==IMU_NONE regardless of solQuality, so a genuinely
 *   solQuality==4 reading with NO IMU configured at all could still
 *   have reached the function and started "correcting" against a
 *   meaningless roll value. The spec's own "ImuType != IMU_NONE"
 *   condition had never actually been implemented anywhere — fixed
 *   with an explicit check inside the function itself, not relying on
 *   the surrounding branch. (2) solQuality is only ever reset to 0 by
 *   zzGNSS_DualF9P.ino/zzGNSS_DualUM980.ino's own solution-lost
 *   handling — UM982's own HPR_Handler never resets it when HPR
 *   sentences stop arriving, so a stale solQuality==4 left over from
 *   before signal loss could satisfy the check in UM982 mode
 *   specifically even with no fresh data coming in. Fixed by also
 *   requiring !hprTimeout (the existing, independent watchdog on time
 *   since the last HPR sentence) before adjusting.
 *   - v0.3.20: a genuine build error fixed — v0.3.19 never actually
 *   compiled. When updateAutoRollAdjust() was originally inserted
 *   directly before imuHandler()'s own definition (v0.3.17), the
 *   "void imuHandler()" signature line itself was accidentally lost
 *   in that edit, leaving imuHandler()'s function body starting with
 *   a bare "{" and no signature at all — "expected unqualified-id
 *   before '{' token". Caught by an actual build attempt, not by any
 *   of this project's own static checks (balance-checking only
 *   verifies brace COUNTS match, not that every brace has a valid
 *   signature before it). Fixed by restoring the missing signature
 *   line. A broader search for the same "orphaned brace" pattern
 *   elsewhere in the file found two more candidates, both confirmed
 *   as genuinely valid, deliberate standalone scope blocks (used to
 *   locally scope a temporary variable inside
 *   loadAlphasFromEEPROM()/saveAlphasToEEPROM()), not further
 *   instances of this bug.
 *   - v0.3.2: autosteerSetup()/EthernetStart() moved back to run early
 *     in setup(), immediately after the basic pin/parser/Serial setup
 *     — restoring their original position from every version through
 *     v0.7 (confirmed against a preserved v0.5 snapshot), after
 *     several TFF sessions had each reasonably inserted new EEPROM-
 *     loading/GNSS-dispatch code directly above the pair without
 *     anyone tracking the cumulative drift. Verified safe in both
 *     directions before moving (not assumed): EthernetStart() depends
 *     on networkAddress, loaded inside autosteerSetup() itself — that
 *     internal pair ordering is unchanged, only the pair's overall
 *     position moved — and nothing in loadAlphasFromEEPROM()/
 *     resolveBoardSlots()/any GNSS source's setup() references
 *     Autosteer_running, Ethernet_running, steerSettings, steerConfig,
 *     or networkAddress, so nothing now-later depends on the pair
 *     having already run. Restored as a genuine, provable structural
 *     regression back to known-good ordering — not because a specific
 *     failure was conclusively traced to the reordering itself.
 * "Everything else is the v0.7 base, unmodified" no longer holds
 * exactly as stated in earlier versions of this comment — CAN steering
 * (V_Bus, CAN_setup()) and autosteer/PID (motorDrive()) both gained a
 * Keya-specific branch in v0.3, checked first, that bypasses the
 * unmodified v0.7 logic entirely when selected. With MotorDriveType at
 * its AIO-default (IBT2) and WasSource at its AIO-default (Normal),
 * every one of those branches evaluates false and control falls
 * straight through to the original, unmodified v0.7 code path — so
 * existing, non-Keya installations see zero behavioural change.
 * =====================================================================
 *
 * OVERVIEW
 * --------
 * Builds a $PAOGI (or $PANDA, in single+IMU mode) sentence from GNSS
 * and IMU input and sends it to AgOpenGPS via UDP and USB at 10 Hz.
 *
 * DUAL mode  (HPR solQuality >= HPR_QUALITY_MIN):
 *   Heading and roll come directly from HPR. Accurate, absolute azimuth.
 *
 * FALLBACK mode  (solQuality < HPR_QUALITY_MIN, e.g. slave disconnected):
 *   Heading and roll come from BNO08x IMU. A heading offset accumulated
 *   while dual was active is applied so BNO heading approximates the
 *   correct absolute azimuth.
 *
 * AgOpenGPS receives $PAOGI in both modes and cannot tell the difference.
 * This GNSS/IMU fallback logic is completely independent of the CAN
 * steering option below — CAN only changes how the desired steer angle
 * leaves the Teensy, never how heading/position is computed.
 *
 *
 * UM982 CONFIGURATION (uPrecise terminal, example uses COM2)
 * ----------------------------------------------------------
 *   FRESET
 *   MODE ROVER SURVEY
 *   CONFIG SIGNALGROUP 3 6
 *   CONFIG PPP ENABLE E6-HAS
 *   CONFIG SMOOTH RTKHEIGHT 0 (factory default?)
 *   CONFIG RTK TIMEOUT 120
 *   CONFIG COM2 460800
 *   //com 2 output
 *   GPGGA  COM2 0.1           master position + satellite count ,10Hz
 *   GPVTG  COM2 0.1           speed + course over ground ,10Hz
 *   GPHPR  COM2 0.1           dual heading + roll ,10Hz
 *   GPGGAH COM2 0.1           slave position + satellite count ,10Hz
 *   //end output
 *   CONFIG HEADING OFFSET 90.0          transverse mount 
 *   CONFIG HEADING VARIABLELENGTH       baseline measured dynamically (recommended)
 *   SAVECONFIG
 *
 * 
 * Antennas: transverse on roof, 1100 mm apart (recommended to be as far apart as practical possible).
 * Master (GPS1) right, slave (GPS2) left.
 *
 *
 * HEADING FUSION  (HEADING_ALPHA, adjustable via Teensy Tool)
 * ----------------------------------------------------------
 * In dual mode, a small amount of BNO heading can be blended into the
 * HPR heading to reduce jitter caused by the short 1.1 m baseline:
 *
 *   output = (1 - HEADING_ALPHA) * HPR + HEADING_ALPHA * (BNO + offset)
 *
 * Set HEADING_ALPHA = 0.0 (default) for pure HPR. Increase to 0.05-0.15
 * if heading looks jumpy while driving. Adjustable live from the UDP
 * monitor without reflashing by using UM982_Monitor.exe
 *
 * ROLL_ALPHA works the same way for roll.
 *
 * Note: the UM982 itself has a CONFIG HEADING SMOOTH command in uPrecise
 * that stabalize the signal on receiver level. 
 * However, HEADING_ALPHA > 0 additionally incorporates the BNO.
 * IMU fusion additionally incorporates gyro dynamics which the
 * UM982 smooth command cannot — making it likely more responsive.
 * SMOOTH has been indicated to be cause unwanted delay
 *
 * CALIBRATION
 * -----------
 * DUAL ANTENNA ROLL (in AgOpenGPS):
 *  1. With slave antenna connected and dual active (RTK fix),
 *     drive to flat ground.
 *  2. In AGO SteerConfig → GPS tab, press "Zero IMU" to null the
 *     dual antenna roll offset. This compensates for antenna mounting
 *     angle. Normally this is the only roll calibration needed.
 *
 * IMU (BNO08x) ROLL OFFSET (in Teensy Tool — only if needed):
 *  If you want fallback roll to match dual roll, you can calibrate
 *  the BNO offset so that switching between dual and fallback does
 *  not cause a visible roll change. 
 *  Procedure:
 *  1. With dual active, note the roll value displayed in AGO.
 *  2. Disconnect the slave antenna (or set Roll Alpha = 1.0 in
 *     Teensy Tool) to switch to IMU-only roll.
 *  3. In Teensy Tool, adjust "BNO ROLL OFFSET" until the roll
 *     value in AGO matches the dual roll value noted in step 1.
 *  4. Reconnect the slave antenna to return to dual mode.
 *
 * HEADING:
 *  Heading offset between BNO and HPR is calibrated automatically
 *  while dual is active (converges within ~5 seconds of dual becoming
 *  available). No manual heading calibration is needed.
 *
 *
 * CAN-BUS STEERING (inherited unchanged from UM982 Fallback v0.6/v0.7)
 * -----------------------------------------
 * Steering output can come from the classic PWM/relay path
 * (c00_Autosteer.ino / c01_AutosteerPID.ino, unchanged from earlier
 * versions) OR from a factory CAN steering valve, selected at runtime
 * by the Brand variable — no reflashing needed to switch:
 *
 *   Brand  Tractor / mode
 *   -----  ----------------------------------------------------
 *     0    Claas
 *     1    Valtra / Massey Ferguson
 *     2    Case IH / New Holland
 *     3    Fendt
 *     4    JCB
 *     5    FendtOne
 *     6    Lindner
 *     7    AgOpenGPS remote CAN/PWM module (CAN stays live in the
 *          background, e.g. for the tractor's own engage button,
 *          but no factory valve is driven)
 *     8    BRAND_NONE — no CAN at all. Default at first boot.
 *
 * Implementation:
 *   zCAN_All_Brands.ino   CAN_setup(), VBus_Send()/VBus_Receive(),
 *                         ISO_Receive(), K_Receive() — per-brand CAN
 *                         message formats for three FlexCAN buses:
 *                         K_Bus (tractor bus), ISO_Bus, V_Bus (valve).
 *   c01_AutosteerPID.ino  motorDrive() additionally computes setCurve
 *                         (the CAN target curve/angle) whenever
 *                         Brand != BRAND_NONE, alongside the existing
 *                         PWM output — the two paths never interfere.
 *
 * With Brand == BRAND_NONE (8), CAN_setup()/VBus_Send()/VBus_Receive()/
 * ISO_Receive()/K_Receive() are all skipped entirely in setup()/loop()
 * below — zero CAN bus activity, byte-for-byte identical behaviour to
 * the original CAN-less firmware.
 *
 * Set the brand with SETBRAND:N via Teensy Tool (UDP port 5556),
 * saved to EEPROM immediately. Reboot the Teensy after changing Brand
 * so CAN filters are set up correctly (CAN_setup() only runs once, in
 * setup()). Current Brand is reported back in the $PDIAG sentence —
 * see sendDiagnostics() in zHandlers.ino for the full field list.
 *
 * =====================================================================
 */

// Suppresses -Wstringop-truncation for THIS header's own internal
// strncpy() call (NMEAParser::addHandler(), confirmed a harmless
// false positive — see the fuller explanation at the three
// parser.addHandler() call sites further down this file, in setup()).
// Wrapped around the #include itself, not the call sites: addHandler()
// is a template method, so the actual strncpy() the warning points at
// is textually inside THIS header's own body once included here — GCC
// diagnostic pragmas apply based on the textual/lexical location of
// the code as the compiler processes it, not the "inlined from" call
// site a warning message displays, so a pragma at the call site (tried
// first, v0.3.14) has no effect on a warning whose actual source line
// lives inside an included header instead.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#include "zNMEAParser.h"
#pragma GCC diagnostic pop
#include <Wire.h>
#include "BNO08x_AOG.h"
#include <SimpleKalmanFilter.h>
#ifdef ARDUINO_TEENSY41
  #include <NativeEthernet.h>
  #include <NativeEthernetUdp.h>
#endif
#include <FlexCAN_T4.h>

// This firmware always targets the All-In-One (AIO) board. Some CAN code
// adapted from the AgOpenGPS-Official/Boards CANBUS firmware branches on
// this with #ifdef isAllInOneBoard (AIO-specific LED pins) vs #else
// (separate CAN module with its own dedicated LED pins). Defining it
// here selects the correct branch for this build.
#define isAllInOneBoard

// =====================================================================
// CANBUS  (optional steering output — see BRAND_NONE below)
// =====================================================================
// Three physical CAN buses on the Teensy 4.1, matching the layout used
// by AgOpenGPS-Official/Boards CANBUS firmware (MechanicTony/gunicsba):
//   K_Bus   (CAN1) — tractor / armrest bus (engage button, etc.)
//   ISO_Bus (CAN2) — ISOBUS (not used by most brands below)
//   V_Bus   (CAN3) — steering valve bus (curve/angle read + write)
//
// BRAND selects which factory CAN protocol (if any) drives steering.
// This is completely independent of the UM982/BNO08x GNSS fallback
// logic above and in zHandlers.ino — CAN only affects how the desired
// steer angle leaves the Teensy, not how heading/position is computed.
//
//   0 = Claas       4 = JCB
//   1 = Valtra / Massey Ferguson   5 = FendtOne
//   2 = Case IH / New Holland      6 = Lindner
//   3 = Fendt                      7 = AgOpenGPS remote CAN/PWM module
//                                      (CAN buses stay live in the
//                                      background, e.g. for reading the
//                                      tractor's own engage button, but
//                                      no factory valve is driven)
//   8 = BRAND_NONE — No CAN at all. CAN_setup() and all VBus/ISO/K bus
//       polling are skipped entirely. Behaviourally identical to the
//       original 000_UM982_Fallback.ino (classic PWM/relay steering
//       via c00_Autosteer.ino, zero CAN bus activity).
//
// Default is BRAND_NONE so a freshly flashed board — or one where the
// EEPROM hasn't been initialised yet — behaves exactly like the
// CAN-less firmware until the user deliberately selects a brand via
// Teensy Tool (SETBRAND:N) or the serial Service Tool.
#define BRAND_NONE 8
uint8_t Brand = BRAND_NONE;   // loaded from EEPROM at startup

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> K_Bus;    // Tractor / control bus
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> ISO_Bus;  // ISO Bus
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> V_Bus;    // Steering valve bus

// CAN-side steering variables (see CAN_All_Brands.ino, c01_AutosteerPID.ino)
uint16_t setCurve = 32128;        // desired curve/angle sent to CAN valve
uint16_t estCurve = 32128;        // actual curve/angle read from CAN valve
int16_t  FendtEstCurve = 0;       // Fendt-specific actual curve
int16_t  FendtSetCurve = 0;       // Fendt-specific desired curve
boolean  engageCAN = 0;           // engage state read from CAN (K_Bus)
boolean  workCAN = 0;             // work-switch state read from CAN
uint8_t  steeringValveReady = 0;  // valve-ready state read from CAN (V_Bus)

// =====================================================================
// Hydraulic hitch/implement lift control via CAN (Fendt/Claas headland
// auto-raise/lower) — ported from the original CAN implementation.
// The low-level CAN button senders this depends on (pressGo/liftGo/
// pressEnd/liftEnd/pressCSM1/pressCSM2, all in zCAN_All_Brands.ino)
// already existed in this codebase from the original CAN merge, but
// were never called by anything — this is the missing orchestration
// layer, plus the two AgOpenGPS PGNs (Machine Data 0xEF, Machine
// Settings 0xEE) that feed it, neither of which this codebase parsed
// before. See SetRelaysFendt()/SetRelaysClaas() in c00_Autosteer.ino
// and their PGN 0xEF/0xEE handlers in ReceiveUdp() there.
uint8_t hydLift = 0;         // from PGN 0xEF (Machine Data): 0=off, 1=lower, 2=raise
uint8_t bitState = 0;        // current "should implement be down" bit
uint8_t bitStateOld = 0;     // previous bitState, for edge detection

struct AogConfig
{
    uint8_t raiseTime = 2;
    uint8_t lowerTime = 4;
    uint8_t isRelayActiveHigh = 0;  // if zero, active low (default)
    uint8_t enableToolLift = 0;     // 1 = AOG allowed to press CAN headland buttons
};  AogConfig aogConfig;      // 4 bytes — saved at EEPROM address 6
#define AOGCONFIG_EEPROM_ADDR 6
uint8_t  pressureReading = 0;     // hitch/pressure sensor value from CAN
uint8_t  currentReading = 0;      // current sensor value from CAN (V_Bus)
uint8_t  ISORearHitch = 250;      // hitch height from ISOBUS (0-250 -> 0-100%)
uint8_t  KBUSRearHitch = 250;     // hitch height from K-Bus (CaseIH tractor bus)
boolean  ShowCANData = 0;         // debug: print raw CAN traffic to Serial
boolean  sendCAN = 0;             // send CAN message every 2nd cycle (kept for parity with original)
boolean  intendToSteer = 0;       // do we intend to steer? Set in autosteerLoop()
                                   // (c00_Autosteer.ino): 1 when actively steering,
                                   // 0 when not — used both by VBus_Send() (brand
                                   // logic) and by the CAN-side WAS derivation below
                                   // (steerAngleActual := estCurve while not steering,
                                   // to avoid a jump at the moment of engage).
uint32_t Time = 0;                // timestamp used by K-Bus button handlers
uint32_t relayTime = 0;           // engage-LED hold timer
elapsedMillis vbusSendTimer;      // rate-limits VBus_Send() to ~50ms (see loop())
// engageLED: reuses the existing autosteer-active LED on the AIO board
// (this firmware always targets the AIO board — see ARDUINO_TEENSY41
// guards elsewhere — so no separate non-AIO LED pin is needed).
#define engageLED AUTOSTEER_ACTIVE_LED

// =====================================================================
// WAS source selection: physical sensor (BRAND_NONE) vs CAN valve
// feedback (all other Brand values) — see autosteerLoop() in
// c00_Autosteer.ino.
// =====================================================================
// When Brand != BRAND_NONE, steerAngleActual is derived entirely from
// the CAN steering valve's own reported curve (setCurve/estCurve) —
// the physical ADS1115 WAS input is not read at all in that case, and
// must not be: with no potentiometer wired to a CAN-steered valve, the
// ADC would float and inject noise into steerAngleActual if it were
// still read (this was the root cause of the "jittery WAS" symptom
// reported from a CAN bench test — two different sources, physical
// ADS1115 and VBus_Receive(), were both writing steerAngleActual).
//
// The raw CAN curve value doesn't map linearly to degrees, so it goes
// through a calibration lookup (multiMap, 21-point piecewise linear).
// inputWAS/outputWAS is an identity curve (adjust outputWAS if your
// specific valve needs correction); Fendt/FendtOne use a separate
// curve (outputWASFendt) because their curve value uses a different
// internal scale (steerSensorCounts * 10, see autosteerLoop()).
float inputWAS[]       = { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0};  // do not adjust
float outputWAS[]      = { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0};  // adjust to calibrate
float outputWASFendt[] = { -60.00, -54.0, -48.0, -42.3, -36.1, -30.1, -23.4, -17.1, -11.0, -5.5, 0, 5.5, 11.0, 17.1, 23.4, 30.1, 36.1, 42.3, 48.0, 54.0, 60.0};  // Fendt 720 SCR, CPD = 80

// Piecewise-linear interpolation between the size points of _in/_out.
// Values outside the input range are clamped to the first/last output.
template <typename T>
T multiMap(T value, T* _in, T* _out, uint8_t size)
{
    if (value <= _in[0]) return _out[0];
    if (value >= _in[size - 1]) return _out[size - 1];

    uint8_t pos = 1;
    while (value > _in[pos]) pos++;

    T inSpan = _in[pos] - _in[pos - 1];
    T outSpan = _out[pos] - _out[pos - 1];
    T valueScaled = (value - _in[pos - 1]) / inSpan;

    return _out[pos - 1] + valueScaled * outSpan;
}

// Fendt / Claas K-Bus "GO"/"END"/CSM button messages (see pressGo() etc.
// in CAN_All_Brands.ino). Byte payloads are fixed CAN messages defined
// by each tractor's protocol — do not change unless you know why.
boolean goDown = false, endDown = false;
byte goPress[8]    = {0x15, 0x20, 0x06, 0xCA, 0x80, 0x01, 0x00, 0x00};  // press big go
byte goLift[8]     = {0x15, 0x20, 0x06, 0xCA, 0x00, 0x02, 0x00, 0x00};  // lift big go
byte endPress[8]   = {0x15, 0x21, 0x06, 0xCA, 0x80, 0x03, 0x00, 0x00};  // press big end
byte endLift[8]    = {0x15, 0x21, 0x06, 0xCA, 0x00, 0x04, 0x00, 0x00};  // lift big end
byte csm1Press[8]  = {0xF1, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x67};  // Claas CSM1, pre-MR tractors
byte csm2Press[8]  = {0xF4, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F};  // Claas CSM2, pre-MR tractors

// Forward declarations for CAN_All_Brands.ino
void CAN_setup();
void VBus_Send();
void VBus_Receive();
void ISO_Receive();
void K_Receive();
void SetRelaysFendt();   // hitch/implement lift control (c00_Autosteer.ino)
void SetRelaysClaas();   // hitch/implement lift control (c00_Autosteer.ino)
void canConfig();        // re-broadcast AGO config onto CAN, Brand==7 only



// =====================================================================
// USER SETTINGS
// =====================================================================

// Output sentence ($PAOGI vs $PANDA) is now selected by GnssMode in
// BuildNmea() — see zHandlers.ino. The old makeOGI bool that used to
// control this was removed here; it had already stopped doing anything
// once BuildNmea() switched to checking GnssMode instead, and leaving
// a dead, unused constant with the same name as the thing it used to
// control was more likely to mislead a future reader than help one.

// Also send PAOGI to AgIO via USB serial (useful for debugging).
const bool sendUSB = true;

// --- Kalman filtering for HPR heading and roll ---
// Carried over from the original Chris Kinal firmware. Start with false.
//
// The UM982 has a built-in CONFIG SMOOTH HEADING <N> command (N = number
// of epochs, range 0-100). At 10 Hz one epoch = 100 ms, so N=10 gives
// 1 second of smoothing. This is a time-window average (moving average)
// over N epochs — all samples in the window carry equal weight.
//
// The Kalman filter used here is fundamentally different: it models the
// process dynamics and weights recent measurements more intelligently,
// reacting faster to real heading changes while suppressing noise.
// Neither approach is strictly better in all conditions; Kalman is
// generally more suited to a moving vehicle, while the UM982 smooth
// command acts at the receiver before data is transmitted (lower lag
// at equal smoothing level since no serial/UDP delay is added).
//
// Recommendation:  
// Primary, try imu fusion 
// Secondary, try CONFIG SMOOTH HEADING on the UM982 first.
// Thirdly, try Kalman filter
//
// Enable these firmware filters only as an additional option if needed.
//
//   e_mea  measurement noise (higher = more smoothing, more lag)
//   e_est  initial estimate uncertainty (set equal to e_mea)
//   q      process variance  (0.001 = slow, 0.01 = medium, 0.1 = fast)
//
// filterRoll/filterHeading (the on/off toggles) are plain booleans
// checked at each call site — safe to change live, no restart needed.
// The six tuning numbers below are baked into the SimpleKalmanFilter
// objects at construction (see rollFilter/headingFilter further down)
// — changing them at runtime does NOT reconfigure an already-built
// filter object (SimpleKalmanFilter has no public setters we can rely
// on being present across library versions), so these follow the same
// "saved to EEPROM, applied at next boot" convention already used for
// GnssMode/Brand/ImuType — see SETROLLKALMAN/SETHEADINGKALMAN in
// receiveMonitorCommands() (zHandlers.ino).
bool  filterRoll    = false;
float rollMEA       = 1.0f;
float rollEST       = 1.0f;
float rollQ         = 0.01f;

bool  filterHeading = false;
float headingMEA    = 1.0f;
float headingEST    = 1.0f;
float headingQ      = 0.01f;

// --- BNO08x roll inversion ---
// Roll inversion is not possible for the BNO at this point.
// invertRoll will be in conflict with dual, BNO has to be mountet correctly
const bool invertRoll = true;

// --- HPR roll field selection ---
// Official Table 7-42: words[2]=Pitch (fore-aft), words[3]=Roll (lateral).
//
// With CONFIG HEADING OFFSET 90.0 (transverse antenna mount), UM982
// internally rotates the coordinate system, which swaps the axes in HPR:
//   words[2] carries lateral tilt (tractor roll)  ← confirmed by field test
//   words[3] carries fore-aft tilt (tractor pitch)
//
// This matches Chris Kinal's original firmware. Confirmed: lifting one
// antenna produced the expected response in words[2].
//
// Only change to false if antennas are mounted longitudinally (fore-aft)
// without CONFIG HEADING OFFSET.
const bool hprRollIsWords2 = true;

// --- Heading fusion (HEADING_ALPHA) ---
// Blends BNO heading into HPR heading in dual mode (see OVERVIEW above).
// 0.0 = pure HPR (recommended). Adjustable live from Teensy Tool.
// See alpha guide in um982_monitor for recommended values.
float HEADING_ALPHA = 0.0f;

// --- Roll fusion (ROLL_ALPHA) ---
// Blends BNO roll into HPR roll in dual mode.
// HPR roll can have jitter from antenna oscillation or vibration.
// BNO roll is gravity-referenced with low noise at low frequency.
//   output = (1 - ROLL_ALPHA) * HPR_roll + ROLL_ALPHA * BNO_roll
// No wraparound handling needed (roll stays well within ±180°).
// 0.0 = pure HPR (recommended starting point).
// Adjustable live from Teensy Tool.
float ROLL_ALPHA = 0.0f;

// --- HPR solution quality threshold ---
// 0 = no solution
// 1 = single-point GPS
// 2 = pseudorange differential
// 4 = RTK fixed   (centimetre accuracy, ambiguity resolved)
// 5 = float RTK   (decimetre accuracy, ambiguity unresolved)
//
// Note: Unicore quality codes are NOT in order of quality.
// 4 = best (fixed), 5 = second best (float).
// The check is solQuality >= HPR_QUALITY_MIN, meaning:
//   HPR_QUALITY_MIN = 4 → accepts both fixed (4) AND float (5)
//   HPR_QUALITY_MIN = 5 → accepts float (5) only (rarely useful)
//   HPR_QUALITY_MIN = 6 → accepts nothing (effectively disables dual)
//
// To require RTK fixed only, use a value between 4 and 5, e.g.:
//   const int HPR_QUALITY_MIN = 5;  // float only — rarely wanted
// The current >= check means 4 accepts both fixed (4) and float (5).
//
// Recommended: leave at 4 (accepts fixed and float).
// Change to 2 to also accept DGPS in poor RTK conditions.
const int HPR_QUALITY_MIN = 4;

// --- Alpha control parameters ---
// HEADING_ALPHA and ROLL_ALPHA are adjusted based on two signals:
//
// 1. HPR solQuality (primary):
//    solQuality < HPR_QUALITY_MIN → alpha = 1.0 immediately (full BNO)
//
// 2. hprSats from HPR words[5] (satellite-based, linear):
//    hprSats >= satsSlaveFull → alpha = initialAlpha
//    hprSats = 0              → alpha = 1.0
//    Linear interpolation between.
//    hprSats = satellites actually used in heading solution —
//    a more direct quality indicator than slave antenna total count.
//
//
// initialHeadingAlpha and initialRollAlpha are the base values (floor)
// used when dual is fully healthy. Set independently from the monitor
// and saved to EEPROM. Default 0.0 = pure HPR.
float initialHeadingAlpha = 0.0f;  // loaded from EEPROM at startup
float initialRollAlpha    = 0.0f;  // loaded from EEPROM at startup

// HPR satellite threshold for full dual quality.
// hprSats (HPR words[5]) >= this value -> alpha stays at initialAlpha.
// hprSats = 0                          -> alpha = 1.0 (full BNO).
// Linear interpolation between.
// Adjustable live from Teensy Tool (SETSATSFULL:N command).
// Saved to EEPROM alongside alpha values.
// Lower value = more tolerant. Higher value = more conservative.
int satsSlaveFull = 7;  // loaded from EEPROM at startup
// --- BNO08x watchdog ---
// Automatically restarts BNO08x if it stops responding (I2C hang).
// Recovers without a Teensy power cycle.
//
//   IMU_TIMEOUT_MS      ms without data before restart is attempted
//   IMU_RESTART_DELAY_MS  minimum wait between restart attempts
//   IMU_MAX_RETRIES     give up after this many failures (0 = unlimited)
#define IMU_TIMEOUT_MS       500
#define IMU_RESTART_DELAY_MS 300
#define IMU_MAX_RETRIES      5

// =====================================================================
// END USER SETTINGS
// =====================================================================


// --- Serial port assignments ---
#define SerialAOG   Serial       // AgIO USB
#define SerialRTK   Serial3      // RTK radio (RTCM input)
HardwareSerialIMXRT* SerialGPS = &Serial7;  // UM982

// =====================================================================
// GNSS mode selection (TFF)
// =====================================================================
// Which GNSS source is active, chosen at runtime via Teensy Tool's
// Receiver Configuration tab (SETGNSSMODE / SETRECEIVERTYPE, see
// receiveMonitorCommands() in zHandlers.ino) and persisted in EEPROM —
// same runtime-selectable pattern already used for CAN Brand, applied
// here to the GNSS source instead. Each source's setup()/update() live
// in its own zGNSS_*.ino file; see the dispatch switches in setup()
// and loop() below.
//
// A reboot is required after changing GnssMode or DualReceiverType, for
// the same reason CAN Brand needs one: serial port initialisation only
// runs once, in setup().
#define MODE_UM982       1   // Dual antenna receiver: UnicoreComm UM982
#define MODE_SINGLE_IMU  2   // Single antenna + IMU → $PANDA output
#define MODE_DUAL        3   // Dual single-antenna receivers, moving-base
uint8_t GnssMode = MODE_UM982;   // loaded from EEPROM at startup

#define RX_F9P    0   // u-blox F9P pair — UBX-NAV-RELPOSNED (binary)
#define RX_UM980  1   // UnicoreComm UM980 pair — UNIHEADING2 (ASCII)
uint8_t DualReceiverType = RX_F9P;   // only meaningful when GnssMode == MODE_DUAL

// GNSS Passthrough — an orthogonal on/off toggle, not another GnssMode
// value, because it means something categorically different: "ignore
// GnssMode and every fusion feature entirely, just relay whatever the
// receiver sends, byte for byte." Restored from Chris Kinal's original
// UM982 firmware (AOG_Teensy_UM982.ino, "udpPassthrough" — same
// mechanism, renamed here to match this project's naming), where a
// receiver configured to output $KSXT (a single self-contained
// UnicoreComm sentence with position, heading, roll, pitch, and
// separate position/heading quality indicators, all computed by the
// receiver's own internal fusion) could be sent to AGO with zero
// Teensy-side processing — no alpha-blend, no Kalman filter, no IMU
// fallback, no headingOffset calibration, nothing. For users who don't
// want any of TFF's fallback engine and would rather trust the
// receiver's own onboard fusion outright.
//
// Genuinely content-agnostic, exactly as in the original: this does
// not parse or even recognise $KSXT specifically — it forwards ANY
// complete "$...\r\n" line the receiver sends, whatever sentence type
// that happens to be. Getting $KSXT out of it specifically is purely a
// receiver-configuration choice (KSXT COM2 1), made outside this
// firmware entirely — same as GGA/VTG/HPR rate configuration already
// is. AGO itself must be able to parse whatever sentence is forwarded;
// this firmware has no part in that and doesn't reformat it.
bool GnssPassthrough = false;   // loaded from EEPROM at startup

// =====================================================================
// Keya CAN steering motor support
// =====================================================================
// Deliberately NOT added as new bits in the shared steerConfig struct
// — that struct's bit layout is a fixed protocol contract with AGO
// itself (synced via PGN 251/252, confirmed in AGO's own C# source,
// FormSteer.cs -> p_251/p_252 -> SendPgnToLoop()), so adding an
// unplanned bit there risks silent conflicts with what AGO thinks that
// bit means, or being overwritten on the next settings sync. These are
// new, independent, TFF-own settings instead — same pattern as
// BoardSlot1/BoardSlot2/ImuType/GnssMode/GnssPassthrough.
//
// Protocol verified against Keya's own official manual (KY170DD01005-
// 08G user manual v2.4, JINAN KEYA ELECTRON SCIENCE AND TECHNOLOGY CO.,
// LTD — fetched in full from
// bsg-i.nbxc.com/upload/669/442/pdf/0d6c506ea3ebf97650f00dc39d.pdf at
// the time this was written; the AgOpenGPS docs site links the same
// manual under the name "Keya KY170DD01005-08G v2.4.pdf" if that URL
// ever moves) — not just community source, which turned out to
// genuinely disagree with itself across different repos (see
// MOTOR_DRIVE_KEYA comment below for the specific discrepancy this
// resolved). Also cross-checked against a tester's direct field
// report: the community "5A firmware" variant (automatic EEPROM
// current-limit write on first detect) was independently reported as
// unreliable in practice — deliberately NOT replicated here;
// current-limit configuration, if ever added, should be an explicit,
// installer-triggered Teensy Tool action, never silent automatic
// EEPROM writes at boot.
#define MOTOR_DRIVE_PWM    0   // traditional PWM output — see below for
                                 // why this isn't split into separate
                                 // IBT2/Cytron values
#define MOTOR_DRIVE_KEYA   2   // CAN-driven Keya steering motor, replaces
                                // PWM output entirely when selected —
                                // see motorDrive() in c01_AutosteerPID.ino.
                                // Kept at value 2 (not renumbered to 1)
                                // for continuity with the very first
                                // version of this setting, even though
                                // nothing had shipped with it saved yet
                                // when this was simplified.
                                //
                                // MOTOR_DRIVE_PWM deliberately does NOT
                                // distinguish IBT2 vs Cytron itself — an
                                // earlier version of this setting had
                                // three values (IBT2/Cytron/Keya), but
                                // motorDrive() (c01_AutosteerPID.ino)
                                // never actually read that distinction:
                                // the real IBT2-vs-Cytron signal choice
                                // was always, and still is, made by the
                                // pre-existing, AGO-synced
                                // steerConfig.CytronDriver (part of the
                                // fixed PGN 251/252 protocol contract).
                                // The three-value version's "IBT2"/
                                // "Cytron" radio buttons in Teensy Tool
                                // looked like they selected between the
                                // two but silently did nothing — found
                                // and fixed before this caused real
                                // confusion in the field. Duplicating a
                                // choice AGO already owns, without it
                                // actually doing anything, is worse than
                                // not offering it at all.
                                //
                                // HARDWARE LIMITATION, not a firmware
                                // choice: AIO v4.x boards expose only ONE
                                // physical CAN channel on the main Ampseal
                                // connector (CanH1/CanL1, pins 16/17) —
                                // confirmed against the official AIO
                                // Board Pinout documentation
                                // (docs.agopengps.com/hardware/boards/
                                // all-in-one-boards/AIO-Board-Pinout/):
                                // the second channel (CanH2/CanL2, pins
                                // 18/19) exists only on v2.x boards; on
                                // v4.x those same two pins are repurposed
                                // for Cytron power input instead. TFF
                                // hardcodes Keya onto V_Bus (see
                                // CAN_setup(), zCAN_All_Brands.ino) — the
                                // same bus a CAN-ready tractor Brand
                                // (Valtra, Fendt, etc.) already needs for
                                // its own engage-detection and valve
                                // feedback. On standard v4.x hardware,
                                // Brand != BRAND_NONE and
                                // MotorDriveType == MOTOR_DRIVE_KEYA
                                // together is therefore not something a
                                // Teensy Tool setting alone can resolve —
                                // it would require genuinely separate
                                // hardware (an external CAN transceiver
                                // wired directly to one of the Teensy's
                                // otherwise-unused CAN2/CAN3 pins,
                                // bypassing the Ampseal connector
                                // entirely) to use both at once. Teensy
                                // Tool warns on this combination rather
                                // than silently allowing it — see
                                // _check_motor_drive_conflict() in
                                // teensy_tool.py.
uint8_t MotorDriveType = MOTOR_DRIVE_PWM;   // loaded from EEPROM; AIO-default
                                              // matches existing behaviour
                                              // exactly (steerConfig.
                                              // CytronDriver alone already
                                              // decided PWM signalling,
                                              // unchanged either way)

#define WAS_SOURCE_NORMAL 0   // physical potentiometer (Brand==BRAND_NONE)
                                // or CAN-brand tractor's own valve
                                // position feedback — i.e. exactly
                                // today's existing behaviour, unchanged
#define WAS_SOURCE_KEYA   1   // Keya's own internal encoder, reusing
                                // steerSensorCounts/AckermanFix — same
                                // calibration values as physical WAS,
                                // confirmed against community source
                                // (steerAngleActual = steeringPosition /
                                // steerSensorCounts, identical formula)
                                //
                                // A separate, explicit setting rather
                                // than repurposing the existing
                                // "Danfoss mode" checkbox, which is the
                                // convention a real, active community
                                // implementation uses instead
                                // (discourse.agopengps.com/t/
                                // was-with-keya/20745 — confirmed
                                // directly from that thread's author,
                                // who also wrote one of the four Keya
                                // source repos consulted for this
                                // feature). Rejected deliberately:
                                // overloading an existing, differently-
                                // named setting with a second, hidden
                                // meaning trades a small amount of UI
                                // real estate for a genuinely confusing
                                // non-obvious behaviour — the same
                                // "clarity over convenience" reasoning
                                // already applied to every other
                                // TFF-own setting in this project.
uint8_t WasSource = WAS_SOURCE_NORMAL;   // loaded from EEPROM; AIO-default
                                           // matches existing behaviour
                                           // exactly

// Second GNSS serial port — the "heading role" receiver on the AIO
// board's slave header, used only in MODE_DUAL (F9P/UM980 pairs).
// SerialGPS (Serial7) stays the "position role" receiver in every
// mode, including MODE_DUAL. Teensy 4.1 has 8 hardware UARTs; Serial3
// (RTK) and Serial7 (position) are already used, leaving six free —
// Serial2 is used here, but the actual pin/UART your board's slave
// header is physically wired to may differ; check your board's
// schematic and change this if needed.
HardwareSerialIMXRT* SerialGPSHeading = &Serial2;

// NEW, found via a genuine field-report of the Teensy hanging (LED
// stopped blinking) after setting Board Configuration to Slot1=Empty,
// Slot2=Master in DUAL mode: SerialGPSHeading only gets reassigned in
// resolveBoardSlots() below if some slot is explicitly SLOT_SLAVE. If
// neither slot is, SerialGPSHeading silently keeps its file-scope
// default above (&Serial2) — which, in that exact configuration, is
// the SAME physical UART SerialGPS just got assigned to via the
// Master rule. dualF9P_setup()/dualUM980_setup() would then call
// .begin()/.addMemoryForRead() a SECOND time on the very same
// already-initialised HardwareSerialIMXRT object, with a DIFFERENT
// buffer — a genuine low-level UART driver collision, not a
// controlled error, matching the observed hang exactly.
// This flag is set false by resolveBoardSlots() whenever that
// collision would occur, and checked by every Dual-mode setup/update
// function before touching SerialGPSHeading at all — see those
// functions' own comments for how they use it.
bool headingReceiverConfigured = true;

// Raw (pre-blend) dual and IMU roll, computed every imuHandler() call
// (zHandlers.ino) but not previously exposed outside it. Added for
// Teensy Tool's IMU tab "Dual Roll vs IMU Roll" comparison panel — lets
// rollZeroOffset be tuned by eye until the two match, without touching
// ROLL_ALPHA or looking anything up in AgOpenGPS. rawDualRollValid is
// false whenever there's no current dual solution (degraded quality,
// or GnssMode == MODE_SINGLE_IMU where a dual solution never exists at
// all) — Teensy Tool should show "--" rather than a stale frozen number
// when this is false.
float rawDualRoll      = 0.0f;
float rawImuRoll       = 0.0f;
bool  rawDualRollValid = false;

const int32_t baudAOG = 115200;
const int32_t baudGPS = 460800;  // UM982 high-speed baud
const int32_t baudRTK = 9600;    // Xbee default

// --- BNO08x I2C ---
#define ImuWire Wire             // SDA=18, SCL=19
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAddresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;
bool useIMU = false;   // "an IMU is active" — true regardless of
                           // which ImuType is selected (see below). The
                           // name is a historical leftover from before
                           // TM171 support existed; kept unchanged
                           // rather than renamed, since ~12 of its 15
                           // call sites across the codebase only ever
                           // ask "is an IMU active", never "is it
                           // specifically a BNO08x" — renaming would
                           // touch every one of those sites for no
                           // functional benefit and real regression
                           // risk. Only setup()/loop()'s IMU dispatch
                           // and checkImuWatchdog() need to know the
                           // actual type — see ImuType below.

// =====================================================================
// ImuType — which physical IMU is in use (TFF)
// =====================================================================
// Unlike BNO08x (auto-detected at boot via I2C address scan — see
// setup()), TM171 has no bus to probe: it's a fixed UART connection,
// either wired there or not. So ImuType is an explicit Teensy Tool
// selection (SETIMUTYPE, saved to EEPROM), not an auto-probe outcome —
// selecting TM171 skips the BNO I2C scan entirely and goes straight to
// TM171_setup() (zIMU_TM171.ino). Reboot required after changing this,
// same reason as GnssMode/Brand: serial/I2C init only runs once, in
// setup().
#define IMU_BNO08X 0
#define IMU_TM171  1
#define IMU_NONE   2   // "No fallback function" — traditional dual-only
                        // operation, no IMU blend/fallback at all. See
                        // imuHandler() (zHandlers.ino) and the
                        // dualReadyRelPos assignment in each
                        // zzGNSS_*.ino for how this is threaded through
                        // — PAOGI is always sent from whatever the dual
                        // source reports, good quality or not, exactly
                        // as a non-fallback firmware would, rather than
                        // ever substituting IMU data during a dip.
uint8_t ImuType = IMU_BNO08X;   // loaded from EEPROM at startup

// =====================================================================
// Board Configuration — which physical connector position each
// logical GNSS/IMU role actually occupies on this specific
// installation. Confirmed by tracing the official AIO v4 Firmware's
// own source (SerialGPS = &Serial7 "Main position receiver (GGA)",
// SerialGPS2 = &Serial2 "Dual heading receiver"), matching this
// project's own field-tested wiring — settled after an earlier,
// unreliable attempt to confirm the same thing from a PDF schematic
// alone, which turned out not to be trustworthy for this purpose.
//
// This exists because TM171 can physically occupy either GNSS
// connector position instead of a real receiver — confirmed
// electrically identical (generic UART + power pins, no
// receiver-specific wiring) — and different installations wire it
// differently (dedicated Serial5 header vs one of the two GNSS
// positions, purely for wiring convenience, e.g. avoiding running a
// new cable to the Serial5 header at all). BoardSlot1/BoardSlot2 let
// the installer say which physical slot has which role, instead of
// firmware assuming the AIO-default wiring unconditionally.
//
// Two validation rules (enforced in loadAlphasFromEEPROM(),
// zHandlers.ino): the two slots must never be equal (catches double-
// Master, double-Slave, double-TM171, and double-Empty all at once),
// and at least one slot must be a real GNSS role (Master or Slave) —
// otherwise there is no position/heading source at all. An invalid
// saved combination resets to the AIO-default rather than being left
// ambiguous.
//
// If either slot is TM171, ImuType is force-set to IMU_TM171 at boot
// (see resolveBoardSlots() below) — deliberately one-way. Physically
// wiring TM171 into a GNSS slot only makes sense if it's actually
// going to be read, so BNO08x is turned off automatically; but
// choosing ImuType = TM171 in the IMU tab without touching Board
// Configuration is still valid on its own (the ordinary dedicated-
// Serial5-header case), so nothing forces Board Configuration in the
// other direction.
#define SLOT_MASTER 0   // real GNSS receiver, position role (AIO-default here)
#define SLOT_SLAVE  1   // real GNSS receiver, heading role (AIO-default here)
#define SLOT_TM171  2   // TM171 wired into this connector position instead
#define SLOT_EMPTY  3   // nothing connected here
uint8_t BoardSlot1 = SLOT_MASTER;   // loaded from EEPROM at startup
uint8_t BoardSlot2 = SLOT_SLAVE;    // loaded from EEPROM at startup

// TM171 UART — Serial5 by default, matching the pin convention already
// used in the AgOpenGPS community's own TM171 reference implementation
// ("Chassis IMU -> Serial5") for the AIO board family, so existing
// TM171 wiring/harnesses need no changes to work with TFF. Can be
// overridden to Serial7 or Serial2 by Board Configuration if TM171 is
// physically wired into one of the GNSS connector positions instead —
// see resolveBoardSlots() and the Board Configuration comment block
// above.
HardwareSerial* SerialImuTM171 = &Serial5;

// Resolves BoardSlot1/BoardSlot2 into the actual SerialGPS/
// SerialGPSHeading/SerialImuTM171 pointers, and force-sets ImuType if
// either slot is TM171. Called once in setup(), after
// loadAlphasFromEEPROM() (so BoardSlot1/BoardSlot2 are already loaded
// and validated) and before any GNSS/IMU source's own setup() runs
// (so they see the correct, final pointer values from the moment they
// first touch them — none of um982_setup()/dualF9P_setup()/
// TM171_setup()/etc. needed any changes for this, since they already
// only ever referred to these pointers, never to a hardcoded Serial
// object directly).
void resolveBoardSlots()
{
    if      (BoardSlot1 == SLOT_MASTER) SerialGPS = &Serial7;
    else if (BoardSlot2 == SLOT_MASTER) SerialGPS = &Serial2;

    if      (BoardSlot1 == SLOT_SLAVE) SerialGPSHeading = &Serial7;
    else if (BoardSlot2 == SLOT_SLAVE) SerialGPSHeading = &Serial2;
    else
    {
        // Neither slot is Slave — SerialGPSHeading just kept its
        // file-scope default (&Serial2), which collides with
        // whatever SerialGPS itself resolved to above whenever THAT
        // also happens to be Serial2 (e.g. Slot1=Empty, Slot2=Master).
        // Flagged here, unconditionally, rather than only when a
        // collision is actually detected — even the non-colliding
        // case (e.g. Slot1=Master, Slot2=Empty, leaving
        // SerialGPSHeading on Serial2 while SerialGPS took Serial7)
        // still means there is genuinely no configured heading
        // receiver, so Dual mode's own setup/update functions
        // shouldn't touch SerialGPSHeading at all either way.
        headingReceiverConfigured = false;
    }

    // SerialImuTM171 keeps its Serial5 default unless a slot claims
    // TM171 — deliberately no "else Serial5" branch needed here, since
    // the file-scope initialiser above already set that default.
    if      (BoardSlot1 == SLOT_TM171) { SerialImuTM171 = &Serial7; ImuType = IMU_TM171; }
    else if (BoardSlot2 == SLOT_TM171) { SerialImuTM171 = &Serial2; ImuType = IMU_TM171; }
}

// BNO08x I2C clock speed.
// 400000 (400 kHz, Fast Mode) — default, works on most setups.
// 100000 (100 kHz, Standard Mode) — use if tractor rotates 360° in AGO,
//         indicating I2C communication errors with BNO08x.
#define IMU_I2C_CLOCK 400000

// BNO08x report interval (ms). 20 ms = 50 Hz, faster than GPS so
// the freshest possible IMU data is always ready when GGA arrives.
#define REPORT_INTERVAL 20
uint32_t READ_BNO_TIME = 0;

// --- BNO08x watchdog state ---
uint32_t lastImuDataTime = 0;
uint32_t imuRestartTime  = 0;
uint8_t  imuRetryCount   = 0;
bool     imuHealthy      = false;

// --- LED pins (AIO standard) ---
#define GGAReceivedLED        13
#define Power_on_LED           5
#define Ethernet_Active_LED    6
#define GPSRED_LED             9
#define GPSGREEN_LED          10
#define AUTOSTEER_STANDBY_LED 11
#define AUTOSTEER_ACTIVE_LED  12
uint32_t gpsReadyTime = 0;

// --- Network ---
struct ConfigIP {
    uint8_t ipOne   = 192;
    uint8_t ipTwo   = 168;
    uint8_t ipThree = 137;
};
ConfigIP networkAddress;
byte Eth_myip[4] = { 0, 0, 0, 0 };
byte mac[]       = { 0x00, 0x00, 0x56, 0x00, 0x00, 0x78 };

unsigned int portMy           = 5120;  // this module's listening port
unsigned int AOGNtripPort     = 2233;  // NTRIP data from AOG
unsigned int AOGAutoSteerPort = 8888;  // autosteer data from AOG
unsigned int portDestination  = 9999;  // AGO listening port
unsigned int portMonitorIn    = 5556;  // monitor → Teensy commands
unsigned int portMonitorOut   = 5555;  // Teensy → monitor diagnostics

char Eth_NTRIP_packetBuffer[512];  // buffer for incoming NTRIP data

EthernetUDP Eth_udpPAOGI;     // PAOGI out   (port 9999)
EthernetUDP Eth_udpNtrip;     // NTRIP in    (port 2233)
EthernetUDP Eth_udpAutoSteer; // autosteer   (port 8888)
EthernetUDP Eth_udpMonitor;   // monitor in  (port 5556) + diag out (port 5555)
IPAddress Eth_ipDestination;

bool Autosteer_running = true;
bool Ethernet_running  = false;   // did Ethernet.begin() succeed at boot — never re-checked after setup()
bool EthernetLinkUp    = false;   // live physical link status, updated every loop() iteration —
                                    // see the loop() LED block, which already called
                                    // Ethernet.linkStatus() every iteration but only used it to
                                    // drive an LED, never recorded it anywhere or acted on it. A
                                    // cable pulled mid-session previously left Ethernet_running
                                    // permanently true, so firmware kept trying to send PAOGI/PDIAG
                                    // over a dead link indefinitely, with nothing anywhere
                                    // reflecting that it had happened — a real, silent failure
                                    // mode, not just a missing diagnostic. Diagnostic only: this
                                    // does not change what firmware does when the link is down
                                    // (Eth_udpPAOGI.write() to a dead link is harmless, it just
                                    // goes nowhere) — only what gets reported.
bool GGA_Available     = false;
bool blink             = false;

// --- Sentence-ready flags ---
// GGA and HPR arrive at 10 Hz but at slightly different times.
// PAOGI is sent only when both have arrived in the same cycle.
bool dualReadyGGA    = false;  // set true by GGA_Handler
bool dualReadyRelPos = false;  // set true by HPR_Handler or fallback path

// --- Raw BNO08x values (degrees, updated at ~50 Hz) ---
float yaw   = 0.0f;   // heading relative to BNO startup orientation
float roll  = 0.0f;   // lateral tilt (positive = right lean)
float pitch = 0.0f;   // fore-aft tilt

double correctionHeading = 0.0;  // raw BNO yaw in radians for offset calc

// --- Heading offset: BNO yaw → absolute azimuth ---
// Tracks the difference between HPR heading and raw BNO yaw via EMA
// while dual is active. Applied during fallback so BNO heading
// approximates the correct absolute azimuth.
double headingOffset = 0.0;

// --- Dual (HPR) values ---
double heading  = 0.0;   // HPR words[1], degrees 0-360
double rollDual = 0.0;   // HPR words[2] (pitch field used as roll)
int    solQuality = 0;   // HPR words[4], 0-5

// Kalman filters — used only when filterHeading / filterRoll = true.
// Declared as pointers, constructed inside setup() AFTER
// loadAlphasFromEEPROM() runs, not as global objects here — a plain
// global "SimpleKalmanFilter rollFilter(rollMEA, rollEST, rollQ);"
// would be constructed by C++ static initialization BEFORE setup()
// (and therefore before loadAlphasFromEEPROM()) ever runs, permanently
// baking in the compile-time default values (1.0/1.0/0.01) regardless
// of what SETROLLKALMAN/SETHEADINGKALMAN saved to EEPROM — no amount
// of rebooting would ever have picked up a tuned value. Found via
// initialization-order analysis while auditing the full package, not
// from a symptom report — fixed here before it ever shipped as a
// working feature that silently wasn't.
SimpleKalmanFilter* rollFilter    = nullptr;
SimpleKalmanFilter* headingFilter = nullptr;

// Serial and NMEA buffers
char msgBuf[254];
int  msgBufLen = 0;
bool gotCR = false, gotLF = false, gotDollar = false;

NMEAParser<4> parser;

// Dual reconnect stability timer.
// When slave reconnects and dual becomes available, we hold alpha at 1.0
// for DUAL_STABLE_HOLD iterations (~3s at 10Hz) to let HPR stabilize,
// then ramp alpha down to initialAlpha over DUAL_RAMP_UPDATES (~5s).
// If dual is lost during hold, counter resets.
// Dual reconnect timing — adjustable from Teensy Tool, saved to EEPROM.
// Times in milliseconds. At 10Hz HPR: 1000ms = 10 iterations.
int  dualHoldUpdates = 30;      // default 3s → stored as 10Hz iterations
int  dualRampUpdates = 50;      // default 5s → stored as 10Hz iterations
bool dualWasHealthy  = false;

elapsedMillis dualHoldTimer;    // counts ms since hold phase started
elapsedMillis dualRampTimer;    // counts ms since ramp phase started
bool dualInHold = false;        // currently in hold phase
bool dualInRamp = false;        // currently in ramp phase

constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];
uint8_t GPStxbuffer[serial_buffer_size];
uint8_t RTKrxbuffer[serial_buffer_size];
// Extra buffer for SerialGPSHeading (the second, "heading role" GNSS
// port, MODE_DUAL only — F9P/UM980 pairs). Without this, the port only
// gets Teensy's small default hardware buffer, which is smaller than a
// single UBX-NAV-RELPOSNED frame (72 bytes) or UNIHEADING2 ASCII line
// (~150-200 chars) at 460800 baud — risking dropped/corrupted bytes if
// loop() doesn't drain it fast enough. Confirmed against the official
// AIO v4 F9P firmware, which allocates the equivalent buffer
// (GPS2rxbuffer/GPS2txbuffer) for exactly this reason.
uint8_t GPSHeadingRxBuffer[serial_buffer_size];
uint8_t GPSHeadingTxBuffer[serial_buffer_size];

byte          velocityPWM_Pin      = 36;
elapsedMillis speedPulseUpdateTimer = 0;
elapsedMillis diagTimer             = 0;  // PDIAG send interval

// Forward declarations
void errorHandler();
void GGA_Handler();
void VTG_Handler();
void HPR_Handler();
void readBNO();
void TM171_setup();   // zIMU_TM171.ino
void TM171_update();  // zIMU_TM171.ino
void TM171_resetParser();  // zIMU_TM171.ino
void updateHeadingOffset(double hprHeading);
double applyHeadingKalman(double rawHeadingDeg);  // wraparound-safe Kalman wrapper, see zHandlers.ino
void imuHandler();
void BuildNmea();
void CalculateChecksum();
void checkImuWatchdog();
void updateAlphaControl();
void updateHeadingOffsetFromCOG(double latRad, double lonRad);
void GGAH_Handler_Raw(const char* buf, int len);
void loadAlphasFromEEPROM();
void resolveBoardSlots();
void keyaSetup();
void keyaSendSpeed(int16_t pwmValue, bool enable);
void keyaReceive();
float keyaGetSteerAngle();

// extern, not a forward declaration the way the functions above are —
// wasLeft/wasRight are VARIABLES actually defined in zHandlers.ino
// (float wasLeft = -45.0f; float wasRight = 45.0f;), which sorts
// AFTER c00_Autosteer.ino in Arduino's concatenation order. Found
// during a later audit pass: c00_Autosteer.ino's WAS plausibility
// check (added earlier, before this specific check was run) uses both
// of these directly, with no forward visibility at all — a genuine
// compile-blocking bug ("wasLeft was not declared in this scope"), not
// a false positive like the handful of known safe comment-only
// mentions elsewhere in this file. A plain, non-extern re-declaration
// here would conflict with zHandlers.ino's actual definition (Arduino
// concatenates every .ino into one single translation unit, so this
// isn't separate-file linking — a second plain "float wasLeft;" would
// be a duplicate definition, not just a second declaration), so this
// needs the explicit extern keyword specifically.
extern float wasLeft;
extern float wasRight;

// Same reasoning as wasLeft/wasRight above — keyaDetected/
// keyaFaultActive are defined in zKeya.ino, which sorts AFTER
// zHandlers.ino in Arduino's concatenation order, but zHandlers.ino's
// own sendDiagnostics() (PDIAG) uses both directly. Found by the same
// cross-reference check that caught wasLeft/wasRight — run
// deliberately again after this addition specifically because of that
// earlier miss, not assumed safe this time.
extern bool keyaDetected;
extern bool keyaFaultActive;
void saveAlphasToEEPROM();
void sendDiagnostics();
// GNSS source dispatch (TFF) — one setup()/update() pair per source,
// see zzGNSS_UM982.ino / zzGNSS_SingleIMU.ino / zzGNSS_DualF9P.ino /
// zzGNSS_DualUM980.ino. Exactly one pair is called, based on GnssMode
// (and DualReceiverType for MODE_DUAL) — see setup()/loop() below.
void um982_setup();      void um982_update();
void singleImu_setup();  void singleImu_update();
void gnssPassthrough_setup();  void gnssPassthrough_update();
void updateDiagnostics();
void readResetCause();
void dualF9P_setup();    void dualF9P_update();
void dualUM980_setup();  void dualUM980_update();
void receiveMonitorCommands();
void autosteerSetup();
void autosteerLoop();
void ReceiveUdp();
void EthernetStart();
void udpNtrip();

// =====================================================================
// Diagnostic-only health monitoring — added for later troubleshooting
// via Teensy Tool's Logfile feature. PURELY DIAGNOSTIC: nothing here
// disables, limits, corrects, or otherwise changes steering, GNSS,
// CAN, or any other behaviour — every flag below is reported via
// PDIAG and read, never acted on, matching the explicit scope agreed
// for this feature (listen, don't control).
//
// Each *Timeout flag follows the same shape: a "last seen" timestamp
// updated wherever that data source is actually touched, checked once
// per loop() against its own threshold, defaulting to false/healthy
// until enough time has passed with no activity to prove otherwise —
// same pattern already used for checkImuWatchdog() and the existing
// >10s "NO SIGNAL" detection in Teensy Tool, just generalised to a few
// more sources that never had it.
// =====================================================================
bool     wasImplausible       = false;  // out-of-calibrated-range or an implausible single-step jump
bool     canKBusTimeout       = false;  // no K_Bus (tractor/control) message in CAN_MSG_TIMEOUT_MS
bool     canISOBusTimeout     = false;  // no ISO_Bus message in CAN_MSG_TIMEOUT_MS
bool     canVBusTimeout       = false;  // no V_Bus (steering valve) message in CAN_MSG_TIMEOUT_MS
bool     canContentImplausible = false; // a CAN message arrived, but its content didn't make sense
bool     gnssWatchdogTimeout  = false;  // no byte at all from SerialGPS in GNSS_WATCHDOG_TIMEOUT_MS
bool     rtkRadioTimeout      = false;  // no byte at all from SerialRTK in RTK_RADIO_TIMEOUT_MS
bool     hprTimeout           = false;  // no successfully-parsed $GPHPR sentence in
                                          // HPR_WATCHDOG_TIMEOUT_MS — deliberately a
                                          // DIFFERENT check than gnssWatchdogTimeout
                                          // above: that one only confirms SOME byte
                                          // arrived on the GNSS serial line (matching
                                          // the CAN watchdogs' own "successful message,
                                          // not just any byte" pattern is the whole
                                          // point here), so a receiver outputting GGA/
                                          // VTG correctly but never completing a valid
                                          // HPR sentence (wrong baud config, garbled
                                          // dual-antenna link, HPR output not actually
                                          // enabled on the receiver) would still show
                                          // gnssWatchdogTimeout==false the entire time.
                                          // Found genuinely missing during a real
                                          // debugging session where exactly that
                                          // happened — solQuality/satsMaster staying at
                                          // 0 was the only visible symptom, easy to
                                          // miss in the moment. Only meaningful for
                                          // GnssMode==MODE_UM982 (the only mode that
                                          // uses HPR_Handler() at all — Dual F9P/UM980
                                          // use UBX-RELPOSNED/UNIHEADING2 instead, an
                                          // entirely different mechanism); permanently
                                          // timed-out and not meaningful in every other
                                          // mode, same "gate interpretation, not
                                          // computation" convention as the CAN
                                          // timeouts (only meaningful when a CAN Brand
                                          // is active).
uint8_t  resetCause           = 0;      // 0=unknown, 1=power-on, 2=watchdog, 3=software — see
                                          // readResetCause() (zHandlers.ino), read once at boot

uint32_t lastKBusMsgTime   = 0;
uint32_t lastISOBusMsgTime = 0;
uint32_t lastVBusMsgTime   = 0;
uint32_t lastGnssByteTime  = 0;
uint32_t lastRtkByteTime   = 0;
uint32_t lastHprMsgTime    = 0;   // set inside HPR_Handler() itself (zzGNSS_UM982.ino),
                                    // which only ever runs when the NMEAParser library
                                    // has already confirmed a complete, checksum-valid
                                    // $GPHPR sentence — same "successful message, not
                                    // just a byte" guarantee the CAN watchdogs already
                                    // have, deliberately not replicating
                                    // lastGnssByteTime's weaker "byte available" check.

#define CAN_MSG_TIMEOUT_MS         3000   // 3s — generous relative to
                                            // typical CAN heartbeat
                                            // rates (tens to hundreds
                                            // of ms), avoids false
                                            // positives from a merely
                                            // slow/quiet bus
#define GNSS_WATCHDOG_TIMEOUT_MS   5000   // 5s — matches the rough
                                            // order of magnitude
                                            // already used for
                                            // "signal lost" in Teensy
                                            // Tool (10s there, but
                                            // that also accounts for
                                            // UDP delivery; this is
                                            // the more immediate,
                                            // serial-level check)
#define RTK_RADIO_TIMEOUT_MS       15000  // 15s — RTCM correction
                                            // streams are typically
                                            // ~1Hz or slower per
                                            // message type, so this
                                            // needs to be looser than
                                            // the GNSS/CAN checks to
                                            // avoid false positives
                                            // during normal operation
#define HPR_WATCHDOG_TIMEOUT_MS    8000   // 8s — deliberately looser
                                            // than GNSS_WATCHDOG_
                                            // TIMEOUT_MS's 5s, not
                                            // tighter: a brief real-
                                            // world dropout (a few
                                            // missed HPR cycles) is
                                            // rarely something the
                                            // driver even notices in
                                            // practice, and steering
                                            // behaviour visibly
                                            // degrading typically
                                            // takes several seconds
                                            // to become apparent —
                                            // this watchdog is meant
                                            // to catch a genuine,
                                            // persistent loss, not
                                            // flag every momentary
                                            // hiccup.


// =====================================================================
// SETUP
// =====================================================================
void setup()
{
    delay(500);

    // As early as possible — before anything else could plausibly
    // touch SRC_SRSR — see readResetCause()'s own header comment
    // (zHandlers.ino) for why this specific register needs reading
    // before it's disturbed.
    readResetCause();

    pinMode(GGAReceivedLED,        OUTPUT);
    pinMode(Power_on_LED,          OUTPUT);
    pinMode(Ethernet_Active_LED,   OUTPUT);
    pinMode(GPSRED_LED,            OUTPUT);
    pinMode(GPSGREEN_LED,          OUTPUT);
    pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
    pinMode(AUTOSTEER_ACTIVE_LED,  OUTPUT);

    // NMEAParser stores max 5 chars of token so G-GGAH truncates to G-GGA,
    // conflicting with GGA_Handler. GGAH is parsed separately in loop().
    //
    // GCC warns about the library's own internal strncpy() call for
    // these 5-character tokens (-Wstringop-truncation) — confirmed a
    // harmless false positive: zNMEAParser.h's mToken is actually
    // char[6] (room for a null terminator), and the library explicitly
    // sets mToken[5]='\0' on the line immediately after that strncpy()
    // call. Suppressed at the #include "zNMEAParser.h" site itself, up
    // near the top of this file — NOT with a push/ignored/pop wrapped
    // around these three call sites (tried that first, in v0.3.14; it
    // does NOT work for this specific case, since addHandler() is a
    // TEMPLATE method — the actual strncpy() the warning points at is
    // textually written inside the template's own body in
    // zNMEAParser.h, not here, and GCC's diagnostic pragmas apply based
    // on the TEXTUAL location of the code being compiled, not the
    // "inlined from" call site the warning message displays. Confirmed
    // by testing: the call-site pragma had zero effect, the exact same
    // warning still appeared verbatim in the next build).
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA",  GGA_Handler);   // master position + sats
    parser.addHandler("G-VTG",  VTG_Handler);   // speed
    parser.addHandler("G-HPR",  HPR_Handler);   // dual heading + roll

    Serial.begin(baudAOG);
    delay(10);
    Serial.println(F("TFF (Teensy Flexible Firmware) starting..."));

    // v0.3.2: autosteerSetup()/EthernetStart() moved back up here —
    // this is a restoration, not a new design. Confirmed (against a
    // preserved v0.5 snapshot, "000_UM982_Fallback") that this pair
    // originally ran early, immediately after the basic pin/parser/
    // Serial setup above, in every version through v0.7. Across
    // several later TFF sessions, new EEPROM-loading and GNSS-source-
    // dispatch code kept getting inserted directly above this pair —
    // each insertion reasonable on its own, but nobody tracking the
    // cumulative effect: by v0.3.1 the pair had drifted to running
    // after the entire GNSS source dispatch, autosteer no longer
    // matching its own proven-working position. Restored here.
    //
    // Verified safe in both directions before moving, not assumed:
    // EthernetStart() itself reads networkAddress (IP octets), which
    // is loaded from EEPROM inside autosteerSetup() itself, not by
    // loadAlphasFromEEPROM() — so the pair's own internal order
    // (autosteerSetup() before EthernetStart()) was never actually
    // the problem and is preserved unchanged here. Separately
    // confirmed that nothing in loadAlphasFromEEPROM()/
    // resolveBoardSlots()/any GNSS source's own setup() references
    // Autosteer_running, Ethernet_running, steerSettings, steerConfig,
    // or networkAddress — so moving the pair earlier, ahead of all of
    // that, doesn't introduce a new reverse dependency either.
    //
    // Whether this specific reordering actually explains any
    // particular observed symptom remains unconfirmed — restored
    // primarily because it's a genuine, provable regression back to
    // known-good structure, not because a specific failure was ever
    // traced to it directly.
    Serial.println(F("Starting AutoSteer..."));
    autosteerSetup();

    Serial.println(F("Starting Ethernet..."));
    EthernetStart();

    // Load GnssMode/DualReceiverType (and everything else EEPROM-backed
    // — alpha settings, IMU calibration, CAN Brand) BEFORE dispatching
    // GNSS source setup below, since that dispatch depends on knowing
    // which mode was actually selected last time, not just the
    // compiled-in default (MODE_UM982).
    loadAlphasFromEEPROM();   // also loads Brand, GnssMode, DualReceiverType

    // Resolve BoardSlot1/BoardSlot2 into actual Serial port pointers —
    // must run after loadAlphasFromEEPROM() (needs validated slot
    // values) and before the GNSS/IMU dispatch below (which reads
    // SerialGPS/SerialGPSHeading/SerialImuTM171 and ImuType).
    resolveBoardSlots();

    // Construct the Kalman filter objects now — AFTER EEPROM load, so
    // rollMEA/rollEST/rollQ/headingMEA/headingEST/headingQ hold whatever
    // was actually saved (or the defaults, on first boot), not the
    // file-scope literal defaults. See the rollFilter/headingFilter
    // declaration comment above for why this ordering matters.
    rollFilter    = new SimpleKalmanFilter(rollMEA,    rollEST,    rollQ);
    headingFilter = new SimpleKalmanFilter(headingMEA, headingEST, headingQ);

    // --- GNSS source dispatch ---
    // Each source's _setup() owns its own serial port init — see the
    // corresponding zGNSS_*.ino file. Exactly one of these runs; the
    // old hardcoded "SerialGPS->begin(...)" that used to sit here
    // directly is now inside um982_setup() specifically, since it's
    // only correct for that one mode.
    Serial.print(F("GNSS mode = "));
    Serial.println(GnssMode);
    Serial.print(F("GNSS passthrough = "));
    Serial.println(GnssPassthrough);
    if (GnssPassthrough)
    {
        gnssPassthrough_setup();
    }
    else switch (GnssMode)
    {
        case MODE_SINGLE_IMU: singleImu_setup(); break;
        case MODE_DUAL:
            Serial.print(F("  Dual receiver type = "));
            Serial.println(DualReceiverType);
            if (DualReceiverType == RX_F9P) dualF9P_setup();
            else                             dualUM980_setup();
            break;
        case MODE_UM982:
        default:               um982_setup();     break;
    }

    SerialRTK.begin(baudRTK);
    SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

    Serial.println(F("Serial ports initialized."));

    // --- CANBUS ---
    // Only touches the CAN hardware/buses at all if a brand other than
    // BRAND_NONE (8) is configured. With Brand == BRAND_NONE this block
    // is fully skipped, so CAN activity is exactly zero — identical to
    // the original CAN-less 000_UM982_Fallback.ino.
    // Also runs for Keya (MotorDriveType == MOTOR_DRIVE_KEYA) even when
    // Brand == BRAND_NONE — a Keya installation typically has no CAN-
    // ready tractor brand at all, but still needs V_Bus begin()'d/
    // filtered (keyaSetup()'s own filter addition lives inside
    // CAN_setup(), zCAN_All_Brands.ino). Safe to call unconditionally
    // here: every brand-specific filter/address-claim line inside
    // CAN_setup() is itself individually gated on Brand 0-7, so
    // calling it with Brand == BRAND_NONE only runs the shared
    // V_Bus/ISO_Bus/K_Bus begin()/baud/FIFO setup plus the new Keya
    // filter — no brand-specific behaviour is triggered.
    if (Brand != BRAND_NONE || MotorDriveType == MOTOR_DRIVE_KEYA)
    {
        Serial.print(F("Starting CANBUS, Brand = "));
        Serial.println(Brand);
        CAN_setup();
        if (MotorDriveType == MOTOR_DRIVE_KEYA)
        {
            Serial.println(F("  Keya CAN steering motor selected."));
            keyaSetup();
        }
    }
    else
    {
        Serial.println(F("CANBUS disabled (Brand = BRAND_NONE)."));
    }

    // --- IMU ---
    // ImuType selects which physical IMU to initialise — see the
    // ImuType comment block earlier in this file for why TM171 is an
    // explicit selection rather than auto-probed like BNO08x is.
    if (ImuType == IMU_NONE)
    {
        // "No fallback function" — deliberately skip all IMU
        // initialisation entirely. useIMU stays at its default
        // false and is never touched again, which is exactly what
        // every downstream gate (readBNO()/TM171_update() polling,
        // checkImuWatchdog(), COG calibration, the imuHandler()
        // fallback branch) already needs to correctly behave as
        // "no IMU present" — see the ImuType/IMU_NONE comment where
        // it's #defined for the full picture of how this threads
        // through the rest of the firmware.
        Serial.println(F("IMU disabled (ImuType = IMU_NONE, dual-only mode)."));
    }
    else if (ImuType == IMU_TM171)
    {
        Serial.println(F("Starting TM171..."));
        TM171_setup();
        useIMU       = true;   // "an IMU is active" — see the
                                    // useIMU comment block earlier;
                                    // true here because TM171_setup()
                                    // doesn't fail-detect the way the
                                    // BNO I2C scan below does — if
                                    // nothing is actually wired to
                                    // SerialImuTM171, no RPY frames will
                                    // ever arrive and checkImuWatchdog()
                                    // will keep retrying harmlessly,
                                    // same as it already does for a
                                    // disconnected/frozen BNO08x.
        imuHealthy      = false;
        lastImuDataTime = millis();
    }
    else
    {
    // Only enableGameRotationVector is called — NOT enableGyro.
    // Enabling both simultaneously seems to cause the BNO08x to stop responding
    // on AIO boards (from in field testing 2026-04).
    Serial.println(F("Starting BNO08x..."));
    ImuWire.begin();

    for (int16_t i = 0; i < nrBNO08xAddresses; i++)
    {
        bno08xAddress = bno08xAddresses[i];
        ImuWire.beginTransmission(bno08xAddress);
        uint8_t error = ImuWire.endTransmission();

        if (error == 0)
        {
            Serial.print(F("  0x")); Serial.print(bno08xAddress, HEX);
            Serial.print(F(" found. Initializing... "));

            if (bno08x.begin(bno08xAddress, ImuWire))
            {
                ImuWire.setClock(IMU_I2C_CLOCK);
                delay(300);
                bno08x.enableGameRotationVector(REPORT_INTERVAL);
                useIMU       = true;
                imuHealthy      = false;
                lastImuDataTime = millis();
                Serial.println(F("OK"));
            }
            else { Serial.println(F("begin() failed.")); }
        }
        else
        {
            Serial.print(F("  0x")); Serial.print(bno08xAddress, HEX);
            Serial.println(F(" not found."));
        }

        if (useIMU) break;
    }
    }

    Serial.print(F("useIMU = ")); Serial.println(useIMU);
    Serial.println(F("Setup complete. Waiting for GPS...\n"));
}


// =====================================================================
// MAIN LOOP
// =====================================================================
void loop()
{
    // --- GNSS source dispatch ---
    // Each source's _update() owns reading its own serial port(s) and
    // populating the shared heading/rollDual/solQuality/hprSats/
    // dualReady* variables that imuHandler()/BuildNmea() below consume
    // — see the corresponding zGNSS_*.ino file. The old hardcoded
    // "feed SerialGPS to parser + GGAH detection" block that used to
    // sit here directly is now inside um982_update() specifically.
    // GNSS serial watchdog — a single, central "did any byte arrive"
    // check here, rather than touching every individual GNSS source
    // file's own read loop (five files: um982/dualF9P/dualUM980/
    // singleImu/passthrough). Four of those five drain fully via their
    // own while (available()) loop each iteration; gnssPassthrough_
    // update() reads only one byte per call instead (matching the
    // confirmed original source's own "if", not "while" — a
    // deliberately faithful port, not an inconsistency introduced
    // here). Either pattern keeps this watchdog meaningful regardless:
    // loop() iterations happen far more often than GNSS bytes actually
    // arrive at any realistic baud rate, so available() reliably
    // reflects "recent" activity either way, not stale leftover bytes
    // from several iterations ago.
    if (SerialGPS->available()) lastGnssByteTime = millis();

    // GnssPassthrough bypasses the normal GNSS source dispatch
    // entirely — see the GnssPassthrough declaration comment in this
    // file for why it's a separate toggle rather than another GnssMode
    // value. The shared RTK/NTRIP forwarding a few lines below still
    // runs unconditionally either way, since a receiver in passthrough
    // mode still needs corrections — gnssPassthrough_update() itself
    // only handles the receiver→AGO direction, not RTK→receiver.
    if (GnssPassthrough)
    {
        gnssPassthrough_update();
    }
    else switch (GnssMode)
    {
        case MODE_SINGLE_IMU: singleImu_update(); break;
        case MODE_DUAL:
            if (DualReceiverType == RX_F9P) dualF9P_update();
            else                             dualUM980_update();
            break;
        case MODE_UM982:
        default:               um982_update();     break;
    }

    // Forward RTCM corrections (radio → GNSS position receiver)
    while (SerialRTK.available())
    {
        lastRtkByteTime = millis();
        SerialGPS->write(SerialRTK.read());
    }

    // Forward NTRIP corrections (AOG → GNSS position receiver)
    udpNtrip();

    // Diagnostic-only health monitoring — see updateDiagnostics()'s
    // own header comment (zHandlers.ino). Runs every iteration,
    // regardless of GnssMode/GnssPassthrough/Brand — the function
    // itself already handles "not applicable in this configuration"
    // internally (e.g. CAN checks stay false when Brand==BRAND_NONE).
    updateDiagnostics();

    // Update alpha with latest solQuality from this iteration — except
    // in MODE_SINGLE_IMU, where HEADING_ALPHA/ROLL_ALPHA are pinned at
    // 1.0 by singleImu_setup()/singleImu_update() and must stay that
    // way; calling this here would be harmless in practice (solQuality
    // is held at -1, which the < HPR_QUALITY_MIN branch already treats
    // as "want alpha 1.0"), but skipping it entirely is clearer and
    // matches the TFF architecture reference's explicit decision that
    // this mode never calls updateAlphaControl() at all. GnssPassthrough
    // is excluded for the same reason — no fusion of any kind happens
    // in that mode, so there is no alpha to update. Both exclusions are
    // belt-and-braces here: dualReadyGGA never actually becomes true in
    // either mode anyway (the normal parser never runs), so this block
    // would already be naturally inert without the explicit check —
    // kept explicit anyway so a future change elsewhere can't
    // accidentally make it fire.
    if (GnssMode != MODE_SINGLE_IMU && !GnssPassthrough)
        updateAlphaControl();

    // Normal path: send PAOGI when GGA + HPR both arrived this cycle
    // This matches original: imuHandler() then BuildNmea() in loop()
    if (!GnssPassthrough && dualReadyGGA && dualReadyRelPos)
    {
        imuHandler();
        BuildNmea();
        dualReadyGGA    = false;
        dualReadyRelPos = false;
    }

    // Fallback path: GGA arrived but no valid HPR → use BNO
    if (!GnssPassthrough && dualReadyGGA && !dualReadyRelPos && useIMU)
    {
        imuHandler();
        BuildNmea();
        dualReadyGGA = false;
    }

    // IMU watchdog (non-blocking soft restart on hang) — see
    // checkImuWatchdog()'s header comment (zHandlers.ino) for how the
    // recovery action branches on ImuType internally.
    if (useIMU) checkImuWatchdog();

    // Poll the active IMU. TM171 is UART-based — TM171_update() just
    // drains whatever bytes have arrived through its own state machine,
    // so it's called unconditionally every iteration (cheap, matches
    // how the GNSS serial sources are drained). BNO08x is I2C-polled at
    // a controlled ~50Hz rate instead — unlike UART, hammering an I2C
    // read every single loop() iteration would be wasteful/could
    // contend with other I2C traffic, so its own rate gate stays as-is.
    if (ImuType == IMU_TM171)
    {
        if (useIMU) TM171_update();
    }
    else if ((systick_millis_count - READ_BNO_TIME) > REPORT_INTERVAL && useIMU)
    {
        READ_BNO_TIME = systick_millis_count;
        readBNO();
    }


    // Receive commands from Teensy Tool (e.g. SETHEADINGALPHA, SETBRAND)
    if (Ethernet_running) receiveMonitorCommands();

    // --- CANBUS polling (only when a CAN brand is configured) ---
    // Brand == BRAND_NONE (8): this whole block is skipped every loop,
    // so there is zero CAN bus traffic — identical to the CAN-less
    // 000_UM982_Fallback.ino. setCurve is still computed harmlessly in
    // motorDrive() above even when skipped here, but nothing is ever
    // sent onto the bus, and nothing is ever read from it.
    if (Brand != BRAND_NONE)
    {
        // VBus_Send() is rate-limited to ~50ms (within the 40-80ms
        // interval documented by AgOpenGPS for CAN steering valve
        // messages — see discourse.agopengps.com CanBus for Beginners).
        // Without this gate it would fire on every loop() iteration,
        // thousands of times per second, flooding V_Bus far faster
        // than any steering controller expects.
        if (vbusSendTimer > 50)
        {
            vbusSendTimer = 0;
            VBus_Send();    // send desired curve/angle to steering valve
        }
        VBus_Receive();     // read actual angle + valve-ready state
        ISO_Receive();      // read ISOBUS traffic (mostly unused)
        K_Receive();        // read tractor bus (e.g. engage button)
    }

    // Keya heartbeat receive — deliberately OUTSIDE the Brand-gated
    // block above (a Keya installation typically has Brand ==
    // BRAND_NONE, where that whole block is skipped) and gated on
    // MotorDriveType instead. Reads whatever V_Bus traffic has arrived
    // since the last call — see keyaReceive() (zKeya.ino) for the
    // heartbeat parsing itself.
    if (MotorDriveType == MOTOR_DRIVE_KEYA)
    {
        keyaReceive();
    }

    if (Brand != BRAND_NONE)
    {

        // Hydraulic hitch/implement lift control — ported from the
        // original CAN implementation ("if (Brand == 3) SetRelaysFendt();
        // if (Brand == 0) SetRelaysClaas();"). Only Fendt and Claas had
        // this feature in the source material; other brands don't call
        // anything here, matching original scope exactly.
        if (Brand == 3) SetRelaysFendt();
        if (Brand == 0) SetRelaysClaas();

        // engageCAN reset — ported from the original CAN implementation
        // (Autosteer_AOGv5_Teensy4_1UDP_SteerReadyCAN_AIOModified.ino,
        // "if ((millis()) > relayTime) { ...; engageCAN = 0; }").
        // engageCAN is set to 1 in several places in zCAN_All_Brands.ino
        // (K_Receive()/VBus_Receive(), whenever the tractor's own CAN
        // engage button is pressed), each time also setting
        // relayTime = millis() + 1000 — but nothing ever reset it back
        // to 0 anywhere in this codebase. Confirmed by grep: engageCAN
        // was written to 1 in a dozen places and never once reset.
        // Without this, engageCAN stays 1 forever after the first CAN
        // engage — which matters because engageCAN==1 is used below
        // (autosteerLoop()'s watchdog-timeout branch) to deliberately
        // hold the safety valve open. Left unreset, that hold would
        // become permanent instead of a 1-second window, defeating the
        // watchdog's ability to ever cut the valve again after the
        // first engage. This is why this fix cannot be added without
        // ALSO adding the reset — the two are safety-coupled.
        //
        // DEVIATION FROM ORIGINAL: the original's equivalent block also
        // force-writes AUTOSTEER_STANDBY_LED/AUTOSTEER_ACTIVE_LED here,
        // every single iteration once relayTime has elapsed (i.e.
        // almost always). In our code those same two LEDs are already
        // correctly driven by autosteerLoop() based on whether PID is
        // actively steering — copying the original's unconditional LED
        // writes here would fight that and cause flicker/incorrect
        // display. Only the engageCAN reset itself is ported; the LED
        // writes are deliberately omitted.
        if (millis() > relayTime) engageCAN = 0;
    }

    // Send PDIAG diagnostic sentence every 2 seconds
    if (Ethernet_running && diagTimer > 2000)
    {
        diagTimer = 0;
        sendDiagnostics();
    }

    if (Autosteer_running) autosteerLoop();
    else ReceiveUdp();

    // Ethernet link LED — now also records live link status in
    // EthernetLinkUp (see its declaration for why this was a real gap,
    // not just a missing diagnostic).
    if      (Ethernet.linkStatus() == LinkOFF)
    { digitalWrite(Power_on_LED, HIGH); digitalWrite(Ethernet_Active_LED, LOW);  EthernetLinkUp = false; }
    else if (Ethernet.linkStatus() == LinkON)
    { digitalWrite(Power_on_LED, LOW);  digitalWrite(Ethernet_Active_LED, HIGH); EthernetLinkUp = true;  }
}
