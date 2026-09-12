# TFF — Teensy Flexible Firmware, for AgOpenGPS

A flexible autosteer firmware for the Teensy 4.1, built to make setting up and tuning an [AgOpenGPS](https://github.com/farmerbriantee/AgOpenGPS) installation dramatically easier — most configuration (GNSS mode, IMU type, board wiring, CAN steering brand, sensor calibration, and more) can be changed live over the network from a companion desktop tool, **without ever needing to re-flash the firmware**.

This repository contains three parts that work together:

```
Firmware/020_TFF/    ← the Teensy 4.1 firmware itself (Arduino, C++)
TeensyToolCs/         ← the configuration/monitoring tool (C#, WinForms)
teensy_tool.py        ← the same tool, as a single-file Python alternative
```

---

## What TFF does

TFF replaces the need to hand-edit and re-upload firmware every time a setting changes. Instead, you flash it **once**, and then configure everything else — live, from a desktop or laptop on the same network — using either companion tool.

### GNSS support (three modes, chosen live)
- **UM982** (UnicoreComm dual-antenna-in-one-receiver, includes onboard heading/roll)
- **Single receiver + IMU** (any single GNSS receiver, heading/roll from the IMU alone)
- **Dual receiver** (two separate GNSS receivers — u-blox F9P moving-baseline, or UnicoreComm UM980)

### IMU support
- BNO08x (I2C)
- TM171 / SYD Dynamics TransducerM (UART)
- Or none at all, if a dual GNSS setup already provides heading/roll

### Sensor fusion
Continuously blends GNSS-derived heading/roll with the IMU's own reading, weighted by live signal quality — not a fixed, one-size-fits-all ratio. Falls back gracefully to IMU-only output the moment the GNSS signal genuinely degrades, and recovers automatically when it improves.

### Auto-calibration
- **Heading offset**: automatically tracks and corrects the IMU's own, otherwise-arbitrary yaw against the GNSS-derived heading, with a rate-of-change plausibility check to reject bad momentary fixes.
- **Auto Roll Adjust**: continuously nudges the roll offset between the dual GNSS signal and the IMU to correct for slow sensor-mounting drift over time — gated on a strict RTK-fixed signal, a post-acquisition settling window, and a configurable noise deadband, so it only ever reacts to genuine, sustained drift, never momentary noise.
- **Keya CAN motor auto-zero**: for installations using a Keya CAN steering motor as the wheel-angle sensor (rather than a physical potentiometer), continuously establishes and maintains the motor's own "straight ahead" reference point — since the motor's encoder only measures *relative* rotation, with no built-in sense of centre at all.

### CAN bus steering
Supports proportional-valve steering over CAN for multiple tractor brands (Claas, Valtra/Massey Ferguson/McCormick, Case IH, Fendt/FendtONE, JCB, Lindner, and AgOpenGPS's own CAN protocol), plus Keya CAN steering motors as a WAS alternative.

### Diagnostics
A continuous, structured `$PDIAG` sentence reports the firmware's full internal state (signal quality, active faults, sensor fusion weights, calibration values, and more) over UDP — this is what both companion tools read to display live status and let you tune settings without ever touching the firmware source.

---

## The companion tools

Both tools talk to the firmware over the same UDP protocol and offer the same core functionality — pick whichever fits your platform:

- **`TeensyToolCs/`** (C#, .NET Framework 4.8, WinForms) — the primary, actively maintained version.
- **`teensy_tool.py`** (Python, Tkinter) — a single, self-contained file with no separate dependencies to install, useful on platforms where running a .NET application isn't convenient.

Both provide: live sensor readings, per-tab configuration for every firmware setting described above, an on-screen numeric keypad for touchscreen use, a diagnostic log file with an automated problem-summary analyzer, and a raw-command field for anything not yet exposed through a dedicated control.

---

## Building the firmware

Open `Firmware/020_TFF/020_TFF.ino` in the Arduino IDE (the folder name must match the main file's name exactly — this is a hard Arduino requirement, not a style choice). Required libraries: `FlexCAN_T4`, `SimpleKalmanFilter`, `NativeEthernet`, `EEPROM` (all installable through the Arduino Library Manager or PlatformIO).

## Building the tools

**C#**: open `TeensyToolCs/` in Visual Studio, or build from the command line with `dotnet build` / `dotnet publish`.
**Python**: requires only Python 3.10+ (for its built-in PNG support in Tkinter) — `python teensy_tool.py`, no other packages needed.

---

## Sources and acknowledgements

TFF is not written from scratch — it builds directly on, and gives credit to, the following community work:

- **UM982 Fallback Firmware v0.7** — the original community firmware TFF is built directly on top of, and reuses unmodified wherever possible.
- **[AgOpenGPS](https://github.com/farmerbriantee/AgOpenGPS)** — the guidance software this firmware and its tools exist to support. TFF communicates with it using AgOpenGPS's own PGN/UDP protocol.
- **[AgOpenGPS-Official/Boards](https://github.com/AgOpenGPS-Official/Boards)** — the community hardware/firmware repository this project's CAN-bus multi-brand steering logic and the companion tool's on-screen keypad controls were adapted from.
- **[Flodu81/AIO_ECU_Keya_WasKeyaFiltre](https://github.com/Flodu81/AIO_ECU_Keya_WasKeyaFiltre)** — the source of the Keya CAN motor auto-zero algorithm, ported into TFF after a real field report of the motor spinning continuously without it.
- **[docs.agopengps.com](https://docs.agopengps.com)** and the **[AgOpenGPS Discourse forum](https://discourse.agopengps.com)** — referenced throughout for hardware wiring conventions, CAN protocol details, and community-reported field issues.
- **FlexCAN_T4**, **SimpleKalmanFilter**, **NativeEthernet** — the third-party Arduino libraries this firmware depends on for CAN bus, sensor filtering, and networking respectively.

If a source used somewhere in this codebase is missing from this list, it's an oversight worth raising as an issue — not an intentional omission.

---

## License

GPLv3 the convention used by AgOpenGPS 
