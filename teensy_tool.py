"""
Teensy Tool  —  v0.3.18
====================
(Renamed from UM982 Fallback Monitor. Part of the TFF (Teensy Flexible
Firmware) project — has its own v0.1-onward version history, separate
from UM982 Fallback Firmware's own v0.1-v0.7 numbering. Drives TFF v0.3.18
firmware: five tabs (Receiver Configuration, IMU, Vehicle Configuration,
Operation, Board Configuration), GNSS mode + dual-receiver-type
selection, Motor Drive / WAS Source (including Keya CAN steering
motor support), a local logfile feature with 30s "STATUS OK" heartbeat
summaries, and the Dual Roll/IMU Roll live comparison panel are all
implemented and wired to real firmware commands — this is no longer
the single-panel tool it started as.)

Listens for $PDIAG sentences from Teensy on UDP port 5555.
Displays dual/fallback status, satellite counts, heading offset,
and current fusion alpha. Allows setting new values live.

$PDIAG format (sent by Teensy every 2 seconds), 22 comma-separated
fields after $PDIAG — see parse_pdiag() below for the authoritative
field order:
  $PDIAG,MODE,SATS_M,SATS_S,HPR_SATS,SOL,HDG_OFF,H_ALPHA,R_ALPHA,
         INIT_H,INIT_R,DUAL_PCT,SATS_FULL,ROLL_ZERO,DUAL_HOLD_S,
         DUAL_RAMP_S,IMU_AXIS,ROLL_INVERT,WAS_LEFT,WAS_RIGHT,
         UTURN_STRENGTH,STEER_ACTUAL,BRAND*CS

  MODE        DUAL or FALLBACK
  SATS_M      satellites on master antenna (from GGA)
  SATS_S      satellites on slave antenna (from GPGGAH)
  HPR_SATS    satellites used in the HPR heading solution
  SOL         HPR solution quality (0-5, 4=RTK fixed, 5=RTK float)
  HDG_OFF     IMU-to-HPR heading offset (degrees)
  H_ALPHA     current HEADING_ALPHA (auto-adjusted, 0=pure HPR, 1=pure IMU)
  R_ALPHA     current ROLL_ALPHA
  BRAND       CAN steering brand (0-7), or 8 = BRAND_NONE (no CAN, PWM only)

Commands sent from monitor to Teensy (UDP port 5556):
  SETHEADINGALPHA:x.xxxx   SETROLLALPHA:x.xxxx
  SETSATSFULL:N            SETROLLZERO:x.xx
  SETDUALHOLD:x.x          SETDUALRAMP:x.x
  SETIMUAXIS:N             SETROLLINVERT:N
  SETWASCURRENT:L/R        SETUTURNSTRENGTH:N
  SETBRAND:N               (0-7 = CAN brand, 8 = no CAN / classic PWM)

ALPHA RECOMMENDATIONS
---------------------
  0.00        Pure HPR (dual only). Recommended starting point.
              No IMU influence. Most accurate when dual is solid.

  0.05-0.10   Light IMU smoothing. Try this if heading looks
              "jumpy" while driving on rough terrain.
              Adds <100ms of lag at 10Hz HPR rate.

  0.15-0.25   Moderate smoothing. Useful with very short baseline
              or noisy RTK conditions. Some lag on fast turns.

  > 0.30      Heavy IMU weighting. Not recommended for normal use.
              May cause slow response to heading changes.

  1.00        Pure IMU (no HPR influence). Only for testing.
"""

import socket
import os
import threading
import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont
import time
import re
import datetime

# -----------------------------------------------------------------------
# U-Turn Dual Boost explanatory diagram — embedded as base64, NOT a
# separate image file. Deliberate: this tool has always been
# distributed and run as ONE single, standalone .py file (no
# installer, no accompanying resources) — requiring a second file
# (the PNG) to sit alongside it would introduce a new, easy way for
# someone to lose/misplace it and crash the tool on startup. Diagram
# provided directly by the person running this project (their own
# explanation of U-Turn Strength, also posted to the AgOpenGPS forum)
# — shows how the DUAL blend factor rises toward 1.0 near "Straight"
# and falls off toward "1/strength" at full lock in either direction.
# tkinter's PhotoImage has native PNG support since Python 3.10 (via
# Tk 8.6) — no Pillow/PIL dependency needed for this.
UTURN_BOOST_DIAGRAM_B64 = (
    "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"
)

# -----------------------------------------------------------------------
# Network configuration
# -----------------------------------------------------------------------
LISTEN_PORT  = 5555   # Teensy sends PDIAG here
SEND_PORT    = 5556   # Monitor sends SETxxx commands here
BROADCAST    = '255.255.255.255'

# -----------------------------------------------------------------------
# Colours (dark terminal theme)
# -----------------------------------------------------------------------
BG      = '#0d1117'
FG      = '#c9d1d9'
GREEN   = '#3fb950'
RED     = '#f85149'
YELLOW  = '#d29922'
BLUE    = '#58a6ff'
DIM     = '#adbac7'
WHITE   = '#ffffff'
PANEL   = '#161b22'
BORDER  = '#30363d'

SOL_LABELS = {
    0: ('No solution',  RED),
    1: ('Single-point', YELLOW),
    2: ('Diff/Float',   YELLOW),
    3: ('Float RTK',    YELLOW),
    4: ('RTK Fixed',    GREEN),
    5: ('Float RTK',    YELLOW),
}


def parse_pdiag(sentence: str) -> dict | None:
    """Parse a $PDIAG sentence. Returns dict or None on error."""
    # Strip whitespace and CRLF
    sentence = sentence.strip()

    # Verify checksum
    m = re.match(r'^\$(.+)\*([0-9A-Fa-f]{2})$', sentence)
    if not m:
        return None
    body, cs_str = m.group(1), m.group(2)
    cs = 0
    for c in body:
        cs ^= ord(c)
    if cs != int(cs_str, 16):
        return None

    # Check sentence ID
    parts = body.split(',')
    if len(parts) < 7 or parts[0] != 'PDIAG':
        return None

    # v0.3.2: per-field resilient, not all-or-nothing. Previously this
    # whole dict was built inside one try/except — a single malformed
    # field anywhere among all 46 (a ValueError from int()/float() on
    # one bad value) discarded the ENTIRE sentence, silently, with no
    # indication of which field was actually the problem. Investigated
    # as a possible explanation for a real "nothing populates" report;
    # never conclusively confirmed as the actual cause (a rigorous
    # field-by-field type check against firmware's own format string
    # found zero mismatches), but the all-or-nothing structure was a
    # genuine, worth-fixing fragility regardless — one bad field
    # should degrade to that one field being unavailable, not silently
    # take out satellite counts, roll, and everything else with it.
    # _f() below mirrors the exact same per-field default values the
    # old inline "if len(parts) > N else default" expressions already
    # used — this changes error-handling behaviour only, not any
    # default value or field name.
    def _f(idx, conv, default=None):
        if len(parts) <= idx:
            return default
        try:
            return conv(parts[idx])
        except (ValueError, IndexError):
            return default

    return {
        'mode':           parts[1] if len(parts) > 1 else '',
        'sats_m':         _f(2, int, 0),
        'sats_s':         _f(3, int, 0),
        'hpr_sats':       _f(4, int, 0),
        'sol':            _f(5, int, 0),
        'hdg_offset':     _f(6, float, 0.0),
        'heading_alpha':  _f(7, float, 0.0),
        'roll_alpha':     _f(8, float, 0.0),
        'init_h':         _f(9, float, 0.0),
        'init_r':         _f(10, float, 0.0),
        'dual_pct':       _f(11, int, 100),
        'sats_full':      _f(12, int, 7),
        'roll_zero':      _f(13, float, 0.0),
        'dual_hold_s':    _f(14, float, 3.0),
        'dual_ramp_s':    _f(15, float, 5.0),
        'imu_axis':       _f(16, int, 1),
        'roll_invert':    _f(17, int, -1),
        'was_left':       _f(18, float, -45.0),
        'was_right':      _f(19, float, 45.0),
        'uturn_strength': _f(20, int, 1),
        'steer_actual':   _f(21, float, 0.0),
        # 22nd field, added in UM982 Fallback firmware v0.6 alongside CAN support.
        # Falls back to None against older (v0.5 and earlier,
        # CAN-less) firmware, so the UI simply leaves the radio
        # buttons untouched rather than erroring out.
        'brand':          _f(22, int, None),
        # Fields 23-26, added alongside TFF (multi-GNSS-source
        # support). Same None-fallback convention as 'brand' above,
        # against pre-TFF firmware that doesn't send them.
        'gnss_mode':        _f(23, int, None),
        'dual_roll_raw':    _f(24, float, None),
        'imu_roll_raw':     _f(25, float, None),
        'roll_valid':       (parts[26] == '1') if len(parts) > 26 else False,
        # Field 27, added alongside TFF's TM171 support. Same
        # None-fallback convention as the other TFF-era fields.
        'imu_type':         _f(27, int, None),
        'board_slot1':      _f(28, int, None),
        'board_slot2':      _f(29, int, None),
        'gnss_passthrough': _f(30, int, None),
        # Diagnostic health fields — see updateDiagnostics() and
        # the EthernetLinkUp/resetCause declaration comments in
        # the firmware's main .ino for what each one means.
        'can_k_timeout':    _f(31, int, None),
        'can_iso_timeout':  _f(32, int, None),
        'can_v_timeout':    _f(33, int, None),
        'can_implausible':  _f(34, int, None),
        'gnss_watchdog':    _f(35, int, None),
        'rtk_timeout':      _f(36, int, None),
        'was_implausible':  _f(37, int, None),
        'ethernet_link_up': _f(38, int, None),
            'reset_cause':      _f(39, int, None),
            # IMU watchdog status — missed in the first pass, added
            # after the fact. use_imu distinguishes "actively unhealthy,
            # still retrying" from "watchdog gave up permanently" —
            # see the imuHealthy/imuRetryCount/useIMU comment in
            # firmware's sendDiagnostics() for the full reasoning.
            'imu_healthy':      _f(40, int, None),
            'imu_retry_count':  _f(41, int, None),
            'use_imu':          _f(42, int, None),
            # Keya CAN steering motor status — only meaningful when
            # motor_drive_type == 2 (see zHandlers.ino's own comment on
            # this in sendDiagnostics()).
            'motor_drive_type': _f(43, int, None),
            'was_source':       _f(44, int, None),
            'keya_detected':    _f(45, int, None),
            'keya_fault_active':_f(46, int, None),
            # HPR-specific watchdog — deliberately distinct from
            # gnss_watchdog: that one only confirms some byte arrived
            # on the GNSS serial line, so a receiver outputting GGA/
            # VTG fine but never completing a valid HPR sentence
            # would show gnss_watchdog==0 (healthy) the whole time.
            # See firmware's own hprTimeout declaration comment
            # (020_TFF.ino) for the full story — found missing during
            # a real debugging session where exactly that happened.
            'hpr_timeout':      _f(47, int, None),
            # Raw age, separate from the hpr_timeout boolean above —
            # lets the UI show a live, continuously-updating "last
            # HPR: 0.3s ago" during active troubleshooting, without
            # waiting for the deliberately generous 8s field-tuned
            # threshold to actually trip. -1.0 from firmware means
            # "never received at all this boot".
            'hpr_age_seconds':  _f(48, float, None),
            # v0.3.10 — Keya-as-WAS auto-zero (zKeyaAutoZero.ino).
            # was_zero_done is the single most important one: whether
            # guidance is even possible right now (see the watchdog
            # gate in c00_Autosteer.ino). Same idx order firmware's
            # own snprintf() argument list uses, not re-ordered here.
            'az_zero_done':     _f(49, int, 0),
            'az_zero_deg':      _f(50, float, 0.0),
            'az_speed_min':     _f(51, float, 2.5),
            'az_yaw_rate_max':  _f(52, float, 0.3),
            'az_gps_hdg_max':   _f(53, float, 0.3),
            'az_time_slow_ms':  _f(54, int, 500),
            'az_time_fast_ms':  _f(55, int, 200),
            'az_speed_slow':    _f(56, float, 3.0),
            'az_speed_fast':    _f(57, float, 12.0),
            'az_use_bno':       _f(58, int, 1),
            'az_use_gps':       _f(59, int, 1),
            'az_beta':          _f(60, float, 0.3),
            # v0.3.17/v0.3.18 — Auto Roll Adjust. rollZeroOffset field
            # index 15/16 area (already parsed above, named
            # 'roll_zero_offset' or similar there) now reports the
            # EFFECTIVE, combined value (rollZeroOffset+
            # rollAutoCorrection) directly from firmware — no separate
            # "combined" field needed here, the existing Current
            # display already shows the right number automatically.
            'auto_roll_adjust': _f(61, int, 0),
            'roll_auto_deadband': _f(62, float, 0.2),
            'roll_auto_alpha':  _f(63, float, 0.0005),
        }


class Monitor:
    def _make_scrollable_tab(self, notebook, tab_title):
        """
        Build one Notebook tab whose content scrolls independently of the
        other tabs. Returns the inner 'content' frame — build widgets into
        that, exactly like the old single-panel layout did.

        Each tab gets its own Canvas+Scrollbar pair rather than sharing
        one globally, because tabs can have very different content
        heights (Receiver Configuration is short, Operation is long) and
        because mouse-wheel scrolling needs to affect only the tab that's
        actually visible — see the <Enter>/<Leave> binding below, which
        only activates wheel scrolling for the canvas currently under the
        mouse pointer, so scrolling one tab never affects another tab
        sitting behind it.
        """
        tab = tk.Frame(notebook, bg=BG)
        notebook.add(tab, text=tab_title)

        canvas = tk.Canvas(tab, bg=BG, highlightthickness=0)
        vscroll = tk.Scrollbar(tab, orient='vertical',
                               command=canvas.yview, width=22)
        canvas.configure(yscrollcommand=vscroll.set)
        vscroll.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)

        content = tk.Frame(canvas, bg=BG)
        canvas_window = canvas.create_window((0, 0), window=content, anchor='nw')

        def _on_content_configure(event, c=canvas):
            c.configure(scrollregion=c.bbox('all'))
        content.bind('<Configure>', _on_content_configure)

        def _on_canvas_configure(event, c=canvas, w=canvas_window):
            c.itemconfig(w, width=event.width)
        canvas.bind('<Configure>', _on_canvas_configure)

        # Mouse wheel only scrolls this tab's canvas while the pointer is
        # actually over it — bound/unbound on Enter/Leave rather than
        # bind_all(), so multiple tabs' canvases never fight over wheel
        # events (a bind_all() from one tab would otherwise still fire
        # even while looking at a different tab).
        def _on_mousewheel(event, c=canvas):
            c.yview_scroll(int(-1 * (event.delta / 120)), 'units')
        def _bind_wheel(event, c=canvas):
            c.bind_all('<MouseWheel>', _on_mousewheel)
            c.bind_all('<Button-4>', lambda e: c.yview_scroll(-1, 'units'))
            c.bind_all('<Button-5>', lambda e: c.yview_scroll(1, 'units'))
        def _unbind_wheel(event, c=canvas):
            c.unbind_all('<MouseWheel>')
            c.unbind_all('<Button-4>')
            c.unbind_all('<Button-5>')
        canvas.bind('<Enter>', _bind_wheel)
        canvas.bind('<Leave>', _unbind_wheel)

        return content

    def __init__(self, root: tk.Tk):
        """
        Builds the entire UI (all five tabs) and starts the background
        receive thread (_rx_thread) — everything the tool does lives
        either directly in this constructor's own widget-building code,
        or in one of the send_xxx()/_apply() methods it wires up here.
        No separate "start" method; the tool is fully live the moment
        this returns.
        """
        self.root = root
        self.root.title("Teensy Tool  v0.3.18")

        # Logfile state — see _log_line()/send_logfile_toggle() near the
        # Operation tab section for the full feature. self.log_file is
        # None when logging is off; the actual file object otherwise.
        self.log_file = None
        self.log_file_path = None
        self._signal_was_lost = False   # avoids re-logging "signal lost"
                                          # every 500ms poll tick while
                                          # already in that state
        # Generic transition tracking for every diagnostic health check
        # (CAN timeouts, GNSS/RTK watchdogs, WAS plausibility, Ethernet
        # link) — one shared dict instead of one near-identical bool
        # per check, see _log_health_transition(). Also drives the
        # periodic "STATUS OK" heartbeat: empty dict values == no
        # active problems.
        self._health_state = {}
        self._last_reset_cause = None   # logged once per new value seen,
                                          # not every poll — see _apply()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.configure(bg=BG)
        self.root.resizable(True, True)
        # Default size tuned for small touch displays (e.g. Panasonic
        # Toughpad). Increased from the original 480x640 to give the
        # larger fonts (see below) room without feeling cramped; content
        # taller than this still scrolls via the scrollbar added below.
        # Ökad från tidigare 560 till 660 — ett verkligt, fältrapporterat
        # problem (fönstret krävde manuell breddning för hand vid
        # varje uppstart), inte en teoretisk beräkning. Windows egna
        # fönsterramar och särskilt DPI-skalning (mycket vanligt på
        # moderna skärmar, 125%/150%) äter en påtaglig del av det
        # angivna bredd-värdet innan innehållet faktiskt får plats.
        self.root.geometry("660x760")
        self.root.minsize(480, 320)

        # Fonts
        # Font sizes increased over the original UM982 Monitor — legibility
        # in bright/moving tractor cab conditions matters more than screen
        # space efficiency, and the scrollable layout already absorbs the
        # extra vertical space this costs.
        fTitle  = tkfont.Font(family='Consolas', size=17, weight='bold')
        fBig    = tkfont.Font(family='Consolas', size=26, weight='bold')
        fMed    = tkfont.Font(family='Consolas', size=14, weight='bold')
        fNorm   = tkfont.Font(family='Consolas', size=12)
        fSmall  = tkfont.Font(family='Consolas', size=12, weight='bold')
        fNote   = tkfont.Font(family='Consolas', size=12, slant='italic')
        # Also stored on self — needed by methods outside __init__ (e.g.
        # _open_numeric_keypad(), built after all the panels that use
        # these fonts as plain local names within __init__ itself).
        self.fTitle, self.fBig, self.fMed = fTitle, fBig, fMed
        self.fNorm, self.fSmall, self.fNote = fNorm, fSmall, fNote

        # ---------------------------------------------------------------
        # Title bar
        # ---------------------------------------------------------------
        bar = tk.Frame(root, bg='#1f6feb', padx=12, pady=6)
        bar.pack(fill='x')
        tk.Label(bar, text="TEENSY  TOOL  v0.3.18", bg='#1f6feb',
                 fg=WHITE, font=fTitle).pack(side='left')

        self.lbl_time = tk.Label(bar, text="--:--:--", bg='#1f6feb',
                                 fg=WHITE, font=fNorm)
        self.lbl_time.pack(side='right')

        self.lbl_age = tk.Label(bar, text="Waiting...", bg='#1f6feb',
                                fg=YELLOW, font=fSmall)
        self.lbl_age.pack(side='right', padx=12)

        # ---------------------------------------------------------------
        # Status bar (packed first, at the bottom, so it reserves its
        # space before the scrollable area below claims the rest)
        # ---------------------------------------------------------------
        status = tk.Frame(root, bg='#010409', padx=10, pady=4)
        status.pack(fill='x', side='bottom')
        self.lbl_status = tk.Label(status,
                                   text=f"Listening on UDP port {LISTEN_PORT}",
                                   bg='#010409', fg=DIM, font=fSmall)
        self.lbl_status.pack(side='left')

        # ---------------------------------------------------------------
        # Tabbed layout — Receiver Configuration / IMU / Vehicle
        # Configuration / Operation. Each tab gets its own scrollable
        # Canvas+Scrollbar (same pattern as the old single-panel layout,
        # just duplicated per tab) via _make_scrollable_tab() below.
        # ---------------------------------------------------------------
        style = ttk.Style()
        style.theme_use('default')
        style.configure('TNotebook', background=BG, borderwidth=0)
        style.configure('TNotebook.Tab', background=PANEL, foreground=FG,
                         padding=[14, 8], font=fSmall, borderwidth=0)
        style.map('TNotebook.Tab',
                  background=[('selected', '#1f6feb')],
                  foreground=[('selected', WHITE)])

        notebook = ttk.Notebook(root)
        notebook.pack(fill='both', expand=True)

        tab_receiver  = self._make_scrollable_tab(notebook, "Receiver Configuration")
        tab_imu       = self._make_scrollable_tab(notebook, "IMU")
        tab_vehicle   = self._make_scrollable_tab(notebook, "Vehicle Configuration")
        tab_board     = self._make_scrollable_tab(notebook, "Board Configuration")
        content       = self._make_scrollable_tab(notebook, "Operation")
        # 'content' keeps its old name deliberately — it's still the
        # parent for every panel that was already in Operation before
        # the tab split, so those blocks below needed no changes beyond
        # this point.

        # ---------------------------------------------------------------
        # Mode panel (large, top)
        # ---------------------------------------------------------------
        mode_frame = tk.Frame(content, bg=PANEL, bd=1, relief='flat', pady=4)
        mode_frame.pack(fill='x', padx=10, pady=(4, 2))

        pct_row = tk.Frame(mode_frame, bg=PANEL)
        pct_row.pack(fill='x', padx=6, pady=2)
        self.lbl_imu_pct = tk.Label(pct_row, text="IMU --%", bg=PANEL,
                                    fg=DIM, font=fSmall)
        self.lbl_imu_pct.pack(side='left', padx=4)
        self.canvas_bar = tk.Canvas(pct_row, width=160, height=12,
                                    bg='#21262d', highlightthickness=0)
        self.canvas_bar.pack(side='left')
        self.lbl_dual_pct2 = tk.Label(pct_row, text="--%  DUAL", bg=PANEL,
                                      fg=DIM, font=fSmall)
        self.lbl_dual_pct2.pack(side='left', padx=4)
        self.lbl_sol = tk.Label(pct_row, text="HPR solution: ---",
                                bg=PANEL, fg=DIM, font=fSmall)
        self.lbl_sol.pack(side='left', padx=(12, 0))

        # ---------------------------------------------------------------
        # Satellite counts
        # ---------------------------------------------------------------
        sat_outer = tk.Frame(content, bg=BG)
        sat_outer.pack(fill='x', padx=10, pady=2)

        for col, label, attr in [
            (0, "MASTER SATS (from GGA)", 'lbl_sats_m'),
            (1, "SLAVE SATS (from GPGGAH)", 'lbl_sats_s'),
        ]:
            f = tk.Frame(sat_outer, bg=PANEL, bd=1, relief='flat',
                         padx=10, pady=3)
            f.grid(row=0, column=col, sticky='nsew',
                   padx=(0, 4) if col == 0 else (0, 0))
            row = tk.Frame(f, bg=PANEL)
            row.pack(anchor='w')
            tk.Label(row, text=label, bg=PANEL, fg=DIM,
                     font=fSmall).pack(side='left', padx=(0, 8))
            lbl = tk.Label(row, text="--", bg=PANEL, fg=DIM, font=fMed)
            lbl.pack(side='left')
            setattr(self, attr, lbl)

        sat_outer.columnconfigure(0, weight=1)
        sat_outer.columnconfigure(1, weight=1)

        # HPR heading solution satellite count
        hpr_row = tk.Frame(content, bg=PANEL, padx=8, pady=2)
        hpr_row.pack(fill='x', padx=10, pady=(0, 2))
        tk.Label(hpr_row, text="HPR heading sats (from HPR):", bg=PANEL,
                 fg=DIM, font=fSmall).pack(side='left')
        self.lbl_hpr_sats = tk.Label(hpr_row, text="--", bg=PANEL,
                                     fg=DIM, font=fSmall)
        self.lbl_hpr_sats.pack(side='left', padx=6)

        # Live "last successfully-parsed HPR" age — deliberately
        # separate from the (generous, field-tuned) hpr_timeout
        # boolean: this updates every PDIAG cycle (~2s), so during
        # active troubleshooting you see the number ticking in real
        # time rather than waiting for an 8s threshold to trip before
        # anything visibly changes.
        tk.Label(hpr_row, text="  |  Last HPR:", bg=PANEL,
                 fg=DIM, font=fSmall).pack(side='left', padx=(12, 0))
        self.lbl_hpr_age = tk.Label(hpr_row, text="--", bg=PANEL,
                                    fg=DIM, font=fSmall)
        self.lbl_hpr_age.pack(side='left', padx=6)

        # ---------------------------------------------------------------
        # Heading offset
        # ---------------------------------------------------------------
        off_frame = tk.Frame(content, bg=PANEL, bd=1, relief='flat',
                             padx=8, pady=3)
        off_frame.pack(fill='x', padx=10, pady=2)

        off_row = tk.Frame(off_frame, bg=PANEL)
        off_row.pack(anchor='w')
        tk.Label(off_row, text="IMU → HPR HEADING OFFSET", bg=PANEL,
                 fg=DIM, font=fSmall).pack(side='left', padx=(0, 10))
        self.lbl_offset = tk.Label(off_row, text="--- °", bg=PANEL,
                                   fg=BLUE, font=fMed)
        self.lbl_offset.pack(side='left')
        tk.Label(off_frame,
                 text="Applied to IMU yaw in fallback. Converges within ~5s of dual activity.",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w', pady=(1, 0))

        # ---------------------------------------------------------------
        # Fusion alpha panels (heading + roll)
        # ---------------------------------------------------------------
        def make_alpha_panel(parent, title, attr_lbl, attr_entry,
                             attr_status, send_fn, entry_default="0.0000",
                             hint=""):
            frame = tk.Frame(parent, bg=PANEL, bd=1, relief='flat',
                             padx=8, pady=4)
            frame.pack(fill='x', padx=10, pady=3)

            tk.Label(frame, text=title, bg=PANEL,
                     fg=DIM, font=fSmall).pack(anchor='w')

            row = tk.Frame(frame, bg=PANEL)
            row.pack(fill='x', pady=(2, 0))

            # Current value
            cf = tk.Frame(row, bg=PANEL)
            cf.pack(side='left')
            tk.Label(cf, text="Current:", bg=PANEL,
                     fg=DIM, font=fSmall).pack(anchor='w')
            lbl = tk.Label(cf, text="0.0000", bg=PANEL, fg=GREEN, font=fMed)
            lbl.pack(anchor='w')
            setattr(self, attr_lbl, lbl)

            tk.Label(row, text="  →  ", bg=PANEL, fg=DIM, font=fMed).pack(side='left')

            # Entry + button
            sf = tk.Frame(row, bg=PANEL)
            sf.pack(side='left')
            tk.Label(sf, text="New value:", bg=PANEL,
                     fg=DIM, font=fSmall).pack(anchor='w')
            er = tk.Frame(sf, bg=PANEL)
            er.pack(anchor='w')
            entry = tk.Entry(er, width=8, bg='#21262d', fg=WHITE,
                             insertbackground=WHITE, font=fNorm,
                             bd=1, relief='solid')
            entry.insert(0, entry_default)
            entry.pack(side='left', padx=(0, 6))
            setattr(self, attr_entry, entry)

            btn = tk.Button(er, text="Set",
                            bg='#1f6feb', fg=WHITE,
                            activebackground='#388bfd',
                            font=fSmall, bd=0, padx=10, pady=4,
                            cursor='hand2', command=send_fn)
            btn.pack(side='left')

            # Hint text to the right of the button
            if hint:
                tk.Label(er, text=hint, bg=PANEL, fg=DIM,
                         font=fSmall, justify='left').pack(
                         side='left', padx=(12, 0))

            slbl = tk.Label(sf, text="", bg=PANEL, fg=GREEN, font=fSmall)
            slbl.pack(anchor='w', pady=(2, 0))
            setattr(self, attr_status, slbl)

            entry.bind('<Return>', lambda e: send_fn())
            return frame

        alpha_row = tk.Frame(content, bg=BG)
        alpha_row.pack(fill='x', padx=10, pady=3)
        alpha_row.columnconfigure(0, weight=1)
        alpha_row.columnconfigure(1, weight=1)

        def make_alpha_inline(parent, col, title, attr_lbl, attr_entry, attr_status, send_fn, hint=""):
            # Builds one Heading/Roll Alpha side-by-side panel (col=0
            # or col=1) — a compact Current/Entry/Set row plus a
            # hint label. attr_lbl/attr_entry/attr_status are the
            # attribute names (strings) the built widgets get stored
            # under on self, so _apply() can update them later by
            # name via getattr().
            frame = tk.Frame(parent, bg=PANEL, bd=1, relief='flat', padx=8, pady=3)
            frame.grid(row=0, column=col, sticky='nsew',
                       padx=(0,3) if col==0 else (3,0))
            tk.Label(frame, text=title, bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')
            row = tk.Frame(frame, bg=PANEL)
            row.pack(anchor='w', pady=(2,0))
            tk.Label(row, text="Current:", bg=PANEL, fg=DIM, font=fSmall).pack(side='left')
            lbl = tk.Label(row, text="0.0000", bg=PANEL, fg=GREEN, font=fMed)
            lbl.pack(side='left', padx=(3,6))
            entry = tk.Entry(row, width=6, bg='#21262d', fg=WHITE,
                             insertbackground=WHITE, font=fNorm, bd=1, relief='solid')
            entry.insert(0, "0.0000")
            entry.pack(side='left', padx=(0,3))
            tk.Button(row, text="Set", bg='#1f6feb', fg=WHITE,
                      activebackground='#388bfd', font=fSmall, bd=0,
                      padx=6, pady=3, cursor='hand2',
                      command=send_fn).pack(side='left')
            if hint:
                tk.Label(row, text=hint, bg=PANEL, fg=DIM,
                         font=fSmall).pack(side='left', padx=(8,0))
            setattr(self, attr_lbl, lbl)
            setattr(self, attr_entry, entry)
            slbl = tk.Label(frame, text="", bg=PANEL, fg=GREEN, font=fSmall)
            slbl.pack(anchor='w')
            setattr(self, attr_status, slbl)
            entry.bind('<Return>', lambda e: send_fn())

        make_alpha_inline(alpha_row, 0,
            "HEADING ALPHA  —  initial value",
            'lbl_heading_alpha', 'entry_heading_alpha', 'lbl_heading_status',
            self.send_heading_alpha, hint="Initial base line: 0.0 full DUAL, 1.0 full Single+IMU")
        make_alpha_inline(alpha_row, 1,
            "ROLL ALPHA  —  initial value",
            'lbl_roll_alpha', 'entry_roll_alpha', 'lbl_roll_status',
            self.send_roll_alpha, hint="Initial base line: 0.0 full DUAL, 1.0 full Single+IMU")

        # satsSlaveFull panel
        make_alpha_panel(content,
            "SATS FULL  —  HPR heading sats threshold for full dual quality",
            'lbl_sats_full', 'entry_sats_full', 'lbl_sats_status',
            self.send_sats_full,
            entry_default="7",
            hint="4-5=tolerant  7=default  10-12=conservative (alpha rises earlier)")

        # =================================================================
        # IMU tab
        # =================================================================
        # Which controls are enabled here depends on the GNSS category
        # chosen in the Receiver Configuration tab (see far below):
        #   - Dual antenna receivers / Dual single receivers (PAOGI):
        #     axis, roll invert AND roll zero offset are all relevant —
        #     roll zero offset calibrates IMU roll to match the dual
        #     (HPR/RELPOSNED/UNIHEADING2-derived) roll.
        #   - Single antenna system, single+IMU (PANDA): only roll AXIS
        #     is relevant (a physical mounting fact that never changes).
        #     Roll invert and roll zero offset are grayed out here,
        #     because AgOpenGPS does its own IMU zeroing for PANDA (its
        #     own "Zero IMU" workflow) — firmware-side roll matching
        #     against a dual signal that doesn't exist in this mode would
        #     be meaningless, and doubling up on AGO's own zeroing would
        #     just cause confusion about which one is "correct".
        # self.gnss_category_var (set in Receiver Configuration, below)
        # drives _update_imu_tab_state(), which does the graying.
        #
        # LIVE as of TFF v0.1: the graying itself is local UI logic (no
        # firmware round-trip needed to decide what to gray out), but
        # self.gnss_category_var is also kept in sync with the real
        # GnssMode reported back in $PDIAG (see _apply() below) once
        # Teensy confirms it after a SETGNSSMODE command + reboot — so
        # this reflects actual firmware state, not just a local guess.
        # -----------------------------------------------------------------
        # IMU type selector — first thing in this tab, per explicit
        # decision: which physical sensor is active governs everything
        # else below it (axis selection is meaningless for TM171, which
        # reports ready-made Euler angles and needs no axis
        # reinterpretation the way BNO08x's quaternion does).
        # -----------------------------------------------------------------
        imu_type_frame = tk.Frame(tab_imu, bg=PANEL, bd=1, relief='flat',
                                  padx=8, pady=4)
        imu_type_frame.pack(fill='x', padx=10, pady=(4, 3))
        tk.Label(imu_type_frame,
                 text="IMU TYPE  —  which physical sensor is fitted",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')

        self.imu_type_var = tk.IntVar(value=0)  # 0=BNO08x, 1=TM171, 2=No fallback (dual-only)
        IMU_TYPE_OPTIONS = [
            (0, "BNO08x (I2C)"),
            (1, "TM171 (UART, SYD Dynamics TransducerM)"),
            (2, "No fallback function (dual-only, no IMU)"),
        ]
        for val, label in IMU_TYPE_OPTIONS:
            rb = tk.Radiobutton(imu_type_frame, text=label,
                           variable=self.imu_type_var, value=val,
                           bg=PANEL, fg=FG, selectcolor='#21262d',
                           activebackground=PANEL, activeforeground=WHITE,
                           font=fSmall, cursor='hand2',
                           command=self.send_imu_type)
            rb.pack(anchor='w', padx=(4, 0))
            if val == 2:
                self.rb_imu_none = rb   # kept for the two-way GNSS-mode/IMU-type lock

        tk.Label(imu_type_frame,
                 text="\"No fallback function\": PAOGI is always sent straight from the dual "
                      "receiver, good quality or not — same as a traditional dual-only system "
                      "with no IMU at all. Not available with \"Single antenna system, "
                      "single+IMU\" in Receiver Configuration, since that mode has no dual "
                      "signal to fall back ON in the first place — the IMU IS the only heading "
                      "source there.",
                 bg=PANEL, fg=DIM, font=fNote, justify='left',
                 wraplength=440).pack(anchor='w', padx=(4, 0), pady=(2, 0))

        tk.Label(imu_type_frame,
                 text="⚠ Requires Teensy restart to take effect (I2C/UART only init at boot)",
                 bg=PANEL, fg=YELLOW, font=fSmall).pack(anchor='w', pady=(2, 0))
        self.lbl_imu_type_status = tk.Label(imu_type_frame, text="", bg=PANEL,
                                            fg=GREEN, font=fSmall)
        self.lbl_imu_type_status.pack(anchor='w', pady=(2, 0))

        # -----------------------------------------------------------------
        # Dual Roll vs IMU Roll comparison panel
        # -----------------------------------------------------------------
        # This is the panel that removes the old back-and-forth workflow
        # (open AGO, note dual roll, flip alpha to 1.0 in this tool, open
        # AGO again, note IMU roll, flip alpha back). Showing both values
        # side by side, always, means rollZeroOffset can be tuned by eye
        # until they match — no alpha changes, no tabbing out to AGO.
        #
        # LIVE as of TFF: firmware's imuHandler() computes both
        # rollDualCorrected (rawDualRoll) and rollImu (rawImuRoll)
        # internally, and now exposes them directly as PDIAG fields
        # 24-26 (DUAL_ROLL_RAW, IMU_ROLL_RAW, ROLL_VALID) — no new
        # firmware logic was needed, just exposing what was already
        # calculated. Against older (pre-TFF) firmware that doesn't
        # send these fields, parse_pdiag() falls back to None and the
        # labels below simply stay at their placeholder dashes.
        roll_compare_frame = tk.Frame(tab_imu, bg=PANEL, bd=1, relief='flat',
                                      padx=8, pady=4)
        roll_compare_frame.pack(fill='x', padx=10, pady=(4, 3))
        tk.Label(roll_compare_frame,
                 text="DUAL ROLL vs IMU ROLL  —  compare live, adjust IMU ROLL OFFSET below until they match",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')

        roll_compare_row = tk.Frame(roll_compare_frame, bg=PANEL)
        roll_compare_row.pack(fill='x', pady=(4, 0))

        dc = tk.Frame(roll_compare_row, bg=PANEL)
        dc.pack(side='left', padx=(0, 30))
        tk.Label(dc, text="Dual Roll (HPR):", bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')
        self.lbl_dual_roll_live = tk.Label(dc, text="--.-°", bg=PANEL, fg=BLUE, font=fBig)
        self.lbl_dual_roll_live.pack(anchor='w')

        ic = tk.Frame(roll_compare_row, bg=PANEL)
        ic.pack(side='left')
        tk.Label(ic, text="IMU Roll:", bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')
        self.lbl_imu_roll_live = tk.Label(ic, text="--.-°", bg=PANEL, fg=GREEN, font=fBig)
        self.lbl_imu_roll_live.pack(anchor='w')

        tk.Label(roll_compare_frame,
                 text="Lean vehicle to ensure correct roll direction "
                      "(at exactly 0° you can't tell if the sign is right).",
                 bg=PANEL, fg=DIM, font=fNote).pack(anchor='w', pady=(4, 0))

        # -----------------------------------------------------------------
        # IMU Roll Zero panel — manual entry. Grayed out in Single+IMU
        # mode (see note above) — widgets stay in the layout so the tab
        # doesn't jump around when switching GNSS category, they just
        # become non-interactive and visually dimmed.
        # -----------------------------------------------------------------
        self.roll_zero_frame = make_alpha_panel(tab_imu,
            "IMU ROLL OFFSET  —  adjusts IMU roll to match DUAL roll",
            'lbl_roll_zero', 'entry_roll_zero', 'lbl_zero_status',
            self.send_roll_zero,
            entry_default="0.00",
            hint="Watch Dual Roll vs IMU Roll above (live, both values at once) and adjust offset until they match.")

        # ---------------------------------------------------------------
        # Auto Roll Adjust (v0.3.17/v0.3.18) — continuously nudges the
        # roll offset above for slow sensor-mounting drift, instead of
        # needing to re-tune it by eye periodically. Only meaningful
        # with a dual signal — grayed out alongside roll_zero_frame in
        # Single+IMU mode (see _update_imu_tab_state()).
        self.auto_roll_frame = tk.Frame(tab_imu, bg=PANEL, bd=1, relief='flat', padx=8, pady=4)
        self.auto_roll_frame.pack(fill='x', padx=10, pady=3)
        tk.Label(self.auto_roll_frame, text="AUTO ROLL ADJUST", bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')
        tk.Label(self.auto_roll_frame,
                 text="Continuously nudges the roll offset above to correct for slow sensor drift "
                      "over time — not a one-time calibration. Only active with a dual RTK-fixed signal.",
                 bg=PANEL, fg=DIM, font=fNote, justify='left', wraplength=440).pack(anchor='w', pady=(2, 6))

        self.auto_roll_var = tk.BooleanVar(value=False)
        auto_roll_cb = tk.Checkbutton(self.auto_roll_frame, text="Auto Roll Adjust", variable=self.auto_roll_var,
                                       bg=PANEL, fg=FG, selectcolor='#21262d', font=fSmall,
                                       command=self.send_auto_roll_adjust)
        auto_roll_cb.pack(anchor='w')
        self.lbl_auto_roll_status = tk.Label(self.auto_roll_frame, text="", bg=PANEL, fg=GREEN, font=fSmall)
        self.lbl_auto_roll_status.pack(anchor='w', pady=(0, 6))

        def make_roll_auto_row(parent, label_text, cmd_prefix, default_val, lo, hi):
            row = tk.Frame(parent, bg=PANEL)
            row.pack(fill='x', pady=1)
            tk.Label(row, text=label_text, bg=PANEL, fg=FG, font=fSmall,
                    width=28, anchor='w').pack(side='left')
            current = tk.Label(row, text="Current: --", bg=PANEL, fg=DIM,
                               font=fSmall, width=14, anchor='w')
            current.pack(side='left')
            entry = tk.Entry(row, width=8, bg='#21262d', fg=WHITE,
                             insertbackground=WHITE, font=fNorm, bd=1, relief='solid')
            entry.insert(0, str(default_val))
            entry.pack(side='left', padx=(6, 0))
            status = tk.Label(row, text="", bg=PANEL, fg=GREEN, font=fSmall)
            tk.Button(row, text="Set", bg='#1f6feb', fg=WHITE,
                     activebackground='#388bfd', font=fSmall, bd=0, padx=6, pady=2,
                     cursor='hand2',
                     command=lambda: self.send_roll_auto_param(entry, cmd_prefix, status, lo, hi)
                     ).pack(side='left', padx=(6, 0))
            status.pack(side='left', padx=(6, 0))
            return (current, entry, default_val)

        # (lo, hi) match firmware's own constrain() ranges exactly
        # (SETROLLAUTODEADBAND/SETROLLAUTOALPHA handlers, zHandlers.ino).
        self._roll_auto_rows = {
            'deadband': make_roll_auto_row(self.auto_roll_frame, "Min angle diff to adjust [deg]",
                                            "SETROLLAUTODEADBAND:", 0.2, 0.02, 2.0),
            'alpha':    make_roll_auto_row(self.auto_roll_frame, "Adjustment speed",
                                            "SETROLLAUTOALPHA:", 0.0005, 0.0001, 0.1),
        }

        def load_roll_auto_defaults():
            # Same "fills fields only, still requires Set per row"
            # philosophy as Keya Auto-Zero's own Load Defaults button
            # — no single "reset everything to firmware right now"
            # action without a review step.
            for _, entry, default in self._roll_auto_rows.values():
                entry.delete(0, 'end')
                entry.insert(0, str(default))

        tk.Button(self.auto_roll_frame, text="Load Defaults (fills fields only, still requires Set per row)",
                 bg='#30363d', fg=FG, activebackground='#484f58', font=fSmall,
                 bd=0, padx=8, pady=3, cursor='hand2',
                 command=load_roll_auto_defaults).pack(anchor='w', pady=(6, 0))

        # ---------------------------------------------------------------
        # IMU Settings panel
        imu_frame = tk.Frame(tab_imu, bg=PANEL, bd=1, relief='flat', padx=8, pady=4)
        imu_frame.pack(fill='x', padx=10, pady=3)
        tk.Label(imu_frame, text="IMU SETTINGS  —  set to match physical mounting (Standard = X axis pointing forward)",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')

        imu_row = tk.Frame(imu_frame, bg=PANEL)
        imu_row.pack(fill='x', pady=(3,0))

        # Axis — always enabled regardless of GNSS mode: this is a
        # physical mounting fact (which BNO axis reads as roll), not
        # something that depends on whether a dual signal exists.
        tk.Label(imu_row, text="Roll axis:", bg=PANEL, fg=DIM, font=fSmall).pack(side='left')
        self.lbl_imu_axis = tk.Label(imu_row, text="X", bg=PANEL, fg=GREEN, font=fMed)
        self.lbl_imu_axis.pack(side='left', padx=(3,4))
        self.entry_imu_axis = tk.Entry(imu_row, width=3, bg='#21262d', fg=WHITE,
                                       insertbackground=WHITE, font=fNorm, bd=1, relief='solid')
        self.entry_imu_axis.insert(0, "0")
        self.entry_imu_axis.pack(side='left', padx=(0,2))
        tk.Label(imu_row, text="0=X 1=Y 2=Z", bg=PANEL, fg=DIM, font=fSmall).pack(side='left', padx=(2,4))
        self.lbl_axis_status = tk.Label(imu_row, text="", bg=PANEL, fg=GREEN, font=fSmall)
        tk.Button(imu_row, text="Set", bg='#1f6feb', fg=WHITE,
                  activebackground='#388bfd', font=fSmall, bd=0, padx=6, pady=3,
                  cursor='hand2', command=self.send_imu_axis).pack(side='left')
        self.lbl_axis_status.pack(side='left', padx=(3,20))

        # Roll invert — checkbox. Grayed out in Single+IMU mode, same
        # reasoning as the roll-zero-offset panel above.
        self.roll_invert_var = tk.BooleanVar(value=False)  # unchecked = X forward (rollInvert=-1)
        self.chk_roll_invert = tk.Checkbutton(
            imu_row, text="Invert Roll", variable=self.roll_invert_var,
            bg=PANEL, fg=DIM, selectcolor='#21262d',
            activebackground=PANEL, activeforeground=WHITE,
            font=fSmall, cursor='hand2',
            command=self.send_roll_invert)
        self.chk_roll_invert.pack(side='left', padx=(0,8))
        self.lbl_invert_status = tk.Label(imu_row, text="", bg=PANEL,
                                          fg=GREEN, font=fSmall)
        self.lbl_invert_status.pack(side='left', padx=(0,0))

        # ---------------------------------------------------------------
        # U-turn dual boost panel
        uturn_frame = tk.Frame(content, bg=PANEL, bd=1, relief='flat', padx=8, pady=4)
        uturn_frame.pack(fill='x', padx=10, pady=3)
        tk.Label(uturn_frame, text="U-TURN DUAL BOOST", bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')

        uturn_row = tk.Frame(uturn_frame, bg=PANEL)
        uturn_row.pack(fill='x', pady=(3,0))

        # WAS Left
        tk.Label(uturn_row, text="WAS left:", bg=PANEL, fg=DIM, font=fSmall).pack(side='left')
        self.lbl_was_left = tk.Label(uturn_row, text="-45.0°", bg=PANEL, fg=GREEN, font=fMed)
        self.lbl_was_left.pack(side='left', padx=(3,4))
        tk.Button(uturn_row, text="Set current", bg='#2ea043', fg=WHITE,
                  activebackground='#3fb950', font=fSmall, bd=0, padx=6, pady=3,
                  cursor='hand2', command=self.send_was_left).pack(side='left', padx=(0,12))

        # WAS Right
        tk.Label(uturn_row, text="WAS right:", bg=PANEL, fg=DIM, font=fSmall).pack(side='left')
        self.lbl_was_right = tk.Label(uturn_row, text="45.0°", bg=PANEL, fg=GREEN, font=fMed)
        self.lbl_was_right.pack(side='left', padx=(3,4))
        tk.Button(uturn_row, text="Set current", bg='#2ea043', fg=WHITE,
                  activebackground='#3fb950', font=fSmall, bd=0, padx=6, pady=3,
                  cursor='hand2', command=self.send_was_right).pack(side='left', padx=(0,16))

        # Steer actual live
        tk.Label(uturn_row, text="Steer angle:", bg=PANEL, fg=DIM, font=fSmall).pack(side='left')
        self.lbl_steer_actual = tk.Label(uturn_row, text="0.0°", bg=PANEL, fg=YELLOW, font=fMed)
        self.lbl_steer_actual.pack(side='left', padx=(3,0))

        uturn_row2 = tk.Frame(uturn_frame, bg=PANEL)
        uturn_row2.pack(fill='x', pady=(4,0))
        tk.Label(uturn_row2, text="Strength (1-100):", bg=PANEL, fg=DIM, font=fSmall).pack(side='left')
        self.lbl_uturn_strength = tk.Label(uturn_row2, text="1", bg=PANEL, fg=GREEN, font=fMed)
        self.lbl_uturn_strength.pack(side='left', padx=(3,4))
        self.entry_uturn_strength = tk.Entry(uturn_row2, width=3, bg='#21262d', fg=WHITE,
                                              insertbackground=WHITE, font=fNorm, bd=1, relief='solid')
        self.entry_uturn_strength.insert(0, "1")
        self.entry_uturn_strength.pack(side='left', padx=(0,3))
        tk.Label(uturn_row2, text="1=off  100=maximum dual at full lock",
                 bg=PANEL, fg=DIM, font=fSmall).pack(side='left', padx=(4,12))
        self.lbl_uturn_status = tk.Label(uturn_row2, text="", bg=PANEL, fg=GREEN, font=fSmall)
        tk.Button(uturn_row2, text="Set", bg='#1f6feb', fg=WHITE,
                  activebackground='#388bfd', font=fSmall, bd=0, padx=6, pady=3,
                  cursor='hand2', command=self.send_uturn_strength).pack(side='left')
        self.lbl_uturn_status.pack(side='left', padx=(3,0))

        # Explanatory diagram — see UTURN_BOOST_DIAGRAM_B64's own
        # comment near the top of this file for why it's embedded as
        # base64 rather than a separate file. "self." reference on
        # the PhotoImage is REQUIRED, not optional — tkinter does not
        # keep its own strong reference, so without one Python's
        # garbage collector can (and eventually will) silently
        # collect the image, leaving a blank Label with no error at
        # all — a well-known tkinter pitfall, not a hypothetical one.
        self.uturn_diagram_photo = tk.PhotoImage(data=UTURN_BOOST_DIAGRAM_B64)
        tk.Label(uturn_frame, image=self.uturn_diagram_photo, bg=PANEL).pack(
            anchor='w', pady=(6, 0))

        # ---------------------------------------------------------------
        # Dual reconnect timing — both settings on one row
        dual_frame = tk.Frame(content, bg=PANEL, bd=1, relief='flat',
                              padx=8, pady=4)
        dual_frame.pack(fill='x', padx=10, pady=3)

        tk.Label(dual_frame, text="DUAL RECONNECT TIMING  (activates at full loss of DUAL)",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')

        drow = tk.Frame(dual_frame, bg=PANEL)
        drow.pack(fill='x', pady=(4, 0))

        # Hold time
        tk.Label(drow, text="Stabilization [s]:", bg=PANEL,
                 fg=DIM, font=fSmall).pack(side='left')
        self.lbl_dual_hold = tk.Label(drow, text="3.0", bg=PANEL,
                                      fg=GREEN, font=fMed)
        self.lbl_dual_hold.pack(side='left', padx=(3, 4))
        self.entry_dual_hold = tk.Entry(drow, width=4, bg='#21262d', fg=WHITE,
                                        insertbackground=WHITE, font=fNorm,
                                        bd=1, relief='solid')
        self.entry_dual_hold.insert(0, "3.0")
        self.entry_dual_hold.pack(side='left', padx=(0, 3))
        self.lbl_hold_status = tk.Label(drow, text="", bg=PANEL,
                                        fg=GREEN, font=fSmall)
        tk.Button(drow, text="Set", bg='#1f6feb', fg=WHITE,
                  activebackground='#388bfd', font=fSmall, bd=0,
                  padx=6, pady=3, cursor='hand2',
                  command=self.send_dual_hold).pack(side='left')
        self.lbl_hold_status.pack(side='left', padx=(3, 16))

        # Ramp time
        tk.Label(drow, text="Transition fusion [s]:", bg=PANEL,
                 fg=DIM, font=fSmall).pack(side='left')
        self.lbl_dual_ramp = tk.Label(drow, text="5.0", bg=PANEL,
                                      fg=GREEN, font=fMed)
        self.lbl_dual_ramp.pack(side='left', padx=(3, 4))
        self.entry_dual_ramp = tk.Entry(drow, width=4, bg='#21262d', fg=WHITE,
                                        insertbackground=WHITE, font=fNorm,
                                        bd=1, relief='solid')
        self.entry_dual_ramp.insert(0, "5.0")
        self.entry_dual_ramp.pack(side='left', padx=(0, 3))
        self.lbl_ramp_status = tk.Label(drow, text="", bg=PANEL,
                                        fg=GREEN, font=fSmall)
        tk.Button(drow, text="Set", bg='#1f6feb', fg=WHITE,
                  activebackground='#388bfd', font=fSmall, bd=0,
                  padx=6, pady=3, cursor='hand2',
                  command=self.send_dual_ramp).pack(side='left')
        self.lbl_ramp_status.pack(side='left', padx=(3, 0))

        # =================================================================
        # Logfile — writes everything this tool already receives and
        # parses to a local text file, timestamped per line, for later
        # troubleshooting. Nothing new is measured or requested from
        # firmware — this only records what already flows through
        # _rx_thread()/_apply() during normal operation.
        #
        # Logs three kinds of entries:
        #   - DATA: the full parsed $PDIAG dict, roughly every 500ms
        #     (capped by _poll()'s own interval, not separately
        #     throttled) — the "sensor data" half of the request.
        #   - COMMUNICATION ERROR: no $PDIAG for >10s (reuses the
        #     existing "NO SIGNAL" detection in _poll(), logged once
        #     per loss rather than every poll tick) — the "communication
        #     errors" half.
        #   - UNPARSEABLE PACKET: a UDP packet arrived but didn't parse
        #     as valid $PDIAG — previously silently dropped with no
        #     record at all; now logged too, since a real-world
        #     intermittent problem (e.g. the map-jump symptom discussed
        #     earlier) is exactly the kind of thing worth having a
        #     timestamped record of after the fact.
        #
        # Off by default. File is opened once per session when turned
        # on (timestamped filename, current working directory) and
        # closed cleanly either when turned back off or when the window
        # closes (_on_close()).
        # =================================================================
        log_frame = tk.Frame(content, bg=PANEL, bd=1, relief='flat',
                             padx=8, pady=4)
        log_frame.pack(fill='x', padx=10, pady=3)
        tk.Label(log_frame,
                 text="LOGFILE  —  record communication errors and sensor data for later troubleshooting",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')
        tk.Label(log_frame,
                 text="Writes a timestamped text file in this tool's working folder while enabled. "
                      "No effect on Teensy — purely local to this PC, takes effect immediately, no "
                      "restart needed.",
                 bg=PANEL, fg=DIM, font=fNote, justify='left',
                 wraplength=440).pack(anchor='w', pady=(3, 6))

        self.logfile_var = tk.BooleanVar(value=False)
        tk.Checkbutton(log_frame,
                       text="Enable logging",
                       variable=self.logfile_var,
                       bg=PANEL, fg=FG, selectcolor='#21262d',
                       activebackground=PANEL, activeforeground=WHITE,
                       font=fSmall, cursor='hand2',
                       command=self.send_logfile_toggle).pack(anchor='w')

        self.lbl_logfile_status = tk.Label(log_frame, text="Logging off", bg=PANEL,
                                           fg=DIM, font=fSmall)
        self.lbl_logfile_status.pack(anchor='w', pady=(3, 0))

        # ---------------------------------------------------------------
        # Steering brand panel — mutually exclusive tractor/brand select
        # ---------------------------------------------------------------
        # Note: these are radio buttons, not checkboxes — only one brand
        # can be active at a time, since it selects which CAN V-Bus
        # protocol (or plain PWM) drives the steering output.
        brand_frame = tk.Frame(tab_vehicle, bg=PANEL, bd=1, relief='flat',
                               padx=8, pady=4)
        brand_frame.pack(fill='x', padx=10, pady=3)
        tk.Label(brand_frame,
                 text="STEERING BRAND  —  select tractor / output mode (one at a time)",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')
        tk.Label(brand_frame,
                 text="⚠ Requires Teensy restart to take effect (CAN filters only init at boot)",
                 bg=PANEL, fg=YELLOW, font=fSmall).pack(anchor='w', pady=(1, 0))

        # (number, label) — must match Brand values in firmware.
        # BRAND_NONE (8) is our own addition on top of the original
        # CAN_All_Brands.ino brand list (0-7): it fully disables CAN bus
        # init/polling in firmware, giving byte-for-byte the same
        # behaviour as today's CAN-less 000_UM982_Fallback.ino. This is
        # deliberately distinct from Brand 7 ("AgOpenGPS — PWM/extern
        # ventil"), which still keeps the CAN buses live in the
        # background (e.g. to read the tractor's own engage button via
        # K_Bus) even though it doesn't drive a factory steering valve.
        BRAND_NONE = 8
        BRAND_OPTIONS = [
            (BRAND_NONE, "No CAN — classic PWM/relay"),
            (0, "Claas"),
            (1, "Valtra / Massey Ferguson"),
            (2, "Case IH / New Holland"),
            (3, "Fendt"),
            (4, "JCB"),
            (5, "FendtOne"),
            (6, "Lindner"),
            (7, "AgOpenGPS — PWM/external valve (CAN active in background)"),
        ]

        self.brand_var = tk.IntVar(value=BRAND_NONE)  # default: no CAN at all

        brand_grid = tk.Frame(brand_frame, bg=PANEL)
        brand_grid.pack(fill='x', pady=(4, 0))

        for i, (num, label) in enumerate(BRAND_OPTIONS):
            r, c = divmod(i, 2)  # two columns, four rows
            rb = tk.Radiobutton(
                brand_grid, text=f"{num} — {label}",
                variable=self.brand_var, value=num,
                bg=PANEL, fg=FG, selectcolor='#21262d',
                activebackground=PANEL, activeforeground=WHITE,
                font=fSmall, cursor='hand2',
                command=self.send_brand)
            rb.grid(row=r, column=c, sticky='w', padx=(0, 12), pady=1)

        self.lbl_brand_status = tk.Label(brand_frame, text="", bg=PANEL,
                                         fg=GREEN, font=fSmall)
        self.lbl_brand_status.pack(anchor='w', pady=(3, 0))

        # =================================================================
        # Motor Drive Type / WAS Source — Keya CAN steering motor support.
        # Placed here (Vehicle Configuration) rather than a dedicated tab
        # since both are vehicle-specific hardware properties, same
        # category as Brand right above — not algorithm/mode choices.
        #
        # WAS Source is a new, explicit setting rather than repurposing
        # the existing "Danfoss mode" checkbox, which is the convention a
        # real, active community implementation uses instead (see
        # firmware's own WAS_SOURCE_KEYA comment, 020_TFF.ino, for the
        # full source and reasoning) — clarity over convenience, same as
        # every other TFF-own setting.
        #
        # Protocol verified against Keya's own official manual, not just
        # community source (which turned out to disagree with itself
        # across repos) — see zKeya.ino's own header comment in firmware
        # for the full story, including a field report that the "5A
        # firmware" variant (silent automatic EEPROM current-limit write)
        # was unreliable in practice. That behaviour is deliberately NOT
        # replicated in TFF.
        # =================================================================
        keya_frame = tk.Frame(tab_vehicle, bg=PANEL, bd=1, relief='flat',
                              padx=8, pady=4)
        keya_frame.pack(fill='x', padx=10, pady=3)
        tk.Label(keya_frame,
                 text="MOTOR/VALVE DRIVE / WAS SOURCE  —  select one of each",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')
        tk.Label(keya_frame,
                 text="⚠ Requires Teensy restart to take effect",
                 bg=PANEL, fg=YELLOW, font=fSmall).pack(anchor='w', pady=(1, 6))

        MOTOR_DRIVE_PWM, MOTOR_DRIVE_KEYA = 0, 2   # deliberately no 1 —
                                                     # see firmware's own
                                                     # MOTOR_DRIVE_PWM
                                                     # comment, 020_TFF.ino
        WAS_SOURCE_NORMAL, WAS_SOURCE_KEYA = 0, 1

        keya_row = tk.Frame(keya_frame, bg=PANEL)
        keya_row.pack(fill='x')

        motor_col = tk.Frame(keya_row, bg=PANEL)
        motor_col.pack(side='left', anchor='n', padx=(0, 24))
        tk.Label(motor_col, text="MOTOR / VALVE DRIVE", bg=PANEL, fg=FG,
                 font=fSmall).pack(anchor='w')
        self.motor_drive_var = tk.IntVar(value=MOTOR_DRIVE_PWM)
        for val, label in [(MOTOR_DRIVE_PWM,  "PWM (default — Cytron/IBT2, see below)"),
                            (MOTOR_DRIVE_KEYA, "Keya (CAN)")]:
            tk.Radiobutton(motor_col, text=label,
                           variable=self.motor_drive_var, value=val,
                           bg=PANEL, fg=FG, selectcolor='#21262d',
                           activebackground=PANEL, activeforeground=WHITE,
                           font=fSmall, cursor='hand2',
                           command=self.send_motor_drive_type).pack(anchor='w', pady=1)

        was_col = tk.Frame(keya_row, bg=PANEL)
        was_col.pack(side='left', anchor='n')
        tk.Label(was_col, text="WAS SOURCE", bg=PANEL, fg=FG,
                 font=fSmall).pack(anchor='w')
        self.was_source_var = tk.IntVar(value=WAS_SOURCE_NORMAL)
        for val, label in [(WAS_SOURCE_NORMAL, "Normal (default)"),
                            (WAS_SOURCE_KEYA,   "Keya encoder")]:
            tk.Radiobutton(was_col, text=label,
                           variable=self.was_source_var, value=val,
                           bg=PANEL, fg=FG, selectcolor='#21262d',
                           activebackground=PANEL, activeforeground=WHITE,
                           font=fSmall, cursor='hand2',
                           command=self.send_was_source).pack(anchor='w', pady=1)

        tk.Label(keya_frame,
                 text="PWM: drives a motor or hydraulic valve directly with PWM+direction signals "
                      "from this board's own output pins — the traditional setup, and AIO's own "
                      "documented Cytron connector. (Whether the exact signal convention is Cytron- "
                      "or IBT2-style is decided separately, by AGO's own Vehicle Settings — this "
                      "toggle doesn't affect that.) "
                      "Keya: sends digital CAN commands to the Keya motor's own built-in controller "
                      "instead — no PWM pins used at all, and (see warning above/below) it needs the "
                      "board's one CAN channel to itself unless a CAN-ready tractor Brand is also set.",
                 bg=PANEL, fg=DIM, font=fNote, justify='left',
                 wraplength=440).pack(anchor='w', pady=(6, 0))

        tk.Label(keya_frame,
                 text="\"Normal\" WAS = physical potentiometer, or the CAN-brand tractor's own "
                      "valve position feedback if a Brand is set. \"Keya encoder\" reuses the same "
                      "Sensor Counts / Ackerman Fix calibration as physical WAS — no separate "
                      "calibration needed when switching.",
                 bg=PANEL, fg=DIM, font=fNote, justify='left',
                 wraplength=440).pack(anchor='w', pady=(6, 6))

        self.lbl_keya_status = tk.Label(keya_frame, text="", bg=PANEL,
                                        fg=GREEN, font=fSmall,
                                        justify='left', wraplength=440)
        self.lbl_keya_status.pack(anchor='w')

        # Reserved purely for the "Sent →..." confirmation from
        # send_motor_drive_type()/send_was_source() — never touched by
        # _check_motor_drive_conflict(), same reason the Board
        # Configuration tab splits these two roles across separate
        # labels (see lbl_board_slot1_status/lbl_board_slot2_status):
        # sharing one label would let the transient "Sent" text silently
        # overwrite the conflict warning.
        self.lbl_keya_sent_status = tk.Label(keya_frame, text="", bg=PANEL,
                                             fg=GREEN, font=fSmall)
        self.lbl_keya_sent_status.pack(anchor='w', pady=(2, 0))

        # =================================================================
        # Keya Auto-Zero tuning (v0.3.10) — only meaningful when WAS
        # SOURCE above is set to "Keya encoder"; shown here regardless
        # (not conditionally hidden) since these are firmware-side
        # EEPROM values that persist and remain worth seeing/adjusting
        # even while a different WAS source is momentarily selected.
        # =================================================================
        az_frame = tk.Frame(tab_vehicle, bg=PANEL, bd=1, relief='flat',
                            padx=8, pady=4)
        az_frame.pack(fill='x', padx=10, pady=3)
        tk.Label(az_frame,
                 text="KEYA AUTO-ZERO TUNING  —  only relevant when WAS Source = Keya encoder",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')
        tk.Label(az_frame,
                 text="Establishes and continuously corrects the Keya encoder's zero-point "
                      "reference using GPS/IMU heading while driving straight — the encoder "
                      "itself only measures RELATIVE rotation, with no built-in sense of "
                      "\"straight ahead\" at all. No guidance is possible until the first zero "
                      "is established (see Zero established below).",
                 bg=PANEL, fg=DIM, font=fNote, justify='left',
                 wraplength=440).pack(anchor='w', pady=(2, 6))

        self.lbl_az_zero_status = tk.Label(az_frame, text="Zero established: --",
                                           bg=PANEL, fg=DIM, font=fSmall)
        self.lbl_az_zero_status.pack(anchor='w', pady=(0, 6))

        def make_az_row(parent, label_text, cmd_prefix, default_val, is_int, lo, hi, width=6):
            # Builds one Keya Auto-Zero tuning row (Current label +
            # Entry + Set button) — the generic version of
            # make_alpha_inline() above, parameterised by validation
            # range (lo/hi) and type (is_int) so all ten azParams
            # fields share this one function rather than ten near-
            # identical copies. Returns (current_label, entry,
            # default_val) — the entry and default are needed by
            # _apply()'s one-time sync and the Load Defaults button
            # respectively, not just the label like most other
            # make_xxx helpers in this file return.
            row = tk.Frame(parent, bg=PANEL)
            row.pack(fill='x', pady=1)
            tk.Label(row, text=label_text, bg=PANEL, fg=FG, font=fSmall,
                    width=22, anchor='w').pack(side='left')
            current = tk.Label(row, text="Current: --", bg=PANEL, fg=DIM,
                               font=fSmall, width=14, anchor='w')
            current.pack(side='left')
            entry = tk.Entry(row, width=width, bg='#21262d', fg=WHITE,
                             insertbackground=WHITE, font=fNorm, bd=1, relief='solid')
            entry.insert(0, str(default_val))
            entry.pack(side='left', padx=(6, 0))
            status = tk.Label(row, text="", bg=PANEL, fg=GREEN, font=fSmall)
            tk.Button(row, text="Set", bg='#1f6feb', fg=WHITE,
                     activebackground='#388bfd', font=fSmall, bd=0, padx=6, pady=2,
                     cursor='hand2',
                     command=lambda: self.send_az_param(entry, cmd_prefix, status, is_int, lo, hi)
                     ).pack(side='left', padx=(6, 0))
            status.pack(side='left', padx=(6, 0))
            # Returns (label, entry, default) now, not just the label —
            # entry needed so _apply() can sync it to the REAL, saved
            # EEPROM value on first PDIAG receipt (see _az_synced below:
            # this pre-filled default_val is only ever a PLACEHOLDER
            # shown before that first real value is known — it does
            # NOT necessarily match what's actually saved, and could
            # silently overwrite it if Set were clicked before the
            # sync happens); default kept too, for the Load Defaults
            # button further down.
            return (current, entry, default_val)

        # (is_int, lo, hi) match firmware's own constrain() ranges
        # exactly (azSet*() functions, zKeyaAutoZero.ino) — not
        # independently chosen, so a value this UI accepts is
        # guaranteed to also be accepted, unclamped, by firmware.
        self._az_rows = {
            'speed_min':    make_az_row(az_frame, "Min speed [km/h]",        "SETAZSPEEDMIN:",    2.5,  False, 0.1,  20.0),
            'yaw_rate_max': make_az_row(az_frame, "Max yaw rate [deg/s]",     "SETAZYAWRATEMAX:",  0.3,  False, 0.01, 10.0),
            'gps_hdg_max':  make_az_row(az_frame, "Max GPS hdg change [deg]", "SETAZGPSHDGMAX:",   0.3,  False, 0.01, 10.0),
            'time_slow':    make_az_row(az_frame, "Stable time @ low speed [ms]",  "SETAZTIMESLOW:", 500, True,  100, 5000),
            'time_fast':    make_az_row(az_frame, "Stable time @ high speed [ms]", "SETAZTIMEFAST:", 200, True,  100, 5000),
            'speed_slow':   make_az_row(az_frame, "\"Low speed\" threshold [km/h]",  "SETAZSPEEDSLOW:", 3.0,  False, 0.1, 30.0),
            'speed_fast':   make_az_row(az_frame, "\"High speed\" threshold [km/h]", "SETAZSPEEDFAST:", 12.0, False, 0.1, 30.0),
            'use_bno':      make_az_row(az_frame, "Use BNO yaw rate (0/1)",   "SETAZUSEBNO:",      1,    True,  0, 1),
            'use_gps':      make_az_row(az_frame, "Use GPS heading (0/1)",    "SETAZUSEGPS:",      1,    True,  0, 1),
            'beta':         make_az_row(az_frame, "Correction gain (beta)",   "SETAZBETA:",        0.3,  False, 0.001, 1.0),
        }
        # Back-compat aliases — _apply() already references these
        # directly for the "Current:" label update; unchanged from
        # before, just now sourced from the dict above instead of
        # separate variables.
        self.lbl_az_speed_min    = self._az_rows['speed_min'][0]
        self.lbl_az_yaw_rate_max = self._az_rows['yaw_rate_max'][0]
        self.lbl_az_gps_hdg_max  = self._az_rows['gps_hdg_max'][0]
        self.lbl_az_time_slow    = self._az_rows['time_slow'][0]
        self.lbl_az_time_fast    = self._az_rows['time_fast'][0]
        self.lbl_az_speed_slow   = self._az_rows['speed_slow'][0]
        self.lbl_az_speed_fast   = self._az_rows['speed_fast'][0]
        self.lbl_az_use_bno      = self._az_rows['use_bno'][0]
        self.lbl_az_use_gps      = self._az_rows['use_gps'][0]
        self.lbl_az_beta         = self._az_rows['beta'][0]

        # Tracks whether the entry fields have already been synced to
        # a REAL received value this session — see _apply()'s own
        # comment on why this only happens ONCE, not every ~2s packet
        # (would otherwise silently overwrite whatever the user is
        # actively typing before they get to click Set).
        self._az_synced_once = False

        def load_az_defaults():
            # Deliberately does NOT send anything to firmware — only
            # fills the entry fields with the compile-time defaults
            # (azParams' own initial values, zKeyaAutoZero.ino) so the
            # user can review them, same as every other field in this
            # tool: still requires an explicit Set click per row to
            # actually save/send. A single "reset everything to
            # firmware right now" button was deliberately avoided —
            # too easy to trigger by accident with no review step.
            for _, entry, default in self._az_rows.values():
                entry.delete(0, 'end')
                entry.insert(0, str(default))

        tk.Button(az_frame, text="Load Defaults (fills fields only, still requires Set per row)",
                 bg='#30363d', fg=FG, activebackground='#484f58', font=fSmall,
                 bd=0, padx=8, pady=3, cursor='hand2',
                 command=load_az_defaults).pack(anchor='w', pady=(6, 0))

        # =================================================================
        # Board Configuration tab
        # =================================================================
        # Which physical connector each logical role occupies on THIS
        # installation — separate from every other tab, which all
        # answer "what should firmware do" (GNSS source, IMU algorithm,
        # CAN protocol). This tab answers "what is physically plugged
        # in where", a genuinely different kind of question tied to
        # wiring, not behaviour.
        #
        # Exists because TM171 can physically occupy either GNSS
        # connector position instead of a real receiver — confirmed
        # electrically identical (generic UART + power, no receiver-
        # specific wiring) — and different installations wire it
        # differently for convenience. Two physical slots (Master's
        # connector, Slave's connector), each independently assignable.
        #
        # AIO-default (Slot 1 = Master, Slot 2 = Slave) matches exactly
        # what firmware already does before this feature existed —
        # confirmed against the official AIO v4 Firmware's own source
        # (SerialGPS = &Serial7, SerialGPS2 = &Serial2), not just this
        # project's own prior assumption.
        #
        # Validation (two rules, both enforced firmware-side at boot —
        # see loadAlphasFromEEPROM() in zHandlers.ino): the two slots
        # must never be equal, and at least one must be Master or
        # Slave. Not enforced live here on every click, deliberately —
        # swapping Master and Slave between the two slots needs two
        # separate commands, and the state is briefly, legitimately
        # "invalid" between them. A warning label reflects the current
        # combination instead of blocking the in-progress swap.
        board_frame = tk.Frame(tab_board, bg=PANEL, bd=1, relief='flat',
                               padx=8, pady=4)
        board_frame.pack(fill='x', padx=10, pady=3)
        tk.Label(board_frame,
                 text="BOARD CONFIGURATION  —  what is physically plugged into each GNSS connector",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')
        tk.Label(board_frame,
                 text="⚠ Requires Teensy restart to take effect (Serial port init only runs once, at boot)",
                 bg=PANEL, fg=YELLOW, font=fSmall).pack(anchor='w', pady=(1, 0))
        tk.Label(board_frame,
                 text="AIO-default: Slot 1 = Master, Slot 2 = Slave — matches today's firmware exactly. "
                      "Only change this if TM171 is physically wired into one of the GNSS connector "
                      "positions instead of a real receiver, or if Master/Slave are physically swapped "
                      "on this particular installation.",
                 bg=PANEL, fg=DIM, font=fNote, justify='left',
                 wraplength=440).pack(anchor='w', pady=(4, 0))

        SLOT_MASTER, SLOT_SLAVE, SLOT_TM171, SLOT_EMPTY = 0, 1, 2, 3

        self.board_slot1_var = tk.IntVar(value=SLOT_MASTER)
        self.board_slot2_var = tk.IntVar(value=SLOT_SLAVE)

        slots_row = tk.Frame(board_frame, bg=PANEL)
        slots_row.pack(fill='x', pady=(8, 0))

        slot1_col = tk.Frame(slots_row, bg=PANEL)
        slot1_col.pack(side='left', anchor='n', padx=(0, 24))
        tk.Label(slot1_col, text="SLOT 1", bg=PANEL, fg=FG,
                 font=fSmall).pack(anchor='w')
        for val, label in [(SLOT_MASTER, "Master GNSS (default)"),
                            (SLOT_TM171,  "TM171"),
                            (SLOT_SLAVE,  "Slave GNSS"),
                            (SLOT_EMPTY,  "Empty")]:
            tk.Radiobutton(slot1_col, text=label,
                           variable=self.board_slot1_var, value=val,
                           bg=PANEL, fg=FG, selectcolor='#21262d',
                           activebackground=PANEL, activeforeground=WHITE,
                           font=fSmall, cursor='hand2',
                           command=self.send_board_slot1).pack(anchor='w', pady=1)
        self.lbl_board_slot1_status = tk.Label(slot1_col, text="", bg=PANEL,
                                               fg=GREEN, font=fSmall)
        self.lbl_board_slot1_status.pack(anchor='w', pady=(4, 0))

        slot2_col = tk.Frame(slots_row, bg=PANEL)
        slot2_col.pack(side='left', anchor='n')
        tk.Label(slot2_col, text="SLOT 2", bg=PANEL, fg=FG,
                 font=fSmall).pack(anchor='w')
        for val, label in [(SLOT_SLAVE,  "Slave GNSS (default)"),
                            (SLOT_TM171,  "TM171"),
                            (SLOT_MASTER, "Master GNSS"),
                            (SLOT_EMPTY,  "Empty")]:
            tk.Radiobutton(slot2_col, text=label,
                           variable=self.board_slot2_var, value=val,
                           bg=PANEL, fg=FG, selectcolor='#21262d',
                           activebackground=PANEL, activeforeground=WHITE,
                           font=fSmall, cursor='hand2',
                           command=self.send_board_slot2).pack(anchor='w', pady=1)
        self.lbl_board_slot2_status = tk.Label(slot2_col, text="", bg=PANEL,
                                               fg=GREEN, font=fSmall)
        self.lbl_board_slot2_status.pack(anchor='w', pady=(4, 0))

        # Reserved purely for the validation-rule warning (see
        # _check_board_slots()) — never touched by _send_cmd(), unlike
        # the two per-slot "Sent →..." labels above, so the warning
        # can't be silently overwritten by the send confirmation the
        # way sharing one label would cause.
        self.lbl_board_status = tk.Label(board_frame, text="", bg=PANEL,
                                         fg=GREEN, font=fSmall, justify='left',
                                         wraplength=440)
        self.lbl_board_status.pack(anchor='w', pady=(8, 0))

        # =================================================================
        # Receiver Configuration tab
        # =================================================================
        # Three top-level GNSS categories, each optionally with a nested
        # receiver-type sub-choice — same nested-radio-button pattern as
        # the Steering Brand panel above (one flat IntVar per level,
        # radio buttons grouped visually under their parent category).
        #
        # LIVE as of TFF v0.1: category 1 (Dual antenna receivers) sends
        # SETGNSSMODE, category 2 (Single+IMU) sends SETGNSSMODE, and
        # category 3's F9P/UM980 sub-choice sends SETRECEIVERTYPE — see
        # send_gnss_mode()/send_receiver_type() below. The two
        # "not implemented yet" dual-antenna sub-options (u-blox X20D,
        # Septentrio Mosaic-H) and the two "not implemented yet"
        # dual-single-receiver sub-options (u-blox X20P, Septentrio
        # Mosaic-X5) remain UI-only — there is no firmware source for
        # them to select, so choosing them has no effect (X20D/Mosaic-H
        # aren't wired to any command at all; X20P/Mosaic-X5 are
        # refused client-side by send_receiver_type() rather than sent).
        # =================================================================
        # GNSS Passthrough — deliberately placed first in this tab, above
        # every other Receiver Configuration section, since turning it on
        # makes all of them irrelevant: GnssMode, the dual-receiver-type
        # sub-choice, and the Kalman filter section below all stop
        # mattering once GnssPassthrough is on, because none of TFF's
        # fusion engine runs at all in that mode.
        #
        # Restored from Chris Kinal's original UM982 firmware
        # ("udpPassthrough") — for users who would rather trust the
        # receiver's own onboard fusion outright (e.g. a UM982 configured
        # to output $KSXT, which already contains position, heading,
        # roll, and pitch, self-computed by the receiver) than use any of
        # TFF's alpha-blend/IMU-fallback/Kalman features. Content-
        # agnostic — firmware forwards whatever complete "$...\r\n" line
        # the receiver sends, unexamined; getting $KSXT specifically out
        # of it is purely a receiver-configuration choice made outside
        # this tool (e.g. "KSXT COM2 1" in a uPrecise terminal session),
        # not something this toggle itself selects.
        # =================================================================
        passthrough_frame = tk.Frame(tab_receiver, bg=PANEL, bd=1, relief='flat',
                                     padx=8, pady=4)
        passthrough_frame.pack(fill='x', padx=10, pady=(4, 3))
        tk.Label(passthrough_frame,
                 text="GNSS PASSTHROUGH  —  bypass every TFF fusion feature entirely",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')
        tk.Label(passthrough_frame,
                 text="⚠ Requires Teensy restart to take effect",
                 bg=PANEL, fg=YELLOW, font=fSmall).pack(anchor='w', pady=(1, 0))
        tk.Label(passthrough_frame,
                 text="When on, every setting below (GNSS mode, dual receiver type, Kalman filter) "
                      "is ignored — the receiver's raw sentence is forwarded to AGO byte for byte, "
                      "with zero processing on the Teensy. Off by default. Only turn this on if the "
                      "receiver has been separately configured to output a self-contained sentence "
                      "AGO can parse on its own (e.g. UnicoreComm $KSXT) — this toggle does not "
                      "configure the receiver itself.",
                 bg=PANEL, fg=DIM, font=fNote, justify='left',
                 wraplength=440).pack(anchor='w', pady=(4, 6))

        self.gnss_passthrough_var = tk.BooleanVar(value=False)
        tk.Checkbutton(passthrough_frame,
                       text="Enable GNSS Passthrough",
                       variable=self.gnss_passthrough_var,
                       bg=PANEL, fg=FG, selectcolor='#21262d',
                       activebackground=PANEL, activeforeground=WHITE,
                       font=fSmall, cursor='hand2',
                       command=self.send_gnss_passthrough).pack(anchor='w')

        self.lbl_passthrough_status = tk.Label(passthrough_frame, text="", bg=PANEL,
                                               fg=GREEN, font=fSmall)
        self.lbl_passthrough_status.pack(anchor='w', pady=(3, 0))

        rx_frame = tk.Frame(tab_receiver, bg=PANEL, bd=1, relief='flat',
                            padx=8, pady=4)
        rx_frame.pack(fill='x', padx=10, pady=(4, 3))
        tk.Label(rx_frame,
                 text="GNSS RECEIVER CONFIGURATION  —  select one (not yet sent to firmware, see note in code)",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')

        # Category radio buttons: 1=Dual antenna receiver (one physical
        # unit with two antenna ports, e.g. UM982), 2=Single antenna +
        # IMU (PANDA), 3=Dual single-antenna receivers (two independent
        # units, moving-base heading, e.g. 2x F9P or 2x UM980).
        self.gnss_category_var = tk.IntVar(value=1)

        # --- Category 1: Dual antenna receivers ---
        cat1_row = tk.Frame(rx_frame, bg=PANEL)
        cat1_row.pack(fill='x', anchor='w', pady=(6, 0))
        tk.Radiobutton(cat1_row, text="Single receiver, dual antenna (e.g. UM982)",
                       variable=self.gnss_category_var, value=1,
                       bg=PANEL, fg=FG, selectcolor='#21262d',
                       activebackground=PANEL, activeforeground=WHITE,
                       font=fSmall, cursor='hand2',
                       command=self.send_gnss_mode).pack(anchor='w')

        # Note: dual_antenna_type_var (which physical dual-antenna
        # receiver — UM982 vs Bynav vs future others) has no firmware
        # command of its own: MODE_UM982 in firmware is UM982-only
        # today, there's nothing to select yet. It's kept as local UI
        # state so the layout is ready once a second dual-antenna
        # source is actually implemented, but selecting anything other
        # than UM982 here currently has no effect on the Teensy at all
        # — the "(not implemented yet)" labels are the honest signal.
        self.dual_antenna_type_var = tk.IntVar(value=0)
        cat1_sub = tk.Frame(rx_frame, bg=PANEL)
        cat1_sub.pack(fill='x', padx=(24, 0), pady=(2, 6))
        DUAL_ANTENNA_OPTIONS = [
            (0, "UnicoreComm UM982"),
            (1, "Bynav C2-M20D"),
            (2, "u-blox X20D (not implemented yet)"),
            (3, "Septentrio Mosaic-H (not implemented yet)"),
        ]
        for val, label in DUAL_ANTENNA_OPTIONS:
            tk.Radiobutton(cat1_sub, text=label,
                           variable=self.dual_antenna_type_var, value=val,
                           bg=PANEL, fg=FG, selectcolor='#21262d',
                           activebackground=PANEL, activeforeground=WHITE,
                           font=fSmall, cursor='hand2').pack(anchor='w')

        # --- Category 2: Single antenna system, single+IMU ---
        cat2_row = tk.Frame(rx_frame, bg=PANEL)
        cat2_row.pack(fill='x', anchor='w', pady=(0, 0))
        self.rb_gnss_single_imu = tk.Radiobutton(cat2_row, text="Single antenna system, single+IMU",
                       variable=self.gnss_category_var, value=2,
                       bg=PANEL, fg=FG, selectcolor='#21262d',
                       activebackground=PANEL, activeforeground=WHITE,
                       font=fSmall, cursor='hand2',
                       command=self.send_gnss_mode)
        self.rb_gnss_single_imu.pack(anchor='w')
        tk.Label(rx_frame,
                 text="(Just requires GGA+VTG from any receiver)",
                 bg=PANEL, fg=DIM, font=fNote).pack(anchor='w', padx=(24, 0), pady=(0, 6))

        # --- Category 3: Dual single antenna receivers ---
        cat3_row = tk.Frame(rx_frame, bg=PANEL)
        cat3_row.pack(fill='x', anchor='w')
        tk.Radiobutton(cat3_row, text="Two receivers, moving-base heading (e.g. 2x F9P)",
                       variable=self.gnss_category_var, value=3,
                       bg=PANEL, fg=FG, selectcolor='#21262d',
                       activebackground=PANEL, activeforeground=WHITE,
                       font=fSmall, cursor='hand2',
                       command=self.send_gnss_mode).pack(anchor='w')

        # dual_single_type_var DOES have a real firmware command
        # (SETRECEIVERTYPE) — F9P (0) and UM980 (1) are both actually
        # implemented (zGNSS_DualF9P.ino / zGNSS_DualUM980.ino). The
        # other two options are still placeholders — send_receiver_type()
        # below refuses to send for those rather than pretending they
        # work, matching firmware's own SETRECEIVERTYPE range check.
        self.dual_single_type_var = tk.IntVar(value=0)
        cat3_sub = tk.Frame(rx_frame, bg=PANEL)
        cat3_sub.pack(fill='x', padx=(24, 0), pady=(2, 4))
        DUAL_SINGLE_OPTIONS = [
            (0, "2 x u-blox F9P"),
            (1, "2 x UnicoreComm UM980 (Not verified — see note below)"),
            (2, "2 x u-blox X20P (not implemented yet)"),
            (3, "2 x Septentrio Mosaic-X5 (not implemented yet)"),
        ]
        for val, label in DUAL_SINGLE_OPTIONS:
            tk.Radiobutton(cat3_sub, text=label,
                           variable=self.dual_single_type_var, value=val,
                           bg=PANEL, fg=FG, selectcolor='#21262d',
                           activebackground=PANEL, activeforeground=WHITE,
                           font=fSmall, cursor='hand2',
                           command=self.send_receiver_type).pack(anchor='w')
        tk.Label(rx_frame,
                 text="F9P: heading confirmed against public protocol docs, roll is a "
                      "well-reasoned inference — verify sign on a bench test.\n"
                      "UM980: field layout confirmed against Unicore's manual, but never "
                      "run against real hardware — not verified end-to-end.",
                 bg=PANEL, fg=DIM, font=fNote, justify='left').pack(anchor='w', padx=(24, 0), pady=(2, 0))

        tk.Label(rx_frame,
                 text="⚠ Requires Teensy restart to take effect (GNSS source only inits at boot)",
                 bg=PANEL, fg=YELLOW, font=fSmall).pack(anchor='w', pady=(2, 0))
        self.lbl_gnss_status = tk.Label(rx_frame, text="", bg=PANEL,
                                        fg=GREEN, font=fSmall)
        self.lbl_gnss_status.pack(anchor='w', pady=(2, 0))

        # =================================================================
        # Kalman filter panel — Receiver Configuration tab, not IMU.
        # Deliberate placement: this filter operates on heading/rollDual
        # (the raw GNSS/HPR-family value), not on the IMU signal — see
        # applyHeadingKalman() in zHandlers.ino. It's shared by all three
        # dual-capable sources (UM982/F9P/UM980), which is also why it
        # lives here rather than under any one receiver's own section.
        # =================================================================
        kalman_frame = tk.Frame(tab_receiver, bg=PANEL, bd=1, relief='flat',
                                padx=8, pady=4)
        kalman_frame.pack(fill='x', padx=10, pady=(4, 10))
        tk.Label(kalman_frame,
                 text="KALMAN FILTER  —  smooths raw GNSS heading/roll before the IMU blend "
                      "(all dual-capable sources share this)",
                 bg=PANEL, fg=DIM, font=fSmall).pack(anchor='w')

        # Toggles — live, no reboot required (firmware applies these on
        # the next update, see SETFILTERHEADING/SETFILTERROLL).
        self.filter_heading_var = tk.BooleanVar(value=False)
        self.filter_roll_var    = tk.BooleanVar(value=False)
        toggle_row = tk.Frame(kalman_frame, bg=PANEL)
        toggle_row.pack(fill='x', pady=(4, 2))
        tk.Checkbutton(toggle_row, text="Filter heading", variable=self.filter_heading_var,
                       bg=PANEL, fg=FG, selectcolor='#21262d',
                       activebackground=PANEL, activeforeground=WHITE,
                       font=fSmall, cursor='hand2',
                       command=self.send_filter_heading).pack(side='left', padx=(0, 16))
        tk.Checkbutton(toggle_row, text="Filter roll", variable=self.filter_roll_var,
                       bg=PANEL, fg=FG, selectcolor='#21262d',
                       activebackground=PANEL, activeforeground=WHITE,
                       font=fSmall, cursor='hand2',
                       command=self.send_filter_roll).pack(side='left')
        self.lbl_filter_status = tk.Label(kalman_frame, text="", bg=PANEL,
                                          fg=GREEN, font=fSmall)
        self.lbl_filter_status.pack(anchor='w')

        tk.Label(kalman_frame,
                 text="When to try this: if Dual Roll (IMU tab) looks noisier than the "
                      "physical tilt actually is — small, fast flickers rather than a real, "
                      "slow drift. Start with just one toggle (heading or roll, not both) so "
                      "you can tell what changed. Note: with ROLL_ALPHA/HEADING_ALPHA already "
                      "weighted heavily toward the IMU (e.g. 0.8), raw GNSS noise is already "
                      "attenuated in the blended output before this filter even runs — so if "
                      "the blended PAOGI signal already looks smooth, this filter may have "
                      "little visible effect regardless of its settings.",
                 bg=PANEL, fg=DIM, font=fNote, justify='left',
                 wraplength=440).pack(anchor='w', pady=(3, 0))

        tk.Label(kalman_frame,
                 text="Tuning (only matters once a toggle above is on): mea = how much you "
                      "distrust the raw reading — higher smooths harder. est = starting "
                      "uncertainty — usually leave at 1.0, it settles within a few seconds "
                      "either way. q = how fast the filter is allowed to follow a real change "
                      "— lower is smoother but laggier during genuine turns, higher reacts "
                      "faster but filters less. Change one number at a time.",
                 bg=PANEL, fg=DIM, font=fNote, justify='left',
                 wraplength=440).pack(anchor='w', pady=(3, 0))

        # Tuning — mea (measurement uncertainty), est (estimate
        # uncertainty), q (process noise). Reboot required — see
        # SETROLLKALMAN/SETHEADINGKALMAN in zHandlers.ino; unlike the
        # toggles above, these reconstruct the SimpleKalmanFilter object
        # itself, which only happens once, in setup().
        def make_kalman_row(parent, label_text, defaults, send_fn, status_attr):
            row = tk.Frame(parent, bg=PANEL)
            row.pack(fill='x', pady=(6, 0))
            tk.Label(row, text=label_text, bg=PANEL, fg=DIM, font=fSmall,
                    width=9, anchor='w').pack(side='left')
            entries = []
            for sub_label, default_val in zip(("mea", "est", "q"), defaults):
                tk.Label(row, text=sub_label, bg=PANEL, fg=DIM, font=fSmall).pack(side='left', padx=(6, 1))
                e = tk.Entry(row, width=6, bg='#21262d', fg=WHITE,
                            insertbackground=WHITE, font=fNorm, bd=1, relief='solid')
                e.insert(0, str(default_val))
                e.pack(side='left')
                entries.append(e)
            tk.Button(row, text="Set", bg='#1f6feb', fg=WHITE,
                     activebackground='#388bfd', font=fSmall, bd=0, padx=6, pady=2,
                     cursor='hand2', command=send_fn).pack(side='left', padx=(8, 0))
            status = tk.Label(row, text="", bg=PANEL, fg=GREEN, font=fSmall)
            status.pack(side='left', padx=(6, 0))
            setattr(self, status_attr, status)
            return entries

        # Defaults match firmware's own compiled-in starting values
        # (020_TFF.ino: rollMEA/rollEST/rollQ, headingMEA/headingEST/
        # headingQ) — shown here so the fields aren't just empty
        # placeholders before the first PDIAG round-trip.
        self.entry_roll_kalman = make_kalman_row(
            kalman_frame, "Roll:", (1.0, 1.0, 0.01),
            lambda: self.send_kalman_tuning('roll'), 'lbl_roll_kalman_status')
        self.entry_heading_kalman = make_kalman_row(
            kalman_frame, "Heading:", (1.0, 1.0, 0.01),
            lambda: self.send_kalman_tuning('heading'), 'lbl_heading_kalman_status')

        tk.Label(kalman_frame,
                 text="⚠ mea/est/q changes require a Teensy restart (filter object is only "
                      "built once, at boot) — the two toggles above do not.",
                 bg=PANEL, fg=YELLOW, font=fNote).pack(anchor='w', pady=(6, 0))

        # ---------------------------------------------------------------
        # State
        # ---------------------------------------------------------------
        self.last_rx      = None
        # Deliberately separate from last_rx — last_rx is set inside
        # _rx_thread() the moment a packet is successfully parsed off
        # the socket, entirely independent of whether _poll()/_apply()
        # (the code that actually updates the UI, health state, and
        # writes DATA: log lines) is still running at all. Found via a
        # real field session: the receive thread kept updating last_rx
        # normally while _apply() had apparently stopped being called,
        # and _log_heartbeat() — which only checked last_rx — kept
        # printing "STATUS OK" the whole time, with zero DATA: lines
        # in the log to show for it. Tracked separately so the
        # heartbeat can tell "receiving fine, not being processed"
        # apart from genuine, full health.
        self.last_successful_apply = None
        self.teensy_ip    = None
        self.sock_rx      = None
        self.sock_tx      = None
        self.lock         = threading.Lock()
        self.pending_data = None

        self._start_udp()
        self._update_clock()
        self._poll()
        self._log_heartbeat()   # harmless no-op via _log_line() while logging is off
        # Set initial IMU-tab graying to match the default GNSS category
        # (1 = Dual antenna receivers, so nothing should be grayed yet).
        self._update_imu_tab_state()

        # Touch-friendly numeric entry — see _bind_numeric_keypad() and
        # _open_numeric_keypad() below. Walks every widget built above
        # and attaches the popup keypad to every tk.Entry found, so all
        # ~15 existing numeric fields (alpha, WAS, Kalman mea/est/q,
        # U-turn strength, IMU axis, dual hold/ramp, sats full...) get it
        # automatically — no need to touch each field's own creation
        # code. Called once, after every panel already exists.
        self._bind_numeric_keypad(self.root)

    # -------------------------------------------------------------------
    # UDP receiver thread
    # -------------------------------------------------------------------
    def _start_udp(self):
        try:
            self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock_rx.bind(('', LISTEN_PORT))
            self.sock_rx.settimeout(0.5)
        except OSError as e:
            self.lbl_status.config(text=f"Socket error: {e}", fg=RED)
            return

        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_tx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        t = threading.Thread(target=self._rx_thread, daemon=True)
        t.start()

    def _rx_thread(self):
        """
        Runs forever on its own background thread (started once from
        __init__), continuously reading UDP packets on the PDIAG
        socket and parsing each into pending_data — the ONLY place
        this tool actually receives Teensy data at all. Never touches
        Tkinter widgets directly (not thread-safe) — just stores the
        parsed dict behind self.lock; _apply() (called from the main
        thread via .after()) picks it up and does the actual UI
        updates. A packet that fails to parse is logged as a genuine
        communication-quality signal rather than silently dropped —
        see the comment inline below for why that distinction matters.
        """
        while True:
            try:
                data, addr = self.sock_rx.recvfrom(512)
                sentence = data.decode('ascii', errors='ignore').strip()
                parsed = parse_pdiag(sentence)
                if parsed:
                    with self.lock:
                        self.pending_data = parsed
                        self.teensy_ip    = addr[0]
                        self.last_rx      = time.time()
                else:
                    # Malformed/unparseable sentence — previously
                    # silently dropped with no record at all. Now
                    # logged (when logging is on) as a genuine
                    # communication-quality signal, not routine noise:
                    # a UDP packet that arrived but didn't parse as a
                    # valid $PDIAG sentence indicates real corruption
                    # or truncation, not just "nothing received yet".
                    self._log_line(f"UNPARSEABLE PACKET from {addr[0]}: {sentence!r}")
            except socket.timeout:
                pass
            except Exception as e:
                self._log_line(f"RX THREAD ERROR: {e}")

    def _log_line(self, text: str):
        # Safe to call from _rx_thread (background thread) as well as
        # the main thread — only ever touches the file object, never a
        # Tkinter widget, so no cross-thread UI-safety concern here.
        # Silently does nothing if logging is off, and never lets a
        # logging failure (e.g. disk full, permissions) crash the rest
        # of the app — troubleshooting output is a nice-to-have, not
        # something that should be allowed to take the whole tool down.
        if self.log_file is None:
            return
        try:
            ts = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            self.log_file.write(f"{ts}  {text}\n")
            self.log_file.flush()
        except Exception:
            pass

    def _log_health_transition(self, name: str, is_problem: bool, problem_text: str):
        # Shared by every diagnostic health check (CAN timeouts, GNSS/
        # RTK watchdogs, WAS plausibility, Ethernet link) instead of
        # one near-identical block per check. Logs exactly once when a
        # problem STARTS and once when it CLEARS — never repeats every
        # poll tick while the state is unchanged, matching the
        # established _signal_was_lost pattern generalized to every
        # check at once. self._health_state also doubles as the "any
        # active problems right now" source for the periodic heartbeat
        # below — deliberately the single source of truth for both,
        # rather than tracking problem state twice.
        was_problem = self._health_state.get(name, False)
        if is_problem and not was_problem:
            self._log_line(f"COMMUNICATION ERROR: {problem_text}")
        elif was_problem and not is_problem:
            self._log_line(f"RESOLVED: {problem_text}")
        self._health_state[name] = is_problem

    def _log_heartbeat(self):
        # Periodic "is everything actually fine" summary — see the
        # discussion this was built from: scanning a long log for the
        # ABSENCE of errors is much harder than seeing a clear,
        # recurring confirmation. Every 30s, three possible states,
        # checked in this order:
        #   1. last_rx stale/missing — no signal at all this session;
        #      the "signal lost" transition logging already covers
        #      that case, nothing added here on top of it.
        #   2. last_rx fresh but last_successful_apply stale/missing —
        #      NEW as of this fix: raw packets are arriving (the
        #      receive thread is running fine) but _poll()/_apply()
        #      itself appears to have stopped. Found from a real field
        #      session where this exact split happened — the old
        #      version of this heartbeat only checked last_rx, so it
        #      kept printing "STATUS OK" for the entire session with
        #      not a single DATA: line logged to show for it. See
        #      last_successful_apply's own declaration comment for the
        #      full story.
        #   3. Both fresh — genuine health, checked via
        #      self._health_state as before.
        active = [name for name, is_problem in self._health_state.items() if is_problem]
        if self.last_rx and (time.time() - self.last_rx) <= 10:
            apply_stale = (not self.last_successful_apply
                            or (time.time() - self.last_successful_apply) > 10)
            if apply_stale:
                self._log_line(
                    "STATUS: receiving data but NOT processing it — "
                    "_poll()/_apply() appears to have stopped (try restarting Teensy Tool)")
            elif active:
                self._log_line(f"STATUS: ongoing issue(s) — {', '.join(active)}")
            else:
                self._log_line("STATUS OK — signal alive, no active issues")
        # else: no signal at all yet this session — the "signal lost"
        # transition logging already covers that case, nothing useful
        # to add here on top of it.
        self.root.after(30000, self._log_heartbeat)

    def _on_close(self):
        # Ensures the log file (if open) is closed cleanly rather than
        # just abandoned mid-write when the window closes — matters
        # because _log_line() calls write() but the OS may not have
        # actually flushed/closed the file handle to disk otherwise.
        if self.log_file is not None:
            self._log_line("=== Teensy Tool closing — log stopped ===")
            try:
                self.log_file.close()
            except Exception:
                pass
        self.root.destroy()

    # -------------------------------------------------------------------
    # Command senders — one method per SETxxx command (imu axis, roll
    # invert, WAS ends, U-turn strength, dual hold/ramp, brand, roll
    # zero, sats full, heading/roll alpha). All funnel through the
    # shared _send_cmd() helper below.
    # -------------------------------------------------------------------
    def _send_cmd(self, cmd_str: str, status_lbl: tk.Label,
                  note: str = "", note_fg: str = None, persist: bool = False):
        dest = self.teensy_ip if self.teensy_ip else BROADCAST
        try:
            self.sock_tx.sendto(cmd_str.encode('ascii'), (dest, SEND_PORT))
            text = f"Sent → {dest}:{SEND_PORT}"
            if note:
                text += f"  —  {note}"
            status_lbl.config(text=text, fg=(note_fg or GREEN))
            if not persist:
                self.root.after(3000, lambda: status_lbl.config(text="", fg=GREEN))
        except Exception as e:
            status_lbl.config(text=f"Send error: {e}", fg=RED)
            self._log_line(f"COMMUNICATION ERROR: failed to send '{cmd_str}' to {dest}:{SEND_PORT} — {e}")

    def send_az_param(self, entry, cmd_prefix, status_lbl, is_int, lo, hi):
        raw = entry.get().strip()
        try:
            # .replace(',', '.') — samma fix som redan finns för Dual
            # Hold/Ramp/Roll Zero, men saknades här (glömdes vid
            # byggnationen av den här specifika panelen) — svensk
            # standard är komma som decimaltecken, Pythons float()
            # kräver alltid punkt oavsett systemets språkinställning.
            v = int(raw) if is_int else float(raw.replace(',', '.'))
        except ValueError:
            status_lbl.config(text="Invalid", fg=RED)
            return
        if v < lo or v > hi:
            status_lbl.config(text=f"{lo}-{hi} only", fg=RED)
            return
        # int() truncates cleanly for the two integer fields (timeSlow/
        # timeFast/useBno/useGps); floats formatted plainly, matching
        # every other numeric SETxxx command in this file.
        self._send_cmd(f"{cmd_prefix}{v}", status_lbl)

    def send_imu_axis(self):
        try:
            v = int(self.entry_imu_axis.get().strip())
            if v not in (0, 1, 2):
                self.lbl_axis_status.config(text="0/1/2 only", fg=RED)
                return
        except ValueError:
            self.lbl_axis_status.config(text="Invalid", fg=RED)
            return
        self._send_cmd(f"SETIMUAXIS:{v}", self.lbl_axis_status)

    def _set_widget_tree_state(self, widget, enabled):
        """
        Recursively enable/disable a widget and all its children. Used to
        gray out the Roll Invert checkbox and the whole IMU Roll Offset
        panel when Single+IMU (PANDA) is selected — see
        _update_imu_tab_state(). Plain container Frames don't support
        the 'state' option, so the TclError from those is expected and
        silently ignored; only interactive widgets (Entry, Button,
        Checkbutton, Radiobutton, Label) actually change appearance.
        """
        state = 'normal' if enabled else 'disabled'
        try:
            widget.configure(state=state)
        except tk.TclError:
            pass
        for child in widget.winfo_children():
            self._set_widget_tree_state(child, enabled)

    def _bind_numeric_keypad(self, widget):
        """
        Recursively walks a widget tree and attaches the touch keypad
        (_open_numeric_keypad) to every tk.Entry found, opened on click.
        Normal typing still works exactly as before — this only adds a
        second, touch-friendly way to fill the same field, it doesn't
        replace or intercept keyboard input. Called once, in __init__,
        over the whole window, after every panel already exists — so
        every current numeric field (alpha, WAS, Kalman mea/est/q,
        U-turn strength, IMU axis, dual hold/ramp, sats full, ...) gets
        this automatically, with no changes needed at each field's own
        creation site.
        """
        if isinstance(widget, tk.Entry):
            widget.bind('<Button-1>', lambda e, w=widget: self._open_numeric_keypad(w))
        for child in widget.winfo_children():
            self._bind_numeric_keypad(child)

    def _open_numeric_keypad(self, entry):
        """
        Touch-friendly numeric entry popup — the same concept as
        AgOpenGPS's own built-in numeric keypad (confirmed to exist
        separately from the Windows touch keyboard, used for AGO's own
        numeric fields like the nudge distance dialog), so this should
        already feel familiar rather than like a new pattern to learn.
        Not a pixel-for-pixel copy of AGO's own keypad — its exact
        layout wasn't available to build against — but the same
        standard grid-of-digits-plus-decimal-plus-sign-plus-clear shape
        anyone who has used AGO, or any till/ATM-style keypad, already
        knows.

        Opens as a small modal window near the tapped field, pre-filled
        with its current value. Digit/decimal/sign buttons edit a
        local working string only — nothing is written back to the
        real Entry until OK is pressed, so Cancel (or tapping outside)
        leaves the field completely untouched.
        """
        popup = tk.Toplevel(self.root)
        popup.title("")
        popup.configure(bg=BG)
        popup.transient(self.root)
        popup.grab_set()   # modal — must be dismissed before returning to the field

        # Position near the field that was tapped, but nudged to stay
        # on-screen on a small tablet display.
        x = entry.winfo_rootx()
        y = entry.winfo_rooty() + entry.winfo_height() + 4
        popup.geometry(f"+{max(0, x)}+{max(0, y)}")

        display_var = tk.StringVar(value=entry.get())
        display = tk.Entry(popup, textvariable=display_var, font=self.fBig,
                           bg='#21262d', fg=WHITE, insertbackground=WHITE,
                           justify='right', bd=1, relief='solid', width=10)
        display.pack(fill='x', padx=6, pady=(6, 4), ipady=6)

        def press(char):
            # One key of the touchscreen numpad popup — appends the
            # pressed character to display_var, with the three special
            # keys ('⌫' backspace, '±' sign toggle, '.' decimal — only
            # one allowed) handled separately from plain digit keys.
            cur = display_var.get()
            if char == '⌫':
                display_var.set(cur[:-1])
            elif char == '±':
                if cur.startswith('-'):
                    display_var.set(cur[1:])
                else:
                    display_var.set('-' + cur)
            elif char == '.':
                if '.' not in cur:
                    display_var.set(cur + '.')
            else:
                display_var.set(cur + char)

        def confirm():
            entry.delete(0, tk.END)
            entry.insert(0, display_var.get())
            popup.destroy()
            # Auto-press this field's own "Set" button — one tap instead
            # of two. A separate, second press on the panel's Set button
            # was easy to miss, especially on a small touchscreen where
            # the popup closing draws the eye away from where the next
            # tap needs to land.
            #
            # Finds the right button by scanning forward through the
            # entry's own parent frame's children, in creation order,
            # for the next Button — not just "any Set button in this
            # parent", because some frames hold more than one field +
            # Set button pair (e.g. the dual reconnect row has both
            # Hold and Ramp fields sharing one parent frame). Each
            # field's own dedicated button is always the very next
            # Button created after that field's Entry, never a later
            # field's — so a forward scan from the entry's own position
            # always lands on the correct one.
            siblings = entry.master.winfo_children()
            if entry in siblings:
                idx = siblings.index(entry)
                for w in siblings[idx + 1:]:
                    if isinstance(w, tk.Button):
                        w.invoke()
                        break

        grid = tk.Frame(popup, bg=BG)
        grid.pack(padx=6, pady=(0, 6))
        rows = [['7', '8', '9'],
                ['4', '5', '6'],
                ['1', '2', '3'],
                ['±', '0', '.']]
        for r, row in enumerate(rows):
            for c, label in enumerate(row):
                tk.Button(grid, text=label, font=self.fMed,
                         bg=PANEL, fg=WHITE, activebackground='#30363d',
                         width=4, height=2, bd=0, cursor='hand2',
                         command=lambda ch=label: press(ch)).grid(row=r, column=c, padx=2, pady=2)

        bottom = tk.Frame(popup, bg=BG)
        bottom.pack(fill='x', padx=6, pady=(0, 6))
        tk.Button(bottom, text="⌫", font=self.fMed,
                 bg=PANEL, fg=WHITE, activebackground='#30363d',
                 width=4, height=2, bd=0, cursor='hand2',
                 command=lambda: press('⌫')).pack(side='left', padx=2)
        tk.Button(bottom, text="Cancel", font=self.fSmall,
                 bg='#3a1d1d', fg=WHITE, activebackground='#4a2525',
                 width=6, height=2, bd=0, cursor='hand2',
                 command=popup.destroy).pack(side='left', padx=2, expand=True, fill='x')
        tk.Button(bottom, text="OK", font=self.fSmall,
                 bg='#1f6feb', fg=WHITE, activebackground='#388bfd',
                 width=6, height=2, bd=0, cursor='hand2',
                 command=confirm).pack(side='left', padx=2, expand=True, fill='x')

    def _update_imu_tab_state(self):
        """
        Grays out Roll Invert and the IMU Roll Offset panel when Single
        antenna + IMU (PANDA, category 2) is selected in Receiver
        Configuration — see the long comment at the top of the IMU tab
        for why: AGO does its own IMU zeroing for PANDA, so firmware-side
        roll matching against a dual signal that doesn't exist in this
        mode would be meaningless. Roll axis is left alone regardless —
        it's a physical mounting fact, not a per-mode setting.

        Also enforces the two-way lock between GNSS category 2
        (Single+IMU) and IMU type 2 ("No fallback function", dual-only):
        the two are mutually exclusive, since Single+IMU has no dual
        signal to ever fall back ON in the first place, and "No
        fallback" has no IMU to fall back TO. Selecting either one, in
        either tab, grays out the other option and — if the other
        option happened to already be selected — forces it back to a
        sane default and actually notifies firmware of that forced
        change (not just a local UI correction), so the two tools never
        silently disagree about what's really configured.

        Called whenever a Receiver Configuration category radio button
        is clicked, whenever the IMU type radio button is clicked (see
        send_imu_type()), and once at startup to set the correct
        initial state for the defaults (category 1, IMU type 0).
        """
        is_single_imu = (self.gnss_category_var.get() == 2)
        is_no_fallback = (self.imu_type_var.get() == 2)

        self._set_widget_tree_state(self.chk_roll_invert, not is_single_imu)
        self._set_widget_tree_state(self.roll_zero_frame, not is_single_imu)
        self._set_widget_tree_state(self.auto_roll_frame, not is_single_imu)

        # Two-way lock — see docstring above.
        self._set_widget_tree_state(self.rb_imu_none, not is_single_imu)
        self._set_widget_tree_state(self.rb_gnss_single_imu, not is_no_fallback)

        if is_single_imu and is_no_fallback:
            # Only reachable if both were set before this lock existed
            # (e.g. an old saved state) — resolve deterministically by
            # keeping GNSS mode as the "primary" choice and forcing IMU
            # type back to a real sensor, rather than leaving an
            # impossible combination in place.
            self.imu_type_var.set(0)
            self._send_cmd(f"SETIMUTYPE:0", self.lbl_imu_type_status,
                            note="Forced to BNO08x — incompatible with Single+IMU",
                            note_fg=YELLOW, persist=True)

    def send_roll_invert(self):
        # Unchecked = X-axis forward (normal AGO mounting) = rollInvert=-1
        # Checked   = Invert Roll = rollInvert=1
        v = 1 if self.roll_invert_var.get() else -1
        self._send_cmd(f"SETROLLINVERT:{v}", self.lbl_invert_status)

    def send_was_left(self):
        # Captures the CURRENT, live steerAngleActual as the new left
        # limit — no typed value at all, unlike most other send_xxx()
        # methods here. See SETWASCURRENT's own comment in
        # zHandlers.ino: firmware itself does the constrain(), this
        # just fires the "capture now" trigger. send_was_right() below
        # is the mirror-image right-limit version.
        self._send_cmd("SETWASCURRENT:L", self.lbl_was_left)

    def send_was_right(self):
        self._send_cmd("SETWASCURRENT:R", self.lbl_was_right)

    def send_uturn_strength(self):
        # 1-100, matching firmware's own constrain() range exactly
        # (SETUTURNSTRENGTH handler, zHandlers.ino) — a value this UI
        # accepts is guaranteed to also be accepted unclamped there.
        try:
            v = int(self.entry_uturn_strength.get().strip())
            v = max(1, min(100, v))
        except ValueError:
            self.lbl_uturn_status.config(text="Invalid", fg=RED)
            return
        self._send_cmd(f"SETUTURNSTRENGTH:{v}", self.lbl_uturn_status)

    def send_dual_hold(self):
        # Dual reconnect timing pair — Hold is the stabilisation delay
        # right after a lost dual signal returns, Ramp is the
        # transition-fusion duration afterward (see the "DUAL
        # RECONNECT TIMING" panel's own explanatory text for the full
        # picture). Both 0-30s, matching firmware's own constrain()
        # range. send_dual_ramp() below follows the identical pattern.
        try:
            val = max(0.0, min(30.0, float(self.entry_dual_hold.get().strip().replace(',','.'))))
        except ValueError:
            self.lbl_hold_status.config(text="Invalid", fg=RED)
            return
        self._send_cmd(f"SETDUALHOLD:{val:.1f}", self.lbl_hold_status)

    def send_dual_ramp(self):
        try:
            val = max(0.0, min(30.0, float(self.entry_dual_ramp.get().strip().replace(',','.'))))
        except ValueError:
            self.lbl_ramp_status.config(text="Invalid", fg=RED)
            return
        self._send_cmd(f"SETDUALRAMP:{val:.1f}", self.lbl_ramp_status)

    def _check_board_slots(self):
        # Mirrors the two validation rules enforced firmware-side (see
        # loadAlphasFromEEPROM() in zHandlers.ino) — purely for local,
        # immediate warning display here, not enforcement. Firmware is
        # the actual authority: an invalid combination still left in
        # place at the next reboot resets to AIO-default there,
        # regardless of what this label says.
        SLOT_MASTER, SLOT_SLAVE = 0, 1
        s1, s2 = self.board_slot1_var.get(), self.board_slot2_var.get()
        if s1 == s2:
            self.lbl_board_status.config(
                text="⚠ Both slots are the same — set the other slot too before rebooting.",
                fg=YELLOW)
        elif s1 not in (SLOT_MASTER, SLOT_SLAVE) and s2 not in (SLOT_MASTER, SLOT_SLAVE):
            self.lbl_board_status.config(
                text="⚠ No Master or Slave GNSS role assigned to either slot.",
                fg=YELLOW)
        else:
            self.lbl_board_status.config(text="", fg=GREEN)

    def _check_motor_drive_conflict(self):
        # AIO v4.x boards have only ONE physical CAN channel on the main
        # connector (confirmed against the official AIO Board Pinout
        # docs: the second channel exists only on v2.x boards; on v4.x
        # those same two pins are repurposed for Cytron power input
        # instead — see the full explanation at the MOTOR_DRIVE_KEYA
        # declaration, 020_TFF.ino). Keya is hardcoded onto V_Bus, the
        # same bus a CAN-ready tractor Brand already needs for its own
        # engage-detection/valve feedback — so Brand != BRAND_NONE and
        # MotorDriveType == Keya together isn't something a setting
        # alone can resolve on standard hardware; it needs a separate,
        # external CAN transceiver wired to an otherwise-unused Teensy
        # CAN pin. Warned here, not blocked — some installations may
        # genuinely have that extra hardware in place.
        BRAND_NONE, MOTOR_DRIVE_KEYA = 8, 2
        if self.brand_var.get() != BRAND_NONE and self.motor_drive_var.get() == MOTOR_DRIVE_KEYA:
            self.lbl_keya_status.config(
                text="⚠ Brand + Keya both need V_Bus — AIO v4.x boards only have one CAN "
                     "channel on the main connector. Needs extra CAN hardware to use both; "
                     "see code comments for details.",
                fg=YELLOW)
        else:
            self.lbl_keya_status.config(text="", fg=GREEN)

    def send_board_slot1(self):
        # Board Configuration slot pair — which physical role (Master/
        # Slave GNSS, TM171, Empty) occupies each connector. Requires
        # a reboot to actually take effect (resolveBoardSlots() only
        # runs once, at setup()) — the "RESTART to apply!" note below
        # reflects that, not a bug. send_board_slot2() follows the
        # identical pattern for the other slot.
        self._check_board_slots()
        v = self.board_slot1_var.get()
        self._send_cmd(f"SETBOARDSLOT1:{v}", self.lbl_board_slot1_status,
                        note="RESTART to apply!",
                        note_fg=YELLOW, persist=True)

    def send_board_slot2(self):
        self._check_board_slots()
        v = self.board_slot2_var.get()
        self._send_cmd(f"SETBOARDSLOT2:{v}", self.lbl_board_slot2_status,
                        note="RESTART to apply!",
                        note_fg=YELLOW, persist=True)

    def send_motor_drive_type(self):
        # PWM (0) vs Keya CAN motor (2) — no value 1, matching
        # firmware's own MOTOR_DRIVE_PWM/MOTOR_DRIVE_KEYA constants
        # (020_TFF.ino). Requires a reboot (CAN bus setup only runs
        # once, at setup()).
        self._check_motor_drive_conflict()
        v = self.motor_drive_var.get()
        self._send_cmd(f"SETMOTORDRIVETYPE:{v}", self.lbl_keya_sent_status,
                        note="RESTART to apply!",
                        note_fg=YELLOW, persist=True)

    def send_was_source(self):
        # Normal (physical potentiometer, 0) vs Keya encoder (1) as
        # the steering-angle source — see zKeyaAutoZero.ino for why
        # the Keya path needs its own zero-calibration on top of just
        # selecting this. Requires a reboot to take effect.
        v = self.was_source_var.get()
        self._send_cmd(f"SETWASSOURCE:{v}", self.lbl_keya_sent_status,
                        note="RESTART to apply!",
                        note_fg=YELLOW, persist=True)

    def send_brand(self):
        # Called automatically when a radio button is clicked
        # (IntVar already holds the new value at this point).
        self._check_motor_drive_conflict()
        v = self.brand_var.get()
        self._send_cmd(f"SETBRAND:{v}", self.lbl_brand_status,
                        note="RESTART TEENSY to apply!",
                        note_fg=YELLOW, persist=True)

    def send_filter_heading(self):
        # Live, no reboot needed — firmware applies this on the next
        # HPR/RELPOSNED/UNIHEADING2 update (see applyHeadingKalman()
        # call sites, gated on this flag, in each zzGNSS_*.ino file).
        v = 1 if self.filter_heading_var.get() else 0
        self._send_cmd(f"SETFILTERHEADING:{v}", self.lbl_filter_status)

    def send_filter_roll(self):
        # Live, no reboot needed — see send_filter_heading() above for
        # why (identical mechanism, just the roll Kalman filter
        # instead of heading).
        v = 1 if self.filter_roll_var.get() else 0
        self._send_cmd(f"SETFILTERROLL:{v}", self.lbl_filter_status)

    def send_kalman_tuning(self, which):
        # which is 'roll' or 'heading' — selects which SimpleKalmanFilter
        # instance and which entry row to read from. Reboot required:
        # unlike the toggles above, this reconstructs the filter object
        # itself (mea/est/q are constructor arguments, not settable
        # after the fact), which only happens once, in setup().
        if which == 'roll':
            entries, status_lbl, cmd = self.entry_roll_kalman, self.lbl_roll_kalman_status, "SETROLLKALMAN"
        else:
            entries, status_lbl, cmd = self.entry_heading_kalman, self.lbl_heading_kalman_status, "SETHEADINGKALMAN"
        try:
            mea, est, q = (float(e.get().strip().replace(',', '.')) for e in entries)
        except ValueError:
            status_lbl.config(text="Invalid number", fg=RED)
            return
        self._send_cmd(f"{cmd}:{mea},{est},{q}", status_lbl,
                        note="RESTART TEENSY to apply!",
                        note_fg=YELLOW, persist=True)

    def send_imu_type(self):
        # Called when the IMU type radio button (BNO08x/TM171/No
        # fallback) is clicked, at the top of the IMU tab. Same
        # restart-required pattern as GnssMode/Brand — I2C (BNO08x) vs
        # UART (TM171) vs nothing (No fallback) init only happens once,
        # in setup().
        self._update_imu_tab_state()   # keep the GNSS-mode/IMU-type lock in sync
        v = self.imu_type_var.get()
        self._send_cmd(f"SETIMUTYPE:{v}", self.lbl_imu_type_status,
                        note="RESTART TEENSY to apply!",
                        note_fg=YELLOW, persist=True)

    def send_logfile_toggle(self):
        # Purely local to this PC — nothing sent to Teensy, unlike
        # every other send_xxx() method in this file. Named to match
        # the established pattern anyway since it's wired to a
        # Checkbutton's command= the same way.
        if self.logfile_var.get():
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = f"tff_log_{ts}.txt"
            # Byggs som en EXPLICIT, absolut sökväg mot skriptfilens EGEN
            # mapp (os.path.dirname(os.path.abspath(__file__))) — INTE
            # bara ett rått filnamn (vilket skriver till Pythons "aktuella
            # arbetskatalog", cwd). Skillnaden spelar normalt ingen roll
            # (dubbelklick i Utforskaren ger oftast cwd = filens egen
            # mapp) — MEN inte garanterat: en verklig, fältrapporterad
            # krasch (Errno 13, Permission denied) visade sig troligen
            # bero på att skriptet öppnades direkt från en tillfällig
            # temp-mapp (skapad av en e-postklient/dokumentvisare vid
            # extrahering av en bilaga), där cwd kan vara en helt annan,
            # mer restriktiv plats än filens egen mapp.
            script_dir = os.path.dirname(os.path.abspath(__file__))
            full_path = os.path.join(script_dir, fname)
            try:
                self.log_file = open(full_path, 'w', encoding='utf-8')
                self.log_file_path = full_path
                self._signal_was_lost = False
                self._log_line(f"=== Teensy Tool logging started (Teensy IP: {self.teensy_ip or 'unknown'}) ===")
                self.lbl_logfile_status.config(text=f"Logging to {fname}", fg=GREEN)
            except OSError as e:
                self.log_file = None
                self.logfile_var.set(False)
                # Tydligare, mer hjälpsamt fel — nämner den troliga,
                # verkliga orsaken (körd från en icke-skrivbar/tillfällig
                # plats) istället för bara det råa OSError-meddelandet.
                self.lbl_logfile_status.config(
                    text=f"Could not open log file (folder not writable — try saving "
                         f"the .py file to Downloads/Documents first): {e}",
                    fg=RED)
        else:
            if self.log_file is not None:
                self._log_line("=== Logging stopped ===")
                try:
                    self.log_file.close()
                except Exception:
                    pass
                self.log_file = None
            self.lbl_logfile_status.config(text="Logging off", fg=DIM)

    def send_gnss_passthrough(self):
        # Bypasses ALL of TFF's own heading/roll/quality processing —
        # raw receiver bytes forwarded straight through unmodified
        # when enabled (gnssPassthrough_update(), zGnssPassthrough.ino).
        # Requires a reboot.
        v = 1 if self.gnss_passthrough_var.get() else 0
        self._send_cmd(f"SETGNSSPASSTHROUGH:{v}", self.lbl_passthrough_status,
                        note="RESTART TEENSY to apply!",
                        note_fg=YELLOW, persist=True)

    def send_gnss_mode(self):
        # Called when a top-level Receiver Configuration category radio
        # button is clicked. Also updates IMU-tab graying immediately
        # (previously this method's job alone, before GNSS mode had a
        # real firmware command — see _update_imu_tab_state()).
        self._update_imu_tab_state()
        v = self.gnss_category_var.get()
        self._send_cmd(f"SETGNSSMODE:{v}", self.lbl_gnss_status,
                        note="RESTART TEENSY to apply!",
                        note_fg=YELLOW, persist=True)

    def send_receiver_type(self):
        # Called when a dual-single-receiver sub-choice (F9P/UM980/...)
        # is clicked. Only F9P (0) and UM980 (1) are actually implemented
        # in firmware — refuse to send for the other two rather than
        # pretending they work, matching firmware's own SETRECEIVERTYPE
        # range check in receiveMonitorCommands().
        v = self.dual_single_type_var.get()
        if v not in (0, 1):
            self.lbl_gnss_status.config(
                text="That receiver pair isn't implemented yet — not sent.",
                fg=YELLOW)
            return
        self._send_cmd(f"SETRECEIVERTYPE:{v}", self.lbl_gnss_status,
                        note="RESTART TEENSY to apply!",
                        note_fg=YELLOW, persist=True)

    def send_roll_zero(self):
        # BNO roll offset, applied in fallback (IMU-only) mode as
        # outRoll = BNO_roll - rollZeroOffset. Set to the current BNO
        # roll reading while parked on flat ground to zero out a
        # mounting-angle offset. -30 to 30°, matching firmware's own
        # constrain() range exactly.
        try:
            val = float(self.entry_roll_zero.get().strip().replace(',', '.'))
            val = max(-30.0, min(30.0, val))
        except ValueError:
            self.lbl_zero_status.config(text="Invalid value", fg=RED)
            return
        self._send_cmd(f"SETROLLZERO:{val:.2f}", self.lbl_zero_status)

    def send_auto_roll_adjust(self):
        v = 1 if self.auto_roll_var.get() else 0
        # No client-side "are you sure" — unchecking simply reverts to
        # today's manual SETROLLZERO control, exactly matching how it
        # already worked before this feature existed. Firmware itself
        # folds the live correction into rollZeroOffset with a single
        # EEPROM write on this transition (see SETAUTOROLLADJUST
        # handler, zHandlers.ino) — nothing client-side to do about that.
        self._send_cmd(f"SETAUTOROLLADJUST:{v}", self.lbl_auto_roll_status)

    def send_roll_auto_param(self, entry, cmd_prefix, status_lbl, lo, hi):
        raw = entry.get().strip()
        try:
            v = float(raw.replace(',', '.'))
        except ValueError:
            status_lbl.config(text="Invalid", fg=RED)
            return
        if v < lo or v > hi:
            status_lbl.config(text=f"{lo}-{hi} only", fg=RED)
            return
        self._send_cmd(f"{cmd_prefix}{v}", status_lbl)

    def send_sats_full(self):
        # HPR heading-satellite threshold above which the dual/HPR
        # signal is considered fully trustworthy for quality
        # weighting. 1-40, matching firmware's own constrain() range
        # exactly — NOT 1-30, a real bug this tool once had until
        # corrected against the actual firmware constraint.
        try:
            val = max(1, min(40, int(self.entry_sats_full.get().strip())))
        except ValueError:
            self.lbl_sats_status.config(text="Invalid value", fg=RED)
            return
        self._send_cmd(f"SETSATSFULL:{val}", self.lbl_sats_status)

    def send_heading_alpha(self):
        # Initial (minimum) alpha pair — heading and roll each have
        # their own. Actual runtime HEADING_ALPHA/ROLL_ALPHA will
        # always be >= this floor, rising further based on live
        # signal quality (see updateAlphaControl(), zHandlers.ino).
        # 0.0-1.0, live (no reboot needed). send_roll_alpha() below is
        # the identical pattern for the roll counterpart.
        try:
            val = max(0.0, min(1.0, float(self.entry_heading_alpha.get().strip().replace(',', '.'))))
        except ValueError:
            self.lbl_heading_status.config(text="Invalid value", fg=RED)
            return
        self._send_cmd(f"SETHEADINGALPHA:{val:.4f}", self.lbl_heading_status)

    def send_roll_alpha(self):
        try:
            val = max(0.0, min(1.0, float(self.entry_roll_alpha.get().strip().replace(',', '.'))))
        except ValueError:
            self.lbl_roll_status.config(text="Invalid value", fg=RED)
            return
        self._send_cmd(f"SETROLLALPHA:{val:.4f}", self.lbl_roll_status)

    # -------------------------------------------------------------------
    # UI update poll (every 500ms)
    # -------------------------------------------------------------------
    def _poll(self):
        with self.lock:
            data = self.pending_data
            self.pending_data = None

        if data:
            self._apply(data)
        elif self.last_rx and (time.time() - self.last_rx) > 10:
            # lbl_mode fanns inte längre (ingen mode-etikett kvar i UI:t
            # alls, av oklar historisk anledning i den här sessionen)
            # — den trasiga raden orsakade en genuin krasch (kraschar
            # TILLBAKA till detta ställe var 500:e ms, i evighet, så
            # fort signalen väl tappats en gång) och togs bort. Bara
            # lbl_age (bekräftat existerande) uppdateras nu.
            self.lbl_age.config(text="Signal lost", fg=RED)
            if not self._signal_was_lost:
                self._log_line("COMMUNICATION ERROR: no $PDIAG received for >10s (signal lost)")
                self._signal_was_lost = True

        self.root.after(500, self._poll)

    def _apply(self, d: dict):
        # Set first, unconditionally — the whole point is to mark that
        # _apply() genuinely got called and started running, distinct
        # from last_rx (which _rx_thread() sets on raw packet arrival,
        # whether or not _poll()/_apply() ever processes it — see the
        # instance variable's own declaration comment for the full
        # story). If anything below this line were ever to raise, this
        # timestamp still correctly reflects "_apply() did start" up
        # to the point of failure, which is the right thing for the
        # heartbeat to see either way.
        self.last_successful_apply = time.time()

        # Every successfully parsed $PDIAG, logged in full when logging
        # is on — the "sensor data" half of the logfile feature. Firing
        # rate here is naturally capped at ~2Hz by _poll()'s own 500ms
        # interval (pending_data gets overwritten if firmware sends
        # faster than that), which keeps log file size reasonable for a
        # troubleshooting session without needing separate throttling.
        self._log_line(f"DATA: {d}")
        if self._signal_was_lost:
            self._log_line("COMMUNICATION: signal restored")
            self._signal_was_lost = False

        # Diagnostic health transitions — see _log_health_transition()
        # for why these are logged as start/clear events, not every
        # poll tick. d.get(...) == 1 correctly treats a missing field
        # (older firmware without this feature) as "not a problem"
        # rather than raising or false-alarming.
        self._log_health_transition('can_k',       d.get('can_k_timeout')   == 1,
                                     "CAN K-Bus timeout (no message received)")
        self._log_health_transition('can_iso',     d.get('can_iso_timeout') == 1,
                                     "CAN ISO-Bus timeout (no message received)")
        self._log_health_transition('can_v',       d.get('can_v_timeout')   == 1,
                                     "CAN V-Bus timeout (no message received)")
        self._log_health_transition('can_content', d.get('can_implausible') == 1,
                                     "CAN content implausible (steeringValveReady out of sane range)")
        self._log_health_transition('gnss_wd',     d.get('gnss_watchdog')   == 1,
                                     "GNSS serial watchdog timeout (no byte received from receiver)")
        self._log_health_transition('hpr_wd',      d.get('hpr_timeout')     == 1,
                                     "HPR watchdog timeout (GNSS bytes arriving, but no valid $GPHPR "
                                     "sentence completed — only meaningful in UM982 mode)")
        self._log_health_transition('rtk',         d.get('rtk_timeout')     == 1,
                                     "RTK radio timeout (no RTCM byte received)")
        self._log_health_transition('was',         d.get('was_implausible') == 1,
                                     "WAS signal implausible (out of calibrated range or sudden jump)")
        self._log_health_transition('eth_link',    d.get('ethernet_link_up') == 0,
                                     "Ethernet link down")

        # IMU watchdog — genuinely missed alongside the other checks
        # above, added after the fact. Skipped entirely when
        # imu_type == 2 (IMU_NONE): no IMU is expected there, so
        # imuHealthy/useIMU being "false" is normal, not a problem.
        # use_imu == 0 (watchdog gave up permanently after max
        # retries) is logged as its own, more severe transition,
        # separate from imu_healthy == 0 while still actively
        # retrying — collapsing the two into one flag would lose the
        # distinction between "still trying to recover" and "already
        # gave up, running dual-only for the rest of the session".
        # imuHealthy is only meaningfully re-checked here while
        # use_imu == 1: once the watchdog has given up,
        # checkImuWatchdog() stops running in firmware and imuHealthy
        # is left frozen at its last value, so re-logging it as an
        # ongoing transition at that point would be redundant with —
        # and could misleadingly look separate from — the
        # already-logged "gave up" event above it.
        imu_type = d.get('imu_type')
        use_imu  = d.get('use_imu')
        if imu_type is not None and imu_type != 2:
            self._log_health_transition(
                'imu_given_up', use_imu == 0,
                "IMU watchdog gave up permanently (exceeded max retries) — running dual-only for the rest of this session")
            if use_imu == 1:
                self._log_health_transition(
                    'imu_unhealthy', d.get('imu_healthy') == 0,
                    "IMU watchdog unhealthy (retrying)")
            else:
                # use_imu == 0: superseded by the more severe
                # "imu_given_up" state above — clear silently rather
                # than logging a misleading "RESOLVED" (it didn't
                # recover, it gave up) or leaving it stuck true forever
                # in the heartbeat's ongoing-issues list.
                self._health_state['imu_unhealthy'] = False

        # Reset cause — reported once per new value seen (i.e. once per
        # actual Teensy reboot detected), not every poll tick.
        rc = d.get('reset_cause')
        if rc is not None and rc != self._last_reset_cause:
            rc_labels = {0: 'unknown/other', 1: 'power-on', 2: 'watchdog', 3: 'software'}
            self._log_line(f"TEENSY RESET CAUSE: {rc_labels.get(rc, rc)} (code {rc})")
            self._last_reset_cause = rc

        # IMU% = average of heading and roll alpha × 100
        # alpha=0 → pure HPR (0% IMU), alpha=1 → pure IMU (100% IMU)
        ha = d.get('heading_alpha', 0.0)
        ra = d.get('roll_alpha',    0.0)
        imu_pct  = int(round((ha + ra) / 2.0 * 100))
        dual_pct = 100 - imu_pct

        imu_color  = GREEN if imu_pct <= 20 else YELLOW if imu_pct <= 60 else RED
        dual_color = GREEN if dual_pct >= 80 else YELLOW if dual_pct >= 40 else RED

        self.lbl_imu_pct.config(text=f"IMU {imu_pct}%",    fg=imu_color)
        self.lbl_dual_pct2.config(text=f"{dual_pct}% DUAL", fg=dual_color)

        # Bidirectional bar — center is neutral (50/50)
        # Left half (red) grows left = more IMU
        # Right half (green) grows right = more DUAL
        self.canvas_bar.delete('all')
        w = 160
        mid = w // 2

        # IMU side (left of center): imu_pct=50 → no fill, imu_pct=100 → full left
        imu_fill = int((imu_pct - 50) / 50.0 * mid) if imu_pct > 50 else 0
        # DUAL side (right of center): dual_pct=50 → no fill, dual_pct=100 → full right
        dual_fill = int((dual_pct - 50) / 50.0 * mid) if dual_pct > 50 else 0

        if imu_fill > 0:
            self.canvas_bar.create_rectangle(
                mid - imu_fill, 0, mid, 12, fill=RED, outline='')
        if dual_fill > 0:
            self.canvas_bar.create_rectangle(
                mid, 0, mid + dual_fill, 12, fill=GREEN, outline='')

        # Center marker — always visible
        self.canvas_bar.create_rectangle(
            mid - 1, 0, mid + 1, 12, fill=DIM, outline='')

        sol = d.get('sol', 0)
        sol_text, sol_color = SOL_LABELS.get(sol, (f"Quality {sol}", YELLOW))
        self.lbl_sol.config(
            text=f"HPR heading solution: {sol_text}  (code {sol})",
            fg=sol_color)
        # Note: HPR quality is separate from GGA position quality.
        # AGO shows position RTK; monitor shows heading RTK.



        self.lbl_sats_m.config(
            text=str(d['sats_m']),
            fg=GREEN if d['sats_m'] >= 10 else YELLOW if d['sats_m'] > 0 else RED)
        self.lbl_sats_s.config(
            text=str(d['sats_s']),
            fg=GREEN if d['sats_s'] >= 10 else YELLOW if d['sats_s'] > 0 else RED)
        hs = d.get('hpr_sats', 0)
        self.lbl_hpr_sats.config(
            text=str(hs),
            fg=GREEN if hs >= 5 else YELLOW if hs > 0 else RED)

        age = d.get('hpr_age_seconds')
        if age is None:
            self.lbl_hpr_age.config(text="--", fg=DIM)
        elif age < 0:
            self.lbl_hpr_age.config(text="never", fg=RED)
        else:
            # Colour matches the same 8s threshold hpr_timeout itself
            # uses (firmware's HPR_WATCHDOG_TIMEOUT_MS) — green while
            # comfortably under it, yellow approaching it, red past it
            # — but this label updates every ~2s regardless, unlike
            # the boolean flag which only flips once the threshold is
            # actually crossed.
            fg = GREEN if age < 4 else YELLOW if age < 8 else RED
            self.lbl_hpr_age.config(text=f"{age:.1f}s ago", fg=fg)

        # Keya auto-zero — zero_done first, the single most important
        # one (whether guidance is even possible right now), coloured
        # distinctly from the ten tuning parameters below it.
        zero_done = d.get('az_zero_done', 0)
        zero_deg  = d.get('az_zero_deg', 0.0)
        if zero_done:
            self.lbl_az_zero_status.config(
                text=f"Zero established: YES  (offset {zero_deg:+.2f}°)", fg=GREEN)
        else:
            self.lbl_az_zero_status.config(
                text="Zero established: NO — guidance blocked until the vehicle "
                     "drives straight for a bit", fg=RED)

        self.lbl_az_speed_min.config(text=f"Current: {d.get('az_speed_min', 0):.2f}")
        self.lbl_az_yaw_rate_max.config(text=f"Current: {d.get('az_yaw_rate_max', 0):.2f}")
        self.lbl_az_gps_hdg_max.config(text=f"Current: {d.get('az_gps_hdg_max', 0):.2f}")
        self.lbl_az_time_slow.config(text=f"Current: {d.get('az_time_slow_ms', 0)}")
        self.lbl_az_time_fast.config(text=f"Current: {d.get('az_time_fast_ms', 0)}")
        self.lbl_az_speed_slow.config(text=f"Current: {d.get('az_speed_slow', 0):.2f}")
        self.lbl_az_speed_fast.config(text=f"Current: {d.get('az_speed_fast', 0):.2f}")
        self.lbl_az_use_bno.config(text=f"Current: {d.get('az_use_bno', 0)}")
        self.lbl_az_use_gps.config(text=f"Current: {d.get('az_use_gps', 0)}")
        self.lbl_az_beta.config(text=f"Current: {d.get('az_beta', 0):.3f}")

        # Sync entry fields to the REAL, received value — but only
        # ONCE per session (guarded by _az_synced_once), not every
        # ~2s PDIAG packet. Without this guard, typing a new value
        # into an entry field would get silently overwritten by the
        # next packet's old value before the user even gets to click
        # Set. The one-time sync still fixes the actual problem this
        # was built for: the entry fields start with a hardcoded
        # PLACEHOLDER default (set when the UI was built) that does
        # NOT necessarily match what's really saved in EEPROM — left
        # unsynced, clicking Set without noticing would silently
        # overwrite a real, previously-saved value with that
        # placeholder.
        if not self._az_synced_once and 'az_speed_min' in d:
            sync_map = [
                ('speed_min',    d.get('az_speed_min'),    '.2f'),
                ('yaw_rate_max', d.get('az_yaw_rate_max'), '.2f'),
                ('gps_hdg_max',  d.get('az_gps_hdg_max'),  '.2f'),
                ('time_slow',    d.get('az_time_slow_ms'), 'd'),
                ('time_fast',    d.get('az_time_fast_ms'), 'd'),
                ('speed_slow',   d.get('az_speed_slow'),   '.2f'),
                ('speed_fast',   d.get('az_speed_fast'),   '.2f'),
                ('use_bno',      d.get('az_use_bno'),      'd'),
                ('use_gps',      d.get('az_use_gps'),      'd'),
                ('beta',         d.get('az_beta'),         '.3f'),
            ]
            for key, value, fmt in sync_map:
                if value is not None:
                    _, entry, _ = self._az_rows[key]
                    entry.delete(0, 'end')
                    entry.insert(0, format(value, fmt))
            self._az_synced_once = True

        self.lbl_offset.config(text=f"{d['hdg_offset']:+.2f} °")

        def colour_alpha(val):
            # 0.0 (pure dual/HPR, best case) = green, small IMU
            # blend = yellow warning, larger IMU blend = red — a
            # quick visual read of how much the fusion currently
            # leans on the IMU instead of dual/HPR, without needing
            # to read the exact number.
            if val == 0.0:    return GREEN
            elif val <= 0.15: return YELLOW
            else:             return RED

        ha = d['heading_alpha']
        ih = d.get('init_h', ha)
        self.lbl_heading_alpha.config(
            text=f"{ha:.4f}  (init: {ih:.4f})", fg=colour_alpha(ha))

        sf = d.get('sats_full', 7)
        self.lbl_sats_full.config(
            text=f"{sf}  sats",
            fg=GREEN if sf >= 5 else YELLOW)

        dh = d.get('dual_hold_s', 3.0)
        dr = d.get('dual_ramp_s', 5.0)
        self.lbl_dual_hold.config(text=f"{dh:.1f}")
        self.lbl_dual_ramp.config(text=f"{dr:.1f}")
        if self.root.focus_get() != self.entry_dual_hold:
            self.entry_dual_hold.delete(0, 'end')
            self.entry_dual_hold.insert(0, f"{dh:.1f}")
        if self.root.focus_get() != self.entry_dual_ramp:
            self.entry_dual_ramp.delete(0, 'end')
            self.entry_dual_ramp.insert(0, f"{dr:.1f}")

        ax = d.get('imu_axis', 1)
        ri = d.get('roll_invert', -1)
        self.roll_invert_var.set(ri > 0)  # checked=inverted(1), unchecked=normal(-1)
        axis_name = ['X','Y','Z'][ax] if ax in (0,1,2) else '?'
        self.lbl_imu_axis.config(text=axis_name)
        if self.root.focus_get() != self.entry_imu_axis:
            self.entry_imu_axis.delete(0,'end')
            self.entry_imu_axis.insert(0, str(ax))

        wl = d.get('was_left', -45.0)
        wr = d.get('was_right', 45.0)
        sa = d.get('steer_actual', 0.0)
        us = d.get('uturn_strength', 1)
        self.lbl_was_left.config(text=f"{wl:.1f}°")
        self.lbl_was_right.config(text=f"{wr:.1f}°")
        self.lbl_steer_actual.config(text=f"{sa:.1f}°")
        self.lbl_uturn_strength.config(text=str(us))
        if self.root.focus_get() != self.entry_uturn_strength:
            self.entry_uturn_strength.delete(0, 'end')
            self.entry_uturn_strength.insert(0, str(us))

        rz = d.get('roll_zero', 0.0)
        self.lbl_roll_zero.config(
            text=f"{rz:.2f} °",
            fg=GREEN if abs(rz) < 5.0 else YELLOW)
        # Only update entry field if user is not actively editing it
        if self.root.focus_get() != self.entry_roll_zero:
            self.entry_roll_zero.delete(0, 'end')
            self.entry_roll_zero.insert(0, f"{rz:.2f}")

        # Auto Roll Adjust — checkbox synced unconditionally (a
        # momentary click, not an ongoing edit, so no focus-guard
        # needed there unlike the entry fields). Entry fields use the
        # same focus-guard pattern as entry_roll_zero just above, for
        # consistency with this tab's own established approach rather
        # than Keya Auto-Zero's "sync once" pattern on a different tab.
        ara = d.get('auto_roll_adjust', 0)
        self.auto_roll_var.set(bool(ara))
        self.lbl_auto_roll_status.config(
            text="Active — correcting live" if ara else "Off — manual control")

        for key, pdiag_key, fmt in [
            ('deadband', 'roll_auto_deadband', '.3f'),
            ('alpha',    'roll_auto_alpha',    '.4f'),
        ]:
            current_lbl, entry, _ = self._roll_auto_rows[key]
            val = d.get(pdiag_key)
            if val is not None:
                current_lbl.config(text=f"Current: {val:{fmt}}")
                if self.root.focus_get() != entry:
                    entry.delete(0, 'end')
                    entry.insert(0, format(val, fmt))

        ra = d['roll_alpha']
        ir = d.get('init_r', ra)
        self.lbl_roll_alpha.config(
            text=f"{ra:.4f}  (init: {ir:.4f})", fg=colour_alpha(ra))

        # Sync brand radio buttons to firmware value, if present.
        # Only touch the widget if the user isn't mid-click on it, to
        # avoid fighting with their own selection.
        br = d.get('brand')
        if br is not None and br != self.brand_var.get():
            self.brand_var.set(br)

        # Sync GNSS mode category radio button to firmware value, if
        # present. Note: this only syncs the top-level category (1/2/3)
        # — firmware doesn't currently report DualReceiverType (F9P vs
        # UM980) in PDIAG, so that sub-choice stays local-UI-only, same
        # as the dual antenna type sub-choice under category 1. A
        # future PDIAG field could close this gap; not done here.
        gm = d.get('gnss_mode')
        if gm is not None and gm != self.gnss_category_var.get():
            self.gnss_category_var.set(gm)
            self._update_imu_tab_state()

        # Sync IMU type radio button to firmware value, if present.
        it = d.get('imu_type')
        if it is not None and it != self.imu_type_var.get():
            self.imu_type_var.set(it)
            self._update_imu_tab_state()

        # Sync Board Configuration radio buttons to firmware values, if
        # present — e.g. after a fresh connect/reconnect, so the UI
        # reflects what's actually saved rather than the compiled-in
        # defaults until the user happens to touch either radio group.
        bs1 = d.get('board_slot1')
        bs2 = d.get('board_slot2')
        board_changed = False
        if bs1 is not None and bs1 != self.board_slot1_var.get():
            self.board_slot1_var.set(bs1)
            board_changed = True
        if bs2 is not None and bs2 != self.board_slot2_var.get():
            self.board_slot2_var.set(bs2)
            board_changed = True
        if board_changed:
            self._check_board_slots()

        # Sync GNSS Passthrough checkbox to firmware value, if present.
        gp = d.get('gnss_passthrough')
        if gp is not None and bool(gp) != self.gnss_passthrough_var.get():
            self.gnss_passthrough_var.set(bool(gp))

        # Sync Motor Drive Type / WAS Source radio buttons to firmware
        # values, if present.
        mdt = d.get('motor_drive_type')
        if mdt is not None and mdt != self.motor_drive_var.get():
            self.motor_drive_var.set(mdt)
        ws = d.get('was_source')
        if ws is not None and ws != self.was_source_var.get():
            self.was_source_var.set(ws)
        self._check_motor_drive_conflict()

        # Keya fault health check — only meaningful when motor_drive_type
        # == 2, matching the same "only meaningful when relevant" pattern
        # already used for the IMU watchdog checks above.
        if mdt == 2:
            self._log_health_transition(
                'keya_fault', d.get('keya_fault_active') == 1,
                "Keya motor fault active (see motor's own LED blink code for specifics)")

        # Dual Roll vs IMU Roll comparison panel (IMU tab). dual_roll_raw
        # is only meaningful when roll_valid is true (a current dual
        # solution actually exists right now) — shows "--.-°" otherwise
        # rather than a stale frozen number. imu_roll_raw is shown
        # whenever present; it's valid any time the IMU is healthy,
        # independent of whether a dual solution currently exists.
        dr = d.get('dual_roll_raw')
        ir_raw = d.get('imu_roll_raw')
        if dr is not None:
            if d.get('roll_valid'):
                self.lbl_dual_roll_live.config(text=f"{dr:.1f}°", fg=BLUE)
            else:
                self.lbl_dual_roll_live.config(text="--.-°", fg=DIM)
        if ir_raw is not None:
            self.lbl_imu_roll_live.config(text=f"{ir_raw:.1f}°", fg=GREEN)

        # Update age label
        age_s = int(time.time() - self.last_rx)
        self.lbl_age.config(text=f"{age_s}s ago", fg=GREEN if age_s < 8 else YELLOW)

        self.lbl_status.config(
            text=f"Receiving from {self.teensy_ip} | port {LISTEN_PORT} | last msg {age_s}s ago",
            fg=DIM)

    def _update_clock(self):
        # Purely local wall-clock display — reschedules itself via
        # .after(), same self-perpetuating pattern as _poll(). No
        # connection to the Teensy at all, just a convenience for
        # someone glancing at the tool during a long field session.
        self.lbl_time.config(text=time.strftime('%H:%M:%S'))
        self.root.after(1000, self._update_clock)


if __name__ == '__main__':
    root = tk.Tk()
    app = Monitor(root)
    root.mainloop()
