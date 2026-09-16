"""
Desktop application for the pSCT focal-plane actuators.

    python -m psct_motors.cli gui
    python -m psct_motors.cli gui --simulate      # no hardware needed

What is on screen, top to bottom:

  STOP                a controlled stop of all three axes, always reachable
  Connection          connect/disconnect, and which config is loaded
  Focal plane         where the plane is now, and where to send it
  Actuators           per-axis position, mode, brake indicator and jog
  Log                 everything the application did, timestamped

Threading
---------
Tkinter is single-threaded and pymodbus calls block, so every motor operation
runs on a worker thread. Tcl is not thread-safe and even `root.after` is not
officially callable from another thread -- doing so raises "main thread is not
in main loop" at best and corrupts the interpreter at worst. So no worker here
touches tkinter at all. Workers push closures onto `_ui_queue`, and a repeating
`_drain_ui` job running on the UI thread executes them. `post()` is the only
way work crosses back.

Command buttons disable themselves while an operation is in flight, so a
second command cannot be queued behind a move.

STOP is the exception: it launches immediately on its own thread whatever else
is running, because a stop button that waits its turn is not a stop button.
`FocalPlanePlatform.stop()` deliberately takes no move lock for the same
reason.
"""

from __future__ import annotations

import queue
import threading
import time
import traceback
from datetime import datetime
from typing import Callable, List, Optional

import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, ttk

from .config import load_config, save_config, default_config_path
from .focus_gauge import FocusGauge
from .plane_view import FocalPlaneView
from .jvl_motor import BrakeState
from .kinematics import Orientation
from .platform import FocalPlanePlatform, PlatformError, PlatformState

# Indicator colours, shared by the brake lamps and the status pills.
COLOR_OK = "#1b8a3a"
COLOR_WARN = "#c77700"
COLOR_BAD = "#b3231f"
COLOR_IDLE = "#8a8a8a"
COLOR_STOP = "#c62828"
COLOR_STOP_DARK = "#7a0000"

#: Brakes get their own two colours, deliberately not the green/red pair used
#: for health. A released brake is not "good" and an engaged one is not "bad":
#: engaged means the camera is clamped, released means it is hanging on the
#: drives. Showing those as green and red read as "all fine" at exactly the
#: moment the load was least supported, so they are blue (clamped) and amber
#: (free, and depending on something else), and the lamp always has the word
#: beside it.
COLOR_BRAKE_ON = "#1f5fa9"
COLOR_BRAKE_OFF = "#e08a00"


#: What each brake state is called on screen. Plain words, because "engaged"
#: and "released" both sound reassuring and only one of them means the camera
#: is held.
BRAKE_WORDS = {
    BrakeState.ENGAGED: "HOLDING",
    BrakeState.RELEASED: "FREE",
    BrakeState.UNKNOWN: "unknown",
}


class Lamp(tk.Canvas):
    """A small coloured indicator light."""

    def __init__(self, parent, diameter: int = 16, **kw):
        super().__init__(parent, width=diameter + 2, height=diameter + 2,
                         highlightthickness=0, **kw)
        self._circle = self.create_oval(2, 2, diameter, diameter,
                                        fill=COLOR_IDLE, outline="#404040")

    def set(self, color: str) -> None:
        self.itemconfigure(self._circle, fill=color)


def pixels_for(font_spec, *samples: str) -> int:
    """Widest of `samples` in `font_spec`, in pixels, with a little slack.

    Widths given to a label are counted in *characters*, and Tk sizes a
    character as the width of "0" in that font. A four-character string of
    wide glyphs is therefore wider than four "characters" and gets clipped --
    which is why `width=4` showed "Top" and "East" but cut "West" off at the
    T. Reserving space in pixels, measured from the strings that will
    actually appear, is the only way to get this right for a proportional
    font, and it keeps working if an actuator is renamed in the config.
    """
    font = tkfont.Font(font=font_spec)
    return max(font.measure(s) for s in samples) + 8


class LoadBar(tk.Canvas):
    """How hard one motor is working, as a bar plus a number.

    These motors do not report amps. What they report is Actual Torque as a
    fraction of the drive's current limit, so that is what is drawn -- and the
    label says "load", not "current", because calling a torque fraction an
    ammeter reading would be inventing a measurement. When the actuator's
    rated current is configured, an approximate figure in amps is shown beside
    it and marked with a tilde.

    Two marks matter and are drawn on the bar: the amber warning level and the
    red line at which a move is aborted. Seeing where a healthy move sits
    relative to those is the whole point -- it is how you tell whether the
    stall threshold is set sensibly for this machine.
    """

    WIDTH = 86
    HEIGHT = 14

    def __init__(self, parent, warn_percent: float = 30.0,
                 stall_percent: float = 45.0, **kw):
        super().__init__(parent, width=self.WIDTH, height=self.HEIGHT,
                         highlightthickness=1, highlightbackground="#bbb",
                         bg="#f4f4f4", **kw)
        self.warn_percent = warn_percent
        self.stall_percent = stall_percent
        self._percent = None
        self._peak = 0.0
        self._bar = self.create_rectangle(1, 1, 1, self.HEIGHT - 1,
                                          fill=COLOR_OK, width=0)
        self._peak_mark = self.create_line(0, 0, 0, 0, fill="#444", width=1,
                                           state="hidden")
        for percent, colour in ((warn_percent, "#c77700"),
                                (stall_percent, COLOR_BAD)):
            x = self._x(percent)
            self.create_line(x, 0, x, self.HEIGHT, fill=colour, width=1,
                             dash=(2, 2))
        self._text = self.create_text(self.WIDTH // 2, self.HEIGHT // 2,
                                      text="--", font=("TkDefaultFont", 7))

    def _x(self, percent: float) -> float:
        return max(1.0, min(self.WIDTH, self.WIDTH * percent / 100.0))

    def set(self, percent, label: str = "") -> None:
        """`percent` of None means the motor does not report torque."""
        self._percent = percent
        if percent is None:
            self.coords(self._bar, 1, 1, 1, self.HEIGHT - 1)
            self.itemconfigure(self._text, text=label or "n/a")
            self.itemconfigure(self._peak_mark, state="hidden")
            return
        self.coords(self._bar, 1, 1, self._x(percent), self.HEIGHT - 1)
        if percent >= self.stall_percent:
            colour = COLOR_BAD
        elif percent >= self.warn_percent:
            colour = COLOR_WARN
        else:
            colour = COLOR_OK
        self.itemconfigure(self._bar, fill=colour)
        self.itemconfigure(self._text, text=label or f"{percent:.0f}%")
        # A high-water mark, because the peak of a move is what tells you
        # whether the threshold has margin -- and it is gone by the time you
        # look at a settled axis.
        if percent > self._peak:
            self._peak = percent
        x = self._x(self._peak)
        self.coords(self._peak_mark, x, 1, x, self.HEIGHT - 1)
        self.itemconfigure(self._peak_mark, state="normal")

    def reset_peak(self) -> None:
        self._peak = 0.0
        self.itemconfigure(self._peak_mark, state="hidden")


class MotorRow:
    """One actuator's line in the actuator table."""

    #: Everything `mode_var` and `brake_var` show in normal operation. The
    #: columns are sized to hold these without moving; a rare long mode name
    #: ("Zero-search / internal mode 15") widens the column rather than being
    #: silently cut in half.
    MODE_SAMPLES = ("Passive", "Velocity", "Position", "Gear", "no comms", "--")
    BRAKE_SAMPLES = ("HOLDING?", "FREE?", "unknown")
    NAME_FONT = ("TkDefaultFont", 11, "bold")

    def __init__(self, parent, name: str, row: int, app: "MotorApp"):
        self.name = name
        self.app = app

        self.name_label = ttk.Label(parent, text=name, anchor="w",
                                    font=self.NAME_FONT)
        self.name_label.grid(row=row, column=0, padx=(6, 2), sticky="w")

        self.position_var = tk.StringVar(value="--")
        ttk.Label(parent, textvariable=self.position_var, width=14, anchor="e",
                  font=("TkFixedFont", 10)).grid(row=row, column=1, padx=2)

        self.counts_var = tk.StringVar(value="--")
        ttk.Label(parent, textvariable=self.counts_var, width=13, anchor="e",
                  font=("TkFixedFont", 9), foreground="#555").grid(row=row, column=2, padx=2)

        self.mode_lamp = Lamp(parent)
        self.mode_lamp.grid(row=row, column=3, padx=(8, 2))
        self.mode_var = tk.StringVar(value="--")
        ttk.Label(parent, textvariable=self.mode_var,
                  anchor="w").grid(row=row, column=4, padx=2, sticky="w")

        self.brake_lamp = Lamp(parent)
        self.brake_lamp.grid(row=row, column=5, padx=(8, 2))
        self.brake_var = tk.StringVar(value="unknown")
        ttk.Label(parent, textvariable=self.brake_var,
                  anchor="w").grid(row=row, column=6, padx=2, sticky="w")

        actuator = app.cfg.actuator(name)
        self.load_bar = LoadBar(parent,
                                warn_percent=actuator.torque_warn_percent,
                                stall_percent=actuator.stall_torque_percent)
        self.load_bar.grid(row=row, column=7, padx=(8, 2))
        self.load_var = tk.StringVar(value="")
        ttk.Label(parent, textvariable=self.load_var, width=9, anchor="w",
                  font=("TkFixedFont", 8), foreground="#555").grid(
            row=row, column=8, padx=(2, 6), sticky="w")

        self.release_btn = ttk.Button(
            parent, text="Release", width=8,
            command=lambda: app.on_brake(name, engage=False))
        self.release_btn.grid(row=row, column=9, padx=2)
        self.engage_btn = ttk.Button(
            parent, text="Engage", width=8,
            command=lambda: app.on_brake(name, engage=True))
        self.engage_btn.grid(row=row, column=10, padx=2)

        ttk.Button(parent, text="▼", width=3,
                   command=lambda: app.on_jog(name, -1)).grid(row=row, column=11, padx=(10, 1))
        ttk.Button(parent, text="▲", width=3,
                   command=lambda: app.on_jog(name, +1)).grid(row=row, column=12, padx=1)

        self.error_var = tk.StringVar(value="")
        ttk.Label(parent, textvariable=self.error_var, foreground=COLOR_BAD,
                  anchor="w").grid(row=row, column=13, padx=(10, 6), sticky="w")

    def update(self, status) -> None:
        if status.comms_error:
            self.position_var.set("--")
            self.counts_var.set("")
            self.mode_var.set("no comms")
            self.load_bar.set(None, "--")
            self.load_var.set("")
            self.mode_lamp.set(COLOR_BAD)
            self.brake_var.set("unknown")
            self.brake_lamp.set(COLOR_IDLE)
            self.error_var.set(status.comms_error[:60])
            return

        self.position_var.set(f"{status.position_mm:+10.4f} mm")
        self.counts_var.set(f"{status.position_counts} ct")
        self.mode_var.set(status.mode_text.split(" (")[0])
        # Green only when the drive is enabled AND settled; amber while moving.
        if status.mode == 2:
            self.mode_lamp.set(COLOR_OK if status.in_position else COLOR_WARN)
        else:
            self.mode_lamp.set(COLOR_IDLE)

        brake = status.brake
        # The word, not just the colour. Somebody glancing at this window has
        # to be able to tell whether the camera is clamped without first
        # remembering what the colours mean.
        self.brake_var.set(BRAKE_WORDS[brake.state]
                           + ("?" if brake.inferred else ""))
        if brake.state is BrakeState.ENGAGED:
            self.brake_lamp.set(COLOR_BRAKE_ON)
        elif brake.state is BrakeState.RELEASED:
            self.brake_lamp.set(COLOR_BRAKE_OFF)
        else:
            self.brake_lamp.set(COLOR_IDLE)

        self.load_bar.set(status.torque_percent)
        if status.torque_percent is None:
            self.load_var.set("no torque")
        elif status.current_a is not None:
            self.load_var.set(f"~{status.current_a:.2f} A")
        else:
            self.load_var.set(f"{status.torque_percent:.0f}% lim")

        self.error_var.set(status.error_text if status.error_bits else "")

    def set_brake_controls_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        self.release_btn.config(state=state)
        self.engage_btn.config(state=state)


class MotorApp:
    def __init__(self, root: tk.Tk, config_path: Optional[str] = None,
                 simulate: bool = False, bench: Optional[str] = None):
        self.root = root
        self.simulate = simulate
        self.bench = bench
        self.config_path = config_path

        self.cfg = load_config(config_path)
        if bench:
            from .cli import apply_bench
            apply_bench(self.cfg, bench)
        self.platform = FocalPlanePlatform(
            cfg=self.cfg, simulate=simulate, logger=self.log_threadsafe,
            config_path=config_path,
        )
        self.root.title(self._window_title())

        self._busy = False
        self._poll_stop = threading.Event()
        self._poll_thread: Optional[threading.Thread] = None
        #: Closures posted by worker threads, executed on the UI thread.
        self._ui_queue: "queue.Queue[Callable[[], None]]" = queue.Queue()
        self._ui_job: Optional[str] = None

        self._build_ui()
        self._drain_ui()
        self.log(f"Configuration: {config_path or default_config_path()}")
        if simulate:
            self.log("SIMULATION MODE -- no hardware is being touched.")
        elif self.platform.is_mixed:
            self.log("BENCH MODE: "
                     + ", ".join(self.platform.simulated_names)
                     + " are simulated. Only "
                     + ", ".join(m.name for m in self.platform.motors
                                 if m.name not in self.platform.simulated_names)
                     + " is a real motor, and everything the others report is "
                       "made up.")
        for actuator in self.cfg.actuators:
            if not actuator.scale_is_measured:
                self.log(
                    f"NOTE: actuator {actuator.name} is using a scale derived from "
                    "the drivetrain, not a measured one. Run "
                    f"`cli calibrate --motor {actuator.name}` before trusting "
                    "millimetre readings."
                )
        self.log("Press Connect to begin.")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.report_callback_exception = self._on_callback_error

    def _on_callback_error(self, exc_type, exc_value, exc_traceback) -> None:
        """Surface a crash in a button or menu command.

        Tk's default is to print a traceback to stderr and carry on, which on a
        windowed application means the control silently does nothing: the
        operator presses it, no dialog appears, no log line appears, and the
        window looks fine. That is indistinguishable from the command having
        run and found nothing to do, and it is the worst way to discover a bug
        -- particularly in front of an audience.

        So it goes in the log, where everything else the application did
        already is, and in a dialog, because a control that failed is not
        something to notice later.
        """
        detail = "".join(traceback.format_exception(exc_type, exc_value,
                                                    exc_traceback))
        try:
            self.log(f"INTERNAL ERROR in a control: {exc_type.__name__}: "
                     f"{exc_value}")
            for line in detail.strip().splitlines():
                self.log("    " + line)
        except Exception:  # noqa: BLE001 -- logging must not mask the error
            pass
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        try:
            messagebox.showerror(
                "Something went wrong inside the application",
                f"{exc_type.__name__}: {exc_value}\n\n"
                "The motors have not been changed by this. The full traceback "
                "is in the log at the bottom of the window.",
            )
        except Exception:  # noqa: BLE001
            pass

    def _window_title(self) -> str:
        """Say in the title bar what is real and what is not.

        Somebody walking up to this window has to be able to tell at a glance
        whether it is driving a telescope. A screenshot of a simulated run and
        a screenshot of a real one are otherwise identical.
        """
        if self.simulate:
            return "pSCT Focal Plane Control  [SIMULATION -- no hardware]"
        if self.platform.is_mixed:
            real = ", ".join(m.name for m in self.platform.motors
                             if m.name not in self.platform.simulated_names)
            return (f"pSCT Focal Plane Control  [BENCH -- only {real} is real]")
        return "pSCT Focal Plane Control"

    # ------------------------------------------------------------------ UI

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        self._build_menu()
        self._build_stop_bar()
        self._build_connection()

        # The working area: focus on the left, the gauge on the right.
        body = ttk.Frame(self.root)
        body.grid(row=2, column=0, sticky="nsew", padx=6, pady=3)
        body.columnconfigure(0, weight=1)
        body.rowconfigure(2, weight=1)

        self._build_focal_plane(body)
        self._build_actuators(body)
        self._build_gauge(body)
        self._build_log()

    def _build_menu(self) -> None:
        """Tip and tilt live here rather than on the main panel.

        Day to day this mechanism is a focus drive: the site's own procedure
        motorises only the optical axis, and the two tilts exist to correct
        the focal plane's orientation, not to be set routinely. Keeping them
        one menu item away means the main window says what the job is, while
        the capability is still there for whoever needs it.
        """
        menubar = tk.Menu(self.root)

        motion = tk.Menu(menubar, tearoff=0)
        motion.add_command(label="Tip and tilt...", command=self.on_open_tilt)
        motion.add_separator()
        motion.add_command(label="Set zero here", command=self.on_set_zero)
        motion.add_command(label="Copy current orientation into the boxes",
                           command=self.on_copy_current)
        menubar.add_cascade(label="Motion", menu=motion)

        view = tk.Menu(menubar, tearoff=0)
        view.add_command(label="Focal plane picture...",
                         command=self.on_open_plane_view)
        menubar.add_cascade(label="View", menu=view)

        tools = tk.Menu(menubar, tearoff=0)
        tools.add_command(label="Connection settings...",
                          command=self.on_edit_connection)
        tools.add_command(label="Motion limits...",
                          command=self.on_edit_limits)
        tools.add_command(label="Find hard stop (calibration)...",
                          command=self.on_find_hard_stop)
        tools.add_command(label="Run safety drills (simulated)...",
                          command=self.on_safety_drills)
        tools.add_separator()
        tools.add_command(label="Clear errors", command=self.on_clear_errors)
        tools.add_command(label="Release all brakes",
                          command=lambda: self.on_brake(None, engage=False))
        tools.add_command(label="Engage all brakes",
                          command=lambda: self.on_brake(None, engage=True))
        menubar.add_cascade(label="Tools", menu=tools)

        self.root.config(menu=menubar)
        self.menubar = menubar

    def _build_stop_bar(self) -> None:
        bar = tk.Frame(self.root, bg=COLOR_STOP_DARK)
        bar.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 2))
        bar.columnconfigure(0, weight=3)
        bar.columnconfigure(1, weight=1)

        tk.Button(
            bar, text="STOP", command=self.on_stop,
            bg=COLOR_STOP, fg="white", activebackground=COLOR_STOP_DARK,
            activeforeground="white", font=("TkDefaultFont", 16, "bold"), height=2,
        ).grid(row=0, column=0, sticky="ew", padx=4, pady=4)

        tk.Button(
            bar, text="EMERGENCY\nhalt + brakes on", command=self.on_passivate,
            bg="#3a3a3a", fg="white", activebackground="#111", activeforeground="white",
            font=("TkDefaultFont", 9, "bold"), height=2,
        ).grid(row=0, column=1, sticky="ew", padx=4, pady=4)

        tk.Label(
            bar,
            text="STOP decelerates and holds position with the drives still on. "
                 "EMERGENCY does that too, then engages the brakes, and turns the "
                 "drives off ONLY if the brakes are confirmed holding -- otherwise "
                 "they stay on, because they are the only thing holding the camera. "
                 "Neither asks for confirmation.",
            bg=COLOR_STOP_DARK, fg="#ffd7d7", font=("TkDefaultFont", 8),
        ).grid(row=1, column=0, columnspan=2, sticky="w", padx=8, pady=(0, 2))

        # What the last STOP or EMERGENCY actually did. These controls act
        # without asking, so the result has to be visible without going to
        # look for it in the log.
        self.action_var = tk.StringVar(value="")
        self.action_label = tk.Label(
            bar, textvariable=self.action_var, bg=COLOR_STOP_DARK, fg="white",
            font=("TkDefaultFont", 9, "bold"), anchor="w", justify="left",
            wraplength=900,
        )
        self.action_label.grid(row=2, column=0, columnspan=2, sticky="ew",
                               padx=8, pady=(0, 4))

    def _build_connection(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Connection")
        frame.grid(row=1, column=0, sticky="ew", padx=6, pady=3)

        self.connect_btn = ttk.Button(frame, text="Connect", command=self.on_connect)
        self.connect_btn.grid(row=0, column=0, padx=6, pady=6)

        self.conn_lamp = Lamp(frame)
        self.conn_lamp.grid(row=0, column=1, padx=(6, 2))
        self.conn_var = tk.StringVar(value="disconnected")
        ttk.Label(frame, textvariable=self.conn_var, width=28).grid(row=0, column=2, padx=2)

        ttk.Button(frame, text="Clear errors",
                   command=self.on_clear_errors).grid(row=0, column=3, padx=6)
        ttk.Button(frame, text="Set zero here",
                   command=self.on_set_zero).grid(row=0, column=4, padx=6)

        self.addresses_var = tk.StringVar()
        ttk.Label(frame, textvariable=self.addresses_var,
                  foreground="#555").grid(row=0, column=5, padx=10, sticky="w")
        ttk.Button(frame, text="Edit...",
                   command=self.on_edit_connection).grid(row=0, column=6, padx=4)
        self._refresh_addresses()

    def _build_legend(self, parent, row: int) -> None:
        """A key to every lamp and colour in the table.

        Without this the indicators are a guessing game: a lamp is only
        self-explanatory to whoever wrote it. The brake colours in particular
        are not the usual green/red pair and would be actively misread --
        "released" is the state where the camera hangs on the drives.
        """
        legend = ttk.Frame(parent)
        legend.grid(row=row, column=0, columnspan=14, sticky="w",
                    padx=8, pady=(8, 0))

        ttk.Label(legend, text="key:", foreground="#555",
                  font=("TkDefaultFont", 8, "bold")).grid(row=0, column=0,
                                                          padx=(0, 6))

        column = 1
        for colour, text in (
            (COLOR_OK, "drive enabled, settled"),
            (COLOR_WARN, "drive enabled, moving"),
            (COLOR_IDLE, "drive passive / unknown"),
            (COLOR_BAD, "fault or no comms"),
        ):
            lamp = Lamp(legend, diameter=11)
            lamp.set(colour)
            lamp.grid(row=0, column=column, padx=(6, 2))
            ttk.Label(legend, text=text, foreground="#555",
                      font=("TkDefaultFont", 8)).grid(row=0, column=column + 1)
            column += 2

        brake_key = ttk.Frame(parent)
        brake_key.grid(row=row + 1, column=0, columnspan=14, sticky="w",
                       padx=8, pady=(2, 0))
        ttk.Label(brake_key, text="brake:", foreground="#555",
                  font=("TkDefaultFont", 8, "bold")).grid(row=0, column=0,
                                                           padx=(0, 6))
        column = 1
        for colour, text in (
            (COLOR_BRAKE_ON, "HOLDING -- clamped, the camera cannot move"),
            (COLOR_BRAKE_OFF, "FREE -- released, the drives are holding it"),
            (COLOR_IDLE, "unknown -- not under software control"),
        ):
            lamp = Lamp(brake_key, diameter=11)
            lamp.set(colour)
            lamp.grid(row=0, column=column, padx=(6, 2))
            ttk.Label(brake_key, text=text, foreground="#555",
                      font=("TkDefaultFont", 8)).grid(row=0, column=column + 1)
            column += 2
        ttk.Label(brake_key,
                  text="   '?' means inferred from the drive mode, not read back",
                  foreground="#777",
                  font=("TkDefaultFont", 8)).grid(row=0, column=column)

        load_key = ttk.Frame(parent)
        load_key.grid(row=row + 2, column=0, columnspan=14, sticky="w",
                      padx=8, pady=(2, 0))
        ttk.Label(load_key, text="load:", foreground="#555",
                  font=("TkDefaultFont", 8, "bold")).grid(row=0, column=0,
                                                           padx=(0, 6))
        ttk.Label(
            load_key,
            text=("torque as % of the drive's current limit (these motors "
                  "report no amps). Dashed lines mark the warning and stall "
                  "levels; the dark tick is the peak since the window opened."),
            foreground="#555", font=("TkDefaultFont", 8), wraplength=760,
            justify="left",
        ).grid(row=0, column=1, sticky="w")

    def _refresh_addresses(self) -> None:
        self.addresses_var.set(
            "   ".join(f"{a.name} {a.ip}:{a.port}" for a in self.cfg.actuators))

    def _build_focal_plane(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="Focus  (along the optical axis)")
        frame.grid(row=0, column=0, sticky="ew", pady=(0, 3))
        frame.columnconfigure(9, weight=1)

        # --- live readout ---
        readout = ttk.Frame(frame)
        readout.grid(row=0, column=0, columnspan=10, sticky="ew", padx=6, pady=(6, 2))

        self.focus_readout_var = tk.StringVar(value="not connected")
        ttk.Label(readout, textvariable=self.focus_readout_var,
                  font=("TkFixedFont", 15, "bold")).grid(row=0, column=0, sticky="w")

        # Kept under its original name: other code and the tests read it, and
        # it is still the full orientation in one line.
        self.orientation_var = tk.StringVar(value="not connected")
        ttk.Label(readout, textvariable=self.orientation_var,
                  foreground="#555",
                  font=("TkFixedFont", 9)).grid(row=1, column=0, sticky="w")
        self.orientation_detail_var = tk.StringVar(value="")
        ttk.Label(readout, textvariable=self.orientation_detail_var,
                  foreground="#777").grid(row=2, column=0, sticky="w")

        ttk.Separator(frame, orient="horizontal").grid(
            row=1, column=0, columnspan=10, sticky="ew", padx=6, pady=6)

        # --- absolute focus command ---
        ttk.Label(frame, text="Go to focus (mm):").grid(row=2, column=0, padx=(8, 2),
                                                        sticky="e")
        self.focus_var = tk.StringVar(value="0.0")
        ttk.Entry(frame, textvariable=self.focus_var, width=12,
                  font=("TkDefaultFont", 11)).grid(row=2, column=1, padx=2, pady=4)
        ttk.Button(frame, text="Preview",
                   command=self.on_preview).grid(row=2, column=2, padx=(6, 2))
        self.move_btn = ttk.Button(frame, text="Move", command=self.on_move)
        self.move_btn.grid(row=2, column=3, padx=2)
        ttk.Label(frame, text="+ towards M1,  − towards M2",
                  foreground="#777").grid(row=2, column=4, padx=(12, 4), sticky="w")

        # --- relative nudges ---
        nudge = ttk.Frame(frame)
        nudge.grid(row=3, column=0, columnspan=10, sticky="w", padx=8, pady=(6, 10))
        ttk.Label(nudge, text="Nudge focus by (mm):").grid(row=0, column=0, padx=(0, 6))
        self.focus_step_var = tk.StringVar(value="0.010")
        ttk.Entry(nudge, textvariable=self.focus_step_var,
                  width=10).grid(row=0, column=1, padx=2)
        tk.Button(nudge, text="−", width=4, font=("TkDefaultFont", 12, "bold"),
                  command=lambda: self.on_nudge("focus", -1)).grid(row=0, column=2, padx=3)
        tk.Button(nudge, text="+", width=4, font=("TkDefaultFont", 12, "bold"),
                  command=lambda: self.on_nudge("focus", +1)).grid(row=0, column=3, padx=3)

        for label, step in (("1 um", 0.001), ("10 um", 0.010),
                            ("50 um", 0.050), ("0.5 mm", 0.500)):
            ttk.Button(nudge, text=label, width=7,
                       command=lambda v=step: self.focus_step_var.set(f"{v:.3f}")
                       ).grid(row=0, column=4 + list(
                           ("1 um", "10 um", "50 um", "0.5 mm")).index(label),
                              padx=2)

        # --- tip/tilt entries exist here but are shown in the dialog --------
        # They live on the app so the dialog, the tests and the move code can
        # all reach them whether or not the dialog is currently open.
        self.tip_var = tk.StringVar(value="0.0")
        self.tilt_var = tk.StringVar(value="0.0")
        self.angle_step_var = tk.StringVar(value="0.010")
        self._tilt_window = None
        self._plane_window = None
        self._hard_stop_window = None
        self._limits_window = None
        self.plane_view = None

    def _build_actuators(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="Actuators")
        self.actuator_frame = frame
        frame.grid(row=1, column=0, sticky="nsew", pady=3)

        headers = ["", "position", "counts", "", "mode", "", "brake",
                   "load", "", "", "", "jog", "", ""]
        for col, text in enumerate(headers):
            if text:
                ttk.Label(frame, text=text, foreground="#555",
                          font=("TkDefaultFont", 8)).grid(row=0, column=col, padx=2)

        self.rows = {}
        for i, actuator in enumerate(self.cfg.actuators):
            self.rows[actuator.name] = MotorRow(frame, actuator.name, i + 1, self)

        # Reserve room for the widest text each of the proportional-font
        # columns will hold, so nothing is clipped and nothing shifts sideways
        # as a value changes.
        frame.columnconfigure(0, minsize=pixels_for(
            MotorRow.NAME_FONT, *(a.name for a in self.cfg.actuators)))
        frame.columnconfigure(4, minsize=pixels_for(
            "TkDefaultFont", *MotorRow.MODE_SAMPLES))
        frame.columnconfigure(6, minsize=pixels_for(
            "TkDefaultFont", *MotorRow.BRAKE_SAMPLES))

        self._build_legend(frame, row=len(self.cfg.actuators) + 1)

        footer = ttk.Frame(frame)
        footer.grid(row=len(self.cfg.actuators) + 2, column=0, columnspan=14,
                    sticky="w", padx=6, pady=(6, 6))
        ttk.Label(footer, text="jog step (mm)").grid(row=0, column=0, padx=(0, 4))
        self.jog_step_var = tk.StringVar(value="0.050")
        ttk.Entry(footer, textvariable=self.jog_step_var, width=8).grid(row=0, column=1)
        ttk.Label(
            footer,
            text="  Jogging moves ONE actuator and tilts the plane. Use the focal "
                 "plane controls above for normal operation.",
            foreground="#777",
        ).grid(row=0, column=2, padx=6)

        ttk.Button(footer, text="Release all brakes",
                   command=lambda: self.on_brake(None, engage=False)).grid(row=0, column=3, padx=(20, 4))
        ttk.Button(footer, text="Engage all brakes",
                   command=lambda: self.on_brake(None, engage=True)).grid(row=0, column=4, padx=4)

    def _build_gauge(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="Distance from zero")
        frame.grid(row=0, column=1, rowspan=3, sticky="ns", padx=(6, 0))
        frame.rowconfigure(0, weight=1)
        self.gauge = FocusGauge(frame,
                                min_mm=self.cfg.limits.min_focus_mm,
                                max_mm=self.cfg.limits.max_focus_mm)
        self.gauge.grid(row=0, column=0, sticky="ns", padx=8, pady=8)
        self._refresh_gauge_limits()

    def _refresh_gauge_limits(self) -> None:
        """Push the current limits and any found hard stops onto the gauge."""
        limits = self.cfg.limits
        self.gauge.set_limits(limits.min_focus_mm, limits.max_focus_mm,
                              hard_stop_low_mm=limits.hard_stop_low_mm,
                              hard_stop_high_mm=limits.hard_stop_high_mm)

    def _build_log(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Log")
        frame.grid(row=3, column=0, sticky="nsew", padx=6, pady=(3, 6))
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self.log_text = tk.Text(frame, height=9, wrap="word", state="disabled",
                                font=("TkFixedFont", 9))
        self.log_text.grid(row=0, column=0, sticky="nsew", padx=(6, 0), pady=6)
        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.log_text.yview)
        scroll.grid(row=0, column=1, sticky="ns", pady=6, padx=(0, 6))
        self.log_text.configure(yscrollcommand=scroll.set)

        ttk.Button(frame, text="Clear log", command=self.on_clear_log).grid(
            row=1, column=0, columnspan=2, sticky="e", padx=6, pady=(0, 6))

    # --------------------------------------------------------------- logging

    def log(self, msg: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{stamp}] {msg}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def log_threadsafe(self, msg: str) -> None:
        """Safe to call from a worker thread."""
        self.post(lambda: self.log(msg))

    def post(self, fn: Callable[[], None]) -> None:
        """Schedule `fn` to run on the UI thread. The only safe way in."""
        self._ui_queue.put(fn)

    def _drain_ui(self) -> None:
        """Run queued UI work. Always called on the UI thread."""
        for _ in range(200):        # bounded, so a flood cannot freeze the UI
            try:
                job = self._ui_queue.get_nowait()
            except queue.Empty:
                break
            try:
                job()
            except Exception as exc:  # noqa: BLE001 - one bad update must not
                try:                  # stop the drain loop forever
                    self.log(f"UI update failed: {exc}")
                except Exception:
                    pass
        self._ui_job = self.root.after(60, self._drain_ui)

    def on_clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    # ------------------------------------------------------- worker plumbing

    def _set_busy(self, busy: bool) -> None:
        self._busy = busy
        state = "disabled" if busy else "normal"
        self.move_btn.config(state=state)
        for row in self.rows.values():
            row.set_brake_controls_enabled(not busy)

    def run_async(self, description: str, fn: Callable[[], None],
                  on_done: Optional[Callable[[], None]] = None) -> None:
        """Run a motor operation off the UI thread, one at a time."""
        if self._busy:
            self.log(f"Busy -- '{description}' ignored. Wait for the current "
                     "operation, or press STOP.")
            return
        self._set_busy(True)

        def worker():
            try:
                fn()
            except Exception as exc:  # noqa: BLE001 - reported, never swallowed
                self.log_threadsafe(f"{description} failed: {exc}")
                if not isinstance(exc, PlatformError):
                    # An unexpected type means a bug rather than a refused
                    # command, so keep the last traceback line for diagnosis.
                    self.log_threadsafe(traceback.format_exc().strip().splitlines()[-1])
            finally:
                self.post(lambda: self._set_busy(False))
                if on_done:
                    self.post(on_done)

        threading.Thread(target=worker, name=description, daemon=True).start()

    # ------------------------------------------------------------ connection

    def on_connect(self) -> None:
        if self.platform.connected:
            self._poll_stop.set()
            self.platform.disconnect()
            self.connect_btn.config(text="Connect")
            self.conn_var.set("disconnected")
            self.conn_lamp.set(COLOR_IDLE)
            self.log("Disconnected.")
            return

        def work():
            self.platform.connect()
            self.log_threadsafe("Connected to all three actuators.")
            self.post(self._after_connect)

        self.run_async("Connect", work)

    def _after_connect(self) -> None:
        self.connect_btn.config(text="Disconnect")
        self.conn_var.set("connected" + (" (simulated)" if self.simulate else ""))
        self.conn_lamp.set(COLOR_OK)
        self._start_polling()

    def _start_polling(self) -> None:
        self._poll_stop.clear()
        interval = self.cfg.poll_interval_s

        def poll():
            while not self._poll_stop.is_set():
                try:
                    state = self.platform.read_state()
                    self.post(lambda s=state: self._apply_state(s))
                except Exception as exc:  # noqa: BLE001 - a poll must never die
                    self.log_threadsafe(f"Status poll error: {exc}")
                self._poll_stop.wait(interval)

        self._poll_thread = threading.Thread(target=poll, name="poll", daemon=True)
        self._poll_thread.start()

    def _apply_state(self, state: PlatformState) -> None:
        for status in state.motors:
            row = self.rows.get(status.name)
            if row:
                row.update(status)

        if state.orientation_valid and state.orientation:
            o = state.orientation
            towards = ("towards M1" if o.focus_mm > 0
                       else "towards M2" if o.focus_mm < 0 else "at zero")
            self.focus_readout_var.set(
                f"focus  {o.focus_mm:+9.4f} mm   ({o.focus_mm * 1000:+.0f} um, {towards})"
                + ("   [MOVING]" if state.moving else "")
            )
            self.orientation_var.set(
                f"tip {o.tip_deg:+8.5f} deg   tilt {o.tilt_deg:+8.5f} deg"
            )
            self.orientation_detail_var.set(
                f"total tilt {o.total_tilt_arcmin:.3f} arcmin "
                f"({o.total_tilt_arcsec:.1f} arcsec) towards azimuth "
                f"{o.tilt_azimuth_deg:.1f} deg"
            )
            target = None
            if state.all_connected:
                try:
                    target = self.platform.geometry.orientation_from_actuators(
                        [m.target_mm for m in state.motors]).focus_mm
                except Exception:      # a UI hint, never worth an exception
                    target = None
            self.gauge.update_position(o.focus_mm, target, valid=True)
            if self.plane_view is not None:
                self.plane_view.update_plane(
                    [m.position_mm for m in state.motors],
                    focus_mm=o.focus_mm, tip_deg=o.tip_deg, tilt_deg=o.tilt_deg)
        else:
            self.focus_readout_var.set("focus unavailable")
            self.orientation_var.set("orientation unavailable")
            self.orientation_detail_var.set(state.message)
            self.gauge.update_position(None, None, valid=False)
            if self.plane_view is not None:
                self.plane_view.update_plane(None, message=state.message
                                             or "no reading")

        if state.any_error:
            self.conn_lamp.set(COLOR_BAD)
        elif state.all_connected:
            self.conn_lamp.set(COLOR_WARN if state.moving else COLOR_OK)

    # ----------------------------------------------------------------- stop

    def _announce(self, message: str, colour: str = "white") -> None:
        """Say what a safety control just did, on the red bar. UI thread."""
        self.action_var.set(message)
        self.action_label.configure(bg=colour, fg="white")

    def _announce_threadsafe(self, message: str, colour: str = "white") -> None:
        self.post(lambda: self._announce(message, colour))

    def on_stop(self) -> None:
        """Always runs, busy or not, on its own thread.

        Guarded on `any_connected`, not `connected`: with one motor off the
        network the other two can still be running, and they are the ones that
        need stopping."""
        if not self.platform.any_connected:
            self.log("STOP pressed, but no motor is reachable.")
            self._announce("STOP pressed, but no motor is reachable.", COLOR_BAD)
            return
        self.log("STOP pressed.")
        self._announce("STOP: decelerating all three axes...", COLOR_WARN)
        threading.Thread(target=self._stop_worker, name="stop", daemon=True).start()

    def _stop_worker(self) -> None:
        stamp = time.strftime("%H:%M:%S")
        try:
            problems = self.platform.stop()
        except Exception as exc:  # noqa: BLE001
            self.log_threadsafe(f"STOP had trouble: {exc}")
            self._announce_threadsafe(f"{stamp}  STOP did not complete: {exc}",
                                      COLOR_BAD)
            self.post(lambda: self._set_busy(False))
            return

        for line in self._stop_report(stamp, problems):
            self.log_threadsafe(line)
        if problems:
            self._announce_threadsafe(
                f"{stamp}  STOP incomplete -- {len(problems)} motor(s) did not "
                f"answer and may still be moving. See the log.", COLOR_BAD)
        else:
            self._announce_threadsafe(
                f"{stamp}  STOP done: motion halted, drives still on and "
                f"holding position.", COLOR_STOP)
        self.post(lambda: self._set_busy(False))

    def _stop_report(self, stamp: str, problems: List[str]) -> List[str]:
        """Where each axis actually came to rest."""
        lines = [f"STOP at {stamp}: motion halted, drives still holding."]
        for problem in problems:
            lines.append(f"  ** could NOT stop {problem}")
        try:
            state = self.platform.read_state()
        except Exception as exc:  # noqa: BLE001
            lines.append(f"  (could not read back the result: {exc})")
            return lines
        for motor in state.motors:
            if motor.comms_error:
                lines.append(f"  {motor.name:<5} no comms -- state unknown: "
                             f"{motor.comms_error}")
            else:
                lines.append(f"  {motor.name:<5} holding at "
                             f"{motor.position_mm:+8.4f} mm  {motor.mode_text}")
        return lines

    def on_passivate(self) -> None:
        """No confirmation. An emergency control that stops to ask a question
        is not an emergency control -- it acts, then reports what it did.

        What it does is *hold*, not cut power. See
        `FocalPlanePlatform.emergency_stop`: the drives are the only thing
        holding this camera unless the brakes are confirmed on, so they are
        the last thing to be turned off and only once something else has
        taken over."""
        if not self.platform.any_connected:
            self.log("EMERGENCY pressed, but no motor is reachable.")
            self._announce("EMERGENCY pressed, but no motor is reachable -- "
                           "nothing could be stopped.", COLOR_BAD)
            return
        self.log("EMERGENCY pressed: halting all three and engaging the brakes.")
        self._announce("EMERGENCY: halting all three, engaging brakes...",
                       COLOR_WARN)
        threading.Thread(target=self._passivate_worker, name="passivate",
                         daemon=True).start()

    def _passivate_worker(self) -> None:
        stamp = time.strftime("%H:%M:%S")
        try:
            result = self.platform.emergency_stop()
        except Exception as exc:  # noqa: BLE001
            self.log_threadsafe(f"EMERGENCY had trouble: {exc}")
            self._announce_threadsafe(
                f"{stamp}  EMERGENCY did not complete: {exc}. Check the focal "
                f"plane physically.", COLOR_BAD)
            self.post(lambda: self._set_busy(False))
            return

        # Report what is true now, read back from the motors, rather than what
        # was commanded. On an emergency control the difference matters.
        self.log_threadsafe(f"EMERGENCY at {stamp}:")
        for line in result.summary().splitlines():
            self.log_threadsafe(line)

        if not result.stopped:
            self._announce_threadsafe(
                f"{stamp}  EMERGENCY incomplete -- "
                f"{len(result.stop_problems)} motor(s) did not answer and may "
                f"still be moving. See the log.", COLOR_BAD)
        elif result.drives_off:
            self._announce_threadsafe(
                f"{stamp}  EMERGENCY done: stopped, brakes engaged, drives off. "
                f"The brakes are holding the focal plane.", COLOR_BAD)
        else:
            self._announce_threadsafe(
                f"{stamp}  EMERGENCY done: stopped and HOLDING. The drives are "
                f"still on, on purpose -- the brakes are not confirmed, and "
                f"cutting power would leave the focal plane held by nothing.",
                COLOR_STOP)
        self.post(lambda: self._set_busy(False))

    # ----------------------------------------------------------------- moves

    def _read_orientation_fields(self) -> Optional[Orientation]:
        try:
            return Orientation(
                focus_mm=float(self.focus_var.get()),
                tip_deg=float(self.tip_var.get()),
                tilt_deg=float(self.tilt_var.get()),
            )
        except ValueError as exc:
            messagebox.showerror("Check the numbers",
                                 f"focus, tip and tilt must all be numbers.\n\n{exc}")
            return None

    def _read_float(self, var: tk.StringVar, label: str) -> Optional[float]:
        try:
            return float(var.get())
        except ValueError:
            messagebox.showerror("Check the numbers", f"{label} must be a number.")
            return None

    def on_copy_current(self) -> None:
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return

        def work():
            o = self.platform.read_orientation()
            def apply():
                self.focus_var.set(f"{o.focus_mm:.4f}")
                self.tip_var.set(f"{o.tip_deg:.5f}")
                self.tilt_var.set(f"{o.tilt_deg:.5f}")
            self.post(apply)
            self.log_threadsafe("Copied the current orientation into the entry boxes.")

        self.run_async("Copy current", work)

    def on_preview(self) -> None:
        target = self._read_orientation_fields()
        if target is None:
            return
        lines = [f"Target: {target.describe()}", "", "Actuator targets:"]
        for name, mm in self.platform.preview(target).items():
            actuator = self.cfg.actuator(name)
            inside = actuator.min_travel_mm <= mm <= actuator.max_travel_mm
            lines.append(f"   {name}: {mm:10.4f} mm" + ("" if inside else "   OUT OF RANGE"))
        try:
            self.platform.check_orientation(target)
            lines.append("")
            lines.append("Within all configured limits.")
        except PlatformError as exc:
            lines.append("")
            lines.append(str(exc))
        self.log("\n".join(lines))
        messagebox.showinfo("Move preview", "\n".join(lines))

    def on_move(self) -> None:
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        target = self._read_orientation_fields()
        if target is None:
            return
        try:
            self.platform.check_orientation(target)
        except PlatformError as exc:
            messagebox.showerror("Move refused", str(exc))
            return

        preview = self.platform.preview(target)
        detail = "\n".join(f"   {n}: {mm:10.4f} mm" for n, mm in preview.items())
        if not messagebox.askyesno(
            "Confirm move",
            f"Move the focal plane to:\n\n{target.describe()}\n\n"
            f"Actuator targets:\n{detail}\n\nProceed?",
        ):
            return

        def work():
            self.platform.move_to_orientation(target)
            self.log_threadsafe("Move complete.")

        self.run_async("Move", work)

    def on_nudge(self, axis: str, sign: int) -> None:
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        if axis == "focus":
            step = self._read_float(self.focus_step_var, "Focus step")
            if step is None:
                return
            kwargs = {"d_focus_mm": sign * step}
        else:
            step = self._read_float(self.angle_step_var, "Angle step")
            if step is None:
                return
            kwargs = {"d_tip_deg": sign * step} if axis == "tip" else {"d_tilt_deg": sign * step}

        def work():
            self.platform.move_relative(**kwargs)
            self.log_threadsafe(f"Nudged {axis} by {sign * step:+g}.")

        self.run_async(f"Nudge {axis}", work)

    def on_jog(self, name: str, sign: int) -> None:
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        step = self._read_float(self.jog_step_var, "Jog step")
        if step is None:
            return

        def work():
            self.platform.move_actuator_mm(name, sign * step, relative=True)
            self.log_threadsafe(f"{name}: jogged {sign * step:+g} mm.")

        self.run_async(f"Jog {name}", work)

    # ---------------------------------------------------------------- brakes

    def on_brake(self, name: Optional[str], engage: bool) -> None:
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        who = name or "all three actuators"
        if not engage and not messagebox.askyesno(
            "Release the brake?",
            f"Release the brake on {who}?\n\nThe drives must be enabled and "
            "holding position first, or the load will be free to move. The "
            "software will refuse if a drive is passive.",
        ):
            return

        def work():
            if name is None:
                results = self.platform.set_all_brakes(engaged=engage)
                for motor_name, result in results.items():
                    self.log_threadsafe(
                        f"{motor_name}: brake "
                        f"{'engage' if engage else 'release'} -> {result}"
                    )
            else:
                motor = self.platform.motor(name)
                if engage:
                    motor.engage_brake()
                else:
                    motor.release_brake()
                self.log_threadsafe(
                    f"{name}: brake {'engaged' if engage else 'released'}."
                )

        self.run_async(f"Brake {who}", work)

    # ---------------------------------------------------- focal plane picture

    def on_open_plane_view(self) -> None:
        """A live picture of the plate on its three actuators.

        Separate from the main window on purpose: it is for watching, and it
        wants room. It updates from the same poll as everything else, so it
        costs no extra traffic to the motors.
        """
        if self._plane_window is not None and self._plane_window.winfo_exists():
            self._plane_window.lift()
            return

        window = tk.Toplevel(self.root)
        self._plane_window = window
        window.title("Focal plane" + ("  [SIMULATION]" if self.simulate else ""))
        window.geometry("520x460")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)

        self.plane_view = FocalPlaneView(
            window,
            points_xy_mm=[a.position_xy_mm for a in self.cfg.actuators],
            names=[a.name for a in self.cfg.actuators],
            focus_span_mm=max(abs(self.cfg.limits.min_focus_mm),
                              abs(self.cfg.limits.max_focus_mm)),
        )
        self.plane_view.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

        tk.Label(
            window, justify="left", anchor="w", fg="#666", wraplength=480,
            text=("Dashed triangle: the zero plane. Solid: where the focal "
                  "plane is now. The orange posts are each actuator's "
                  "extension.\n\nVertical travel is exaggerated by the factor "
                  "shown -- the plate is about a metre across and moves "
                  "millimetres, so a true-scale drawing would be a flat line."),
        ).grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 6))

        ttk.Button(window, text="Close", command=window.destroy).grid(
            row=2, column=0, pady=(0, 10))

        def forget(_event=None):
            self.plane_view = None
            self._plane_window = None
        window.bind("<Destroy>", forget)

    # ------------------------------------------------------- tip/tilt dialog

    def on_open_tilt(self) -> None:
        """The tip/tilt controls, deliberately behind a menu."""
        if self._tilt_window is not None and self._tilt_window.winfo_exists():
            self._tilt_window.lift()
            return

        window = tk.Toplevel(self.root)
        self._tilt_window = window
        window.title("Tip and tilt")
        window.transient(self.root)

        tk.Label(
            window, justify="left", anchor="w", wraplength=520, fg="#555",
            text=("Tip and tilt change the ORIENTATION of the focal plane, not "
                  "its focus. Day to day this mechanism is a focus drive, which "
                  "is why these are kept out of the main window.\n\n"
                  "tip  = rotation about the east-west axis\n"
                  "tilt = rotation about the vertical axis\n\n"
                  "A tilt costs actuator travel in proportion to the actuator "
                  "radius, so check Preview before committing to one."),
        ).grid(row=0, column=0, columnspan=8, sticky="w", padx=12, pady=(12, 8))

        ttk.Separator(window, orient="horizontal").grid(
            row=1, column=0, columnspan=8, sticky="ew", padx=12, pady=4)

        ttk.Label(window, text="tip (deg)").grid(row=2, column=0, sticky="e",
                                                 padx=(12, 2), pady=6)
        ttk.Entry(window, textvariable=self.tip_var,
                  width=12).grid(row=2, column=1, padx=2)
        ttk.Label(window, text="tilt (deg)").grid(row=2, column=2, sticky="e", padx=(12, 2))
        ttk.Entry(window, textvariable=self.tilt_var,
                  width=12).grid(row=2, column=3, padx=2)
        ttk.Button(window, text="Preview",
                   command=self.on_preview).grid(row=2, column=4, padx=(12, 2))
        ttk.Button(window, text="Move",
                   command=self.on_move).grid(row=2, column=5, padx=2)

        nudge = ttk.Frame(window)
        nudge.grid(row=3, column=0, columnspan=8, sticky="w", padx=12, pady=(6, 4))
        ttk.Label(nudge, text="Nudge by (deg):").grid(row=0, column=0, padx=(0, 6))
        ttk.Entry(nudge, textvariable=self.angle_step_var,
                  width=10).grid(row=0, column=1, padx=2)
        ttk.Label(nudge, text="tip").grid(row=0, column=2, padx=(12, 2))
        ttk.Button(nudge, text="−", width=3,
                   command=lambda: self.on_nudge("tip", -1)).grid(row=0, column=3, padx=1)
        ttk.Button(nudge, text="+", width=3,
                   command=lambda: self.on_nudge("tip", +1)).grid(row=0, column=4, padx=1)
        ttk.Label(nudge, text="tilt").grid(row=0, column=5, padx=(12, 2))
        ttk.Button(nudge, text="−", width=3,
                   command=lambda: self.on_nudge("tilt", -1)).grid(row=0, column=6, padx=1)
        ttk.Button(nudge, text="+", width=3,
                   command=lambda: self.on_nudge("tilt", +1)).grid(row=0, column=7, padx=1)

        ttk.Button(window, text="Level (tip = tilt = 0)",
                   command=self.on_level).grid(row=4, column=0, columnspan=2,
                                               sticky="w", padx=12, pady=(8, 12))
        ttk.Button(window, text="Close", command=window.destroy).grid(
            row=4, column=5, sticky="e", padx=12, pady=(8, 12))

    def on_level(self) -> None:
        """Zero both tilts, keeping the current focus."""
        self.tip_var.set("0.0")
        self.tilt_var.set("0.0")
        if not self.platform.connected:
            return

        def work():
            current = self.platform.read_orientation()
            self.platform.move_to_orientation(
                Orientation(current.focus_mm, 0.0, 0.0))
            self.log_threadsafe("Levelled: tip and tilt set to zero.")

        self.run_async("Level", work)

    # -------------------------------------------------- connection settings

    def on_edit_limits(self) -> None:
        """Edit the motion limits without going to the configuration file.

        The limits are the thing most likely to be wrong on a machine that has
        not been commissioned yet: they ship as a guess, and the numbers that
        replace them come out of `find-stop`, which is run from this same
        window. Making that a text-file edit means somebody has to find the
        text file, in the dark, on a telescope.
        """
        window = tk.Toplevel(self.root)
        window.title("Motion limits")
        window.transient(self.root)
        self._limits_window = window
        limits = self.cfg.limits

        tk.Label(window, justify="left", anchor="w", fg="#555", wraplength=520,
                 text=("What the software will refuse. These are soft limits: "
                       "they must sit INSIDE the mechanism's own end stops, "
                       "with margin. Changes apply immediately and can be "
                       "saved to the configuration file.")
                 ).grid(row=0, column=0, columnspan=3, sticky="w",
                        padx=12, pady=(12, 8))

        fields = [
            ("min_focus_mm", "Focus, lowest (mm)",
             "towards M2 (secondary)"),
            ("max_focus_mm", "Focus, highest (mm)",
             "towards M1 (primary)"),
            ("max_tilt_deg", "Max total tilt (deg)",
             "magnitude, from the optical axis"),
            ("max_step_mm", "Max single step (mm)",
             "guards against a typo or a unit mistake"),
            ("max_tilt_step_deg", "Max single tilt step (deg)", ""),
            ("max_hard_stop_spread_mm", "Max drift apart, find-stop (mm)",
             "how far the three may diverge before the search is abandoned"),
        ]
        entries = {}
        for index, (attr, label, note) in enumerate(fields):
            ttk.Label(window, text=label).grid(row=1 + index, column=0,
                                               sticky="e", padx=(12, 4), pady=3)
            var = tk.StringVar(value=f"{getattr(limits, attr):g}")
            ttk.Entry(window, textvariable=var, width=12).grid(
                row=1 + index, column=1, sticky="w", padx=4)
            if note:
                ttk.Label(window, text=note, foreground="#777",
                          font=("TkDefaultFont", 8)).grid(
                    row=1 + index, column=2, sticky="w", padx=(4, 12))
            entries[attr] = var

        # --- what find-stop found, and a one-click way to use it -------------
        row = 1 + len(fields)
        found = ttk.LabelFrame(window, text="Ends of travel found by Find hard stop")
        found.grid(row=row, column=0, columnspan=3, sticky="ew",
                   padx=12, pady=(10, 4))

        def describe(value, which):
            if value is None:
                return f"{which}: not found yet"
            return f"{which}: {value:+.4f} mm"

        ttk.Label(found, text=describe(limits.hard_stop_low_mm, "lower")).grid(
            row=0, column=0, sticky="w", padx=8, pady=(6, 0))
        ttk.Label(found, text=describe(limits.hard_stop_high_mm, "upper")).grid(
            row=1, column=0, sticky="w", padx=8)

        margin_var = tk.StringVar(value="1.0")
        ttk.Label(found, text="keep this much margin (mm):").grid(
            row=2, column=0, sticky="e", padx=8, pady=4)
        ttk.Entry(found, textvariable=margin_var, width=8).grid(
            row=2, column=1, sticky="w")

        def from_stops() -> None:
            try:
                margin = float(margin_var.get())
            except ValueError:
                messagebox.showerror("Check the number",
                                     "Margin must be a number.", parent=window)
                return
            if margin < 0:
                messagebox.showerror("Check the number",
                                     "Margin cannot be negative.", parent=window)
                return
            if limits.hard_stop_low_mm is None and limits.hard_stop_high_mm is None:
                messagebox.showwarning(
                    "Nothing found yet",
                    "Run Tools > Find hard stop in each direction first.",
                    parent=window)
                return
            if limits.hard_stop_low_mm is not None:
                entries["min_focus_mm"].set(
                    f"{limits.hard_stop_low_mm + margin:g}")
            if limits.hard_stop_high_mm is not None:
                entries["max_focus_mm"].set(
                    f"{limits.hard_stop_high_mm - margin:g}")

        ttk.Button(found, text="Set focus limits from these",
                   command=from_stops).grid(row=2, column=2, padx=8, pady=4)

        def apply(persist: bool) -> None:
            values = {}
            for attr, label, _note in fields:
                try:
                    values[attr] = float(entries[attr].get())
                except ValueError:
                    messagebox.showerror("Check the numbers",
                                         f"{label} must be a number.",
                                         parent=window)
                    return

            # Validate on a copy, so a rejected edit cannot leave the live
            # limits half-applied.
            import dataclasses
            candidate = dataclasses.replace(limits, **values)
            try:
                candidate.validate()
                self._check_limits_against_stops(candidate)
            except ValueError as exc:
                messagebox.showerror("These limits will not do", str(exc),
                                     parent=window)
                return

            for attr, value in values.items():
                setattr(limits, attr, value)
            self._refresh_gauge_limits()
            self.log(f"Motion limits updated: focus "
                     f"{limits.min_focus_mm:+g} to {limits.max_focus_mm:+g} mm, "
                     f"max tilt {limits.max_tilt_deg:g} deg, max step "
                     f"{limits.max_step_mm:g} mm.")
            if persist:
                path = save_config(self.cfg, self.config_path)
                self.log(f"Saved to {path}")
            window.destroy()
            self._limits_window = None

        buttons = ttk.Frame(window)
        buttons.grid(row=row + 1, column=0, columnspan=3, pady=(6, 12))
        ttk.Button(buttons, text="Use for this session",
                   command=lambda: apply(False)).grid(row=0, column=0, padx=6)
        ttk.Button(buttons, text="Use and save",
                   command=lambda: apply(True)).grid(row=0, column=1, padx=6)
        ttk.Button(buttons, text="Cancel",
                   command=window.destroy).grid(row=0, column=2, padx=6)

    @staticmethod
    def _check_limits_against_stops(limits) -> None:
        """Refuse soft limits that sit outside the mechanism's own ends.

        A soft limit outside the hard stop is not a limit at all: every move it
        allows would end by driving into the end of travel. This is the one
        combination that is wrong on its face rather than merely unusual, so it
        is refused rather than warned about.
        """
        low, high = limits.hard_stop_low_mm, limits.hard_stop_high_mm
        if high is not None and limits.max_focus_mm > high:
            raise ValueError(
                f"The upper focus limit ({limits.max_focus_mm:+g} mm) is beyond "
                f"the end of travel found at {high:+g} mm. A move to that limit "
                f"would drive into the stop. Set it below {high:+g} mm."
            )
        if low is not None and limits.min_focus_mm < low:
            raise ValueError(
                f"The lower focus limit ({limits.min_focus_mm:+g} mm) is beyond "
                f"the end of travel found at {low:+g} mm. A move to that limit "
                f"would drive into the stop. Set it above {low:+g} mm."
            )

    def on_edit_connection(self) -> None:
        """Edit each motor's IP and port without leaving the application.

        Addresses change: a motor gets swapped, the subnet is renumbered, or
        the bench and the telescope are simply not the same network. Making
        that a text-file edit means someone has to find the text file.
        """
        window = tk.Toplevel(self.root)
        window.title("Connection settings")
        window.transient(self.root)

        tk.Label(window, justify="left", anchor="w", fg="#555", wraplength=460,
                 text=("Address of each motor. Changes take effect on the next "
                       "Connect, and can be saved into the configuration file "
                       "so they persist.")
                 ).grid(row=0, column=0, columnspan=4, sticky="w",
                        padx=12, pady=(12, 8))

        entries = {}
        for index, actuator in enumerate(self.cfg.actuators):
            ttk.Label(window, text=actuator.name, width=8,
                      font=("TkDefaultFont", 10, "bold")).grid(
                row=1 + index, column=0, sticky="e", padx=(12, 4), pady=4)
            ip_var = tk.StringVar(value=actuator.ip)
            port_var = tk.StringVar(value=str(actuator.port))
            ttk.Entry(window, textvariable=ip_var, width=18).grid(
                row=1 + index, column=1, padx=4)
            ttk.Label(window, text="port").grid(row=1 + index, column=2, padx=(8, 2))
            ttk.Entry(window, textvariable=port_var, width=8).grid(
                row=1 + index, column=3, padx=(0, 12))
            entries[actuator.name] = (ip_var, port_var)

        def apply(persist: bool) -> None:
            for actuator in self.cfg.actuators:
                ip_var, port_var = entries[actuator.name]
                ip = ip_var.get().strip()
                if not ip:
                    messagebox.showerror(
                        "Address needed",
                        f"{actuator.name} has no IP address.", parent=window)
                    return
                try:
                    port = int(port_var.get())
                except ValueError:
                    messagebox.showerror(
                        "Check the port",
                        f"{actuator.name}: port must be a whole number.",
                        parent=window)
                    return
                if not (0 < port < 65536):
                    messagebox.showerror(
                        "Check the port",
                        f"{actuator.name}: port must be 1..65535.", parent=window)
                    return
                actuator.ip, actuator.port = ip, port

            self._refresh_addresses()
            self._rebuild_platform()
            self.log("Connection settings updated. Press Connect to use them.")
            if persist:
                path = save_config(self.cfg, self.config_path)
                self.log(f"Saved to {path}")
            window.destroy()

        buttons = ttk.Frame(window)
        buttons.grid(row=1 + len(self.cfg.actuators), column=0, columnspan=4,
                     pady=(10, 12))
        ttk.Button(buttons, text="Use for this session",
                   command=lambda: apply(False)).grid(row=0, column=0, padx=6)
        ttk.Button(buttons, text="Use and save",
                   command=lambda: apply(True)).grid(row=0, column=1, padx=6)
        ttk.Button(buttons, text="Cancel",
                   command=window.destroy).grid(row=0, column=2, padx=6)

    def _rebuild_platform(self) -> None:
        """Rebuild the platform so new addresses are actually used.

        A JVLMotor holds its transport, and the transport holds the address it
        was built with, so editing the config alone would change the label and
        nothing else.
        """
        was_connected = self.platform.connected
        self._poll_stop.set()
        try:
            self.platform.disconnect()
        except Exception:
            pass
        self.platform = FocalPlanePlatform(
            cfg=self.cfg, simulate=self.simulate, logger=self.log_threadsafe,
            config_path=self.config_path,
        )
        self.connect_btn.config(text="Connect")
        self.conn_var.set("disconnected")
        self.conn_lamp.set(COLOR_IDLE)
        if was_connected:
            self.log("Disconnected because the addresses changed.")

    # ------------------------------------------------------ hard-stop search

    def on_find_hard_stop(self) -> None:
        """Run the actuators out until the travel ends.

        Always all three, continuously and together. Taking one actuator to
        its end stop on its own tilts the focal plane about the other two ball
        joints, and the site's experience is that this can break it, so there
        is no way to ask for that here.
        """
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return

        window = tk.Toplevel(self.root)
        window.title("Find hard stop")
        window.transient(self.root)
        self._hard_stop_window = window

        tk.Label(
            window, justify="left", anchor="w", wraplength=560,
            text=("Runs all three actuators out together until they will not "
                  "go further, then backs the commands off so nothing is left "
                  "pressed against a stop.\n\n"
                  "They move continuously, at a speed matched in millimetres "
                  "per second, so the plate stays flat the whole way. Torque "
                  "and progress are watched throughout: the first axis to stop "
                  "halts the other two in the same instant, and they are then "
                  "backed off to match it.\n\n"
                  "Where it finds the end is remembered and drawn on the gauge, "
                  "so you can see how much room is left."),
            fg="#333",
        ).grid(row=0, column=0, columnspan=4, sticky="w", padx=12, pady=(12, 8))

        ttk.Label(window, text="Direction").grid(row=1, column=0, sticky="e",
                                                 padx=(12, 4), pady=4)
        direction_var = tk.StringVar(value="+  towards M1")
        ttk.OptionMenu(window, direction_var, direction_var.get(),
                       "+  towards M1", "−  towards M2").grid(
            row=1, column=1, sticky="w", padx=4)

        ttk.Label(window, text="Give up after (mm)").grid(row=1, column=2, sticky="e",
                                                          padx=(12, 4))
        budget_var = tk.StringVar(value="30.0")
        ttk.Entry(window, textvariable=budget_var, width=10).grid(row=1, column=3,
                                                                  sticky="w", padx=(0, 12))

        ttk.Label(window, text="Speed (% of configured)").grid(
            row=2, column=0, sticky="e", padx=(12, 4), pady=4)
        speed_var = tk.StringVar(value="25")
        ttk.Entry(window, textvariable=speed_var, width=10).grid(row=2, column=1,
                                                                 sticky="w", padx=4)

        actuator = self.cfg.actuators[0]
        ttk.Label(window,
                  text=(f"Torque limit {actuator.stall_torque_percent:.0f}% of "
                        f"the drive's current limit over "
                        f"{actuator.stall_persist_samples} consecutive readings. "
                        f"Abandoned if the three drift more than "
                        f"{self.cfg.limits.max_hard_stop_spread_mm:.2f} mm apart."),
                  foreground="#777", wraplength=560, justify="left").grid(
            row=3, column=0, columnspan=4, sticky="w", padx=12, pady=(4, 8))

        def start() -> None:
            try:
                budget_mm = float(budget_var.get())
                speed = float(speed_var.get()) / 100.0
            except ValueError:
                messagebox.showerror("Check the numbers",
                                     "Budget and speed must be numbers.",
                                     parent=window)
                return
            direction = 1 if direction_var.get().startswith("+") else -1
            window.destroy()
            self._hard_stop_window = None
            self._run_hard_stop_together(direction, budget_mm, speed)

        buttons = ttk.Frame(window)
        buttons.grid(row=4, column=0, columnspan=4, pady=(6, 12))
        ttk.Button(buttons, text="Find the stop",
                   command=start).grid(row=0, column=0, padx=6)
        ttk.Button(buttons, text="Cancel",
                   command=window.destroy).grid(row=0, column=1, padx=6)

    def _run_hard_stop_together(self, direction: int, budget_mm: float,
                                speed: float) -> None:
        def work():
            self.log_threadsafe(
                f"Hard-stop search on all three actuators, {direction:+d} "
                f"direction, continuous at {speed:.0%} of configured speed, up "
                f"to {budget_mm:.1f} mm."
            )
            last = [0.0]

            def progress(step):
                # 50 ms of readings would bury the log; half a second is enough
                # to watch it advance.
                now = time.monotonic()
                if now - last[0] < 0.5:
                    return
                last[0] = now
                self.log_threadsafe(
                    "  " + "  ".join(f"{n} {mm:+8.4f}"
                                     for n, mm in step.positions_mm.items())
                    + f"   apart by {step.spread_mm:.4f} mm"
                    + f"   load {max(step.torque_percent.values()):.0f}%"
                )

            result = self.platform.seek_hard_stop_together(
                direction=direction, budget_mm=budget_mm,
                speed_fraction=speed, progress=progress,
            )
            for line in result.summary().splitlines():
                self.log_threadsafe(line)
            self.post(lambda: self._record_hard_stop(direction, result))

        self.run_async("Find hard stop", work)

    def _record_hard_stop(self, direction: int, result) -> None:
        """Remember where the end of travel is, and draw it on the gauge.

        A hard stop is a fact about the machine, so it is worth keeping. The
        focus position of the stop is the mean of where the three ended up,
        which is what the kinematics call focus when the plate is flat -- and
        it is flat, because the search levels it.
        """
        # stop_mm is where the travel actually ended. positions_mm is where
        # the actuators are *now*, which is half a millimetre short of it,
        # because the search backs off rather than leaving the mechanism
        # resting on its stop. Marking the parked position would put the end
        # of travel in the wrong place by exactly the back-off.
        ends = result.stop_mm or result.positions_mm
        focus_mm = sum(ends.values()) / len(ends)
        limits = self.cfg.limits
        if direction > 0:
            limits.hard_stop_high_mm = focus_mm
        else:
            limits.hard_stop_low_mm = focus_mm
        self._refresh_gauge_limits()
        self.log(f"End of travel recorded at {focus_mm:+.4f} mm and marked on "
                 f"the gauge.")

        inside = (limits.min_focus_mm <= focus_mm <= limits.max_focus_mm)
        if inside:
            self.log("  That is INSIDE the configured focus limits, which means "
                     "the limits are wrong -- they should sit inside the travel, "
                     "not outside it.")
        if messagebox.askyesno(
            "Save the end of travel?",
            f"The {'upper' if direction > 0 else 'lower'} end of travel was "
            f"found at {focus_mm:+.4f} mm.\n\n"
            "Save it to the configuration file, so it stays marked on the "
            "gauge next time?",
        ):
            try:
                self.platform.save()
                self.log("Saved to the configuration file.")
            except Exception as exc:  # noqa: BLE001
                messagebox.showerror("Could not save", str(exc))


    def on_safety_drills(self) -> None:
        """Provoke each dangerous situation and check the software refuses it.

        Safe to run at any time, including while connected to the telescope:
        every drill builds its own simulated platform and never touches these
        motors. What it proves is that the guards still fire -- a check nobody
        has seen fire is a check nobody should trust.
        """
        def work():
            from .safety import run_all

            self.log_threadsafe(
                "Safety drills: provoking each dangerous situation in "
                "simulation. Nothing here touches the real motors.")

            def report(result):
                self.log_threadsafe(f"  [{result.verdict}] {result.name}")
                self.log_threadsafe(f"        did:    {result.what_was_done}")
                self.log_threadsafe(f"        result: {result.what_happened}")

            outcome = run_all(report=report)
            passed = len(outcome.results) - len(outcome.failures)
            self.log_threadsafe(
                f"Safety drills: {passed} of {len(outcome.results)} passed.")
            if outcome.failures:
                self.log_threadsafe(
                    "  A failing drill means a guard is missing or has stopped "
                    "working. Do not rely on the software to refuse that "
                    "situation until it is fixed.")

        self.run_async("Safety drills", work)

    # ----------------------------------------------------------------- misc

    def on_clear_errors(self) -> None:
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return

        def work():
            for name, value in self.platform.clear_all_errors().items():
                if value == 0:
                    self.log_threadsafe(f"{name}: errors clear.")
                elif value < 0:
                    self.log_threadsafe(f"{name}: could not read errors back.")
                else:
                    self.log_threadsafe(
                        f"{name}: still 0x{value:08X} -- latched fault, needs "
                        "MacTalk's clear or a power cycle."
                    )

        self.run_async("Clear errors", work)

    def on_set_zero(self) -> None:
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        if not messagebox.askyesno(
            "Set the zero reference?",
            "This defines the CURRENT position as focus 0, tip 0, tilt 0, and "
            "saves it to the configuration file.\n\nEvery later command is "
            "measured from here, so only do this with the focal plane at a "
            "position you have independently established.\n\nSet zero here?",
        ):
            return

        def work():
            result = self.platform.set_zero_here(persist=True)
            self.log_threadsafe(f"Zero reference set: {result}")

        self.run_async("Set zero", work)

    def on_close(self) -> None:
        if self.platform.connected and not messagebox.askyesno(
            "Quit?",
            "Closing this window does NOT stop the motors or change the brakes. "
            "Anything still moving keeps moving.\n\nQuit anyway?",
        ):
            return
        self.shutdown()
        self.root.destroy()

    def shutdown(self) -> None:
        """Stop the poller and the UI drain, and drop the connections.

        Deliberately does not touch the motors: closing a window is not a
        command to move, stop or change a brake, and silently passivating on
        exit would drop a loaded axis.
        """
        self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
            self._poll_thread = None
        if self._ui_job is not None:
            try:
                self.root.after_cancel(self._ui_job)
            except Exception:
                pass
            self._ui_job = None
        # Drop queued closures. They capture widgets, and letting them be
        # collected later -- possibly on a worker thread, after Tk has gone --
        # is what produces "main thread is not in main loop" at exit.
        try:
            while True:
                self._ui_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self.platform.disconnect()
        except Exception:
            pass


def main(config_path: Optional[str] = None, simulate: bool = False,
         bench: Optional[str] = None) -> int:
    root = tk.Tk()
    MotorApp(root, config_path=config_path, simulate=simulate, bench=bench)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
