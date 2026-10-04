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

import hashlib
import hmac
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
from .focus_gauge import REFERENCE_TITLES, REFERENCES, FocusGauge
from .history import MoveRecord, default_history_path
from .saved_positions import SUGGESTED_NAMES, default_saved_positions_path
from .plane_view import FocalPlaneView
from .jvl_motor import BrakeState
from .kinematics import Orientation
from .platform import FocalPlanePlatform, PlatformError, PlatformState
from .registers import describe_errors_short
from .external_brake import (BrakeController, BrakeError, ExternalBrakeConfig,
                             SimulatedBrakeController)

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


#: What each brake state is called on screen. ENGAGED means the brake is
#: clamping the camera; DISENGAGED means it is off and the drives are what
#: holds the camera.
BRAKE_WORDS = {
    BrakeState.ENGAGED: "ENGAGED",
    BrakeState.RELEASED: "DISENGAGED",
    BrakeState.UNKNOWN: "unknown",
}


#: Where 0 can be, everywhere the window shows or takes a position, and what
#: to call it.
POSITION_REFERENCES = {
    "zero": "motor zero",
    "top": "top stop",
    "bottom": "bottom stop",
}
#: The hint beside the Go to box for each.
GOTO_HINTS = {
    "zero": "+ towards M1,  − towards M2",
    "top": "from the top stop: 0 = top stop,  − is below it",
    "bottom": "from the bottom stop: 0 = bottom stop,  + is above it",
}

#: Offered in the Load and torque window, readings per second while idle.
READING_RATES = ("2", "5", "10")

#: The focal plane window (tip, tilt and single-actuator jogs) and changing
#: the torque limit ask for a password. Only a salted hash is kept here, so the password itself is not
#: written in the source. To change it, put the new hash here:
#:   python -c "import hashlib; print(hashlib.sha256(b'psct-tilt-lock:NEW').hexdigest())"
TILT_PASSWORD_SALT = "psct-tilt-lock:"
TILT_PASSWORD_SHA256 = "790025c677383208073807d644fad547bae45f5e95d4661ee0926879f9abbff5"


def tilt_password_matches(text: str) -> bool:
    digest = hashlib.sha256((TILT_PASSWORD_SALT + text).encode("utf-8")).hexdigest()
    return hmac.compare_digest(digest, TILT_PASSWORD_SHA256)


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
        self._marks = [self.create_line(0, 0, 0, 0, fill=colour, width=1,
                                        dash=(2, 2))
                       for colour in ("#c77700", COLOR_BAD)]
        self._text = self.create_text(self.WIDTH // 2, self.HEIGHT // 2,
                                      text="--", font=("TkDefaultFont", 7))
        self.set_thresholds(warn_percent, stall_percent)

    def resize(self, width: int, height: int) -> None:
        self.WIDTH, self.HEIGHT = width, height
        self.configure(width=width, height=height)
        self.coords(self._text, width // 2, height // 2)
        self.set_thresholds(self.warn_percent, self.stall_percent)
        if self._percent is not None:
            self.set(self._percent)

    def set_thresholds(self, warn_percent: float, stall_percent: float) -> None:
        """Move the warning and stop marks, e.g. after the limit is changed."""
        self.warn_percent = warn_percent
        self.stall_percent = stall_percent
        for mark, percent in zip(self._marks, (warn_percent, stall_percent)):
            x = self._x(percent)
            self.coords(mark, x, 0, x, self.HEIGHT)

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
    BRAKE_SAMPLES = tuple(BRAKE_WORDS.values())
    NAME_FONT = ("TkDefaultFont", 11, "bold")
    #: How many grid columns a row occupies. Anything spanning the table --
    #: the footer -- spans this many, so adding a column here cannot leave it
    #: one short.
    NCOLS = 13

    def __init__(self, parent, name: str, row: int, app: "MotorApp"):
        self.name = name
        self.app = app

        self.name_label = ttk.Label(parent, text=name, anchor="w",
                                    font=self.NAME_FONT)
        self.name_label.grid(row=row, column=0, padx=(6, 2), sticky="w")

        self.position_var = tk.StringVar(value="--")
        ttk.Label(parent, textvariable=self.position_var, width=12, anchor="e",
                  font=("TkFixedFont", 10)).grid(row=row, column=1, padx=2)

        self.counts_var = tk.StringVar(value="--")
        ttk.Label(parent, textvariable=self.counts_var, width=12, anchor="e",
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
        ttk.Label(parent, textvariable=self.load_var, width=11, anchor="w",
                  font=("TkFixedFont", 10, "bold"), foreground="#333").grid(
            row=row, column=8, padx=(2, 6), sticky="w")

        # The drive's supply. Volts once `cli supply` has recorded the scale,
        # the raw register value until then -- and labelled as raw, because
        # the drive's units are not documented and are not guessed at here.
        self.supply_var = tk.StringVar(value="--")
        self.supply_label = ttk.Label(parent, textvariable=self.supply_var,
                                      width=9, anchor="e", cursor="hand2",
                                      font=("TkFixedFont", 10), foreground="#333")
        self.supply_label.grid(row=row, column=9, padx=(2, 6))
        # Clicking it is the quickest way to the scale, which is what turns
        # "raw" into volts.
        self.supply_label.bind("<Button-1>",
                               lambda _event: app.on_set_supply_scale())

        self.release_btn = ttk.Button(
            parent, text="Release", width=8,
            command=lambda: app.on_brake(name, engage=False))
        self.release_btn.grid(row=row, column=10, padx=2)
        self.engage_btn = ttk.Button(
            parent, text="Engage", width=8,
            command=lambda: app.on_brake(name, engage=True))
        self.engage_btn.grid(row=row, column=11, padx=2)

        # Short form in the row -- the hex and the first name -- because the
        # full decode with its caveats is a sentence long and stretched the
        # whole table. The full text is in the log whenever the bits change.
        self.error_var = tk.StringVar(value="")
        ttk.Label(parent, textvariable=self.error_var, foreground=COLOR_BAD,
                  anchor="w").grid(row=row, column=12, padx=(10, 6), sticky="w")

    def update(self, status) -> None:
        if status.comms_error:
            self.position_var.set("--")
            self.counts_var.set("")
            self.mode_var.set("no comms")
            self.load_bar.set(None, "--")
            self.load_var.set("")
            self.supply_var.set("--")
            self.mode_lamp.set(COLOR_BAD)
            self.brake_var.set("unknown")
            self.brake_lamp.set(COLOR_IDLE)
            self.error_var.set(status.comms_error[:60])
            return

        self.position_var.set(f"{self.app._shown(status.position_mm):+10.4f} mm")
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
        # remembering what the colours mean. Whether the state is measured or
        # only the relay's is said once, on the Brakes line above the table.
        self.brake_var.set(BRAKE_WORDS[brake.state])
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
            self.load_var.set(f"{status.torque_percent:>3.0f}% ~{status.current_a:.2f}A")
        else:
            self.load_var.set(f"{status.torque_percent:>3.0f}% load")

        if status.bus_voltage is None:
            self.supply_var.set("--")
        elif status.supply_volts is not None:
            self.supply_var.set(f"{status.supply_volts:.1f} V")
        else:
            self.supply_var.set(f"{status.bus_voltage} raw")

        self.error_var.set(describe_errors_short(status.error_bits)
                           if status.error_bits else "")

    def set_brake_controls_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        self.release_btn.config(state=state)
        self.engage_btn.config(state=state)


class MotorApp:
    def __init__(self, root: tk.Tk, config_path: Optional[str] = None,
                 simulate: bool = False, bench: Optional[str] = None,
                 sim_speed: Optional[float] = None,
                 poll_interval_s: Optional[float] = None,
                 use_real_brakes: bool = False):
        self.root = root
        self.simulate = simulate
        self.bench = bench
        self.use_real_brakes = use_real_brakes
        self.config_path = config_path

        self.cfg = load_config(config_path)
        if bench:
            from .cli import apply_bench
            apply_bench(self.cfg, bench)
        if sim_speed:
            self.cfg.simulated_speed_mm_per_s = sim_speed
        if poll_interval_s:
            self.cfg.poll_interval_s = poll_interval_s
            # The idle rate can never be the faster of the two, or asking for
            # a slow poll would silently make the stationary case quicker.
            self.cfg.idle_poll_interval_s = max(poll_interval_s,
                                                self.cfg.idle_poll_interval_s)
        self._busy = False
        self._poll_stop = threading.Event()
        self._poll_thread: Optional[threading.Thread] = None
        #: Closures posted by worker threads, executed on the UI thread.
        #: Created before the platform, whose logger posts here.
        self._ui_queue: "queue.Queue[Callable[[], None]]" = queue.Queue()
        self._ui_job: Optional[str] = None
        self._history_window = None
        self._history_tree = None
        self._history_rows: List[MoveRecord] = []
        #: How many records the log window last drew, so a poll can notice a
        #: move that was recorded without the change callback firing.
        self._history_drawn = -1

        self.platform = self._make_platform()
        self.root.title(self._window_title())

        self._build_ui()
        self._refresh_saved_positions()
        self._drain_ui()
        self.log(f"Configuration: {config_path or default_config_path()}")
        if simulate and "PLC IS REAL" in self._window_title():
            self.log("SIMULATED MOTORS with the REAL brake PLC "
                     f"({self.platform.external_brake.describe()}). Its relays "
                     "WILL switch.")
        elif simulate:
            self.log("SIMULATION MODE -- no hardware is being touched.")
            self.log(f"Simulated actuators run at "
                     f"{self.cfg.simulated_speed_mm_per_s:g} mm/s at full "
                     f"velocity. Change simulated_speed_mm_per_s in the "
                     f"configuration, or pass --sim-speed, to slow it down or "
                     f"speed it up.")
        elif self.platform.is_mixed:
            self.log("BENCH MODE: "
                     + ", ".join(self.platform.simulated_names)
                     + " are simulated. Only "
                     + ", ".join(m.name for m in self.platform.motors
                                 if m.name not in self.platform.simulated_names)
                     + " is a real motor, and everything the others report is "
                       "made up.")
            if self.platform.copied_names:
                self.log(" and ".join(self.platform.copied_names) + " copy "
                         + self.platform.copy_source + ": they sit wherever it "
                         "is and go wherever it is sent, so the plane stays "
                         "flat. Tip, tilt and jogging them on their own are "
                         "refused.")
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

    def _make_platform(self) -> FocalPlanePlatform:
        """The platform, with its position history written beside the config.

        The history file is per machine, like the configuration, so it goes
        in the same place -- and it is passed explicitly because a platform
        built for a test or a safety drill must not write one.
        """
        return FocalPlanePlatform(
            cfg=self.cfg, simulate=self.simulate, logger=self.log_threadsafe,
            config_path=self.config_path,
            history_path=default_history_path(self.config_path),
            on_history_change=lambda: self.post(self._refresh_position_log),
            use_real_brakes=self.use_real_brakes,
            saved_positions_path=default_saved_positions_path(self.config_path),
        )

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
        real_brakes = self.platform.external_brake.available and not isinstance(
            self.platform.external_brake, SimulatedBrakeController)
        if self.simulate and real_brakes:
            return ("pSCT Focal Plane Control  [SIMULATED MOTORS -- the brake "
                    "PLC IS REAL]")
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
        """Tip, tilt and single-actuator jogs live here, not on the main panel.

        Day to day this mechanism is a focus drive: the site's own procedure
        motorises only the optical axis, and the two tilts exist to correct
        the focal plane's orientation, not to be set routinely. Keeping them
        one menu item away means the main window says what the job is, while
        the capability is still there for whoever needs it.
        """
        menubar = tk.Menu(self.root)

        # Everyday controls are on the window itself; the menus hold what is
        # not, each thing in one place. Motion: moving and how moves behave.
        # View: windows to look at. Setup: done once, or when something
        # changes.
        motion = tk.Menu(menubar, tearoff=0)
        motion.add_command(label="Focal plane: tilt and jog...",
                           command=self.on_open_tilt)
        motion.add_command(label="Go back to the previous position",
                           command=self.on_go_back)
        motion.add_command(label="Copy current position into Go to",
                           command=self.on_copy_current)
        motion.add_separator()
        motion.add_command(label="Find hard stop (calibration)...",
                           command=self.on_find_hard_stop)
        motion.add_command(label="Motion settings...",
                           command=self.on_edit_limits)
        menubar.add_cascade(label="Motion", menu=motion)

        view = tk.Menu(menubar, tearoff=0)
        view.add_command(label="Position log...",
                         command=self.on_open_position_log)
        view.add_command(label="Load and torque...",
                         command=self.on_open_load_view)
        menubar.add_cascade(label="View", menu=view)

        setup = tk.Menu(menubar, tearoff=0)
        setup.add_command(label="Motor connections...",
                          command=self.on_edit_connection)
        setup.add_command(label="Brake controller (PLC)...",
                          command=self.on_edit_brake_controller)
        setup.add_command(label="Supply voltage...",
                          command=self.on_set_supply_scale)
        setup.add_command(label="Distances to M1 and M2...",
                          command=self.on_edit_reference_distances)
        menubar.add_cascade(label="Setup", menu=setup)

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

        explanation = tk.Label(
            bar,
            text="STOP decelerates and holds position with the drives still on. "
                 "EMERGENCY does that too, then engages the brakes and, once the "
                 "PLC reports them engaged, turns the drives off. If the PLC "
                 "cannot confirm the brakes, the drives stay on and holding. "
                 "Neither asks for confirmation.",
            bg=COLOR_STOP_DARK, fg="#ffd7d7", font=("TkDefaultFont", 8),
            justify="left", anchor="w",
        )
        explanation.grid(row=1, column=0, columnspan=2, sticky="ew", padx=8,
                         pady=(0, 2))

        # What the last STOP or EMERGENCY actually did. These controls act
        # without asking, so the result has to be visible without going to
        # look for it in the log.
        self.action_var = tk.StringVar(value="")
        self.action_label = tk.Label(
            bar, textvariable=self.action_var, bg=COLOR_STOP_DARK, fg="white",
            font=("TkDefaultFont", 9, "bold"), anchor="w", justify="left",
        )
        self.action_label.grid(row=2, column=0, columnspan=2, sticky="ew",
                               padx=8, pady=(0, 4))

        # Both labels wrap to the bar's width, whatever that turns out to be.
        # Without this the explanation ran as one line about 1700 px long, and
        # since a window is never narrower than its widest child, it pushed
        # the whole application past the right edge of any ordinary screen --
        # the gauge, the log's Clear button and the end of the text itself
        # were simply off-screen.
        self._wrap_to_width(bar, explanation, self.action_label)

    @staticmethod
    def _wrap_to_width(container, *labels, margin: int = 24) -> None:
        """Keep `labels` wrapped to `container`'s current width.

        Tk labels do not wrap on their own: `wraplength` is a fixed number of
        pixels, and a label with none set is exactly as wide as its text. So
        the wrap length follows the container as it is resized, and a long
        sentence can never be the thing that decides how wide the window is.
        """
        def resize(event) -> None:
            width = max(200, event.width - margin)
            for label in labels:
                label.configure(wraplength=width)
        container.bind("<Configure>", resize, add="+")

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

        self.addresses_var = tk.StringVar()
        ttk.Label(frame, textvariable=self.addresses_var,
                  foreground="#555").grid(row=0, column=5, padx=10, sticky="w")
        self._refresh_addresses()

        # The brake controller, on its own line: which device, what it says,
        # and whether that is the brake or only its relay. Bench testing the
        # PLC is mostly a matter of watching this line change.
        self.brake_lamp = Lamp(frame)
        self.brake_lamp.grid(row=1, column=1, padx=(6, 2), pady=(0, 6))
        self.brake_ctrl_var = tk.StringVar(value=self._brake_controller_text())
        self.brake_ctrl_label = ttk.Label(frame, textvariable=self.brake_ctrl_var,
                                          anchor="w", justify="left")
        self.brake_ctrl_label.grid(row=1, column=2, columnspan=4, sticky="w",
                                   pady=(0, 6))
        self._wrap_to_width(frame, self.brake_ctrl_label, margin=160)

    def _brake_controller_text(self) -> str:
        """What the brake line says before anything has been read."""
        controller = self.platform.external_brake
        if not controller.available:
            return "Brakes: not under software control (Setup > Brake controller to set up the PLC)"
        return f"Brakes: {controller.describe()} -- not read yet"

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
        # In the chosen position reference (Motion > Motion settings), so with
        # the top stop as 0, "-10" is 10 mm below the top stop.
        ttk.Label(frame, text="Go to focus (mm):").grid(
            row=2, column=0, padx=(8, 2), sticky="e")
        self.focus_var = tk.StringVar(value="0.0")
        ttk.Entry(frame, textvariable=self.focus_var, width=12,
                  font=("TkDefaultFont", 11)).grid(row=2, column=1, padx=2, pady=4)
        ttk.Button(frame, text="Preview",
                   command=self.on_preview).grid(row=2, column=2, padx=(6, 2))
        self.move_btn = ttk.Button(frame, text="Move", command=self.on_move)
        self.move_btn.grid(row=2, column=3, padx=2)
        self.goto_note_var = tk.StringVar(value=GOTO_HINTS["zero"])
        ttk.Label(frame, textvariable=self.goto_note_var,
                  foreground="#777").grid(row=2, column=4, padx=(12, 4), sticky="w")

        # --- fine adjustment ---
        fine = ttk.Frame(frame)
        fine.grid(row=3, column=0, columnspan=10, sticky="w", padx=8, pady=(6, 2))
        ttk.Label(fine, text="Fine adjust focus by (mm):").grid(row=0, column=0, padx=(0, 6))
        self.focus_step_var = tk.StringVar(value="0.010")
        ttk.Entry(fine, textvariable=self.focus_step_var,
                  width=10).grid(row=0, column=1, padx=2)
        tk.Button(fine, text="−", width=4, font=("TkDefaultFont", 12, "bold"),
                  command=lambda: self.on_nudge("focus", -1)).grid(row=0, column=2, padx=3)
        tk.Button(fine, text="+", width=4, font=("TkDefaultFont", 12, "bold"),
                  command=lambda: self.on_nudge("focus", +1)).grid(row=0, column=3, padx=3)

        for label, step in (("1 um", 0.001), ("10 um", 0.010),
                            ("50 um", 0.050), ("0.5 mm", 0.500)):
            ttk.Button(fine, text=label, width=7,
                       command=lambda v=step: self.focus_step_var.set(f"{v:.3f}")
                       ).grid(row=0, column=4 + list(
                           ("1 um", "10 um", "50 um", "0.5 mm")).index(label),
                              padx=2)

        # --- saved positions ---
        saved = ttk.Frame(frame)
        saved.grid(row=4, column=0, columnspan=10, sticky="w", padx=8, pady=(6, 10))
        ttk.Label(saved, text="Saved position:").grid(row=0, column=0, padx=(0, 6))
        self.saved_choice_var = tk.StringVar(value="")
        self.saved_combo = ttk.Combobox(saved, textvariable=self.saved_choice_var,
                                        state="readonly", width=22)
        self.saved_combo.grid(row=0, column=1, padx=2)
        ttk.Button(saved, text="Go to",
                   command=lambda: self._go_to_saved(self.saved_choice_var.get())
                   ).grid(row=0, column=2, padx=(6, 2))
        ttk.Button(saved, text="Save current as...",
                   command=self.on_save_position).grid(row=0, column=3, padx=2)
        ttk.Button(saved, text="All saved...",
                   command=self.on_open_saved_positions).grid(row=0, column=4, padx=2)

        # --- tip/tilt and jog entries exist here but are shown in the -------
        # focal plane window. They live on the app so that window, the tests
        # and the move code can all reach them whether or not it is open.
        self.tip_var = tk.StringVar(value="0.0")
        self.tilt_var = tk.StringVar(value="0.0")
        self.angle_step_var = tk.StringVar(value="0.010")
        self.jog_step_var = tk.StringVar(value="0.050")
        self._tilt_window = None
        self._password_window = None
        self._saved_window = None
        self._save_dialog = None
        self._supply_window = None
        self._saved_tree = None
        self._last_state = None
        self._hard_stop_window = None
        self._limits_window = None
        self._load_window = None
        self._brake_window = None
        self._brake_window_read = None
        self._load_rows = {}
        self._limit_entries = ()
        self._stops_unlocked = False
        self._password_action = None
        self.plane_view = None

    def _build_actuators(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="Actuators")
        self.actuator_frame = frame
        frame.grid(row=1, column=0, sticky="nsew", pady=3)

        headers = ["", "position", "counts", "", "mode", "", "brake",
                   "load", "", "supply", "", "", ""]
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

        footer = ttk.Frame(frame)
        footer.grid(row=len(self.cfg.actuators) + 1, column=0,
                    columnspan=MotorRow.NCOLS, sticky="w", padx=6, pady=(6, 6))

        # In the order they are used: drives on and holding, then brakes off.
        ttk.Button(footer, text="Enable drives",
                   command=self.on_enable_drives).grid(row=0, column=0, padx=(0, 4))
        ttk.Button(footer, text="Disable drives",
                   command=self.on_disable_drives).grid(row=0, column=1, padx=4)
        ttk.Button(footer, text="Release all brakes",
                   command=lambda: self.on_brake(None, engage=False)).grid(row=0, column=2, padx=(12, 4))
        ttk.Button(footer, text="Engage all brakes",
                   command=lambda: self.on_brake(None, engage=True)).grid(row=0, column=3, padx=4)
        self.rest_after_var = tk.BooleanVar(value=self.cfg.rest_on_brakes_after_moves)
        ttk.Checkbutton(footer, text="Disable drives after each move",
                        variable=self.rest_after_var,
                        command=self.on_rest_after_moves_changed).grid(
            row=0, column=4, padx=(16, 0))

    def _build_gauge(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text=REFERENCE_TITLES["zero"])
        frame.grid(row=0, column=1, rowspan=3, sticky="ns", padx=(6, 0))
        frame.rowconfigure(1, weight=1)
        self.gauge_frame = frame

        self.gauge = FocusGauge(frame,
                                min_mm=self.cfg.limits.min_focus_mm,
                                max_mm=self.cfg.limits.max_focus_mm)
        self.gauge.grid(row=1, column=0, sticky="ns", padx=8, pady=(6, 4))

        # How far it is to each mirror, once those distances are entered.
        self.mirror_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.mirror_var, foreground="#555",
                  font=("TkDefaultFont", 8), justify="center").grid(
            row=2, column=0, pady=(0, 8))

        self._refresh_gauge_limits()

    def _refresh_gauge_limits(self) -> None:
        """Push the current limits and any found hard stops onto the gauge."""
        limits = self.cfg.limits
        self.gauge.set_limits(limits.min_focus_mm, limits.max_focus_mm,
                              hard_stop_low_mm=limits.hard_stop_low_mm,
                              hard_stop_high_mm=limits.hard_stop_high_mm)
        # Measuring from an end of travel depends on these.
        if getattr(self, "gauge_frame", None) is not None:
            self._apply_reference()

    # ------------------------------------------------- position reference

    def _reference(self) -> str:
        """Where 0 is: "zero", "top" or "bottom". The motors' zero while the
        chosen end of travel is not known."""
        ref = self.cfg.position_reference
        limits = self.cfg.limits
        if ref == "top" and limits.hard_stop_high_mm is None:
            return "zero"
        if ref == "bottom" and limits.hard_stop_low_mm is None:
            return "zero"
        return ref

    def _offset(self) -> float:
        ref = self._reference()
        if ref == "top":
            return -self.cfg.limits.hard_stop_high_mm
        if ref == "bottom":
            return -self.cfg.limits.hard_stop_low_mm
        return 0.0

    def _shown(self, mm: float) -> float:
        """A focus or actuator position in the motors' zero, as shown."""
        return mm + self._offset()

    def _from_shown(self, value: float) -> float:
        """A position typed in the chosen reference, in the motors' zero."""
        return value - self._offset()

    def _ref_words(self) -> str:
        """" from the top stop", or nothing for the motors' zero."""
        ref = self._reference()
        return "" if ref == "zero" else f" from the {POSITION_REFERENCES[ref]}"

    def _describe(self, o: Orientation) -> str:
        """Orientation.describe(), with focus in the chosen reference."""
        return (f"focus {self._shown(o.focus_mm):+.4f} mm{self._ref_words()}, "
                f"tip {o.tip_deg:+.5f} deg, tilt {o.tilt_deg:+.5f} deg "
                f"(total {o.total_tilt_arcmin:.3f} arcmin "
                f"towards {o.tilt_azimuth_deg:.1f} deg)")

    def _apply_reference(self) -> None:
        """Make everything show positions from the chosen reference."""
        ref = self._reference()
        self.platform.shown_offset_mm = self._offset()
        self.platform.shown_from = self._ref_words()
        self.gauge.set_reference(ref)
        self.gauge_frame.configure(text=REFERENCE_TITLES[ref])
        self.goto_note_var.set(GOTO_HINTS[ref])
        if self.plane_view is not None:
            self.plane_view.label_offset_mm = self._offset()
        if self._history_tree is not None:
            self._refresh_position_log()
        if self._saved_tree is not None:
            self._refresh_saved_positions()

    def _set_reference(self, ref: str) -> None:
        """Change where 0 is, keeping the Go to box on the same place."""
        try:
            focus = self._from_shown(float(self.focus_var.get()))
        except ValueError:
            focus = None
        self.cfg.position_reference = ref
        self._apply_reference()
        if focus is not None:
            self.focus_var.set(f"{self._shown(focus):.4f}")
        self.log("Positions are now shown from the "
                 + POSITION_REFERENCES[self._reference()] + ".")

    def on_edit_reference_distances(self) -> None:
        """Enter how far the zero reference is from each mirror.

        These are the numbers the "to M1" and "to M2" references need, and
        nobody has them yet. They can change -- re-setting the zero moves
        both, and a re-survey moves either -- so they are edited here rather
        than typed into a file.
        """
        window = tk.Toplevel(self.root)
        window.title("Distances from zero to the mirrors")
        window.transient(self.root)

        tk.Label(window, justify="left", anchor="w", fg="#555", wraplength=500,
                 text=("Distance along the optical axis from the zero reference "
                       "(focus 0) to each mirror, in "
                       "millimetres. With these entered, the distance from "
                       "the focal plane to each mirror is shown under the "
                       "gauge.\n\n"
                       "Leave a box empty if the distance is not known -- "
                       "nothing is shown rather than a made-up number.")
                 ).grid(row=0, column=0, columnspan=3, sticky="w",
                        padx=12, pady=(12, 8))

        def as_text(value):
            return "" if value is None else f"{value:g}"

        m1_var = tk.StringVar(value=as_text(self.cfg.zero_to_m1_mm))
        m2_var = tk.StringVar(value=as_text(self.cfg.zero_to_m2_mm))
        ttk.Label(window, text="zero to M1 (mm)").grid(row=1, column=0, sticky="e",
                                                       padx=(12, 4), pady=4)
        ttk.Entry(window, textvariable=m1_var, width=14).grid(row=1, column=1,
                                                              sticky="w")
        ttk.Label(window, text="+ direction, the primary mirror",
                  foreground="#777", font=("TkDefaultFont", 8)).grid(
            row=1, column=2, sticky="w", padx=(4, 12))
        ttk.Label(window, text="zero to M2 (mm)").grid(row=2, column=0, sticky="e",
                                                       padx=(12, 4), pady=4)
        ttk.Entry(window, textvariable=m2_var, width=14).grid(row=2, column=1,
                                                              sticky="w")
        ttk.Label(window, text="− direction, the secondary mirror",
                  foreground="#777", font=("TkDefaultFont", 8)).grid(
            row=2, column=2, sticky="w", padx=(4, 12))


        def parse(var, label):
            text = var.get().strip()
            if not text:
                return None, True
            try:
                value = float(text)
            except ValueError:
                messagebox.showerror("Check the number",
                                     f"{label} must be a number, or empty.",
                                     parent=window)
                return None, False
            if value <= 0:
                messagebox.showerror("Check the number",
                                     f"{label} must be a positive distance.",
                                     parent=window)
                return None, False
            return value, True

        def apply(persist: bool) -> None:
            m1, ok1 = parse(m1_var, "zero to M1")
            if not ok1:
                return
            m2, ok2 = parse(m2_var, "zero to M2")
            if not ok2:
                return
            self.cfg.zero_to_m1_mm = m1
            self.cfg.zero_to_m2_mm = m2
            self.log("Zero-to-mirror distances: "
                     f"M1 {as_text(m1) or 'not set'} mm, "
                     f"M2 {as_text(m2) or 'not set'} mm.")
            if persist:
                path = save_config(self.cfg, self.config_path)
                self.log(f"Saved to {path}")
            window.destroy()

        buttons = ttk.Frame(window)
        buttons.grid(row=4, column=0, columnspan=3, pady=(8, 12))
        ttk.Button(buttons, text="Use for this session",
                   command=lambda: apply(False)).grid(row=0, column=0, padx=6)
        ttk.Button(buttons, text="Use and save",
                   command=lambda: apply(True)).grid(row=0, column=1, padx=6)
        ttk.Button(buttons, text="Cancel",
                   command=window.destroy).grid(row=0, column=2, padx=6)

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
        """Poll fast while something is moving, slowly when nothing is.

        Every poll costs one Modbus round trip per register per motor, so a
        single fixed rate is either too slow to watch a move or heavier traffic
        than a stationary machine deserves. Moving, it runs at
        `poll_interval_s`; idle, at `idle_poll_interval_s`. Temperature and bus
        voltage -- which move over minutes -- are refreshed only every
        `slow_poll_every` polls, so the fast path is two round trips lighter
        per motor.
        """
        # One poller at a time. A disconnect-then-connect used to start a
        # second thread while the first was still sleeping out its interval,
        # and from then on the motors were polled twice as often as configured.
        previous = self._poll_thread
        if previous is not None and previous.is_alive():
            self._poll_stop.set()
            previous.join(timeout=max(2.0, self.cfg.idle_poll_interval_s * 2))
        self._poll_stop.clear()

        def poll():
            tick = 0
            while not self._poll_stop.is_set():
                interval = self.cfg.idle_poll_interval_s
                try:
                    include_slow = (tick % max(1, self.cfg.slow_poll_every)) == 0
                    state = self.platform.read_state(include_slow=include_slow)
                    self.post(lambda s=state: self._apply_state(s))
                    interval = (self.cfg.poll_interval_s if state.moving
                                else self.cfg.idle_poll_interval_s)
                except Exception as exc:  # noqa: BLE001 - a poll must never die
                    self.log_threadsafe(f"Status poll error: {exc}")
                tick += 1
                self._poll_stop.wait(interval)

        self._poll_thread = threading.Thread(target=poll, name="poll", daemon=True)
        self._poll_thread.start()

    def _apply_state(self, state: PlatformState) -> None:
        self._last_state = state
        alarm = self.platform.pop_fall_alarm()
        if alarm:
            self._announce(alarm, COLOR_BAD)
        for status in state.motors:
            row = self.rows.get(status.name)
            if row:
                row.update(status)

        if state.orientation_valid and state.orientation:
            o = state.orientation
            shown = self._shown(o.focus_mm)
            if self._reference() == "zero":
                where = ("towards M1" if shown > 0
                         else "towards M2" if shown < 0 else "at zero")
            else:
                where = POSITION_REFERENCES[self._reference()].replace(
                    "stop", "stop = 0")
            self.focus_readout_var.set(
                f"focus  {shown:+9.4f} mm   ({shown * 1000:+.0f} um, {where})"
                + ("   [MOVING]" if state.moving else "")
            )
            mirrors = []
            if self.cfg.zero_to_m1_mm is not None:
                mirrors.append(f"to M1 {self.cfg.zero_to_m1_mm - o.focus_mm:.3f} mm")
            if self.cfg.zero_to_m2_mm is not None:
                mirrors.append(f"to M2 {self.cfg.zero_to_m2_mm + o.focus_mm:.3f} mm")
            self.mirror_var.set("\n".join(mirrors))
            self.orientation_var.set(
                f"tip {o.tip_deg:+8.5f} deg   tilt {o.tilt_deg:+8.5f} deg"
            )
            self.orientation_detail_var.set(
                f"total tilt {o.total_tilt_arcmin:.3f} arcmin "
                f"({o.total_tilt_arcsec:.1f} arcsec) towards azimuth "
                f"{o.tilt_azimuth_deg:.1f} deg"
            )
            # Only while something is moving. At rest the drive's command and
            # the encoder differ by the standing lag (about 1.4 um on these
            # motors, and settling leaves the command offset by it on purpose),
            # so comparing them kept the target drawn after arrival.
            target = None
            if state.all_connected and state.moving:
                try:
                    target = self.platform.geometry.orientation_from_actuators(
                        [m.target_mm for m in state.motors]).focus_mm
                except Exception:      # a UI hint, never worth an exception
                    target = None
            self.gauge.update_position(o.focus_mm, target, valid=True)
            if self.plane_view is not None:
                self.plane_view.label_offset_mm = self._offset()
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

        self._update_load_view(state)
        if (self._history_tree is not None
                and len(self.platform.history) != self._history_drawn):
            self._refresh_position_log()

        if state.any_error:
            self.conn_lamp.set(COLOR_BAD)
        elif state.all_connected:
            self.conn_lamp.set(COLOR_WARN if state.moving else COLOR_OK)
        self._apply_brake_summary(state)

    def _apply_brake_summary(self, state: PlatformState) -> None:
        if not self.platform.external_brake.available:
            self.brake_ctrl_var.set(self._brake_controller_text())
            self.brake_lamp.set(COLOR_IDLE)
            return
        self.brake_ctrl_var.set("Brakes: " + (state.brake_summary or "not read yet"))
        states = {m.brake.state for m in state.motors if not m.comms_error}
        if "NOT READABLE" in state.brake_summary:
            self.brake_lamp.set(COLOR_BAD)
        elif states == {BrakeState.ENGAGED}:
            self.brake_lamp.set(COLOR_BRAKE_ON)
        elif states == {BrakeState.RELEASED}:
            self.brake_lamp.set(COLOR_BRAKE_OFF)
        else:
            self.brake_lamp.set(COLOR_IDLE)

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
                f"still on, on purpose: the PLC did not confirm the brakes "
                f"engaged, and cutting power would leave the focal plane held "
                f"by nothing.",
                COLOR_STOP)
        self.post(lambda: self._set_busy(False))

    # ----------------------------------------------------------------- moves

    def _read_orientation_fields(self) -> Optional[Orientation]:
        """The boxes as an orientation. Go to is in the chosen position
        reference, and converted to the motors' zero here."""
        try:
            return Orientation(
                focus_mm=self._from_shown(float(self.focus_var.get())),
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
                self.focus_var.set(f"{self._shown(o.focus_mm):.4f}")
                self.tip_var.set(f"{o.tip_deg:.5f}")
                self.tilt_var.set(f"{o.tilt_deg:.5f}")
            self.post(apply)
            self.log_threadsafe("Copied the current orientation into the entry boxes.")

        self.run_async("Copy current", work)

    def on_preview(self) -> None:
        target = self._read_orientation_fields()
        if target is None:
            return
        lines = [f"Target: {self._describe(target)}", "", "Actuator targets:"]
        for name, mm in self.platform.preview(target).items():
            actuator = self.cfg.actuator(name)
            inside = actuator.min_travel_mm <= mm <= actuator.max_travel_mm
            lines.append(f"   {name}: {self._shown(mm):10.4f} mm"
                         + ("" if inside else "   OUT OF RANGE"))
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
        self._move_with_confirmation(
            target, kind="move", note="",
            reason="Move the focal plane to:")

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
            self.log_threadsafe(f"Fine adjusted {axis} by {sign * step:+g}.")

        self.run_async(f"Fine adjust {axis}", work)

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
            # Through the platform either way, so a row's button reaches the
            # PLC when that is where the brakes are. It used to go straight
            # to the motor's own brake output, which on this telescope is not
            # connected to anything, and every press failed.
            if name is None:
                results = self.platform.set_all_brakes(engaged=engage)
            else:
                results = self.platform.set_brake(name, engaged=engage)
            for target, result in results.items():
                label = "all brakes" if target == "all" else f"{target} brake"
                self.log_threadsafe(
                    f"{label}: {'engage' if engage else 'release'} -> {result}")

        self.run_async(f"Brake {who}", work)

    def on_enable_drives(self) -> None:
        """Turn every passive drive on, holding exactly where it is.

        The step before releasing the brakes, which the interlock refuses to
        do while any drive is passive. Nothing moves: each drive's target is
        set to where its shaft already is before it is enabled.
        """
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return

        def work():
            # The platform logs the drives it enabled; only the no-op needs
            # saying here, or a press would appear to do nothing.
            if not self.platform.enable_drives():
                self.log_threadsafe("All drives were already enabled and holding.")

        self.run_async("Enable drives", work)

    def _brakes_are_only_relay(self) -> bool:
        """True when the brake reading is the PLC's relay, not a sensor."""
        brake = self.platform.external_brake
        return bool(brake.available) and not brake.state_is_measured("all")

    def _confirm_relay_brakes(self) -> bool:
        """Ask once per session to accept the relay's word for "engaged".

        Nothing senses the brakes themselves, so "engaged" means the PLC was
        told to clamp. A person who knows the brakes work can accept that; the
        encoders are still watched after the drives go off, and the drives
        come straight back on if anything moves.
        """
        if not self._brakes_are_only_relay() or self.platform.trust_relay_brakes:
            return True
        if not messagebox.askyesno(
                "Rely on the brake relay?",
                "Nothing senses the brakes themselves: \"engaged\" means the PLC "
                "was told to clamp them.\n\nThe drives will only go off once the "
                "relay reads engaged, and for "
                f"{self.cfg.rest_watch_s:g} s afterwards the encoders are watched. "
                "If anything moves more than "
                f"{self.cfg.rest_sink_limit_mm:g} mm the drives come straight "
                "back on.\n\nRely on the relay for this session?"):
            return False
        self.platform.trust_relay_brakes = True
        self.log("Relying on the brake relay for this session (encoders watched "
                 "after the drives go off).")
        return True

    def on_disable_drives(self) -> None:
        """Brakes on, drives off, and check the brakes hold.

        So a parked focal plane is held by the brakes alone, with no small
        corrections from the motors. If the brakes cannot be confirmed, or
        anything moves once the drives are off, the drives stay (or come back)
        on and holding.
        """
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        if not self._confirm_relay_brakes():
            return

        def work():
            self.log_threadsafe(self.platform.rest_on_brakes(
                trust_relay=self.platform.trust_relay_brakes))

        self.run_async("Disable drives", work)

    def on_rest_after_moves_changed(self) -> None:
        wanted = bool(self.rest_after_var.get())
        if wanted and not self._confirm_relay_brakes():
            self.rest_after_var.set(False)
            return
        # For this session only: whether the relay can be relied on is asked
        # per session, so a saved "yes" would only fail quietly next time.
        self.platform.rest_after_moves = wanted
        self.log("After each move the drives will be turned off and the plane "
                 "left on the brakes." if wanted else
                 "After each move the drives stay on and holding.")

    # ---------------------------------------------------- focal plane picture

    def on_open_load_view(self) -> None:
        """How hard each motor is working, big enough to read across a room.

        The load bars in the actuator table are small by necessity -- they sit
        in a crowded row. This is the same numbers with space around them, for
        watching during a move or a calibration run, which is when they matter.

        It updates from the same poll as everything else, so it costs no extra
        traffic to the motors.
        """
        if self._load_window is not None and self._load_window.winfo_exists():
            self._load_window.lift()
            return

        window = tk.Toplevel(self.root)
        window.title("Load and torque")
        window.transient(self.root)
        self._load_window = window

        tk.Label(window, justify="left", anchor="w", wraplength=520, fg="#555",
                 text=("Torque as a percentage of each drive's current limit "
                       "(Actual Torque / CL: Current Max). These motors have "
                       "no register that reports amps, so this is the honest "
                       "measure of how hard they are working.")
                 ).grid(row=0, column=0, columnspan=6, sticky="w",
                        padx=12, pady=(12, 8))

        headers = ("motor", "load now", "", "peak", "temp", "supply")
        for column, text in enumerate(headers):
            if text:
                ttk.Label(window, text=text, foreground="#555",
                          font=("TkDefaultFont", 8)).grid(row=1, column=column,
                                                          padx=6)

        self._load_rows = {}
        for index, actuator in enumerate(self.cfg.actuators):
            row = 2 + index
            ttk.Label(window, text=actuator.name,
                      font=("TkDefaultFont", 13, "bold")).grid(
                row=row, column=0, sticky="w", padx=(12, 6), pady=6)

            now_var = tk.StringVar(value="--")
            ttk.Label(window, textvariable=now_var, width=7, anchor="e",
                      font=("TkFixedFont", 20, "bold")).grid(row=row, column=1,
                                                             padx=4)
            bar = LoadBar(window,
                          warn_percent=actuator.torque_warn_percent,
                          stall_percent=actuator.stall_torque_percent)
            # Resized through the bar, so its marks move with it. Setting the
            # size from outside left them where an 86-pixel bar has them.
            bar.resize(170, 20)
            bar.grid(row=row, column=2, padx=6)

            peak_var = tk.StringVar(value="--")
            ttk.Label(window, textvariable=peak_var, width=8, anchor="e",
                      font=("TkFixedFont", 10)).grid(row=row, column=3, padx=6)
            temp_var = tk.StringVar(value="--")
            ttk.Label(window, textvariable=temp_var, width=8, anchor="e",
                      font=("TkFixedFont", 10), foreground="#555").grid(
                row=row, column=4, padx=6)
            supply_var = tk.StringVar(value="--")
            ttk.Label(window, textvariable=supply_var, width=9, anchor="e",
                      font=("TkFixedFont", 10), foreground="#555").grid(
                row=row, column=5, padx=(6, 12))

            self._load_rows[actuator.name] = (now_var, bar, peak_var,
                                              temp_var, supply_var)

        footer = ttk.Frame(window)
        footer.grid(row=2 + len(self.cfg.actuators), column=0, columnspan=6,
                    sticky="ew", padx=12, pady=(4, 4))
        ttk.Label(footer, text="Readings per second:").grid(row=0, column=0,
                                                            sticky="w")
        self.rate_var = tk.StringVar(value=self._readings_per_second_text())
        rate = ttk.Combobox(footer, textvariable=self.rate_var, width=5,
                            values=READING_RATES, state="readonly")
        rate.grid(row=0, column=1, sticky="w", padx=(4, 12))
        rate.bind("<<ComboboxSelected>>",
                  lambda _event: self._set_readings_per_second(self.rate_var.get()))
        ttk.Button(footer, text="Reset peaks",
                   command=self._reset_load_peaks).grid(row=0, column=2, padx=4)

        # --- the torque limit, behind the password ---
        limits = ttk.LabelFrame(window, text="Torque limit (all three motors)")
        limits.grid(row=3 + len(self.cfg.actuators), column=0, columnspan=6,
                    sticky="ew", padx=12, pady=(6, 12))
        actuator = self.cfg.actuators[0]
        self.warn_limit_var = tk.StringVar(value=f"{actuator.torque_warn_percent:g}")
        self.stall_limit_var = tk.StringVar(value=f"{actuator.stall_torque_percent:g}")
        ttk.Label(limits, text="Amber above (%):").grid(row=0, column=0, sticky="e",
                                                       padx=(8, 4), pady=4)
        warn_entry = ttk.Entry(limits, textvariable=self.warn_limit_var, width=6,
                               state="disabled")
        warn_entry.grid(row=0, column=1, sticky="w")
        ttk.Label(limits, text="Stop a move above (%):").grid(
            row=0, column=2, sticky="e", padx=(16, 4))
        stall_entry = ttk.Entry(limits, textvariable=self.stall_limit_var, width=6,
                                state="disabled")
        stall_entry.grid(row=0, column=3, sticky="w")
        self._limit_entries = (warn_entry, stall_entry)
        self._limit_status_var = tk.StringVar(value=(
            f"A move is stopped when torque stays above the limit for "
            f"{actuator.stall_persist_samples} readings in a row. Changing "
            "these needs the password."))
        ttk.Label(limits, textvariable=self._limit_status_var, foreground="#777",
                  wraplength=520, justify="left").grid(
            row=1, column=0, columnspan=6, sticky="w", padx=8, pady=(2, 4))
        buttons = ttk.Frame(limits)
        buttons.grid(row=2, column=0, columnspan=6, sticky="w", padx=8, pady=(0, 8))
        self._unlock_limits_btn = ttk.Button(
            buttons, text="Change...",
            command=lambda: self._ask_password(
                "Changing the torque limit needs the password.",
                self._unlock_torque_limits))
        self._unlock_limits_btn.grid(row=0, column=0, padx=(0, 6))
        self._save_limits_btn = ttk.Button(buttons, text="Apply and save",
                                           command=self._apply_torque_limits,
                                           state="disabled")
        self._save_limits_btn.grid(row=0, column=1)

        def closed() -> None:
            self._load_window = None
            self._load_rows = {}
            self._limit_entries = ()
            window.destroy()

        window.protocol("WM_DELETE_WINDOW", closed)

    def _readings_per_second_text(self) -> str:
        return f"{1.0 / self.cfg.idle_poll_interval_s:g}"

    def _set_readings_per_second(self, text: str) -> None:
        """How often the motors are read while nothing is moving.

        While something moves they are read at least this often, and at
        least ten times a second. Saved, so it sticks between sessions.
        """
        try:
            rate = float(text)
        except ValueError:
            return
        if not 0.5 <= rate <= 20:
            return
        self.cfg.idle_poll_interval_s = 1.0 / rate
        self.cfg.poll_interval_s = min(0.1, 1.0 / rate)
        try:
            save_config(self.cfg, self.config_path)
        except Exception as exc:  # noqa: BLE001 -- the rate still applies
            self.log(f"Update rate set to {rate:g}/s for this session; not saved: {exc}")
            return
        self.log(f"Readings now {rate:g} per second while idle "
                 f"({1.0 / self.cfg.poll_interval_s:g} per second while moving).")

    def _unlock_torque_limits(self) -> None:
        if not self._limit_entries:
            return
        for entry in self._limit_entries:
            entry.configure(state="normal")
        self._save_limits_btn.configure(state="normal")
        self._unlock_limits_btn.configure(state="disabled")
        self._limit_status_var.set("Unlocked. Enter the new limits and press "
                                   "Apply and save.")

    def _apply_torque_limits(self) -> bool:
        """Set the warning and stop levels on all three motors, and save."""
        try:
            warn = float(self.warn_limit_var.get())
            stall = float(self.stall_limit_var.get())
        except ValueError:
            self._limit_status_var.set("Enter both limits as numbers, e.g. 30 and 45.")
            return False
        if not (0 < warn < stall <= 100):
            self._limit_status_var.set(
                "The amber level has to be above 0 and below the stop level, "
                "and the stop level at most 100%.")
            return False
        for actuator in self.cfg.actuators:
            actuator.torque_warn_percent = warn
            actuator.stall_torque_percent = stall
        for row in self.rows.values():
            row.load_bar.set_thresholds(warn, stall)
        for _now, bar, _peak, _temp, _supply in self._load_rows.values():
            bar.set_thresholds(warn, stall)
        try:
            path = save_config(self.cfg, self.config_path)
        except Exception as exc:  # noqa: BLE001 -- applied, but say it is not saved
            self._limit_status_var.set(f"Applied for this session, but not saved: {exc}")
            return False
        self.log(f"Torque limit changed: amber above {warn:g}%, a move is stopped "
                 f"above {stall:g}%. Saved to {path}.")
        self._limit_status_var.set(f"Saved: amber above {warn:g}%, stop above {stall:g}%.")
        for entry in self._limit_entries:
            entry.configure(state="disabled")
        self._save_limits_btn.configure(state="disabled")
        self._unlock_limits_btn.configure(state="normal")
        return True

    def _reset_load_peaks(self) -> None:
        for _now, bar, _peak, _temp, _supply in self._load_rows.values():
            bar.reset_peak()
        for row in self.rows.values():
            row.load_bar.reset_peak()

    def _update_load_view(self, state) -> None:
        """Fill the load window from the poll everybody else already used."""
        if not self._load_rows:
            return
        for status in state.motors:
            entry = self._load_rows.get(status.name)
            if entry is None:
                continue
            now_var, bar, peak_var, temp_var, supply_var = entry
            if status.comms_error or status.torque_percent is None:
                now_var.set("--")
                bar.set(None, "no reading")
                temp_var.set("--")
                supply_var.set("--")
                continue
            now_var.set(f"{status.torque_percent:.0f}%")
            label = (f"~{status.current_a:.2f} A" if status.current_a is not None
                     else f"{status.torque_percent:.1f}%")
            bar.set(status.torque_percent, label)
            peak_var.set(f"{bar._peak:.0f}%")
            temp_var.set("--" if status.temperature is None
                         else f"{status.temperature} C")
            if status.bus_voltage is None:
                supply_var.set("--")
            elif status.supply_volts is not None:
                supply_var.set(f"{status.supply_volts:.1f} V")
            else:
                supply_var.set(f"{status.bus_voltage} raw")

    def on_open_tilt(self) -> None:
        """Ask for the password, then open the focal plane window."""
        if self._tilt_window is not None and self._tilt_window.winfo_exists():
            self._tilt_window.lift()
            return
        self._ask_password("Tilt and jog controls are password protected.",
                           self._open_tilt_window)

    def _ask_password(self, message: str, on_success: Callable[[], None]) -> None:
        """Ask for the password, then run `on_success`.

        Asked in a small window of its own rather than a blocking dialog, so
        nothing else in the application stops while it is up. A wrong password
        is said in that window, not in a message box.
        """
        if self._password_window is not None and self._password_window.winfo_exists():
            self._password_window.destroy()
        self._password_action = on_success

        window = tk.Toplevel(self.root)
        self._password_window = window
        window.title("Password")
        window.transient(self.root)
        window.resizable(False, False)
        ttk.Label(window, text=message).grid(row=0, column=0, columnspan=2,
                                             sticky="w", padx=12, pady=(12, 6))
        ttk.Label(window, text="Password:").grid(row=1, column=0, sticky="e",
                                                 padx=(12, 4))
        password_var = tk.StringVar()
        entry = ttk.Entry(window, textvariable=password_var, show="•", width=22)
        entry.grid(row=1, column=1, sticky="w", padx=(0, 12))
        status_var = tk.StringVar(value="")
        ttk.Label(window, textvariable=status_var, foreground=COLOR_BAD).grid(
            row=2, column=0, columnspan=2, sticky="w", padx=12)

        def submit(_event=None):
            if self._submit_password(password_var.get()):
                return
            password_var.set("")
            status_var.set("Wrong password.")
            entry.focus_set()

        buttons = ttk.Frame(window)
        buttons.grid(row=3, column=0, columnspan=2, sticky="e", padx=12, pady=(6, 12))
        ttk.Button(buttons, text="Open", command=submit).grid(row=0, column=0, padx=4)
        ttk.Button(buttons, text="Cancel", command=window.destroy).grid(row=0, column=1)
        entry.bind("<Return>", submit)
        entry.focus_set()

        def forget(event=None):
            if (event is None or event.widget is window) \
                    and self._password_window is window:
                self._password_window = None
        window.bind("<Destroy>", forget)

    def _submit_password(self, text: str) -> bool:
        """Carry on with whatever asked, if `text` is the password."""
        if not tilt_password_matches(text):
            self.log("Wrong password.")
            return False
        action, self._password_action = self._password_action, None
        if self._password_window is not None and self._password_window.winfo_exists():
            self._password_window.destroy()
        self._password_window = None
        if action is not None:
            action()
        return True

    #: The name the tests and older code use.
    _submit_tilt_password = _submit_password

    def _open_tilt_window(self) -> None:
        """The focal plane window: the picture, tip/tilt and actuator jogs.

        Everything that changes the plane's ORIENTATION is here and nowhere
        else. Day to day this mechanism is a focus drive, so the main window
        only does focus; whoever needs to tilt the plane, or move a single
        actuator, opens this and sees what they are doing as they do it. The
        picture updates from the same poll as everything else, so it costs
        no extra traffic to the motors.
        """
        if self._tilt_window is not None and self._tilt_window.winfo_exists():
            self._tilt_window.lift()
            return

        window = tk.Toplevel(self.root)
        self._tilt_window = window
        window.title("Focal plane: tilt and jog"
                     + ("  [SIMULATION]" if self.simulate else ""))
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)

        # --- the picture, on the left ---
        picture = ttk.Frame(window)
        picture.grid(row=0, column=0, sticky="nsew", padx=(8, 4), pady=8)
        picture.columnconfigure(0, weight=1)
        picture.rowconfigure(0, weight=1)
        self.plane_view = FocalPlaneView(
            picture,
            points_xy_mm=[a.position_xy_mm for a in self.cfg.actuators],
            names=[a.name for a in self.cfg.actuators],
            focus_span_mm=max(abs(self.cfg.limits.min_focus_mm),
                              abs(self.cfg.limits.max_focus_mm)),
        )
        self.plane_view.grid(row=0, column=0, sticky="nsew")
        tk.Label(
            picture, justify="left", anchor="w", fg="#666", wraplength=450,
            text=("Dashed triangle: the zero plane. Solid: where the focal "
                  "plane is now. The orange posts are each actuator's "
                  "extension. Vertical travel is exaggerated by the factor "
                  "shown, since the plate is about a metre across and moves "
                  "millimetres."),
        ).grid(row=1, column=0, sticky="ew", pady=(6, 0))

        controls = ttk.Frame(window)
        controls.grid(row=0, column=1, sticky="n", padx=(4, 8), pady=8)

        # --- tip and tilt ---
        tilt = ttk.LabelFrame(controls, text="Tip and tilt")
        tilt.grid(row=0, column=0, sticky="ew")
        tk.Label(
            tilt, justify="left", anchor="w", wraplength=380, fg="#555",
            text=("These change the ORIENTATION of the focal plane, not its "
                  "focus. tip = rotation about the east-west axis, tilt = "
                  "rotation about the vertical axis. A tilt costs actuator "
                  "travel, so check Preview first."),
        ).grid(row=0, column=0, columnspan=6, sticky="w", padx=8, pady=(6, 6))

        ttk.Label(tilt, text="tip (deg)").grid(row=1, column=0, sticky="e",
                                               padx=(8, 2), pady=4)
        ttk.Entry(tilt, textvariable=self.tip_var,
                  width=9).grid(row=1, column=1, padx=2)
        ttk.Label(tilt, text="tilt (deg)").grid(row=1, column=2, sticky="e",
                                                padx=(10, 2))
        ttk.Entry(tilt, textvariable=self.tilt_var,
                  width=9).grid(row=1, column=3, padx=2)
        ttk.Button(tilt, text="Preview",
                   command=self.on_preview).grid(row=2, column=0, columnspan=2,
                                                 sticky="w", padx=8, pady=4)
        ttk.Button(tilt, text="Move",
                   command=self.on_move).grid(row=2, column=2, columnspan=2,
                                              sticky="w", padx=2, pady=4)

        fine = ttk.Frame(tilt)
        fine.grid(row=3, column=0, columnspan=6, sticky="w", padx=8, pady=(6, 4))
        ttk.Label(fine, text="Fine adjust by (deg):").grid(row=0, column=0,
                                                          padx=(0, 6))
        ttk.Entry(fine, textvariable=self.angle_step_var,
                  width=8).grid(row=0, column=1, padx=2)
        buttons = ttk.Frame(fine)
        buttons.grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))
        for column, (axis, sign, text) in enumerate((
                ("tip", -1, "tip −"), ("tip", +1, "tip +"),
                ("tilt", -1, "tilt −"), ("tilt", +1, "tilt +"))):
            ttk.Button(buttons, text=text, width=6,
                       command=lambda a=axis, s=sign: self.on_nudge(a, s)).grid(
                row=0, column=column, padx=(0 if column == 0 else 2,
                                            10 if column == 1 else 0))

        ttk.Button(tilt, text="Level (tip = tilt = 0)",
                   command=self.on_level).grid(row=4, column=0, columnspan=6,
                                               sticky="w", padx=8, pady=(6, 8))

        # --- one actuator at a time ---
        jog = ttk.LabelFrame(controls, text="Jog one actuator")
        jog.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        tk.Label(
            jog, justify="left", anchor="w", wraplength=380, fg="#555",
            text="Moves ONE actuator, which tilts the plane.",
        ).grid(row=0, column=0, columnspan=4, sticky="w", padx=8, pady=(6, 4))
        ttk.Label(jog, text="step (mm)").grid(row=1, column=0, sticky="e",
                                              padx=(8, 2), pady=(0, 4))
        ttk.Entry(jog, textvariable=self.jog_step_var, width=8).grid(
            row=1, column=1, sticky="w", padx=2, pady=(0, 4))
        for i, actuator in enumerate(self.cfg.actuators):
            name = actuator.name
            row = i + 2
            ttk.Label(jog, text=name, font=MotorRow.NAME_FONT).grid(
                row=row, column=0, sticky="w", padx=(8, 2), pady=2)
            # The main table's own variable, so the two can never disagree.
            ttk.Label(jog, textvariable=self.rows[name].position_var, width=13,
                      anchor="e", font=("TkFixedFont", 10)).grid(
                row=row, column=1, padx=2)
            ttk.Button(jog, text="▼", width=3,
                       command=lambda n=name: self.on_jog(n, -1)).grid(
                row=row, column=2, padx=(8, 1))
            ttk.Button(jog, text="▲", width=3,
                       command=lambda n=name: self.on_jog(n, +1)).grid(
                row=row, column=3, padx=(1, 8))
        ttk.Frame(jog, height=4).grid(row=len(self.cfg.actuators) + 2, column=0)

        ttk.Button(controls, text="Close", command=window.destroy).grid(
            row=2, column=0, sticky="e", pady=(10, 0))

        def forget(event=None):
            if event is not None and event.widget is not window:
                return
            self.plane_view = None
            self._tilt_window = None
        window.bind("<Destroy>", forget)

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
        window.title("Motion settings")
        window.transient(self.root)
        self._limits_window = window
        limits = self.cfg.limits

        tk.Label(window, justify="left", anchor="w", fg="#555", wraplength=520,
                 text=("Where 0 is, and what the software will refuse. The "
                       "limits and ends of travel below are in the motors' "
                       "own zero; the soft limits must sit INSIDE the ends of "
                       "travel, with margin.")
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
        found = ttk.LabelFrame(window, text="Ends of travel (hard stops)")
        found.grid(row=row, column=0, columnspan=3, sticky="ew",
                   padx=12, pady=(10, 4))

        # Found by Find hard stop, or entered here behind the password: they
        # are what the soft limits are checked against, so a wrong one is a
        # safety problem, not a typo.
        def shown(value):
            return "" if value is None else f"{value:+.4f}"

        self.stop_low_var = tk.StringVar(value=shown(limits.hard_stop_low_mm))
        self.stop_high_var = tk.StringVar(value=shown(limits.hard_stop_high_mm))
        stop_entries = []
        for r, (text, var) in enumerate((("lower end of travel (mm)", self.stop_low_var),
                                         ("upper end of travel (mm)", self.stop_high_var))):
            ttk.Label(found, text=text).grid(row=r, column=0, sticky="e", padx=8,
                                            pady=(6 if r == 0 else 2, 0))
            entry = ttk.Entry(found, textvariable=var, width=12, state="disabled")
            entry.grid(row=r, column=1, sticky="w", pady=(6 if r == 0 else 2, 0))
            stop_entries.append(entry)
        self._stop_entries = stop_entries
        stops_note = tk.StringVar(value="blank = not known. Changing these "
                                        "needs the password.")
        ttk.Label(found, textvariable=stops_note, foreground="#777",
                  font=("TkDefaultFont", 8)).grid(row=0, column=2, rowspan=2,
                                                  sticky="w", padx=8)

        def unlock_stops() -> None:
            for entry in stop_entries:
                entry.configure(state="normal")
            stops_note.set("Unlocked: edit, then Use for this session or "
                           "Use and save.")
            self._stops_unlocked = True
        self._stops_unlocked = False
        self._unlock_stops = unlock_stops
        ttk.Button(found, text="Change ends of travel...",
                   command=lambda: self._ask_password(
                       "Changing the ends of travel needs the password.",
                       unlock_stops)).grid(row=3, column=0, sticky="w",
                                           padx=8, pady=(4, 0))

        def read_stops():
            """(low, high) from the boxes; None for blank. Raises ValueError."""
            out = []
            for label, var in (("Lower end of travel", self.stop_low_var),
                               ("Upper end of travel", self.stop_high_var)):
                text = var.get().strip()
                if not text:
                    out.append(None)
                    continue
                try:
                    out.append(float(text))
                except ValueError:
                    raise ValueError(f"{label} must be a number, or blank.")
            low, high = out
            if low is not None and high is not None and not low < high:
                raise ValueError("The lower end of travel has to be below the upper.")
            return low, high

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
            try:
                low, high = read_stops()
            except ValueError as exc:
                messagebox.showerror("Check the number", str(exc), parent=window)
                return
            if low is None and high is None:
                messagebox.showwarning(
                    "Nothing known yet",
                    "Run Motion > Find hard stop in each direction first, or "
                    "enter the ends of travel.",
                    parent=window)
                return
            if low is not None:
                entries["min_focus_mm"].set(f"{low + margin:g}")
            if high is not None:
                entries["max_focus_mm"].set(f"{high - margin:g}")

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

            if self._stops_unlocked:
                try:
                    values["hard_stop_low_mm"], values["hard_stop_high_mm"] = read_stops()
                except ValueError as exc:
                    messagebox.showerror("Check the numbers", str(exc), parent=window)
                    return
            stops_changed = self._stops_unlocked and (
                values["hard_stop_low_mm"] != limits.hard_stop_low_mm
                or values["hard_stop_high_mm"] != limits.hard_stop_high_mm)

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
            reference = self.reference_choice_var.get()
            needed = {"top": candidate.hard_stop_high_mm,
                      "bottom": candidate.hard_stop_low_mm}.get(reference, 0.0)
            if needed is None:
                messagebox.showerror(
                    "That end of travel is not known",
                    f"To show positions from the {POSITION_REFERENCES[reference]}, "
                    "it has to be known: run Motion > Find hard stop, or enter "
                    "it under Ends of travel.", parent=window)
                return

            for attr, value in values.items():
                setattr(limits, attr, value)
            self._refresh_gauge_limits()
            self.log(f"Motion settings updated: focus "
                     f"{limits.min_focus_mm:+g} to {limits.max_focus_mm:+g} mm, "
                     f"max tilt {limits.max_tilt_deg:g} deg, max step "
                     f"{limits.max_step_mm:g} mm.")
            if stops_changed:
                def say(v):
                    return "not known" if v is None else f"{v:+.4f} mm"
                self.log(f"Ends of travel changed by hand: lower "
                         f"{say(limits.hard_stop_low_mm)}, upper "
                         f"{say(limits.hard_stop_high_mm)}.")
            if reference != self.cfg.position_reference:
                self._set_reference(reference)
            else:
                self._apply_reference()      # the stop it measures from may have moved
            if persist:
                path = save_config(self.cfg, self.config_path)
                self.log(f"Saved to {path}")
            window.destroy()
            self._limits_window = None

        # --- where 0 is, everywhere in the window ----------------------------
        where = ttk.LabelFrame(window, text="Show positions from")
        where.grid(row=row + 1, column=0, columnspan=3, sticky="ew",
                   padx=12, pady=(6, 4))
        self.reference_choice_var = tk.StringVar(value=self.cfg.position_reference)
        for column, (key, text) in enumerate((
                ("zero", "motor zero"),
                ("top", "top stop = 0  (below it is negative)"),
                ("bottom", "bottom stop = 0  (above it is positive)"))):
            ttk.Radiobutton(where, text=text, value=key,
                            variable=self.reference_choice_var).grid(
                row=0, column=column, sticky="w", padx=8, pady=(4, 2))
        ttk.Label(where, foreground="#777", font=("TkDefaultFont", 8),
                  wraplength=600, justify="left",
                  text=("Every position in the window (the focus readout, the "
                        "gauge, the actuators, Go to, saved positions, the "
                        "log) is then shown from there. A stop has to be known "
                        "(found, or entered above) to be used.")).grid(
            row=1, column=0, columnspan=3, sticky="w", padx=8, pady=(0, 6))

        buttons = ttk.Frame(window)
        buttons.grid(row=row + 2, column=0, columnspan=3, pady=(6, 12))
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
        window.title("Motor connections")
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
            self.log("Motor connections updated. Press Connect to use them.")
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
        trusted = self.platform.trust_relay_brakes
        rest_after = self.platform.rest_after_moves
        self.platform = self._make_platform()
        self.platform.shown_offset_mm = self._offset()
        self.platform.shown_from = self._ref_words()
        self.platform.trust_relay_brakes = trusted
        self.platform.rest_after_moves = rest_after
        self._refresh_position_log()
        self._refresh_saved_positions()
        self.connect_btn.config(text="Connect")
        self.conn_var.set("disconnected")
        self.conn_lamp.set(COLOR_IDLE)
        if was_connected:
            self.log("Disconnected because the addresses changed.")

    # ------------------------------------------------------ brake controller

    #: What the brake settings call each mode.
    BRAKE_MODE_LABELS = {
        "none": "not under software control",
        "controlbyweb": "ControlByWeb X-432 (web PLC)",
    }

    def on_edit_brake_controller(self) -> None:
        """Set up the PLC that switches the brakes, and watch its I/O live.

        Nothing here switches a relay. Working out which relay is which is
        done from the PLC's own web page, where somebody is looking at it on
        purpose; this window only reads, so it can be left open while the
        wiring is traced and every relay and input can be seen change.
        """
        settings = self.cfg.external_brake
        names = [a.name for a in self.cfg.actuators]
        window = tk.Toplevel(self.root)
        window.title("Brake controller (PLC)")
        window.transient(self.root)
        self._brake_window = window

        tk.Label(window, justify="left", anchor="w", fg="#555", wraplength=640,
                 text=("The focal-plane brakes are switched by a ControlByWeb "
                       "X-432, not by the motors. Enter its address and which "
                       "relay drives the brakes. 'Read the PLC' shows every "
                       "relay and input live, which is how to check the wiring: "
                       "switch a relay from the PLC's own web page and watch "
                       "which light changes here. This window never switches "
                       "anything itself.")
                 ).grid(row=0, column=0, columnspan=2, sticky="w", padx=12,
                        pady=(12, 8))

        # --- device --------------------------------------------------------
        device = ttk.LabelFrame(window, text="Device")
        device.grid(row=1, column=0, sticky="nsew", padx=(12, 6), pady=4)
        labels = dict(self.BRAKE_MODE_LABELS)
        if settings.mode not in labels:
            labels[settings.mode] = f"{settings.mode} (set in the configuration file)"
        mode_var = tk.StringVar(value=labels[settings.mode])
        ttk.Label(device, text="brakes are").grid(row=0, column=0, sticky="e",
                                                  padx=(8, 4), pady=3)
        ttk.Combobox(device, textvariable=mode_var, state="readonly", width=30,
                     values=list(labels.values())).grid(row=0, column=1,
                                                        columnspan=3, sticky="w")
        host_var = tk.StringVar(value=settings.host)
        port_var = tk.StringVar(value=str(settings.http_port))
        user_var = tk.StringVar(value=settings.username)
        pass_var = tk.StringVar(value=settings.password)
        https_var = tk.BooleanVar(value=settings.use_https)
        rows = [("IP address", host_var, 18, ""), ("port", port_var, 6, ""),
                ("user", user_var, 12, ""), ("password", pass_var, 12, "*")]
        for index, (label, var, width, show) in enumerate(rows, start=1):
            ttk.Label(device, text=label).grid(row=index, column=0, sticky="e",
                                               padx=(8, 4), pady=3)
            ttk.Entry(device, textvariable=var, width=width, show=show).grid(
                row=index, column=1, sticky="w")
        ttk.Checkbutton(device, text="HTTPS", variable=https_var).grid(
            row=2, column=2, sticky="w", padx=8)
        ttk.Label(device, text="(leave the password empty if the PLC has none)",
                  foreground="#777", font=("TkDefaultFont", 8)).grid(
            row=5, column=0, columnspan=4, sticky="w", padx=8, pady=(0, 6))

        # --- relays ----------------------------------------------------------
        wiring = ttk.LabelFrame(window, text="Which relay switches the brakes")
        wiring.grid(row=2, column=0, sticky="nsew", padx=(12, 6), pady=4)
        per_axis = bool(settings.relays) and "all" not in settings.relays
        relay_mode = tk.StringVar(value="each" if per_axis else "one")
        one_relay = tk.StringVar(value=str(settings.relays.get("all", 1))
                                 if not per_axis else "1")
        ttk.Radiobutton(wiring, text="one relay for all three:", value="one",
                        variable=relay_mode).grid(row=0, column=0, sticky="w",
                                                  padx=8, pady=3)
        ttk.Entry(wiring, textvariable=one_relay, width=5).grid(row=0, column=1,
                                                                sticky="w")
        ttk.Radiobutton(wiring, text="a relay for each actuator (separate):",
                        value="each", variable=relay_mode).grid(
            row=1, column=0, sticky="w", padx=8, pady=3)
        relay_vars = {}
        for index, name in enumerate(names):
            ttk.Label(wiring, text=name).grid(row=2 + index, column=0, sticky="e",
                                              padx=(8, 4))
            var = tk.StringVar(value=str(settings.relays.get(name, ""))
                               if per_axis else "")
            ttk.Entry(wiring, textvariable=var, width=5).grid(
                row=2 + index, column=1, sticky="w", pady=1)
            relay_vars[name] = var
        tk.Label(wiring, justify="left", anchor="w", fg="#555", wraplength=330,
                 font=("TkDefaultFont", 8),
                 text=("Separate: each row's Release/Engage switches only that "
                       "brake, and a jog releases only the brake of the "
                       "actuator it moves. A focus move still releases all "
                       "three.")).grid(row=5, column=0, columnspan=3, sticky="w",
                                       padx=8, pady=(2, 0))
        energized_var = tk.BooleanVar(value=settings.energized_releases)
        ttk.Checkbutton(
            wiring, variable=energized_var,
            text="relay ON releases the brakes"
        ).grid(row=6, column=0, columnspan=3, sticky="w", padx=8, pady=(6, 0))
        tk.Label(wiring, justify="left", anchor="w", fg="#555", wraplength=330,
                 font=("TkDefaultFont", 8),
                 text=("Leave ticked for fail-safe brakes (a dead-man's "
                       "arrangement): with no power they clamp, so powering "
                       "them through the relay is what releases them. Check it "
                       "once by hand: release from here, and the brake should "
                       "be free.")).grid(row=7, column=0, columnspan=3,
                                         sticky="w", padx=8, pady=(0, 6))

        # --- feedback --------------------------------------------------------
        feedback = ttk.LabelFrame(window, text="Brake feedback inputs (optional)")
        feedback.grid(row=3, column=0, sticky="nsew", padx=(12, 6), pady=4)
        tk.Label(feedback, justify="left", anchor="w", fg="#555", wraplength=300,
                 text=("Not needed: the PLC's relay reading is taken as the "
                       "brake state. If a switch on the brake is ever wired "
                       "to a PLC input, give it here and that reading is used "
                       "instead.")
                 ).grid(row=0, column=0, columnspan=3, sticky="w", padx=8, pady=4)
        fb = settings.feedback_inputs
        fb_mode = tk.StringVar(value=("none" if not fb else
                                      "one" if "all" in fb else "each"))
        fb_one = tk.StringVar(value=str(fb.get("all", "")))
        ttk.Radiobutton(feedback, text="none", value="none",
                        variable=fb_mode).grid(row=1, column=0, sticky="w", padx=8)
        ttk.Radiobutton(feedback, text="one input for all three:", value="one",
                        variable=fb_mode).grid(row=2, column=0, sticky="w", padx=8)
        ttk.Entry(feedback, textvariable=fb_one, width=5).grid(row=2, column=1,
                                                               sticky="w")
        ttk.Radiobutton(feedback, text="an input for each actuator:", value="each",
                        variable=fb_mode).grid(row=3, column=0, sticky="w", padx=8)
        fb_vars = {}
        for index, name in enumerate(names):
            ttk.Label(feedback, text=name).grid(row=4 + index, column=0,
                                                sticky="e", padx=(8, 4))
            var = tk.StringVar(value=str(fb.get(name, "")) if "all" not in fb else "")
            ttk.Entry(feedback, textvariable=var, width=5).grid(
                row=4 + index, column=1, sticky="w", pady=1)
            fb_vars[name] = var
        fb_released_var = tk.BooleanVar(value=settings.feedback_on_means_released)
        ttk.Checkbutton(feedback, variable=fb_released_var,
                        text="input ON means the brake is released").grid(
            row=7, column=0, columnspan=3, sticky="w", padx=8, pady=(4, 6))

        # --- live view -------------------------------------------------------
        live = ttk.LabelFrame(window, text="The PLC right now")
        live.grid(row=1, column=1, rowspan=3, sticky="nsew", padx=(6, 12), pady=4)
        lamps = {"relays": {}, "inputs": {}}
        for kind, count, column in (("relays", 16, 0), ("inputs", 18, 2)):
            ttk.Label(live, text="relay" if kind == "relays" else "input",
                      font=("TkDefaultFont", 8, "bold"), foreground="#555").grid(
                row=0, column=column, columnspan=2, pady=(6, 2))
            for n in range(1, count + 1):
                lamp = Lamp(live, diameter=11)
                lamp.grid(row=n, column=column, padx=(10, 2))
                text = tk.StringVar(value=str(n))
                ttk.Label(live, textvariable=text, font=("TkDefaultFont", 8),
                          width=12, anchor="w").grid(row=n, column=column + 1,
                                                     sticky="w")
                lamps[kind][n] = (lamp, text)
        live_var = tk.StringVar(value="Not read yet.")
        tk.Label(live, textvariable=live_var, justify="left", anchor="w",
                 wraplength=260, fg="#333").grid(row=20, column=0, columnspan=4,
                                                 sticky="w", padx=8, pady=(6, 4))
        keep_reading = tk.BooleanVar(value=False)
        buttons_live = ttk.Frame(live)
        buttons_live.grid(row=21, column=0, columnspan=4, sticky="w", padx=6,
                          pady=(0, 8))
        in_flight = [False]

        def candidate():
            """A settings object built from the fields, or an error string."""
            from dataclasses import replace
            chosen = next((k for k, v in labels.items() if v == mode_var.get()),
                          "none")

            def number(text, what):
                text = str(text).strip()
                if not text:
                    return None
                try:
                    value = int(text)
                except ValueError:
                    raise ValueError(f"{what} must be a whole number.")
                if value < 1:
                    raise ValueError(f"{what} must be 1 or more.")
                return value

            try:
                port = number(port_var.get(), "The port") or 80
                if relay_mode.get() == "one":
                    relays = {"all": number(one_relay.get(), "The relay")}
                    if relays["all"] is None:
                        relays = {}
                else:
                    relays = {}
                    for name, var in relay_vars.items():
                        value = number(var.get(), f"{name}'s relay")
                        if value is not None:
                            relays[name] = value
                    if relays and len(relays) != len(names):
                        raise ValueError("Give every actuator a relay, or use "
                                         "one relay for all three.")
                if fb_mode.get() == "none":
                    feedbacks = {}
                elif fb_mode.get() == "one":
                    value = number(fb_one.get(), "The feedback input")
                    feedbacks = {"all": value} if value else {}
                else:
                    feedbacks = {}
                    for name, var in fb_vars.items():
                        value = number(var.get(), f"{name}'s feedback input")
                        if value is not None:
                            feedbacks[name] = value
            except ValueError as exc:
                return None, str(exc)
            new = replace(settings, mode=chosen, host=host_var.get().strip(),
                          http_port=port, username=user_var.get().strip(),
                          password=pass_var.get(), use_https=bool(https_var.get()),
                          relays=relays, feedback_inputs=feedbacks,
                          # A relay per actuator means separate brakes: each
                          # can be switched on its own. One relay means one.
                          all_or_nothing=(relay_mode.get() == "one"),
                          energized_releases=bool(energized_var.get()),
                          feedback_on_means_released=bool(fb_released_var.get()))
            try:
                new.validate()
            except ValueError as exc:
                return None, str(exc)
            return new, ""

        def show(io, error):
            if not window.winfo_exists():
                return
            in_flight[0] = False
            new, _ = candidate()
            relays = new.relays if new else {}
            feedbacks = new.feedback_inputs if new else {}
            relay_names = {v: k for k, v in relays.items()}
            input_names = {v: k for k, v in feedbacks.items()}
            for kind, mapping in (("relays", relay_names), ("inputs", input_names)):
                for n, (lamp, text) in lamps[kind].items():
                    label = f"{n}"
                    if n in mapping:
                        who = mapping[n]
                        label += f"  {'brakes' if who == 'all' else who}"
                    text.set(label)
                    if io is None:
                        lamp.set(COLOR_IDLE)
                        continue
                    value = io[kind].get(n)
                    lamp.set(COLOR_IDLE if value is None else
                             COLOR_OK if value else "#ffffff")
            if error:
                live_var.set(f"Could not read the PLC: {error}")
            else:
                on_relays = [n for n, v in io["relays"].items() if v]
                on_inputs = [n for n, v in io["inputs"].items() if v]
                live_var.set(
                    f"Read at {time.strftime('%H:%M:%S')}. Relays on: "
                    f"{', '.join(map(str, on_relays)) or 'none'}. Inputs on: "
                    f"{', '.join(map(str, on_inputs)) or 'none'}. Green is on.")
            if keep_reading.get():
                window.after(1000, read_now)

        def read_now():
            if in_flight[0] or not window.winfo_exists():
                return
            new, problem = candidate()
            if new is None:
                live_var.set(problem)
                return
            if new.mode != "controlbyweb":
                live_var.set("Choose ControlByWeb X-432 above to read the PLC.")
                return
            in_flight[0] = True
            live_var.set(f"Reading {new.host}...")
            names_now = list(names)

            def work():
                try:
                    controller = BrakeController(
                        ExternalBrakeConfig.from_settings(new), names=names_now)
                    io, error = controller.read_io(fresh=True), ""
                except (BrakeError, ValueError) as exc:
                    io, error = None, str(exc)
                self.post(lambda: show(io, error))

            threading.Thread(target=work, name="plc-read", daemon=True).start()

        ttk.Button(buttons_live, text="Read the PLC",
                   command=read_now).grid(row=0, column=0, padx=(0, 8))
        ttk.Checkbutton(buttons_live, text="keep reading (every second)",
                        variable=keep_reading,
                        command=lambda: keep_reading.get() and read_now()).grid(
            row=0, column=1)

        # --- apply -----------------------------------------------------------
        def apply(persist: bool) -> None:
            new, problem = candidate()
            if new is None:
                messagebox.showerror("Check the brake settings", problem,
                                     parent=window)
                return
            from dataclasses import fields
            for f in fields(new):
                setattr(settings, f.name, getattr(new, f.name))
            try:
                self.cfg.validate()
            except ValueError as exc:
                messagebox.showerror("Check the brake settings", str(exc),
                                     parent=window)
                return
            self.platform.reload_external_brake()
            self.root.title(self._window_title())
            self.brake_ctrl_var.set(self._brake_controller_text())
            controller = self.platform.external_brake
            self.log(f"Brake controller: {controller.describe()}.")
            if (new.mode != "none"
                    and isinstance(controller, SimulatedBrakeController)):
                self.log("  This is a simulation, so the PLC is NOT being used. "
                         "Start with --real-brakes to switch it from a "
                         "simulation, or --bench to test it with one real motor.")
            if (new.mode == "controlbyweb" and not new.feedback_inputs
                    and not new.trust_relay_state):
                self.log("  No feedback inputs: the brake state shown is the "
                         "relay's, and EMERGENCY will keep the drives on.")
            if persist:
                self.log(f"Saved to {save_config(self.cfg, self.config_path)}")
            keep_reading.set(False)
            window.destroy()

        buttons = ttk.Frame(window)
        buttons.grid(row=4, column=0, columnspan=2, pady=(8, 12))
        ttk.Button(buttons, text="Use for this session",
                   command=lambda: apply(False)).grid(row=0, column=0, padx=6)
        ttk.Button(buttons, text="Use and save",
                   command=lambda: apply(True)).grid(row=0, column=1, padx=6)
        ttk.Button(buttons, text="Cancel",
                   command=lambda: (keep_reading.set(False), window.destroy())
                   ).grid(row=0, column=2, padx=6)
        self._brake_window_read = read_now

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
        """Adopt the end of travel just found as the corresponding limit.

        The soft limits ship as a guess; a hard stop is a measurement. So the
        limit becomes the stop, less the configured safety margin, rather than
        staying at whatever somebody typed before the travel was known. If the
        total travel is configured, the far end follows from it.
        """
        # stop_mm is where the travel actually ended. positions_mm is where
        # the actuators are *now*, which is half a millimetre short of it,
        # because the search backs off rather than leaving the mechanism
        # resting on its stop. Marking the parked position would put the end
        # of travel in the wrong place by exactly the back-off.
        ends = result.stop_mm or result.positions_mm
        focus_mm = sum(ends.values()) / len(ends)

        try:
            notes = self.platform.adopt_hard_stop(direction, focus_mm)
        except PlatformError as exc:
            messagebox.showerror("Those ends of travel will not do", str(exc))
            return

        self.log(f"End of travel recorded at {focus_mm:+.4f} mm.")
        for note in notes:
            self.log("  " + note)
        self._refresh_gauge_limits()

        limits = self.cfg.limits
        if messagebox.askyesno(
            "Save the new limits?",
            f"The {'upper' if direction > 0 else 'lower'} end of travel was "
            f"found at {focus_mm:+.4f} mm.\n\n"
            f"Focus limits are now "
            f"{limits.min_focus_mm:+.3f} to {limits.max_focus_mm:+.3f} mm.\n\n"
            "Save them to the configuration file?",
        ):
            try:
                self.log(f"Saved to {self.platform.save()}")
            except Exception as exc:  # noqa: BLE001
                messagebox.showerror("Could not save", str(exc))

    def on_open_position_log(self) -> None:
        """Where the focal plane has been, one line per move.

        The motors remember nothing, so this is the only record of previous
        positions there is. Each line says where the plane was before the
        move, where it was sent, where it ended up and whether the move
        finished -- and any line's "before" or "after" can be sent to the
        motors again, through exactly the checks an ordinary move gets.
        """
        if self._history_window is not None and self._history_window.winfo_exists():
            self._history_window.lift()
            return

        window = tk.Toplevel(self.root)
        window.title("Position log")
        self._history_window = window
        window.geometry("980x420")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(1, weight=1)

        tk.Label(window, justify="left", anchor="w", fg="#555", wraplength=940,
                 text=("Every move this software has commanded, newest first: "
                       "where the focal plane came from, where it was sent, and "
                       "the position it actually ended up at. Positions are "
                       "focus mm from zero, then tip and tilt in degrees. A "
                       "move that was halted is listed too, because its "
                       "position is where the plane really is.")
                 ).grid(row=0, column=0, sticky="ew", padx=12, pady=(10, 6))

        columns = ("when", "what", "from", "sent to", "position", "result", "note")
        tree = ttk.Treeview(window, columns=columns, show="headings",
                            selectmode="browse", height=12)
        widths = {"when": 130, "what": 68, "from": 180, "sent to": 180,
                  "position": 180, "result": 100, "note": 160}
        for column in columns:
            tree.heading(column, text=column)
            tree.column(column, width=widths[column], anchor="w",
                        stretch=(column == "note"))
        tree.grid(row=1, column=0, sticky="nsew", padx=(12, 0))
        scroll = ttk.Scrollbar(window, orient="vertical", command=tree.yview)
        scroll.grid(row=1, column=1, sticky="ns", padx=(0, 12))
        tree.configure(yscrollcommand=scroll.set)
        self._history_tree = tree

        buttons = ttk.Frame(window)
        buttons.grid(row=2, column=0, columnspan=2, sticky="w", padx=12,
                     pady=(8, 10))
        # One button: go to the position on the selected line, which is where
        # that move actually ended up. Double-clicking a line does the same.
        ttk.Button(buttons, text="Go to selected position",
                   command=self._go_to_selected).grid(row=0, column=0,
                                                      padx=(0, 8))
        ttk.Button(buttons, text="Go back one move",
                   command=self.on_go_back).grid(row=0, column=1, padx=4)
        ttk.Label(buttons, foreground="#777", font=("TkDefaultFont", 8),
                  text="  Double-click a line to go there. Every one of these "
                       "is an ordinary move: limits and step size are checked, "
                       "and you are asked to confirm.").grid(
            row=0, column=2, padx=(12, 0))
        tree.bind("<Double-1>", lambda _event: self._go_to_selected())

        self._history_path_var = tk.StringVar()
        ttk.Label(window, textvariable=self._history_path_var, foreground="#777",
                  font=("TkDefaultFont", 8)).grid(row=3, column=0, columnspan=2,
                                                  sticky="w", padx=12,
                                                  pady=(0, 8))

        def closed() -> None:
            self._history_window = None
            self._history_tree = None
            window.destroy()

        window.protocol("WM_DELETE_WINDOW", closed)
        self._refresh_position_log()

    def _orientation_cell(self, o: Optional[Orientation]) -> str:
        if o is None:
            return "--"
        focus = self._shown(o.focus_mm)
        if abs(o.tip_deg) < 5e-6 and abs(o.tilt_deg) < 5e-6:
            return f"{focus:+.4f} mm"
        return f"{focus:+.4f} mm  {o.tip_deg:+.4f}/{o.tilt_deg:+.4f}°"

    def _refresh_position_log(self) -> None:
        """Redraw the log window from the platform's history. UI thread."""
        tree = self._history_tree
        if tree is None or not tree.winfo_exists():
            return
        for item in tree.get_children():
            tree.delete(item)
        self._history_rows = self.platform.history.records(newest_first=True)
        self._history_drawn = len(self._history_rows)
        for index, record in enumerate(self._history_rows):
            tree.insert("", "end", iid=str(index), values=(
                record.when, record.kind,
                self._orientation_cell(record.before),
                self._orientation_cell(record.commanded),
                self._orientation_cell(record.after),
                "done" if record.completed else record.outcome,
                record.note,
            ))
        path = self.platform.history.path
        self._history_path_var.set(
            f"{len(self._history_rows)} move(s) on record"
            + (f", written to {path}" if path else ", this session only"))

    def _go_to_selected(self) -> None:
        """Go to the selected line's position: where that move ended up.

        For the position before a move, pick the line above it (the move
        before), or use Go back one move.
        """
        tree = self._history_tree
        if tree is None:
            return
        selected = tree.selection()
        if not selected:
            messagebox.showinfo("Nothing selected",
                                "Select a line in the log first.",
                                parent=self._history_window)
            return
        try:
            record = self._history_rows[int(selected[0])]
        except (ValueError, IndexError):
            return
        if record.after is None:
            messagebox.showwarning(
                "Not recorded",
                "That line has no recorded position: the motors could not be "
                "read at the time.", parent=self._history_window)
            return
        self._move_with_confirmation(
            record.after, kind="go-back",
            note=f"to where the {record.kind} at {record.when} ended",
            reason=f"Go to the position recorded after the {record.kind} "
                   f"at {record.when}:")

    def on_go_back(self) -> None:
        """Back to where the focal plane was before the most recent move."""
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        record = self.platform.history.last_with_before()
        if record is None:
            messagebox.showinfo(
                "Nothing to go back to",
                "No move has been recorded yet, so there is no previous "
                "position on record.")
            return
        self._move_with_confirmation(
            record.before, kind="go-back",
            note=f"back to before the {record.kind} at {record.when}",
            reason=f"Go back to where the focal plane was before the "
                   f"{record.kind} at {record.when}:")

    # ------------------------------------------------------- saved positions

    def _refresh_saved_positions(self) -> None:
        """Redraw everything that lists saved positions. UI thread."""
        names = self.platform.saved_positions.names()
        self.saved_combo.configure(values=names)
        if self.saved_choice_var.get() not in names:
            self.saved_choice_var.set(names[0] if names else "")
        tree = self._saved_tree
        if tree is None or not tree.winfo_exists():
            return
        selected = tree.selection()
        for item in tree.get_children():
            tree.delete(item)
        for position in self.platform.saved_positions.all():
            o = position.orientation
            tree.insert("", "end", iid=position.name, values=(
                position.name, f"{self._shown(o.focus_mm):+.4f}", f"{o.tip_deg:+.5f}",
                f"{o.tilt_deg:+.5f}", position.when, position.note))
        if selected and tree.exists(selected[0]):
            tree.selection_set(selected[0])
        self._show_saved_detail()

    def on_save_position(self) -> None:
        """Ask for a name, then remember where the focal plane is now."""
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        if self._save_dialog is not None and self._save_dialog.winfo_exists():
            self._save_dialog.lift()
            return
        try:
            current = self.platform.read_orientation()
        except Exception as exc:  # noqa: BLE001 -- say so, do not guess
            messagebox.showerror("Cannot save",
                                 f"The current position could not be read: {exc}")
            return

        window = tk.Toplevel(self.root)
        self._save_dialog = window
        window.title("Save current position")
        window.transient(self.root)
        window.resizable(False, False)

        ttk.Label(window, text="The focal plane is now at:").grid(
            row=0, column=0, columnspan=2, sticky="w", padx=12, pady=(12, 0))
        ttk.Label(window, text=self._describe(current), font=("TkFixedFont", 10)).grid(
            row=1, column=0, columnspan=2, sticky="w", padx=12, pady=(0, 8))

        form = ttk.Frame(window)
        form.grid(row=2, column=0, columnspan=2, sticky="w", padx=12)
        ttk.Label(form, text="Name:").grid(row=0, column=0, sticky="e",
                                           padx=(0, 4), pady=3)
        existing = self.platform.saved_positions.names()
        choices = list(existing) + [n for n in SUGGESTED_NAMES
                                    if n.casefold() not in
                                    {e.casefold() for e in existing}]
        name_var = tk.StringVar(value="" if existing else SUGGESTED_NAMES[0])
        name_box = ttk.Combobox(form, textvariable=name_var, values=choices,
                                width=28)
        name_box.grid(row=0, column=1, sticky="w", pady=3)
        ttk.Label(form, text="Note:").grid(row=1, column=0, sticky="e",
                                           padx=(0, 4), pady=3)
        note_var = tk.StringVar()
        ttk.Entry(form, textvariable=note_var, width=40).grid(
            row=1, column=1, sticky="w", pady=3)
        ttk.Label(window, foreground="#777", font=("TkDefaultFont", 8),
                  wraplength=380, justify="left",
                  text=("Pick a name from the list or type a new one. Saving "
                        "under a name that already exists replaces it.")).grid(
            row=4, column=0, columnspan=2, sticky="w", padx=12, pady=(2, 0))
        status_var = tk.StringVar()
        ttk.Label(window, textvariable=status_var, foreground=COLOR_BAD).grid(
            row=5, column=0, columnspan=2, sticky="w", padx=12)

        def save(_event=None):
            name = name_var.get().strip()
            if not name:
                status_var.set("Give it a name first.")
                return
            if self._save_position(name, note_var.get(), parent=window):
                window.destroy()

        buttons = ttk.Frame(window)
        buttons.grid(row=6, column=0, columnspan=2, sticky="e", padx=12,
                     pady=(6, 12))
        ttk.Button(buttons, text="Save", command=save).grid(row=0, column=0, padx=4)
        ttk.Button(buttons, text="Cancel", command=window.destroy).grid(row=0, column=1)
        name_box.bind("<Return>", save)
        name_box.focus_set()

    def _save_position(self, name: str, note: str = "", parent=None) -> bool:
        """Save the current position as `name`, asking before replacing one.
        True if it was saved. UI thread."""
        if self._busy or (self._last_state is not None and self._last_state.moving):
            messagebox.showwarning(
                "Still moving", "Wait for the move to finish, then save.",
                parent=parent or self.root)
            return False
        existing = self.platform.saved_positions.get(name)
        if existing is not None and not messagebox.askyesno(
                "Replace?",
                f"There is already a saved position called {existing.name!r}:\n\n"
                f"{existing.orientation.describe()}\n(saved {existing.when})\n\n"
                "Replace it with where the focal plane is now?",
                parent=parent or self.root):
            return False
        try:
            position, _replaced = self.platform.save_position(name, note)
        except Exception as exc:  # noqa: BLE001 -- reported to the operator
            messagebox.showerror("Not saved", str(exc), parent=parent or self.root)
            return False
        self.saved_choice_var.set(position.name)
        self._refresh_saved_positions()
        return True

    def _go_to_saved(self, name: str) -> None:
        """An ordinary checked, confirmed move to a saved position."""
        if not name:
            messagebox.showinfo(
                "No saved position",
                "Pick a saved position first, or save one with "
                "\"Save current as...\".")
            return
        position = self.platform.saved_positions.get(name)
        if position is None:
            messagebox.showwarning("Not found",
                                   f"There is no saved position called {name!r}.")
            return
        try:
            target = self.platform.saved_position_target(position)
        except PlatformError as exc:
            messagebox.showerror("Cannot go there", str(exc))
            return
        self._move_with_confirmation(
            target, kind="saved position", note=position.name,
            reason=f"Go to the saved position \"{position.name}\" "
                   f"(saved {position.when}):")

    def on_open_saved_positions(self) -> None:
        """Every saved position: its name, where it goes, and when it was saved."""
        if self._saved_window is not None and self._saved_window.winfo_exists():
            self._saved_window.lift()
            return

        window = tk.Toplevel(self.root)
        window.title("Saved positions")
        self._saved_window = window
        window.geometry("820x420")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(1, weight=1)

        tk.Label(window, justify="left", anchor="w", fg="#555", wraplength=780,
                 text=("Named positions, in the order they were first saved. "
                       "Focus is mm from zero, tip and tilt are degrees. Select "
                       "one to see exactly where it will send each actuator.")
                 ).grid(row=0, column=0, columnspan=2, sticky="ew", padx=12,
                        pady=(10, 6))

        columns = ("name", "focus (mm)", "tip (deg)", "tilt (deg)", "saved", "note")
        tree = ttk.Treeview(window, columns=columns, show="headings",
                            selectmode="browse", height=8)
        widths = {"name": 150, "focus (mm)": 90, "tip (deg)": 90,
                  "tilt (deg)": 90, "saved": 140, "note": 200}
        for column in columns:
            tree.heading(column, text=column)
            tree.column(column, width=widths[column], anchor="w",
                        stretch=(column == "note"))
        tree.grid(row=1, column=0, sticky="nsew", padx=(12, 0))
        scroll = ttk.Scrollbar(window, orient="vertical", command=tree.yview)
        scroll.grid(row=1, column=1, sticky="ns", padx=(0, 12))
        tree.configure(yscrollcommand=scroll.set)
        self._saved_tree = tree

        self._saved_detail_var = tk.StringVar(value="")
        ttk.Label(window, textvariable=self._saved_detail_var, justify="left",
                  font=("TkFixedFont", 9)).grid(row=2, column=0, columnspan=2,
                                                sticky="w", padx=12, pady=(8, 0))

        buttons = ttk.Frame(window)
        buttons.grid(row=3, column=0, columnspan=2, sticky="w", padx=12,
                     pady=(8, 12))
        ttk.Button(buttons, text="Go to selected",
                   command=lambda: self._go_to_saved(self._selected_saved())
                   ).grid(row=0, column=0, padx=(0, 12))
        ttk.Button(buttons, text="Save current as...",
                   command=self.on_save_position).grid(row=0, column=1, padx=2)
        ttk.Button(buttons, text="Rename...",
                   command=self._rename_saved).grid(row=0, column=2, padx=2)
        ttk.Button(buttons, text="Delete",
                   command=self._delete_saved).grid(row=0, column=3, padx=2)
        ttk.Button(buttons, text="Close", command=window.destroy).grid(
            row=0, column=4, padx=(24, 0))

        tree.bind("<<TreeviewSelect>>", lambda _event: self._show_saved_detail())
        tree.bind("<Double-1>",
                  lambda _event: self._go_to_saved(self._selected_saved()))

        def forget(event=None):
            if event is None or event.widget is window:
                self._saved_window = None
                self._saved_tree = None
        window.bind("<Destroy>", forget)
        self._refresh_saved_positions()

    def _selected_saved(self) -> str:
        tree = self._saved_tree
        if tree is None or not tree.winfo_exists():
            return ""
        selected = tree.selection()
        return selected[0] if selected else ""

    def _show_saved_detail(self) -> None:
        """Where the selected saved position will send each actuator."""
        if self._saved_tree is None:
            return
        position = self.platform.saved_positions.get(self._selected_saved())
        if position is None:
            self._saved_detail_var.set("Select a position to see where it goes.")
            return
        try:
            target = self.platform.saved_position_target(position)
        except PlatformError as exc:
            self._saved_detail_var.set(str(exc))
            return
        preview = self.platform.preview(target)
        lines = [f"Goes to:  {self._describe(target)}",
                 "          " + "   ".join(f"{n} {self._shown(mm):+.4f} mm"
                                           for n, mm in preview.items())]
        o = position.orientation
        if (abs(target.focus_mm - o.focus_mm) > 1e-4
                or abs(target.tip_deg - o.tip_deg) > 1e-6
                or abs(target.tilt_deg - o.tilt_deg) > 1e-6):
            lines.append("The zero has been set again since this was saved. It "
                         "still goes to the same physical place; the numbers "
                         "above are measured from the new zero.")
        if position.note:
            lines.append(f"Note:     {position.note}")
        self._saved_detail_var.set("\n".join(lines))

    def _rename_saved(self) -> None:
        name = self._selected_saved()
        if not name:
            messagebox.showinfo("Nothing selected", "Select a position first.",
                                parent=self._saved_window)
            return
        from tkinter import simpledialog
        new = simpledialog.askstring("Rename", f"New name for {name!r}:",
                                     initialvalue=name, parent=self._saved_window)
        if not new or new.strip() == name:
            return
        try:
            self.platform.saved_positions.rename(name, new)
        except (KeyError, ValueError) as exc:
            messagebox.showerror("Not renamed", str(exc), parent=self._saved_window)
            return
        self.log(f"Saved position {name!r} renamed to {new.strip()!r}.")
        self._refresh_saved_positions()

    def _delete_saved(self) -> None:
        name = self._selected_saved()
        if not name:
            messagebox.showinfo("Nothing selected", "Select a position first.",
                                parent=self._saved_window)
            return
        if not messagebox.askyesno("Delete?", f"Delete the saved position {name!r}?",
                                   parent=self._saved_window):
            return
        self.platform.saved_positions.delete(name)
        self.log(f"Saved position {name!r} deleted.")
        self._refresh_saved_positions()

    # -------------------------------------------------------- supply scale

    def on_set_supply_scale(self) -> None:
        """Turn the supply column from raw numbers into volts.

        The drive reports its supply in its own units. Tell it once what the
        supply is really at (MacTalk shows it, or use a meter) and every
        reading after that is shown in volts, scaled from that one pair.
        """
        if self._supply_window is not None and self._supply_window.winfo_exists():
            self._supply_window.lift()
            return
        window = tk.Toplevel(self.root)
        self._supply_window = window
        window.title("Supply voltage")
        window.transient(self.root)
        window.resizable(False, False)

        tk.Label(window, justify="left", anchor="w", fg="#555", wraplength=440,
                 text=("The drives report their supply in their own units. "
                       "They are shown in volts using the scale measured on "
                       "the pSCT motor: 1804 raw = 48.0 V, as MacTalk showed. "
                       "Only if a motor disagrees with MacTalk or a meter: "
                       "with the supply on, enter the voltage it is really at "
                       "and press Record. Each motor's reading right now is "
                       "stored beside that voltage and used for that motor "
                       "from then on.")).grid(
            row=0, column=0, columnspan=3, sticky="w", padx=12, pady=(12, 8))

        readings_var = tk.StringVar(value="")
        ttk.Label(window, textvariable=readings_var, justify="left",
                  font=("TkFixedFont", 10)).grid(row=1, column=0, columnspan=3,
                                                 sticky="w", padx=12)

        def current_raw() -> dict:
            state = self._last_state
            if state is None:
                return {}
            return {m.name: m.bus_voltage for m in state.motors
                    if not m.comms_error and m.bus_voltage}

        def show() -> None:
            raw = current_raw()
            lines = []
            for actuator in self.cfg.actuators:
                reading = raw.get(actuator.name)
                if reading is None:
                    lines.append(f"{actuator.name:<6} not read")
                    continue
                line = f"{actuator.name:<6} reads {reading:>6} raw"
                if actuator.supply_raw_at_nominal and actuator.supply_nominal_v:
                    volts = reading * actuator.supply_nominal_v / actuator.supply_raw_at_nominal
                    line += (f"  = {volts:.1f} V  (recorded {actuator.supply_raw_at_nominal}"
                             f" = {actuator.supply_nominal_v:g} V)")
                else:
                    volts = reading / actuator.supply_raw_per_volt
                    line += f"  = {volts:.1f} V  (measured scale)"
                lines.append(line)
            readings_var.set("\n".join(lines))
        show()

        form = ttk.Frame(window)
        form.grid(row=2, column=0, columnspan=3, sticky="w", padx=12, pady=(10, 4))
        ttk.Label(form, text="Supply is at (V):").grid(row=0, column=0, padx=(0, 4))
        volts_var = tk.StringVar(value="")
        ttk.Entry(form, textvariable=volts_var, width=10).grid(row=0, column=1)
        status_var = tk.StringVar(value="")
        status = ttk.Label(window, textvariable=status_var, foreground=COLOR_BAD)
        status.grid(row=3, column=0, columnspan=3, sticky="w", padx=12)

        def record() -> None:
            try:
                volts = float(volts_var.get())
            except ValueError:
                status.configure(foreground=COLOR_BAD)
                status_var.set("Enter the voltage as a number, e.g. 48.")
                return
            if not 5.0 <= volts <= 100.0:
                status.configure(foreground=COLOR_BAD)
                status_var.set("That is outside what these drives run on (5 to 100 V).")
                return
            raw = current_raw()
            if not raw:
                status.configure(foreground=COLOR_BAD)
                status_var.set("No motor has reported its supply yet. Connect first.")
                return
            done = []
            for actuator in self.cfg.actuators:
                if actuator.name in raw:
                    actuator.supply_nominal_v = volts
                    actuator.supply_raw_at_nominal = int(raw[actuator.name])
                    done.append(f"{actuator.name} {raw[actuator.name]} raw")
            try:
                path = save_config(self.cfg, self.config_path)
            except Exception as exc:  # noqa: BLE001 -- reported to the operator
                status.configure(foreground=COLOR_BAD)
                status_var.set(f"Recorded for this session, but not saved: {exc}")
                return
            self.log(f"Supply scale recorded at {volts:g} V: {', '.join(done)}. "
                     f"Saved to {path}.")
            status.configure(foreground=COLOR_OK)
            status_var.set(f"Recorded at {volts:g} V and saved.")
            show()

        buttons = ttk.Frame(window)
        buttons.grid(row=4, column=0, columnspan=3, sticky="e", padx=12, pady=(6, 12))
        ttk.Button(buttons, text="Record", command=record).grid(row=0, column=0, padx=4)
        ttk.Button(buttons, text="Close", command=window.destroy).grid(row=0, column=1)

        def forget(event=None):
            if event is None or event.widget is window:
                self._supply_window = None
        window.bind("<Destroy>", forget)

    def _move_with_confirmation(self, target: Orientation, kind: str,
                                note: str, reason: str) -> None:
        """The one path every absolute move takes: check, show, confirm, go.

        The check is made against where the plane is *now*, so a move that
        would be refused for its step size is refused here, in a dialog,
        rather than after the operator has confirmed it -- where the only
        trace of the refusal used to be a line in the log.
        """
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return
        try:
            current = self.platform.read_orientation()
        except Exception as exc:  # noqa: BLE001 -- refuse, do not guess
            messagebox.showerror("Cannot move",
                                 f"The current position could not be read: {exc}")
            return
        try:
            self.platform.check_orientation(target, current=current)
        except PlatformError as exc:
            messagebox.showerror("Move refused", str(exc))
            return
        preview = self.platform.preview(target)
        detail = "\n".join(f"   {n}: {self._shown(mm):10.4f} mm"
                            for n, mm in preview.items())
        if not messagebox.askyesno(
            "Confirm move",
            f"{reason}\n\n{self._describe(target)}\n\n"
            f"Actuator targets:\n{detail}\n\nProceed?",
        ):
            return

        def work():
            self.platform.move_to_orientation(target, kind=kind, note=note)
            self.log_threadsafe("Move complete.")

        self.run_async({"move": "Move", "go-back": "Go back",
                        "saved position": f"Go to {note}"}.get(kind, "Move"), work)

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
         bench: Optional[str] = None, sim_speed: Optional[float] = None,
         poll_interval_s: Optional[float] = None,
         use_real_brakes: bool = False) -> int:
    root = tk.Tk()
    MotorApp(root, config_path=config_path, simulate=simulate, bench=bench,
             sim_speed=sim_speed, poll_interval_s=poll_interval_s,
             use_real_brakes=use_real_brakes)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
