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
import traceback
from datetime import datetime
from typing import Callable, Optional

import tkinter as tk
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


class Lamp(tk.Canvas):
    """A small coloured indicator light."""

    def __init__(self, parent, diameter: int = 16, **kw):
        super().__init__(parent, width=diameter + 2, height=diameter + 2,
                         highlightthickness=0, **kw)
        self._circle = self.create_oval(2, 2, diameter, diameter,
                                        fill=COLOR_IDLE, outline="#404040")

    def set(self, color: str) -> None:
        self.itemconfigure(self._circle, fill=color)


class MotorRow:
    """One actuator's line in the actuator table."""

    def __init__(self, parent, name: str, row: int, app: "MotorApp"):
        self.name = name
        self.app = app

        ttk.Label(parent, text=name, width=4,
                  font=("TkDefaultFont", 11, "bold")).grid(row=row, column=0, padx=(6, 2))

        self.position_var = tk.StringVar(value="--")
        ttk.Label(parent, textvariable=self.position_var, width=14, anchor="e",
                  font=("TkFixedFont", 10)).grid(row=row, column=1, padx=2)

        self.counts_var = tk.StringVar(value="--")
        ttk.Label(parent, textvariable=self.counts_var, width=13, anchor="e",
                  font=("TkFixedFont", 9), foreground="#555").grid(row=row, column=2, padx=2)

        self.mode_lamp = Lamp(parent)
        self.mode_lamp.grid(row=row, column=3, padx=(8, 2))
        self.mode_var = tk.StringVar(value="--")
        ttk.Label(parent, textvariable=self.mode_var, width=16,
                  anchor="w").grid(row=row, column=4, padx=2)

        self.brake_lamp = Lamp(parent)
        self.brake_lamp.grid(row=row, column=5, padx=(8, 2))
        self.brake_var = tk.StringVar(value="unknown")
        ttk.Label(parent, textvariable=self.brake_var, width=15,
                  anchor="w").grid(row=row, column=6, padx=2)

        self.release_btn = ttk.Button(
            parent, text="Release", width=8,
            command=lambda: app.on_brake(name, engage=False))
        self.release_btn.grid(row=row, column=7, padx=2)
        self.engage_btn = ttk.Button(
            parent, text="Engage", width=8,
            command=lambda: app.on_brake(name, engage=True))
        self.engage_btn.grid(row=row, column=8, padx=2)

        ttk.Button(parent, text="▼", width=3,
                   command=lambda: app.on_jog(name, -1)).grid(row=row, column=9, padx=(10, 1))
        ttk.Button(parent, text="▲", width=3,
                   command=lambda: app.on_jog(name, +1)).grid(row=row, column=10, padx=1)

        self.error_var = tk.StringVar(value="")
        ttk.Label(parent, textvariable=self.error_var, foreground=COLOR_BAD,
                  anchor="w").grid(row=row, column=11, padx=(10, 6), sticky="w")

    def update(self, status) -> None:
        if status.comms_error:
            self.position_var.set("--")
            self.counts_var.set("")
            self.mode_var.set("no comms")
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
        label = brake.state.value + (" (inferred)" if brake.inferred else "")
        self.brake_var.set(label)
        if brake.state is BrakeState.ENGAGED:
            self.brake_lamp.set(COLOR_BAD)
        elif brake.state is BrakeState.RELEASED:
            self.brake_lamp.set(COLOR_OK)
        else:
            self.brake_lamp.set(COLOR_IDLE)

        self.error_var.set(status.error_text if status.error_bits else "")

    def set_brake_controls_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        self.release_btn.config(state=state)
        self.engage_btn.config(state=state)


class MotorApp:
    def __init__(self, root: tk.Tk, config_path: Optional[str] = None,
                 simulate: bool = False):
        self.root = root
        self.simulate = simulate
        self.config_path = config_path
        self.root.title(
            "pSCT Focal Plane Control" + ("  [SIMULATION]" if simulate else "")
        )

        self.cfg = load_config(config_path)
        self.platform = FocalPlanePlatform(
            cfg=self.cfg, simulate=simulate, logger=self.log_threadsafe,
            config_path=config_path,
        )

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
        tools.add_command(label="Find hard stop (calibration)...",
                          command=self.on_find_hard_stop)
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
            bar, text="EMERGENCY\nbrakes on, drives off", command=self.on_passivate,
            bg="#3a3a3a", fg="white", activebackground="#111", activeforeground="white",
            font=("TkDefaultFont", 9, "bold"), height=2,
        ).grid(row=0, column=1, sticky="ew", padx=4, pady=4)

        tk.Label(
            bar,
            text="STOP decelerates and holds position with the drives still on. "
                 "EMERGENCY cuts drive power -- the load is then held by the brakes alone.",
            bg=COLOR_STOP_DARK, fg="#ffd7d7", font=("TkDefaultFont", 8),
        ).grid(row=1, column=0, columnspan=2, sticky="w", padx=8, pady=(0, 4))

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
        self.plane_view = None

    def _build_actuators(self, parent) -> None:
        frame = ttk.LabelFrame(parent, text="Actuators")
        frame.grid(row=1, column=0, sticky="nsew", pady=3)

        headers = ["", "position", "counts", "", "mode", "", "brake",
                   "", "", "", "jog", ""]
        for col, text in enumerate(headers):
            if text:
                ttk.Label(frame, text=text, foreground="#555",
                          font=("TkDefaultFont", 8)).grid(row=0, column=col, padx=2)

        self.rows = {}
        for i, actuator in enumerate(self.cfg.actuators):
            self.rows[actuator.name] = MotorRow(frame, actuator.name, i + 1, self)

        footer = ttk.Frame(frame)
        footer.grid(row=len(self.cfg.actuators) + 1, column=0, columnspan=12,
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

    def on_stop(self) -> None:
        """Always runs, busy or not, on its own thread."""
        if not self.platform.connected:
            self.log("STOP pressed, but nothing is connected.")
            return
        self.log("STOP pressed.")
        threading.Thread(target=self._stop_worker, name="stop", daemon=True).start()

    def _stop_worker(self) -> None:
        try:
            self.platform.stop()
        except Exception as exc:  # noqa: BLE001
            self.log_threadsafe(f"STOP had trouble: {exc}")
        finally:
            self.post(lambda: self._set_busy(False))

    def on_passivate(self) -> None:
        if not self.platform.connected:
            self.log("EMERGENCY pressed, but nothing is connected.")
            return
        if not messagebox.askyesno(
            "Turn the drives off?",
            "This engages the brakes and removes drive power from all three "
            "motors.\n\nWith the drives off the motors hold nothing: the load "
            "rests on the brakes and screw friction alone.\n\nIf you just want "
            "motion to stop, use STOP instead -- it holds position under power."
            "\n\nTurn the drives off?",
        ):
            return
        self.log("EMERGENCY passivate pressed.")
        threading.Thread(target=self._passivate_worker, name="passivate",
                         daemon=True).start()

    def _passivate_worker(self) -> None:
        try:
            self.platform.emergency_passivate()
        except Exception as exc:  # noqa: BLE001
            self.log_threadsafe(f"Passivate had trouble: {exc}")
        finally:
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
        """Drive one actuator until it physically stops.

        This is the site's calibration procedure -- run the actuator out to its
        end -- done under torque supervision so it stops when something
        resists rather than continuing to push.
        """
        if not self.platform.connected:
            messagebox.showwarning("Not connected", "Connect first.")
            return

        window = tk.Toplevel(self.root)
        window.title("Find hard stop")
        window.transient(self.root)

        tk.Label(
            window, justify="left", anchor="w", wraplength=520,
            text=("Drives ONE actuator until it will not go further, then backs "
                  "the command off so it is not left pressed against the stop.\n\n"
                  "It walks out in small steps and watches the motor's torque. "
                  "If torque passes the configured limit, or a step barely "
                  "moves, that is the stop.\n\n"
                  "This moves one actuator on its own, which tilts the focal "
                  "plane. Use it for calibration, not for observing."),
            fg="#333",
        ).grid(row=0, column=0, columnspan=4, sticky="w", padx=12, pady=(12, 8))

        ttk.Label(window, text="Actuator").grid(row=1, column=0, sticky="e",
                                                padx=(12, 4), pady=4)
        motor_var = tk.StringVar(value=self.cfg.actuators[0].name)
        ttk.OptionMenu(window, motor_var, motor_var.get(),
                       *[a.name for a in self.cfg.actuators]).grid(
            row=1, column=1, sticky="w", padx=4)

        ttk.Label(window, text="Direction").grid(row=1, column=2, sticky="e", padx=(12, 4))
        direction_var = tk.StringVar(value="+  towards M1")
        ttk.OptionMenu(window, direction_var, direction_var.get(),
                       "+  towards M1", "−  towards M2").grid(
            row=1, column=3, sticky="w", padx=(0, 12))

        ttk.Label(window, text="Step (mm)").grid(row=2, column=0, sticky="e",
                                                 padx=(12, 4), pady=4)
        step_var = tk.StringVar(value="0.20")
        ttk.Entry(window, textvariable=step_var, width=10).grid(row=2, column=1,
                                                                sticky="w", padx=4)
        ttk.Label(window, text="Give up after (mm)").grid(row=2, column=2, sticky="e",
                                                          padx=(12, 4))
        budget_var = tk.StringVar(value="30.0")
        ttk.Entry(window, textvariable=budget_var, width=10).grid(row=2, column=3,
                                                                  sticky="w", padx=(0, 12))

        actuator = self.cfg.actuator(motor_var.get())
        ttk.Label(window,
                  text=(f"Torque limit {actuator.stall_torque_percent:.0f}% "
                        f"of the drive's current limit, over "
                        f"{actuator.stall_persist_samples} consecutive readings."),
                  foreground="#777").grid(row=3, column=0, columnspan=4,
                                          sticky="w", padx=12, pady=(4, 8))

        def start() -> None:
            try:
                step_mm = float(step_var.get())
                budget_mm = float(budget_var.get())
            except ValueError:
                messagebox.showerror("Check the numbers",
                                     "Step and budget must be numbers.",
                                     parent=window)
                return
            name = motor_var.get()
            direction = 1 if direction_var.get().startswith("+") else -1
            if not messagebox.askyesno(
                "Run into the end stop?",
                f"{name} will be driven {direction_var.get().strip()} until it "
                f"stops, up to {budget_mm} mm.\n\nThis moves one actuator "
                "alone, which tilts the focal plane.\n\nProceed?",
                parent=window,
            ):
                return
            window.destroy()
            self._run_hard_stop(name, direction, step_mm, budget_mm)

        buttons = ttk.Frame(window)
        buttons.grid(row=4, column=0, columnspan=4, pady=(6, 12))
        ttk.Button(buttons, text="Find the stop",
                   command=start).grid(row=0, column=0, padx=6)
        ttk.Button(buttons, text="Cancel",
                   command=window.destroy).grid(row=0, column=1, padx=6)

    def _run_hard_stop(self, name: str, direction: int,
                       step_mm: float, budget_mm: float) -> None:
        def work():
            motor = self.platform.motor(name)
            scale = motor.cfg.resolved_counts_per_mm
            step_counts = max(1, int(round(step_mm * scale)))
            budget_counts = max(step_counts, int(round(budget_mm * scale)))
            self.log_threadsafe(
                f"{name}: searching for the hard stop, {direction:+d} direction, "
                f"{step_counts} counts per step, up to {budget_counts} counts."
            )

            def progress(counts, torque):
                self.log_threadsafe(
                    f"{name}: at {counts} counts, peak torque {torque:.0f}%")

            stop_counts = motor.seek_hard_stop(
                direction=direction, step_counts=step_counts,
                max_counts=budget_counts, progress=progress,
            )
            self.log_threadsafe(
                f"{name}: hard stop at {stop_counts} counts "
                f"({motor.cfg.counts_to_mm(stop_counts):+.4f} mm on the current "
                "zero). Use Motion > Set zero here if this is your reference, "
                "and narrow the travel limits in the config to keep moves "
                "inside it."
            )

        self.run_async(f"Find hard stop ({name})", work)

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


def main(config_path: Optional[str] = None, simulate: bool = False) -> int:
    root = tk.Tk()
    MotorApp(root, config_path=config_path, simulate=simulate)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
