"""
Single-motor bench GUI: watch one motor, break it on purpose, see why.

    python -m psct_motors.cli motor-gui --motor A
    python -m psct_motors.cli motor-gui --simulate     # no hardware needed

Built for one motor on a bench, so everything is in **counts, revolutions and
degrees of shaft** -- no millimetres, no kinematics, no calibration required.

What is on screen
-----------------
  STOP              always live, always first
  Connection        connect / reconnect, and how long a failed read can block
  Live state        position, target, mode, velocity, brake -- and errors,
                    large and red, with a Clear button
  Move              absolute and relative moves in revolutions
  Diagnose          "why is it not moving?" -- runs the checklist in
                    diagnostics.py against the live motor and explains
  Inject a fault    arm any of the simulated faults against the live link, to
                    prove the error handling works
  Event log         timestamped, colour-coded, recording every change rather
                    than every poll, written to a file as it happens

Why the event log matters
-------------------------
The hard failures are the ones you notice minutes after they started. The log
records mode changes, error bits appearing and clearing, V_SOLL being
overwritten, the target changing, the link dropping, and any transaction that
took longer than a threshold -- each with a timestamp. When the motor stops
taking commands, the answer is usually already in the log, above the point
where you noticed.

Threading
---------
Same rules as gui.py: no worker thread touches tkinter. Workers push closures
onto a queue that a repeating UI-thread job drains. `post()` is the only way
back.
"""

from __future__ import annotations

import queue
import threading
import traceback
from datetime import datetime
from typing import Callable, Optional

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .config import load_config
from .diagnostics import BLOCKING, OK, SUSPECT, UNKNOWN, diagnose
from .eventlog import DEBUG, ERROR, INFO, WARNING, EventLog, MotorWatcher, default_log_path
from .faults import Fault, wrap_motor
from .jvl_motor import BrakeState, JVLMotor, MotorFault
from .registers import MotorMode, describe_errors, describe_mode
from .transport import ModbusError

COLOR_OK = "#1b8a3a"
COLOR_WARN = "#c77700"
COLOR_BAD = "#b3231f"
COLOR_IDLE = "#8a8a8a"
COLOR_STOP = "#c62828"
COLOR_STOP_DARK = "#7a0000"
COLOR_ERROR_BG = "#ffe4e4"
COLOR_OK_BG = "#e8f5e9"

SEVERITY_COLOR = {
    DEBUG: "#777777",
    INFO: "#111111",
    WARNING: COLOR_WARN,
    ERROR: COLOR_BAD,
}


class Lamp(tk.Canvas):
    def __init__(self, parent, diameter: int = 16, **kw):
        super().__init__(parent, width=diameter + 2, height=diameter + 2,
                         highlightthickness=0, **kw)
        self._circle = self.create_oval(2, 2, diameter, diameter,
                                        fill=COLOR_IDLE, outline="#404040")

    def set(self, color: str) -> None:
        self.itemconfigure(self._circle, fill=color)


class SingleMotorApp:
    def __init__(self, root: tk.Tk, motor_name: str = "Top",
                 config_path: Optional[str] = None, simulate: bool = False,
                 log_path: Optional[str] = None):
        self.root = root
        self.simulate = simulate
        self.motor_name = motor_name
        self.root.title(
            f"pSCT Motor {motor_name} -- bench control"
            + ("  [SIMULATION]" if simulate else "")
        )

        self.cfg = load_config(config_path)
        actuator = self.cfg.actuator(motor_name)
        self.counts_per_rev = float(actuator.counts_per_rev)

        if simulate:
            from .simulator import simulated_motor
            self.motor: JVLMotor = simulated_motor(actuator)
        else:
            self.motor = JVLMotor(actuator, timeout_s=self.cfg.modbus_timeout_s,
                                  retries=self.cfg.modbus_retries)

        self.log_path = log_path or default_log_path(motor_name)
        self.log = EventLog(path=self.log_path)
        self.injector = wrap_motor(self.motor)
        self.watcher = MotorWatcher(self.motor, self.log,
                                    interval_s=self.cfg.poll_interval_s)

        self._busy = False
        self._ui_queue: "queue.Queue[Callable[[], None]]" = queue.Queue()
        self._ui_job: Optional[str] = None
        self._log_cursor = 0

        self._build_ui()
        self._drain_ui()
        self.log.info("session", f"Bench GUI opened for motor {motor_name}"
                                 + (" (simulated)" if simulate else ""))
        self.log.info("session", f"Recording to {self.log_path}")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------------------------------------------------------- units

    def revs(self, counts: float) -> float:
        return counts / self.counts_per_rev

    def counts(self, revs: float) -> int:
        return int(round(revs * self.counts_per_rev))

    # ------------------------------------------------------------------- UI

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(5, weight=1)
        self._build_stop_bar()
        self._build_connection()
        self._build_state()
        self._build_controls()
        self._build_faults()
        self._build_log()

    def _build_stop_bar(self) -> None:
        bar = tk.Frame(self.root, bg=COLOR_STOP_DARK)
        bar.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 2))
        bar.columnconfigure(0, weight=3)
        bar.columnconfigure(1, weight=1)
        tk.Button(bar, text="STOP", command=self.on_stop,
                  bg=COLOR_STOP, fg="white", activebackground=COLOR_STOP_DARK,
                  activeforeground="white",
                  font=("TkDefaultFont", 15, "bold"), height=2
                  ).grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        tk.Button(bar, text="PASSIVE\ndrive off", command=self.on_passivate,
                  bg="#3a3a3a", fg="white", activebackground="#111",
                  activeforeground="white",
                  font=("TkDefaultFont", 9, "bold"), height=2
                  ).grid(row=0, column=1, sticky="ew", padx=4, pady=4)

    def _build_connection(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Connection")
        frame.grid(row=1, column=0, sticky="ew", padx=6, pady=3)

        self.connect_btn = ttk.Button(frame, text="Connect", command=self.on_connect)
        self.connect_btn.grid(row=0, column=0, padx=6, pady=6)
        ttk.Button(frame, text="Reconnect",
                   command=self.on_reconnect).grid(row=0, column=1, padx=4)

        self.conn_lamp = Lamp(frame)
        self.conn_lamp.grid(row=0, column=2, padx=(10, 2))
        self.conn_var = tk.StringVar(value="disconnected")
        ttk.Label(frame, textvariable=self.conn_var, width=22).grid(row=0, column=3)

        self.watch_btn = ttk.Button(frame, text="Start recording",
                                    command=self.on_toggle_watch)
        self.watch_btn.grid(row=0, column=4, padx=(14, 4))

        transport = getattr(self.injector, "inner", None)
        worst = getattr(transport, "worst_case_transaction_s", None)
        detail = f"{self.motor.cfg.ip}:{self.motor.cfg.port}"
        if worst:
            detail += f"   a failed read blocks at most {worst:g}s"
        ttk.Label(frame, text=detail, foreground="#555").grid(
            row=0, column=5, padx=10, sticky="w")

    def _build_state(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Live state")
        frame.grid(row=2, column=0, sticky="ew", padx=6, pady=3)
        frame.columnconfigure(7, weight=1)

        self.position_var = tk.StringVar(value="--")
        ttk.Label(frame, text="Position").grid(row=0, column=0, sticky="e", padx=(8, 2))
        ttk.Label(frame, textvariable=self.position_var, width=34, anchor="w",
                  font=("TkFixedFont", 12, "bold")).grid(row=0, column=1,
                                                         columnspan=3, sticky="w")

        self.target_var = tk.StringVar(value="--")
        ttk.Label(frame, text="Target").grid(row=1, column=0, sticky="e", padx=(8, 2))
        ttk.Label(frame, textvariable=self.target_var, width=34, anchor="w",
                  font=("TkFixedFont", 10)).grid(row=1, column=1, columnspan=3,
                                                 sticky="w")

        self.mode_lamp = Lamp(frame)
        self.mode_lamp.grid(row=0, column=4, padx=(16, 2))
        self.mode_var = tk.StringVar(value="--")
        ttk.Label(frame, text="Mode").grid(row=0, column=5, sticky="e")
        ttk.Label(frame, textvariable=self.mode_var, width=18,
                  anchor="w").grid(row=0, column=6, sticky="w", padx=4)

        self.brake_lamp = Lamp(frame)
        self.brake_lamp.grid(row=1, column=4, padx=(16, 2))
        self.brake_var = tk.StringVar(value="--")
        ttk.Label(frame, text="Brake").grid(row=1, column=5, sticky="e")
        ttk.Label(frame, textvariable=self.brake_var, width=18,
                  anchor="w").grid(row=1, column=6, sticky="w", padx=4)

        self.vsoll_var = tk.StringVar(value="--")
        ttk.Label(frame, text="V_SOLL").grid(row=2, column=5, sticky="e")
        ttk.Label(frame, textvariable=self.vsoll_var, width=18,
                  anchor="w").grid(row=2, column=6, sticky="w", padx=4)

        # Follow error: the number that says whether the SHAFT arrived, as
        # opposed to whether the profile generator did. Shown beside the
        # position because the two together are the whole story.
        self.follow_var = tk.StringVar(value="--")
        ttk.Label(frame, text="Follow error").grid(row=2, column=0, sticky="e",
                                                   padx=(8, 2))
        ttk.Label(frame, textvariable=self.follow_var, width=34, anchor="w",
                  font=("TkFixedFont", 10)).grid(row=2, column=1, columnspan=3,
                                                 sticky="w")

        # --- the error panel, deliberately prominent ---
        self.error_frame = tk.Frame(frame, bg=COLOR_OK_BG, bd=1, relief="solid")
        self.error_frame.grid(row=3, column=0, columnspan=8, sticky="ew",
                              padx=8, pady=(10, 8))
        self.error_frame.columnconfigure(1, weight=1)

        self.error_title = tk.Label(self.error_frame, text="ERRORS",
                                    bg=COLOR_OK_BG, fg="#333",
                                    font=("TkDefaultFont", 9, "bold"))
        self.error_title.grid(row=0, column=0, sticky="w", padx=8, pady=(6, 0))

        self.error_var = tk.StringVar(value="not read yet")
        self.error_label = tk.Label(self.error_frame, textvariable=self.error_var,
                                    bg=COLOR_OK_BG, anchor="w", justify="left",
                                    font=("TkFixedFont", 10, "bold"),
                                    wraplength=760)
        self.error_label.grid(row=1, column=0, columnspan=2, sticky="ew",
                              padx=8, pady=(0, 6))

        buttons = tk.Frame(self.error_frame, bg=COLOR_OK_BG)
        buttons.grid(row=0, column=1, sticky="e", padx=8)
        ttk.Button(buttons, text="Clear errors",
                   command=self.on_clear_errors).grid(row=0, column=0, padx=3)
        ttk.Button(buttons, text="Why is it not moving?",
                   command=self.on_diagnose).grid(row=0, column=1, padx=3)
        ttk.Button(buttons, text="Motor history",
                   command=self.on_show_history).grid(row=0, column=2, padx=3)

    def _build_controls(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Move  (revolutions of motor shaft)")
        frame.grid(row=3, column=0, sticky="ew", padx=6, pady=3)

        ttk.Label(frame, text="Go to (rev from zero):").grid(row=0, column=0,
                                                             sticky="e", padx=(8, 2))
        self.target_rev_var = tk.StringVar(value="0.0")
        ttk.Entry(frame, textvariable=self.target_rev_var,
                  width=10).grid(row=0, column=1, padx=2, pady=6)
        self.move_btn = ttk.Button(frame, text="Move", command=self.on_move)
        self.move_btn.grid(row=0, column=2, padx=4)

        ttk.Label(frame, text="Step (rev):").grid(row=0, column=3, sticky="e",
                                                  padx=(20, 2))
        self.step_var = tk.StringVar(value="0.25")
        ttk.Entry(frame, textvariable=self.step_var, width=8).grid(row=0, column=4, padx=2)
        ttk.Button(frame, text="◀ −", width=5,
                   command=lambda: self.on_jog(-1)).grid(row=0, column=5, padx=2)
        ttk.Button(frame, text="+ ▶", width=5,
                   command=lambda: self.on_jog(+1)).grid(row=0, column=6, padx=2)

        ttk.Label(frame, text="V_SOLL:").grid(row=0, column=7, sticky="e", padx=(20, 2))
        self.velocity_var = tk.StringVar(value=str(self.motor.cfg.velocity_raw))
        ttk.Entry(frame, textvariable=self.velocity_var,
                  width=8).grid(row=0, column=8, padx=2)
        ttk.Button(frame, text="Set",
                   command=self.on_set_velocity).grid(row=0, column=9, padx=2)

        ttk.Button(frame, text="Enable Position mode",
                   command=self.on_enable_position).grid(row=1, column=0,
                                                         columnspan=2, padx=8,
                                                         pady=(0, 8), sticky="w")
        ttk.Button(frame, text="Set zero here",
                   command=self.on_set_zero).grid(row=1, column=2, pady=(0, 8))
        ttk.Label(frame, text="Zero is this GUI's reference only; nothing is "
                              "written to the motor.",
                  foreground="#777").grid(row=1, column=3, columnspan=7,
                                          sticky="w", padx=8, pady=(0, 8))

    def _build_faults(self) -> None:
        frame = ttk.LabelFrame(
            self.root,
            text="Inject a fault  (tampers with register traffic; never moves anything)")
        frame.grid(row=4, column=0, sticky="ew", padx=6, pady=3)

        self.fault_var = tk.StringVar(value=Fault.NONE.value)
        choices = [f.value for f in Fault]
        ttk.Label(frame, text="Fault:").grid(row=0, column=0, sticky="e", padx=(8, 2))
        ttk.OptionMenu(frame, self.fault_var, self.fault_var.get(),
                       *choices).grid(row=0, column=1, padx=2, pady=6, sticky="w")
        ttk.Button(frame, text="Arm",
                   command=self.on_arm_fault).grid(row=0, column=2, padx=4)
        ttk.Button(frame, text="Clear",
                   command=self.on_clear_fault).grid(row=0, column=3, padx=4)

        self.fault_lamp = Lamp(frame)
        self.fault_lamp.grid(row=0, column=4, padx=(16, 2))
        self.fault_status_var = tk.StringVar(value="no fault armed")
        ttk.Label(frame, textvariable=self.fault_status_var,
                  width=46, anchor="w").grid(row=0, column=5, sticky="w")

        ttk.Label(
            frame,
            text="Armed faults make the motor APPEAR to misbehave so you can watch "
                 "the handling. They only alter values read back and whether a "
                 "transaction succeeds -- nothing is written to the motor.",
            foreground="#777", wraplength=880, justify="left",
        ).grid(row=1, column=0, columnspan=6, sticky="w", padx=8, pady=(0, 8))

    def _build_log(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Event log")
        frame.grid(row=5, column=0, sticky="nsew", padx=6, pady=(3, 6))
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        controls = ttk.Frame(frame)
        controls.grid(row=0, column=0, columnspan=2, sticky="ew", padx=6, pady=(6, 0))

        ttk.Label(controls, text="Show:").grid(row=0, column=0, padx=(0, 4))
        self.severity_var = tk.StringVar(value=INFO)
        ttk.OptionMenu(controls, self.severity_var, INFO, DEBUG, INFO, WARNING, ERROR,
                       command=lambda _=None: self._refresh_log()
                       ).grid(row=0, column=1)
        ttk.Button(controls, text="Export...",
                   command=self.on_export_log).grid(row=0, column=2, padx=(14, 4))
        ttk.Button(controls, text="Clear view",
                   command=self.on_clear_log_view).grid(row=0, column=3, padx=4)
        self.log_counts_var = tk.StringVar(value="")
        ttk.Label(controls, textvariable=self.log_counts_var,
                  foreground="#555").grid(row=0, column=4, padx=14)
        ttk.Label(controls, text=f"file: {self.log_path}", foreground="#777"
                  ).grid(row=0, column=5, padx=10, sticky="w")

        self.log_text = tk.Text(frame, height=14, wrap="word", state="disabled",
                                font=("TkFixedFont", 9))
        self.log_text.grid(row=1, column=0, sticky="nsew", padx=(6, 0), pady=6)
        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.log_text.yview)
        scroll.grid(row=1, column=1, sticky="ns", pady=6, padx=(0, 6))
        self.log_text.configure(yscrollcommand=scroll.set)
        for severity, color in SEVERITY_COLOR.items():
            self.log_text.tag_configure(severity, foreground=color)
        self.log_text.tag_configure(ERROR, foreground=COLOR_BAD,
                                    font=("TkFixedFont", 9, "bold"))

    # ------------------------------------------------------- worker plumbing

    def post(self, fn: Callable[[], None]) -> None:
        self._ui_queue.put(fn)

    def _drain_ui(self) -> None:
        for _ in range(200):
            try:
                job = self._ui_queue.get_nowait()
            except queue.Empty:
                break
            try:
                job()
            except Exception as exc:  # noqa: BLE001
                try:
                    self.log.error("ui", f"UI update failed: {exc}")
                except Exception:
                    pass
        self._refresh_log()
        self._ui_job = self.root.after(150, self._drain_ui)

    def _set_busy(self, busy: bool) -> None:
        self._busy = busy
        self.move_btn.config(state="disabled" if busy else "normal")

    def run_async(self, description: str, fn: Callable[[], None]) -> None:
        if self._busy:
            self.log.warning("ui", f"Busy -- '{description}' ignored.")
            return
        self._set_busy(True)

        def worker():
            try:
                fn()
            except (ModbusError, MotorFault) as exc:
                self.log.error("command", f"{description} failed: {exc}")
            except Exception as exc:  # noqa: BLE001
                self.log.error("command", f"{description} failed: {exc}")
                self.log.debug("command",
                               traceback.format_exc().strip().splitlines()[-1])
            finally:
                self.post(lambda: self._set_busy(False))

        threading.Thread(target=worker, name=description, daemon=True).start()

    # ------------------------------------------------------------ connection

    def on_connect(self) -> None:
        if self.motor.connected:
            self.watcher.stop()
            self.motor.disconnect()
            self.connect_btn.config(text="Connect")
            self.watch_btn.config(text="Start recording")
            self.conn_var.set("disconnected")
            self.conn_lamp.set(COLOR_IDLE)
            self.log.info("comms", "Disconnected by operator")
            return

        def work():
            self.motor.connect(verify_word_order=True)
            self.log.info("comms", f"Connected to {self.motor.describe()}")
            self.post(self._after_connect)

        self.run_async("Connect", work)

    def _after_connect(self) -> None:
        self.connect_btn.config(text="Disconnect")
        self.conn_var.set("connected" + (" (simulated)" if self.simulate else ""))
        self.conn_lamp.set(COLOR_OK)
        if not self.watcher.running:
            self.watcher.start()
            self.watch_btn.config(text="Stop recording")

    def on_reconnect(self) -> None:
        def work():
            self.log.info("comms", "Reconnecting (rebuilding the client)")
            self.motor.reconnect()
            self.log.info("comms", "Reconnected")
            self.post(self._after_connect)

        self.run_async("Reconnect", work)

    def on_toggle_watch(self) -> None:
        if self.watcher.running:
            self.watcher.stop()
            self.watch_btn.config(text="Start recording")
        else:
            if not self.motor.connected:
                messagebox.showwarning("Not connected", "Connect first.")
                return
            self.watcher.start()
            self.watch_btn.config(text="Stop recording")

    # ---------------------------------------------------------------- state

    def _apply_status(self) -> None:
        """Refresh the readout from the watcher's most recent poll."""
        snapshot = self.watcher._previous
        if not snapshot.reachable:
            self.position_var.set("--")
            self.target_var.set("--")
            self.mode_var.set("no comms")
            self.mode_lamp.set(COLOR_BAD)
            self.conn_lamp.set(COLOR_BAD)
            self._show_errors(None, snapshot.comms_error)
            return

        position = snapshot.position or 0
        target = snapshot.target or 0
        self.position_var.set(
            f"{position:>10d} ct   {self.revs(position):+8.4f} rev   "
            f"{self.revs(position) * 360:+8.2f}°")
        self.target_var.set(
            f"{target:>10d} ct   {self.revs(target):+8.4f} rev   "
            f"({position - target:+d} ct away)")
        try:
            follow = self.motor.get_follow_error()
            window = self.motor.cfg.follow_error_window_counts
            self.follow_var.set(
                f"{follow:>10d} ct   window {window}"
                + ("   << OUTSIDE" if abs(follow) > window else ""))
        except (ModbusError, MotorFault):
            self.follow_var.set("--")
        self.mode_var.set(describe_mode(snapshot.mode).split(" (")[0]
                          if snapshot.mode is not None else "?")
        self.mode_lamp.set(COLOR_OK if snapshot.mode == int(MotorMode.POSITION)
                           else COLOR_IDLE)
        self.vsoll_var.set(str(snapshot.velocity_setting)
                           + ("   << zero!" if snapshot.velocity_setting == 0 else ""))
        self.conn_lamp.set(COLOR_WARN if snapshot.moving else COLOR_OK)
        self._show_errors(snapshot.errors, "")

        try:
            brake = self.motor.get_brake_status()
            self.brake_var.set(brake.state.value
                               + (" (inferred)" if brake.inferred else ""))
            self.brake_lamp.set(COLOR_BAD if brake.state is BrakeState.ENGAGED
                                else COLOR_OK if brake.state is BrakeState.RELEASED
                                else COLOR_IDLE)
        except (ModbusError, MotorFault):
            self.brake_var.set("unknown")
            self.brake_lamp.set(COLOR_IDLE)

    def _show_errors(self, errors: Optional[int], comms_error: str) -> None:
        if comms_error:
            background, text = COLOR_ERROR_BG, f"NO COMMUNICATION: {comms_error}"
        elif errors:
            background, text = COLOR_ERROR_BG, describe_errors(errors)
        else:
            background, text = COLOR_OK_BG, "none  (ERR_BITS = 0)"
        self.error_frame.configure(bg=background)
        self.error_title.configure(bg=background)
        self.error_label.configure(bg=background, fg=COLOR_BAD
                                   if background == COLOR_ERROR_BG else "#222")
        for child in self.error_frame.winfo_children():
            if isinstance(child, tk.Frame):
                child.configure(bg=background)
        self.error_var.set(text)

    # -------------------------------------------------------------- commands

    def _read_float(self, var: tk.StringVar, label: str) -> Optional[float]:
        try:
            return float(var.get())
        except ValueError:
            messagebox.showerror("Check the number", f"{label} must be a number.")
            return None

    def on_move(self) -> None:
        if not self._require_connection():
            return
        revs = self._read_float(self.target_rev_var, "Target")
        if revs is None:
            return
        target = self.motor.cfg.zero_counts + self.counts(revs)

        def work():
            self.log.info("command", f"Move commanded to {revs:+g} rev "
                                     f"({target} counts)", target=target)
            self.motor.clear_cancel()
            self.motor.ensure_position_mode()
            self.motor.command_position_counts(target)
            if self.motor.wait_for_in_position(timeout_s=60.0):
                self.log.info("command", "Move complete",
                              position=self.motor.get_position_counts())
            else:
                self.log.warning("command",
                                 "Move did not report arrival; the axis was halted")

        self.run_async("Move", work)

    def on_jog(self, sign: int) -> None:
        if not self._require_connection():
            return
        step = self._read_float(self.step_var, "Step")
        if step is None:
            return

        def work():
            start = self.motor.get_position_counts()
            target = start + sign * self.counts(step)
            self.log.info("command", f"Jog {sign * step:+g} rev "
                                     f"({start} -> {target} counts)", target=target)
            self.motor.clear_cancel()
            self.motor.ensure_position_mode()
            self.motor.command_position_counts(target)
            if not self.motor.wait_for_in_position(timeout_s=60.0):
                self.log.warning("command",
                                 "Jog did not report arrival; the axis was halted")

        self.run_async("Jog", work)

    def on_set_velocity(self) -> None:
        if not self._require_connection():
            return
        value = self._read_float(self.velocity_var, "V_SOLL")
        if value is None:
            return

        def work():
            self.motor.set_velocity(int(value))
            self.log.info("command", f"V_SOLL set to {int(value)}")

        self.run_async("Set velocity", work)

    def on_enable_position(self) -> None:
        if not self._require_connection():
            return

        def work():
            self.motor.set_mode(MotorMode.POSITION)
            self.log.info("command", "Position mode enabled and verified")

        self.run_async("Enable position mode", work)

    def on_set_zero(self) -> None:
        if not self._require_connection():
            return

        def work():
            counts = self.motor.set_zero_here()
            self.log.info("command", f"Zero reference set at {counts} counts")

        self.run_async("Set zero", work)

    def on_stop(self) -> None:
        if not self.motor.connected:
            self.log.warning("command", "STOP pressed, but not connected")
            return
        self.log.warning("command", "STOP pressed")
        threading.Thread(target=self._stop_worker, name="stop", daemon=True).start()

    def _stop_worker(self) -> None:
        try:
            self.motor.stop()
            self.log.info("command", "Stopped and holding",
                          position=self.motor.get_position_counts())
        except (ModbusError, MotorFault) as exc:
            self.log.error("command", f"STOP failed: {exc}")
        finally:
            self.post(lambda: self._set_busy(False))

    def on_passivate(self) -> None:
        """Acts first and reports afterwards: a safety control that stops to
        ask a question is not a safety control."""
        if not self._require_connection():
            return

        def work():
            self.motor.passivate()
            # Report the state that is true afterwards, read back from the
            # motor, not the state that was commanded.
            try:
                mode = describe_mode(self.motor.get_mode())
                counts = self.motor.get_position_counts()
            except (ModbusError, MotorFault) as exc:
                self.log.error("command",
                               f"Drive passivated, but the result could not be "
                               f"read back: {exc}")
                return
            self.log.warning(
                "command",
                f"Drive passivated by operator: MODE_REG = 0, {mode}. The drive "
                f"is no longer holding -- on a loaded axis the load now rests on "
                f"the brake and friction. Enable position mode to resume.",
                position=counts,
            )

        self.run_async("Passivate", work)

    def on_clear_errors(self) -> None:
        if not self._require_connection():
            return

        def work():
            before = self.motor.get_errors()
            after = self.motor.clear_errors()
            if before and not after:
                self.log.info("error", f"Cleared errors (was "
                                       f"0x{before:08X}, now 0)")
            elif after:
                self.log.error("error",
                               f"Errors did not clear: still {describe_errors(after)}. "
                               "A latched or still-present fault needs MacTalk's "
                               "own clear or a power cycle.")
            else:
                self.log.info("error", "No errors to clear")

        self.run_async("Clear errors", work)

    def _require_connection(self) -> bool:
        if not self.motor.connected:
            messagebox.showwarning("Not connected", "Connect to the motor first.")
            return False
        return True

    # ------------------------------------------------------------- diagnose

    def on_diagnose(self) -> None:
        if not self._require_connection():
            return

        def work():
            result = diagnose(self.motor, probe_writes=True)
            self.log.info("diagnose", result.summary())
            for finding in result.findings:
                if finding.verdict == BLOCKING:
                    self.log.error("diagnose", f"{finding.title}: {finding.detail}",
                                   remedy=finding.remedy)
                elif finding.verdict == SUSPECT:
                    self.log.warning("diagnose", f"{finding.title}: {finding.detail}",
                                     remedy=finding.remedy)
                elif finding.verdict == UNKNOWN:
                    self.log.debug("diagnose", f"{finding.title}: {finding.detail}")
            self.post(lambda: self._show_diagnosis(result))

        self.run_async("Diagnose", work)

    def _show_diagnosis(self, result) -> None:
        window = tk.Toplevel(self.root)
        window.title("Why is it not moving?")
        window.geometry("820x520")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(1, weight=1)

        headline = tk.Label(window, text=result.summary(), anchor="w",
                            justify="left", wraplength=780,
                            font=("TkDefaultFont", 11, "bold"),
                            fg=COLOR_BAD if not result.healthy else COLOR_OK)
        headline.grid(row=0, column=0, sticky="ew", padx=12, pady=(12, 6))

        text = tk.Text(window, wrap="word", font=("TkFixedFont", 9))
        text.grid(row=1, column=0, sticky="nsew", padx=12, pady=6)
        scroll = ttk.Scrollbar(window, orient="vertical", command=text.yview)
        scroll.grid(row=1, column=1, sticky="ns", pady=6, padx=(0, 12))
        text.configure(yscrollcommand=scroll.set)

        text.tag_configure(BLOCKING, foreground=COLOR_BAD,
                           font=("TkFixedFont", 9, "bold"))
        text.tag_configure(SUSPECT, foreground=COLOR_WARN)
        text.tag_configure(OK, foreground=COLOR_OK)
        text.tag_configure(UNKNOWN, foreground="#777")
        text.tag_configure("remedy", foreground="#111")

        for finding in result.findings:
            text.insert("end", f"[{finding.verdict}] {finding.title}\n", finding.verdict)
            text.insert("end", f"    {finding.detail}\n")
            if finding.remedy and finding.verdict in (BLOCKING, SUSPECT):
                text.insert("end", f"    -> {finding.remedy}\n", "remedy")
            text.insert("end", "\n")
        text.configure(state="disabled")

        ttk.Button(window, text="Close", command=window.destroy).grid(
            row=2, column=0, columnspan=2, pady=(0, 12))

    def on_show_history(self) -> None:
        """The only two things the motor itself remembers."""
        if not self._require_connection():
            return

        def work():
            def read(name):
                try:
                    return self.motor.read_register(name)
                except (ModbusError, MotorFault):
                    return None
            values = {
                "follow_error_max": read("FLWERR_MAX"),
                "follow_error": read("FLWERR"),
                "bus_voltage": read("BUS_VOLTAGE"),
                "bus_voltage_min": read("BUS_VOLTAGE_MIN"),
                "ticks": read("TICKS"),
                "errors": read("ERR_BITS"),
                "warnings": read("WARN_BITS"),
            }
            self.log.info("history",
                          f"Follow Error Max {values['follow_error_max']}, "
                          f"Bus Voltage Min {values['bus_voltage_min']}, "
                          f"ticks {values['ticks']}")
            self.post(lambda: self._show_history(values))

        self.run_async("Motor history", work)

    def _show_history(self, values: dict) -> None:
        window = tk.Toplevel(self.root)
        window.title("What the motor remembers")
        window.geometry("720x430")
        window.columnconfigure(0, weight=1)
        window.rowconfigure(1, weight=1)

        tk.Label(window,
                 text="This motor keeps no error history.",
                 anchor="w", justify="left",
                 font=("TkDefaultFont", 11, "bold")).grid(
            row=0, column=0, sticky="ew", padx=12, pady=(12, 4))

        text = tk.Text(window, wrap="word", font=("TkFixedFont", 9))
        text.grid(row=1, column=0, sticky="nsew", padx=12, pady=6)
        text.insert("end",
                    "Registers 35 (Errors) and 36 (Warnings) are instantaneous "
                    "bit fields. A fault that has since cleared leaves no trace "
                    "in the drive at all -- there is no event log to read.\n\n"
                    "Two registers ARE latched extremes, and they survive both a "
                    "cleared error and a completed move. After an intermittent "
                    "fault they are often the only evidence left in the motor:\n\n")
        text.insert("end", f"  Follow Error Max (reg 22)   {values['follow_error_max']}\n",
                    "key")
        text.insert("end",
                    "      The largest lag between the commanded profile and the\n"
                    "      encoder since this was last cleared. A large value means\n"
                    "      the shaft fell behind at some point -- a stall, an\n"
                    "      obstruction, or a brake that did not release.\n"
                    f"      Right now the follow error is {values['follow_error']}.\n\n")
        text.insert("end", f"  Bus Voltage Min (reg 98)    {values['bus_voltage_min']}\n",
                    "key")
        text.insert("end",
                    "      The lowest supply voltage seen since this was last\n"
                    "      cleared, in the same raw units as the live reading of\n"
                    f"      {values['bus_voltage']}. A big gap is evidence of a\n"
                    "      brown-out, though it can also just be the supply ramping\n"
                    "      up at power-on.\n\n")
        text.insert("end", f"  Ticks (reg 202)             {values['ticks']}\n", "key")
        text.insert("end",
                    "      A free-running counter. If it is lower than last time you\n"
                    "      looked, the motor restarted in between -- which would\n"
                    "      explain a mode reverting to its startup value.\n\n")
        text.insert("end",
                    "Both extremes are resettable: write 0 to register 22 or 98,\n"
                    "then watch whether they climb again. That turns a value with no\n"
                    "timestamp into one with a known starting point.\n\n"
                    "For anything finer-grained, the event log this application\n"
                    "records is the history -- the motor has none to give.")
        text.tag_configure("key", font=("TkFixedFont", 10, "bold"))
        text.configure(state="disabled")

        buttons = ttk.Frame(window)
        buttons.grid(row=2, column=0, pady=(0, 12))
        ttk.Button(buttons, text="Reset both extremes",
                   command=lambda: (self.on_reset_extremes(), window.destroy())
                   ).grid(row=0, column=0, padx=6)
        ttk.Button(buttons, text="Close",
                   command=window.destroy).grid(row=0, column=1, padx=6)

    def on_reset_extremes(self) -> None:
        """Zero the latched high/low-water marks, so they mean 'since now'."""
        def work():
            for name in ("FLWERR_MAX", "BUS_VOLTAGE_MIN"):
                try:
                    self.motor.write_register(name, 0)
                    self.log.info("history", f"{name} reset to 0")
                except (ModbusError, MotorFault) as exc:
                    self.log.warning("history", f"Could not reset {name}: {exc}")

        self.run_async("Reset extremes", work)

    # ---------------------------------------------------------------- faults

    def on_arm_fault(self) -> None:
        if not self._require_connection():
            return
        fault = Fault(self.fault_var.get())
        if fault is Fault.NONE:
            self.on_clear_fault()
            return
        self.injector.arm(fault)
        self.fault_lamp.set(COLOR_BAD)
        self.fault_status_var.set(f"ARMED: {fault.label}")
        self.log.warning("inject", f"Fault armed: {fault.value} -- {fault.label}",
                         fault=fault.value)

    def on_clear_fault(self) -> None:
        self.injector.clear()
        self.fault_lamp.set(COLOR_IDLE)
        self.fault_status_var.set("no fault armed")
        self.log.info("inject", "Fault cleared")

    # ------------------------------------------------------------------- log

    def _refresh_log(self) -> None:
        events = self.log.events(min_severity=self.severity_var.get())
        if len(events) == self._log_cursor:
            return
        new = events[self._log_cursor:] if len(events) > self._log_cursor else events
        if len(events) < self._log_cursor:      # filter changed; redraw
            self.log_text.configure(state="normal")
            self.log_text.delete("1.0", "end")
            self.log_text.configure(state="disabled")
            new = events
        self._log_cursor = len(events)

        self.log_text.configure(state="normal")
        for event in new:
            self.log_text.insert("end", event.as_line() + "\n", event.severity)
            if event.data.get("remedy"):
                self.log_text.insert("end", f"{'':>10}-> {event.data['remedy']}\n",
                                     DEBUG)
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

        counts = self.log.counts()
        self.log_counts_var.set(
            f"{counts[ERROR]} error, {counts[WARNING]} warning, "
            f"{counts[INFO]} info")
        self._apply_status()

    def on_export_log(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Export event log",
            defaultextension=".txt",
            initialfile=f"motor-{self.motor_name}-"
                        f"{datetime.now():%Y%m%d-%H%M%S}.txt",
            filetypes=[("Text", "*.txt"), ("All files", "*.*")],
        )
        if not path:
            return
        self.log.export_text(path, min_severity=self.severity_var.get())
        self.log.info("session", f"Log exported to {path}")
        messagebox.showinfo("Exported", f"Written to:\n{path}\n\n"
                                        f"The full JSONL recording is at:\n{self.log_path}")

    def on_clear_log_view(self) -> None:
        self.log.clear()
        self._log_cursor = 0
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.log.info("session", "Log view cleared (the file on disk is unaffected)")

    # ----------------------------------------------------------------- close

    def on_close(self) -> None:
        if self.motor.connected and not messagebox.askyesno(
            "Quit?",
            "Closing does NOT stop the motor or change the brake. Anything "
            "still moving keeps moving.\n\nQuit anyway?",
        ):
            return
        self.shutdown()
        self.root.destroy()

    def shutdown(self) -> None:
        try:
            self.watcher.stop()
        except Exception:
            pass
        if self._ui_job is not None:
            try:
                self.root.after_cancel(self._ui_job)
            except Exception:
                pass
            self._ui_job = None
        try:
            while True:
                self._ui_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self.injector.clear()
            self.motor.disconnect()
        except Exception:
            pass
        try:
            self.log.info("session", "Session closed")
            self.log.close()
        except Exception:
            pass


def main(motor_name: str = "Top", config_path: Optional[str] = None,
         simulate: bool = False, log_path: Optional[str] = None) -> int:
    root = tk.Tk()
    SingleMotorApp(root, motor_name=motor_name, config_path=config_path,
                   simulate=simulate, log_path=log_path)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
