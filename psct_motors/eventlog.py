"""
A timestamped record of what the motor did, so a hang can be explained later.

The problem this solves
-----------------------
"It works, and then at some point it stops taking position commands." By the
time you notice, the thing that caused it has already happened, and the live
readout only tells you the state *now*. What you need is what changed, and in
what order, in the seconds before it stopped.

`MotorWatcher` polls the motor and writes an event whenever something
meaningful changes -- the mode, the error bits, the target, the connection --
rather than on every poll. A five-minute recording of a healthy motor is a
handful of lines; the same recording across a failure has the failure in it,
in order, with timestamps.

Transaction timing is recorded too. A stall is not always an error: a
transaction that takes four seconds and then succeeds leaves no error trace at
all, but it is exactly what "the application froze" feels like. Any transaction
slower than `slow_transaction_s` is logged as a warning with its duration.

Files
-----
Events go to a JSONL file (one JSON object per line), which appends safely,
survives a crash, and can be read back with `read_log`. The GUI keeps the last
few hundred in memory for display. `export_text` writes the human-readable
version for pasting into an email or a commissioning log.
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Any, Callable, Deque, Dict, List, Optional

from .jvl_motor import JVLMotor, MotorFault
from .registers import describe_errors, describe_mode
from .transport import ModbusError

# Severities, ordered.
DEBUG = "DEBUG"
INFO = "INFO"
WARNING = "WARNING"
ERROR = "ERROR"

_SEVERITY_ORDER = {DEBUG: 0, INFO: 1, WARNING: 2, ERROR: 3}


@dataclass
class Event:
    """One thing that happened, at a time."""

    timestamp: float                      # unix time
    severity: str
    category: str                         # "comms", "mode", "error", "motion", "command", ...
    message: str
    data: Dict[str, Any] = field(default_factory=dict)

    @property
    def clock(self) -> str:
        return datetime.fromtimestamp(self.timestamp).strftime("%H:%M:%S.%f")[:-3]

    @property
    def iso(self) -> str:
        return datetime.fromtimestamp(self.timestamp).isoformat(timespec="milliseconds")

    def as_line(self) -> str:
        return f"[{self.clock}] {self.severity:<7} {self.category:<8} {self.message}"

    def as_dict(self) -> Dict[str, Any]:
        payload = asdict(self)
        payload["iso"] = self.iso
        return payload


class EventLog:
    """An in-memory ring of events, optionally mirrored to a JSONL file.

    Thread-safe: the watcher thread and the UI thread both touch it.
    """

    def __init__(self, path: Optional[str] = None, capacity: int = 2000,
                 on_event: Optional[Callable[[Event], None]] = None):
        self.path = path
        self.capacity = capacity
        self._events: Deque[Event] = deque(maxlen=capacity)
        self._lock = threading.RLock()
        self._on_event = on_event
        self._file = None
        if path:
            directory = os.path.dirname(os.path.abspath(path))
            if directory:
                os.makedirs(directory, exist_ok=True)
            # Line-buffered append: each event hits the disk as it happens, so
            # a recording survives the process being killed -- which is the
            # case this exists for.
            self._file = open(path, "a", encoding="utf-8", buffering=1)

    # ----------------------------------------------------------------- write

    def add(self, severity: str, category: str, message: str, **data) -> Event:
        event = Event(time.time(), severity, category, message, data)
        with self._lock:
            self._events.append(event)
            if self._file is not None:
                try:
                    self._file.write(json.dumps(event.as_dict()) + "\n")
                except Exception:
                    pass          # never let logging break the thing being logged
        if self._on_event is not None:
            try:
                self._on_event(event)
            except Exception:
                pass
        return event

    def debug(self, category: str, message: str, **data) -> Event:
        return self.add(DEBUG, category, message, **data)

    def info(self, category: str, message: str, **data) -> Event:
        return self.add(INFO, category, message, **data)

    def warning(self, category: str, message: str, **data) -> Event:
        return self.add(WARNING, category, message, **data)

    def error(self, category: str, message: str, **data) -> Event:
        return self.add(ERROR, category, message, **data)

    # ------------------------------------------------------------------ read

    def events(self, min_severity: str = DEBUG,
               categories: Optional[List[str]] = None) -> List[Event]:
        floor = _SEVERITY_ORDER.get(min_severity, 0)
        with self._lock:
            snapshot = list(self._events)
        return [
            e for e in snapshot
            if _SEVERITY_ORDER.get(e.severity, 0) >= floor
            and (categories is None or e.category in categories)
        ]

    def counts(self) -> Dict[str, int]:
        out = {DEBUG: 0, INFO: 0, WARNING: 0, ERROR: 0}
        for event in self.events():
            out[event.severity] = out.get(event.severity, 0) + 1
        return out

    def clear(self) -> None:
        with self._lock:
            self._events.clear()

    # ---------------------------------------------------------------- export

    def export_text(self, path: str, min_severity: str = DEBUG) -> str:
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(f"psct_motors event log, exported {datetime.now().isoformat()}\n")
            fh.write("=" * 78 + "\n")
            for event in self.events(min_severity=min_severity):
                fh.write(event.as_line() + "\n")
                for key, value in event.data.items():
                    fh.write(f"{'':>12}{key} = {value}\n")
        return path

    def close(self) -> None:
        with self._lock:
            if self._file is not None:
                try:
                    self._file.close()
                except Exception:
                    pass
                self._file = None


def read_log(path: str) -> List[Event]:
    """Read a JSONL log back. Bad lines are skipped rather than fatal."""
    events: List[Event] = []
    with open(path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
                events.append(Event(
                    timestamp=payload["timestamp"],
                    severity=payload.get("severity", INFO),
                    category=payload.get("category", ""),
                    message=payload.get("message", ""),
                    data=payload.get("data", {}) or {},
                ))
            except (ValueError, KeyError):
                continue
    return events


# --------------------------------------------------------------------------
# The watcher
# --------------------------------------------------------------------------

@dataclass
class WatchSnapshot:
    """The fields the watcher compares between polls."""

    reachable: bool = False
    mode: Optional[int] = None
    errors: Optional[int] = None
    target: Optional[int] = None
    position: Optional[int] = None
    velocity_setting: Optional[int] = None
    moving: bool = False
    comms_error: str = ""


class MotorWatcher:
    """Polls one motor and records what changes.

    Deliberately records changes, not samples. A poller that logs every read
    produces a file nobody will read; a poller that logs transitions produces
    a file where the interesting moment is visible.
    """

    def __init__(self, motor: JVLMotor, log: EventLog,
                 interval_s: float = 0.5,
                 slow_transaction_s: float = 1.0,
                 heartbeat_s: float = 60.0,
                 position_change_counts: int = 0):
        self.motor = motor
        self.log = log
        self.interval_s = interval_s
        #: Any poll slower than this is logged. A transaction that stalls and
        #: then succeeds leaves no error behind, but it is what a "hang" is.
        self.slow_transaction_s = slow_transaction_s
        #: Periodic "still here" line, so a quiet log is distinguishable from
        #: a recorder that died.
        self.heartbeat_s = heartbeat_s
        #: Log position changes above this many counts. 0 logs only
        #: start/stop of motion, which is usually what you want.
        self.position_change_counts = position_change_counts

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._previous = WatchSnapshot()
        self._last_heartbeat = 0.0
        self._first_poll = True

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._first_poll = True
        self.log.info("watch", f"Started watching {self.motor.name} "
                               f"every {self.interval_s:g}s",
                      interval_s=self.interval_s,
                      slow_transaction_s=self.slow_transaction_s)
        self._thread = threading.Thread(target=self._loop, name="watcher", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        self.log.info("watch", f"Stopped watching {self.motor.name}")

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ---------------------------------------------------------------- polling

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                self.poll_once()
            except Exception as exc:  # noqa: BLE001 - a watcher must not die
                self.log.error("watch", f"Watcher poll raised: {exc}")
            self._stop.wait(self.interval_s)

    def poll_once(self) -> WatchSnapshot:
        """One poll: read, time it, and record anything that changed."""
        started = time.monotonic()
        current = WatchSnapshot()
        try:
            current.mode = self.motor.get_mode()
            current.errors = self.motor.get_errors()
            current.target = self.motor.get_target_counts()
            current.position = self.motor.get_position_counts()
            current.velocity_setting = self.motor.read_register("V_SOLL")
            current.reachable = True
            # On the very first poll there is no previous position, so nothing
            # is known about motion. Treating "different from None" as moving
            # made every recording open with a spurious "Stopped moving".
            current.moving = (
                self._previous.position is not None
                and current.position != self._previous.position
            )
        except (ModbusError, MotorFault) as exc:
            current.reachable = False
            current.comms_error = str(exc)

        elapsed = time.monotonic() - started
        if elapsed >= self.slow_transaction_s:
            # The single most useful line in the whole log when chasing a
            # freeze: it says the link was alive but glacial, which no error
            # message would have told you.
            self.log.warning(
                "timing",
                f"Poll took {elapsed:.2f}s (threshold {self.slow_transaction_s:g}s). "
                "A stalled-but-successful transaction is what an application "
                "hang usually is.",
                seconds=round(elapsed, 3), reachable=current.reachable,
            )

        self._report_changes(self._previous, current)
        self._previous = current
        self._maybe_heartbeat(current)
        self._first_poll = False
        return current

    # --------------------------------------------------------------- changes

    def _report_changes(self, before: WatchSnapshot, now: WatchSnapshot) -> None:
        first = self._first_poll

        # --- connection ---------------------------------------------------
        if now.reachable != before.reachable or first:
            if now.reachable:
                self.log.info("comms", f"{self.motor.name} is answering",
                              position=now.position)
            else:
                self.log.error("comms", f"{self.motor.name} stopped answering: "
                                        f"{now.comms_error}",
                               detail=now.comms_error)
        if not now.reachable:
            return                       # nothing else is meaningful

        # --- errors -------------------------------------------------------
        if now.errors != before.errors or (first and now.errors):
            if now.errors:
                self.log.error("error",
                               f"ERR_BITS set: {describe_errors(now.errors)}",
                               raw=now.errors, position=now.position,
                               mode=now.mode)
            elif before.errors:
                self.log.info("error", "ERR_BITS cleared", raw=0)

        # --- mode ---------------------------------------------------------
        if now.mode != before.mode or first:
            severity = INFO
            note = ""
            if before.mode == 2 and now.mode == 0 and not first:
                # The classic silent failure: something dropped the drive, and
                # from then on every position command is accepted and ignored.
                severity = ERROR
                note = (" -- the drive went passive on its own. Position "
                        "commands will be accepted and do nothing until it is "
                        "put back into Position mode.")
            self.log.add(severity, "mode",
                         f"MODE_REG {describe_mode(before.mode) if before.mode is not None else '?'}"
                         f" -> {describe_mode(now.mode)}{note}",
                         mode=now.mode, position=now.position)

        # --- velocity setting --------------------------------------------
        if now.velocity_setting != before.velocity_setting or first:
            if now.velocity_setting == 0:
                self.log.error("config",
                               "V_SOLL is 0. Position commands will be accepted "
                               "and the motor will never move.",
                               v_soll=0)
            elif before.velocity_setting is not None and not first:
                self.log.info("config",
                              f"V_SOLL {before.velocity_setting} -> "
                              f"{now.velocity_setting}",
                              v_soll=now.velocity_setting)

        # --- target -------------------------------------------------------
        if now.target != before.target and not first:
            self.log.info("motion", f"P_SOLL {before.target} -> {now.target}",
                          target=now.target, position=now.position)

        # --- motion start / stop ------------------------------------------
        if now.moving and not before.moving and not first:
            self.log.info("motion", "Started moving", position=now.position,
                          target=now.target)
        elif before.moving and not now.moving and not first:
            settled = now.target is not None and now.position is not None
            error_counts = (now.position - now.target) if settled else None
            self.log.info("motion", "Stopped moving",
                          position=now.position, target=now.target,
                          error_counts=error_counts)
            if settled and error_counts and abs(error_counts) > 0:
                self.log.debug("motion",
                               f"Settled {error_counts:+d} counts from target")

        if (self.position_change_counts and before.position is not None
                and now.position is not None
                and abs(now.position - before.position) >= self.position_change_counts):
            self.log.debug("motion", f"Position {before.position} -> {now.position}",
                           position=now.position)

    def _maybe_heartbeat(self, now: WatchSnapshot) -> None:
        if not self.heartbeat_s:
            return
        stamp = time.monotonic()
        if stamp - self._last_heartbeat < self.heartbeat_s:
            return
        self._last_heartbeat = stamp
        self.log.debug(
            "watch",
            f"Still watching. position={now.position} target={now.target} "
            f"mode={describe_mode(now.mode) if now.mode is not None else '?'}",
            position=now.position, target=now.target, mode=now.mode,
        )


def default_log_path(motor_name: str = "Top") -> str:
    """Where a recording goes if no path is given."""
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "logs", f"motor-{motor_name}-{stamp}.jsonl",
    )


__all__ = [
    "Event", "EventLog", "MotorWatcher", "WatchSnapshot", "read_log",
    "default_log_path", "DEBUG", "INFO", "WARNING", "ERROR",
]
