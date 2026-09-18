"""
A record of where the focal plane has been.

The motors keep no history at all -- register 35 is instantaneous, and there
is no event buffer anywhere in the drive -- so anything anybody wants to know
about *previous* positions has to be written down by this software at the
time. This is that record: one entry per move, saying where the focal plane
was, where it was told to go, where it actually ended up, and whether the move
finished.

It exists for two questions asked at the telescope:

  * "Where was it before that?"  -- answered by reading the list.
  * "Put it back where it was."  -- answered by handing the `before` of a
    record to `move_to_orientation`, which then runs every check an ordinary
    move gets. Going back is not a special, unchecked path.

Entries are kept in memory for the window and appended to a JSON-lines file
beside the configuration, so the record survives the application closing.
The file is append-only: nothing here ever rewrites or deletes an entry.
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Callable, Deque, Dict, List, Optional

from .kinematics import Orientation

#: Where the record lives when no path is given: next to the configuration.
DEFAULT_HISTORY_FILENAME = "positions.jsonl"


def _orientation_dict(o: Optional[Orientation]) -> Optional[dict]:
    if o is None:
        return None
    return {"focus_mm": o.focus_mm, "tip_deg": o.tip_deg, "tilt_deg": o.tilt_deg}


def _orientation_from(d: Optional[dict]) -> Optional[Orientation]:
    if not d:
        return None
    return Orientation(float(d["focus_mm"]), float(d.get("tip_deg", 0.0)),
                       float(d.get("tilt_deg", 0.0)))


@dataclass(frozen=True)
class MoveRecord:
    """One move: where it started, where it was sent, where it ended."""

    #: Seconds since the epoch, when the move was commanded.
    timestamp: float
    #: What kind of command it was: "move", "nudge", "jog", "level",
    #: "find-stop", "go-back" ... free text, for the eye.
    kind: str
    #: The orientation read from the motors just before the command.
    before: Optional[Orientation]
    #: What was asked for. None for a search whose end is not known in advance.
    commanded: Optional[Orientation]
    #: The orientation read from the motors afterwards, whether or not the
    #: move completed -- which is the point: after a halted move this is the
    #: only honest statement of where the focal plane is.
    after: Optional[Orientation]
    #: Per-actuator positions, mm, before and after.
    actuators_before: Dict[str, float] = field(default_factory=dict)
    actuators_after: Dict[str, float] = field(default_factory=dict)
    #: "done", or the reason it was not.
    outcome: str = "done"
    #: Anything else worth writing down: which axis a search stopped on, the
    #: nudge size, the jogged actuator.
    note: str = ""

    @property
    def completed(self) -> bool:
        return self.outcome == "done"

    @property
    def when(self) -> str:
        return datetime.fromtimestamp(self.timestamp).strftime("%Y-%m-%d %H:%M:%S")

    def as_dict(self) -> dict:
        d = asdict(self)
        d["before"] = _orientation_dict(self.before)
        d["commanded"] = _orientation_dict(self.commanded)
        d["after"] = _orientation_dict(self.after)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "MoveRecord":
        return cls(
            timestamp=float(d["timestamp"]),
            kind=str(d.get("kind", "move")),
            before=_orientation_from(d.get("before")),
            commanded=_orientation_from(d.get("commanded")),
            after=_orientation_from(d.get("after")),
            actuators_before={k: float(v) for k, v in (d.get("actuators_before") or {}).items()},
            actuators_after={k: float(v) for k, v in (d.get("actuators_after") or {}).items()},
            outcome=str(d.get("outcome", "done")),
            note=str(d.get("note", "")),
        )


class PositionHistory:
    """The list of moves, in memory and on disk.

    Thread-safe: moves are recorded from worker threads and read from the UI
    thread. `on_change` is called (on the recording thread) after every new
    entry, so a window can refresh itself rather than polling the list.
    """

    def __init__(self, path: Optional[str] = None, capacity: int = 2000,
                 on_change: Optional[Callable[[], None]] = None,
                 logger: Optional[Callable[[str], None]] = None):
        self.path = path
        self._records: Deque[MoveRecord] = deque(maxlen=capacity)
        self._lock = threading.Lock()
        self._on_change = on_change
        self._log = logger or (lambda msg: None)
        self._load_warned = False
        if path:
            self._load()

    # --------------------------------------------------------------- access

    def records(self, newest_first: bool = True) -> List[MoveRecord]:
        with self._lock:
            items = list(self._records)
        return items[::-1] if newest_first else items

    def __len__(self) -> int:
        with self._lock:
            return len(self._records)

    def last(self) -> Optional[MoveRecord]:
        with self._lock:
            return self._records[-1] if self._records else None

    def last_with_before(self) -> Optional[MoveRecord]:
        """The most recent record that knows where the plane was before it.

        That is the one "go back" wants: the latest move, completed or not,
        whose starting orientation was read. A halted move counts -- after a
        halt, "where it was before" is exactly what somebody wants back.
        """
        with self._lock:
            for record in reversed(self._records):
                if record.before is not None:
                    return record
        return None

    # ------------------------------------------------------------ recording

    def add(self, record: MoveRecord) -> MoveRecord:
        with self._lock:
            self._records.append(record)
        self._append_to_file(record)
        if self._on_change is not None:
            try:
                self._on_change()
            except Exception:  # noqa: BLE001 -- a listener must not break a move
                pass
        return record

    def record(self, kind: str, before: Optional[Orientation],
               commanded: Optional[Orientation], after: Optional[Orientation],
               actuators_before: Optional[Dict[str, float]] = None,
               actuators_after: Optional[Dict[str, float]] = None,
               outcome: str = "done", note: str = "",
               timestamp: Optional[float] = None) -> MoveRecord:
        return self.add(MoveRecord(
            timestamp=time.time() if timestamp is None else timestamp,
            kind=kind, before=before, commanded=commanded, after=after,
            actuators_before=dict(actuators_before or {}),
            actuators_after=dict(actuators_after or {}),
            outcome=outcome, note=note,
        ))

    # ----------------------------------------------------------------- disk

    def _load(self) -> None:
        if not self.path or not os.path.exists(self.path):
            return
        loaded = 0
        bad = 0
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        self._records.append(MoveRecord.from_dict(json.loads(line)))
                        loaded += 1
                    except (ValueError, KeyError, TypeError):
                        bad += 1
        except OSError as exc:
            self._log(f"Position history could not be read from {self.path}: {exc}")
            return
        if bad:
            self._log(f"Position history: {bad} unreadable line(s) in {self.path} "
                      "were skipped.")

    def _append_to_file(self, record: MoveRecord) -> None:
        if not self.path:
            return
        try:
            directory = os.path.dirname(os.path.abspath(self.path))
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(record.as_dict(), sort_keys=True) + "\n")
        except OSError as exc:
            if not self._load_warned:
                self._load_warned = True
                self._log(f"Position history could not be written to {self.path}: "
                          f"{exc}. Moves are still recorded for this session.")


def default_history_path(config_path: Optional[str]) -> str:
    """Beside the configuration file, so both belong to the same machine."""
    from .config import default_config_path
    base = config_path or default_config_path()
    return os.path.join(os.path.dirname(os.path.abspath(base)), DEFAULT_HISTORY_FILENAME)


__all__ = ["MoveRecord", "PositionHistory", "default_history_path",
           "DEFAULT_HISTORY_FILENAME"]
