"""
Named positions the operator can save and come back to.

"Default", "Window open", "Survey 2026-10" ... each is a place the focal plane
has actually been, written down with a name so that anyone can send it back
there without knowing the numbers.

What is stored is each actuator's *motor* position in raw encoder counts, as
well as the orientation it meant at the time. The counts are the ground
truth. Focus, tip and tilt are measured from the zero set by "Set zero here",
and if somebody sets a new zero later, a position stored only as "focus
+2.0 mm" would quietly start meaning a different physical place. Stored as
counts, a saved position goes back to the same place however the zero has
moved since; the orientation is kept alongside for the eye, and so the list
can say when the two no longer agree.

Going to a saved position is an ordinary move: the target is handed to
`move_to_orientation`, which runs every check a typed move gets.

The file is plain JSON beside the configuration, rewritten whole on each
change (through a temporary file, so a crash mid-write cannot lose the list).
"""

from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable, Dict, List, Optional

from .kinematics import Orientation

#: Where the list lives when no path is given: next to the configuration.
DEFAULT_SAVED_POSITIONS_FILENAME = "saved_positions.json"

#: Offered when saving, because these are the ones the site asked for. Any
#: other name can be typed.
SUGGESTED_NAMES = ("Default", "Window open")


@dataclass(frozen=True)
class SavedPosition:
    """One named place the focal plane has been."""

    name: str
    #: The orientation when it was saved, measured from the zero of the time.
    orientation: Orientation
    #: Each actuator's raw encoder counts: the physical position.
    actuator_counts: Dict[str, int]
    #: Each actuator's travel in mm from the zero of the time, for display.
    actuator_mm: Dict[str, float] = field(default_factory=dict)
    #: Seconds since the epoch.
    saved_at: float = 0.0
    note: str = ""

    @property
    def when(self) -> str:
        return datetime.fromtimestamp(self.saved_at).strftime("%Y-%m-%d %H:%M")

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "orientation": {"focus_mm": self.orientation.focus_mm,
                            "tip_deg": self.orientation.tip_deg,
                            "tilt_deg": self.orientation.tilt_deg},
            "actuator_counts": dict(self.actuator_counts),
            "actuator_mm": dict(self.actuator_mm),
            "saved_at": self.saved_at,
            "note": self.note,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "SavedPosition":
        o = d["orientation"]
        return cls(
            name=str(d["name"]),
            orientation=Orientation(float(o["focus_mm"]),
                                    float(o.get("tip_deg", 0.0)),
                                    float(o.get("tilt_deg", 0.0))),
            actuator_counts={str(k): int(v)
                             for k, v in d["actuator_counts"].items()},
            actuator_mm={str(k): float(v)
                         for k, v in d.get("actuator_mm", {}).items()},
            saved_at=float(d.get("saved_at", 0.0)),
            note=str(d.get("note", "")),
        )


class SavedPositions:
    """The list of saved positions, kept in memory and on disk."""

    def __init__(self, path: Optional[str] = None,
                 logger: Optional[Callable[[str], None]] = None):
        self.path = path
        self._log = logger or (lambda message: None)
        self._lock = threading.Lock()
        self._positions: List[SavedPosition] = []
        self._load()

    # ------------------------------------------------------------- reading

    def all(self) -> List[SavedPosition]:
        """In the order they were first saved."""
        with self._lock:
            return list(self._positions)

    def names(self) -> List[str]:
        return [p.name for p in self.all()]

    def get(self, name: str) -> Optional[SavedPosition]:
        index = self._index(name)
        return None if index is None else self._positions[index]

    def __len__(self) -> int:
        return len(self._positions)

    def _index(self, name: str) -> Optional[int]:
        """Names match without regard to case or surrounding spaces, so
        "window open" and "Window Open " cannot become two entries."""
        key = name.strip().casefold()
        for i, p in enumerate(self._positions):
            if p.name.strip().casefold() == key:
                return i
        return None

    # ------------------------------------------------------------- writing

    def put(self, position: SavedPosition) -> bool:
        """Add `position`, replacing one of the same name. True if replaced."""
        if not position.name.strip():
            raise ValueError("A saved position needs a name.")
        with self._lock:
            index = self._index(position.name)
            if index is None:
                self._positions.append(position)
            else:
                self._positions[index] = position
            self._write()
        return index is not None

    def delete(self, name: str) -> bool:
        with self._lock:
            index = self._index(name)
            if index is None:
                return False
            del self._positions[index]
            self._write()
        return True

    def rename(self, old: str, new: str) -> None:
        new = new.strip()
        if not new:
            raise ValueError("A saved position needs a name.")
        with self._lock:
            index = self._index(old)
            if index is None:
                raise KeyError(old)
            clash = self._index(new)
            if clash is not None and clash != index:
                raise ValueError(f"There is already a saved position called {new!r}.")
            p = self._positions[index]
            self._positions[index] = SavedPosition(
                new, p.orientation, p.actuator_counts, p.actuator_mm,
                p.saved_at, p.note)
            self._write()

    # ------------------------------------------------------------ the file

    def _load(self) -> None:
        if not self.path or not os.path.exists(self.path):
            return
        try:
            with open(self.path, encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError) as exc:
            self._log(f"Saved positions could not be read from {self.path}: {exc}")
            return
        bad = 0
        for entry in data.get("positions", []):
            try:
                self._positions.append(SavedPosition.from_dict(entry))
            except (KeyError, TypeError, ValueError):
                bad += 1
        if bad:
            self._log(f"Saved positions: {bad} unreadable entr"
                      f"{'y' if bad == 1 else 'ies'} in {self.path} were skipped.")

    def _write(self) -> None:
        if not self.path:
            return
        data = {"version": 1,
                "positions": [p.as_dict() for p in self._positions]}
        directory = os.path.dirname(os.path.abspath(self.path))
        try:
            os.makedirs(directory, exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(data, fh, indent=2, sort_keys=True)
                fh.write("\n")
            os.replace(tmp, self.path)
        except OSError as exc:
            self._log(f"Saved positions could not be written to {self.path}: "
                      f"{exc}. They are kept for this session only.")


def make_saved_position(name: str, orientation: Orientation,
                        actuator_counts: Dict[str, int],
                        actuator_mm: Dict[str, float],
                        note: str = "") -> SavedPosition:
    return SavedPosition(name.strip(), orientation, dict(actuator_counts),
                         dict(actuator_mm), time.time(), note.strip())


def default_saved_positions_path(config_path: Optional[str]) -> str:
    """Beside the configuration file, so both belong to the same machine."""
    from .config import default_config_path
    base = config_path or default_config_path()
    return os.path.join(os.path.dirname(os.path.abspath(base)),
                        DEFAULT_SAVED_POSITIONS_FILENAME)


__all__ = ["SavedPosition", "SavedPositions", "SUGGESTED_NAMES",
           "make_saved_position", "default_saved_positions_path",
           "DEFAULT_SAVED_POSITIONS_FILENAME"]
