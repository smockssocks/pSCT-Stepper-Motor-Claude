"""
Flat function API for LabVIEW.

Two ways to drive the motors from LabVIEW, both served from this file:

Route A -- TCP bridge (recommended)
-----------------------------------
Run `python -m psct_motors.cli server` and talk to it with LabVIEW's TCP
primitives. Nothing here is needed; see `labview/README.md`. This route does
not care which Python version or bitness LabVIEW was built against, which is
the usual reason a Python node refuses to load.

Route B -- LabVIEW's native Python node
---------------------------------------
Point the Python node at this file and call the `lv_*` functions below.

Every function here obeys the rules that make a Python node painless:

  * only plain scalars in: strings, doubles, int32
  * only plain scalars out: a string (JSON) or a double array
  * no exceptions ever cross the boundary -- errors come back inside the JSON
    as {"ok": false, "error": "..."}, because an exception raised through the
    Python node surfaces in LabVIEW as an opaque code with no message
  * no objects, dicts, or keyword arguments in the signatures
  * one module-level session, opened by `lv_open`, so the block diagram does
    not have to carry a handle around

Typical block diagram order::

    lv_open("C:\\psct\\config\\psct_motors.json", 0)   -> check "ok"
    lv_move(25.0, 0.1, 0.0, 1)                         -> check "ok"
    lv_orientation_array()                             -> [focus, tip, tilt, ...]
    lv_close()

Version note: LabVIEW's Python node supports a specific set of Python versions
per LabVIEW release, and LabVIEW's bitness must match Python's (64-bit LabVIEW
needs 64-bit Python). If the node fails to load this module at all, that
mismatch is the first thing to check -- and Route A avoids the question
entirely.
"""

from __future__ import annotations

import json
import traceback
from typing import Any, Dict, List, Optional

from .config import load_config
from .kinematics import Orientation
from .platform import FocalPlanePlatform

# One session per Python process, which is what the Python node gives us.
_platform: Optional[FocalPlanePlatform] = None
_last_error: str = ""
_log_lines: List[str] = []
_MAX_LOG_LINES = 500


def _log(msg: str) -> None:
    _log_lines.append(msg)
    del _log_lines[:-_MAX_LOG_LINES]


def _ok(result: Any = None) -> str:
    return json.dumps({"ok": True, "result": result})


def _fail(message: str) -> str:
    global _last_error
    _last_error = message
    _log(f"ERROR: {message}")
    return json.dumps({"ok": False, "error": message})


def _guard(fn) -> str:
    """Run `fn`, turning any exception into a JSON error string."""
    try:
        return _ok(fn())
    except Exception as exc:  # noqa: BLE001 - the whole point is to not raise
        _log(traceback.format_exc().strip().splitlines()[-1])
        return _fail(f"{type(exc).__name__}: {exc}")


def _require_platform() -> FocalPlanePlatform:
    if _platform is None:
        raise RuntimeError("No session. Call lv_open first.")
    return _platform


# --------------------------------------------------------------------------
# Session
# --------------------------------------------------------------------------

def lv_open(config_path: str = "", simulate: int = 0) -> str:
    """Load the configuration and connect to all three motors.

    config_path : path to the JSON config; "" uses the default location
    simulate    : 1 to run against fake motors, 0 for real hardware
    """
    def work():
        global _platform
        if _platform is not None:
            try:
                _platform.disconnect()
            except Exception:
                pass
        cfg = load_config(config_path or None)
        _platform = FocalPlanePlatform(
            cfg=cfg, simulate=bool(int(simulate)), logger=_log,
            config_path=config_path or None,
        )
        _platform.connect()
        return {
            "connected": True,
            "simulated": _platform.simulate,
            "actuators": _platform.names,
        }

    return _guard(work)


def lv_close() -> str:
    """Disconnect. Does not stop motion or change the brakes."""
    def work():
        global _platform
        if _platform is not None:
            _platform.disconnect()
            _platform = None
        return {"connected": False}

    return _guard(work)


def lv_is_connected() -> int:
    """1 when connected, 0 otherwise. Never raises."""
    try:
        return 1 if (_platform is not None and _platform.connected) else 0
    except Exception:
        return 0


def lv_last_error() -> str:
    """The most recent error message, or "" if the last call succeeded."""
    return _last_error


def lv_get_log() -> str:
    """Everything the session has logged, newest last, newline separated."""
    return "\n".join(_log_lines)


# --------------------------------------------------------------------------
# Reading
# --------------------------------------------------------------------------

def lv_status() -> str:
    """Full status as a JSON string: all three motors plus the orientation."""
    return _guard(lambda: _require_platform().read_state().as_dict())


def lv_orientation_array() -> List[float]:
    """Orientation as a 5-element double array, for wiring straight to indicators.

        [0] focus_mm
        [1] tip_deg
        [2] tilt_deg
        [3] total_tilt_deg
        [4] tilt_azimuth_deg

    Returns five NaNs if the orientation cannot be read; check lv_last_error.
    """
    try:
        o = _require_platform().read_orientation()
        return [o.focus_mm, o.tip_deg, o.tilt_deg, o.total_tilt_deg, o.tilt_azimuth_deg]
    except Exception as exc:  # noqa: BLE001
        _fail(f"{type(exc).__name__}: {exc}")
        return [float("nan")] * 5


def lv_actuator_positions_mm() -> List[float]:
    """The three actuator positions in mm, in configured order.

    Returns three NaNs on failure; check lv_last_error.
    """
    try:
        return list(_require_platform().read_actuator_positions_mm())
    except Exception as exc:  # noqa: BLE001
        _fail(f"{type(exc).__name__}: {exc}")
        return [float("nan")] * 3


def lv_brake_states() -> str:
    """JSON object mapping actuator name to "engaged"/"released"/"unknown"."""
    return _guard(
        lambda: {n: s.value for n, s in _require_platform().brake_states().items()}
    )


def lv_preview(focus_mm: float, tip_deg: float, tilt_deg: float) -> str:
    """What a move would do, without touching the motors."""

    def work():
        platform = _require_platform()
        target = Orientation(float(focus_mm), float(tip_deg), float(tilt_deg))
        payload: Dict[str, Any] = {
            "orientation": target.as_dict(),
            "actuator_targets_mm": platform.preview(target),
            "within_limits": True,
            "limit_message": "",
        }
        try:
            platform.check_orientation(target)
        except Exception as exc:  # noqa: BLE001
            payload["within_limits"] = False
            payload["limit_message"] = str(exc)
        return payload

    return _guard(work)


# --------------------------------------------------------------------------
# Moving
# --------------------------------------------------------------------------

def lv_move(focus_mm: float, tip_deg: float, tilt_deg: float, wait: int = 1) -> str:
    """Move to an absolute orientation.

    wait = 1 blocks until all three axes arrive; 0 returns once commanded.
    A refused move (limits, faults, a disconnected motor) comes back as
    {"ok": false, ...} with the reason, and nothing will have moved.
    """
    return _guard(lambda: _require_platform().move_to_orientation(
        Orientation(float(focus_mm), float(tip_deg), float(tilt_deg)),
        wait=bool(int(wait)),
    ).as_dict())


def lv_move_relative(d_focus_mm: float, d_tip_deg: float, d_tilt_deg: float,
                     wait: int = 1) -> str:
    """Move relative to the current orientation."""
    return _guard(lambda: _require_platform().move_relative(
        float(d_focus_mm), float(d_tip_deg), float(d_tilt_deg), wait=bool(int(wait))
    ).as_dict())


def lv_move_polar(focus_mm: float, total_tilt_deg: float, azimuth_deg: float,
                  wait: int = 1) -> str:
    """Absolute move, tilt given as magnitude plus azimuth."""
    return _guard(lambda: _require_platform().move_to_polar_tilt(
        float(focus_mm), float(total_tilt_deg), float(azimuth_deg), wait=bool(int(wait))
    ).as_dict())


def lv_jog_actuator(name: str, mm: float, wait: int = 1) -> str:
    """Move ONE actuator by `mm`. Commissioning only -- this tilts the plane."""
    return _guard(lambda: _require_platform().move_actuator_mm(
        str(name), float(mm), relative=True, wait=bool(int(wait))
    ).as_dict())


# --------------------------------------------------------------------------
# Stopping and brakes
# --------------------------------------------------------------------------

def lv_stop() -> str:
    """Controlled stop: decelerate and hold, drives stay enabled.

    Safe to call from a parallel loop while a `lv_move` with wait=1 is still
    running -- the platform's stop path takes no move lock precisely so that a
    stop request never queues behind the move it is meant to interrupt.
    """
    return _guard(lambda: (_require_platform().stop(), {"stopped": True})[1])


def lv_passivate() -> str:
    """Brakes on, drives off. The load then rests on the brakes alone."""
    return _guard(
        lambda: (_require_platform().emergency_passivate(), {"passivated": True})[1]
    )


def lv_brake(action: str) -> str:
    """action is "engage", "release" or "status"."""

    def work():
        platform = _require_platform()
        act = str(action).lower()
        if act == "status":
            return {n: s.value for n, s in platform.brake_states().items()}
        if act in ("engage", "release"):
            return platform.set_all_brakes(engaged=(act == "engage"))
        raise ValueError(
            f"brake action must be 'engage', 'release' or 'status', got {action!r}"
        )

    return _guard(work)


def lv_clear_errors() -> str:
    return _guard(lambda: _require_platform().clear_all_errors())


def lv_set_zero() -> str:
    """Define the current position as focus 0, tip 0, tilt 0, and save it."""
    return _guard(lambda: _require_platform().set_zero_here(persist=True))


# --------------------------------------------------------------------------
# Socket route helper
# --------------------------------------------------------------------------

def send_json_command(command: str, args_json: str = "{}",
                      host: str = "127.0.0.1", port: int = 5020,
                      timeout_s: float = 180.0) -> str:
    """Send one command to a running bridge and return the JSON reply.

    Useful if you want the robustness of the socket route but would rather
    write the JSON in Python than assemble it on the block diagram.
    """
    from .server import send_command
    try:
        args = json.loads(args_json) if args_json else {}
        return json.dumps(send_command(command, args, host=host, port=int(port),
                                       timeout_s=float(timeout_s)))
    except Exception as exc:  # noqa: BLE001
        return _fail(f"{type(exc).__name__}: {exc}")


__all__ = [
    "lv_open", "lv_close", "lv_is_connected", "lv_last_error", "lv_get_log",
    "lv_status", "lv_orientation_array", "lv_actuator_positions_mm",
    "lv_brake_states", "lv_preview",
    "lv_move", "lv_move_relative", "lv_move_polar", "lv_jog_actuator",
    "lv_stop", "lv_passivate", "lv_brake", "lv_clear_errors", "lv_set_zero",
    "send_json_command",
]
