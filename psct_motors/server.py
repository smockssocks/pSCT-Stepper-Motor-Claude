"""
JSON-over-TCP bridge to the focal-plane platform.

This is the recommended way to drive the motors from LabVIEW, and it is also
useful from MATLAB, a shell script, or an observatory control system.

Why a socket rather than LabVIEW's Python node
-----------------------------------------------
LabVIEW's native Python node is fussy: it only supports a fixed set of Python
versions per LabVIEW release, the bitness of LabVIEW and of Python must match,
and a mismatch shows up as an unhelpful error rather than a clear message.
That is the usual reason "LabVIEW cannot talk to my Python script".

A TCP socket sidesteps all of it. LabVIEW's TCP Open Connection / Write / Read
primitives have worked unchanged for twenty years and care nothing about which
Python is installed. You get the same API from any language, you can talk to it
by hand with netcat while debugging, and the Python process can live on a
different machine from LabVIEW if that suits the telescope's layout.

`labview_api.py` provides the Python-node route as well, for anyone who
prefers it.

Protocol
--------
One JSON object per line, request and response both. Send a line, read a line.

    --> {"command": "status"}
    <-- {"ok": true, "result": {...}}

    --> {"command": "move", "args": {"focus_mm": 25.0, "tip_deg": 0.1}}
    <-- {"ok": true, "result": {...}}

    --> {"command": "move", "args": {"focus_mm": 999}}
    <-- {"ok": false, "error": "Move refused, nothing was commanded: ..."}

Errors are always a well-formed response with "ok": false, never a dropped
connection, so a LabVIEW read never hangs waiting for a reply that is not
coming. `send_json_command` in labview_api.py speaks this protocol.

Concurrency and safety
----------------------
Several clients may connect at once, but commands are serialised by a single
lock, so two clients cannot interleave halves of a move. `stop` deliberately
bypasses that lock: a stop request from a second connection must get through
while the first is mid-move. That is the whole point of having one.

The listener binds to 127.0.0.1 by default. There is no authentication here,
so if you bind it to a routable address you are trusting everything that can
reach the port with the focal plane. Put it behind the instrument network's
own access control, or leave it on localhost and run LabVIEW on the same box.
"""

from __future__ import annotations

import json
import socket
import socketserver
import threading
from typing import Any, Callable, Dict, Optional

from .config import load_config
from .jvl_motor import MotorFault
from .kinematics import Orientation
from .platform import FocalPlanePlatform, PlatformError
from .transport import ModbusError

PROTOCOL_VERSION = 1


class CommandDispatcher:
    """Maps protocol commands onto platform operations."""

    #: Commands that must NOT queue behind an in-flight move.
    #:
    #: Everything that changes the machine is serialised by `_lock`, so two
    #: clients cannot interleave halves of a move. These are exempt:
    #:
    #:   stop         a stop that waits its turn is not a stop
    #:   status,      polling progress during a long move is the normal way to
    #:   orientation  drive an indicator; blocking those for the whole move
    #:   ping         would make the bridge look hung
    #:   preview      pure arithmetic, touches no hardware
    #:
    #: The reads are safe to run concurrently because each motor serialises
    #: its own register transactions internally, so a poll can never land in
    #: the middle of another thread's 32-bit access.
    UNSERIALISED = frozenset({"stop", "ping", "status", "orientation", "preview"})

    def __init__(self, platform: FocalPlanePlatform,
                 logger: Optional[Callable[[str], None]] = None):
        self.platform = platform
        self._lock = threading.RLock()
        self._log = logger or (lambda msg: None)

    # ------------------------------------------------------------- dispatch

    def handle(self, request: Dict[str, Any]) -> Dict[str, Any]:
        command = request.get("command")
        if not command:
            return {"ok": False, "error": "Request has no 'command' field."}
        args = request.get("args") or {}
        if not isinstance(args, dict):
            return {"ok": False, "error": "'args' must be an object."}

        handler = getattr(self, f"do_{command}", None)
        if handler is None:
            available = sorted(
                name[3:] for name in dir(self) if name.startswith("do_")
            )
            return {
                "ok": False,
                "error": f"Unknown command {command!r}. Available: {', '.join(available)}",
            }

        try:
            if command in self.UNSERIALISED:
                result = handler(**args)
            else:
                with self._lock:
                    result = handler(**args)
            return {"ok": True, "result": result}
        except TypeError as exc:
            return {"ok": False, "error": f"Bad arguments for {command!r}: {exc}"}
        except (PlatformError, MotorFault, ModbusError, ValueError, KeyError) as exc:
            return {"ok": False, "error": str(exc)}
        except Exception as exc:  # noqa: BLE001 - never drop the connection
            return {"ok": False, "error": f"Unexpected {type(exc).__name__}: {exc}"}

    # ------------------------------------------------------------- commands

    def do_ping(self) -> Dict[str, Any]:
        return {
            "protocol_version": PROTOCOL_VERSION,
            "connected": self.platform.connected,
            "simulated": self.platform.simulate,
            "actuators": self.platform.names,
        }

    def do_connect(self) -> Dict[str, Any]:
        if not self.platform.connected:
            self.platform.connect()
        return {"connected": self.platform.connected}

    def do_disconnect(self) -> Dict[str, Any]:
        self.platform.disconnect()
        return {"connected": self.platform.connected}

    def do_status(self) -> Dict[str, Any]:
        return self.platform.read_state().as_dict()

    def do_orientation(self) -> Dict[str, Any]:
        return self.platform.read_orientation().as_dict()

    def do_preview(self, focus_mm: float, tip_deg: float = 0.0,
                   tilt_deg: float = 0.0) -> Dict[str, Any]:
        target = Orientation(float(focus_mm), float(tip_deg), float(tilt_deg))
        result: Dict[str, Any] = {
            "orientation": target.as_dict(),
            "actuator_targets_mm": self.platform.preview(target),
            "within_limits": True,
            "limit_message": "",
        }
        try:
            self.platform.check_orientation(target)
        except PlatformError as exc:
            result["within_limits"] = False
            result["limit_message"] = str(exc)
        return result

    def do_move(self, focus_mm: float, tip_deg: float = 0.0, tilt_deg: float = 0.0,
                wait: bool = True) -> Dict[str, Any]:
        target = Orientation(float(focus_mm), float(tip_deg), float(tilt_deg))
        self._log(f"move -> {target.describe()}")
        return self.platform.move_to_orientation(target, wait=bool(wait)).as_dict()

    def do_move_relative(self, d_focus_mm: float = 0.0, d_tip_deg: float = 0.0,
                         d_tilt_deg: float = 0.0, wait: bool = True) -> Dict[str, Any]:
        return self.platform.move_relative(
            float(d_focus_mm), float(d_tip_deg), float(d_tilt_deg), wait=bool(wait)
        ).as_dict()

    def do_move_polar(self, focus_mm: float, total_tilt_deg: float,
                      azimuth_deg: float, wait: bool = True) -> Dict[str, Any]:
        return self.platform.move_to_polar_tilt(
            float(focus_mm), float(total_tilt_deg), float(azimuth_deg), wait=bool(wait)
        ).as_dict()

    def do_jog_actuator(self, name: str, mm: float, relative: bool = True,
                        wait: bool = True) -> Dict[str, Any]:
        return self.platform.move_actuator_mm(
            str(name), float(mm), relative=bool(relative), wait=bool(wait)
        ).as_dict()

    def do_stop(self) -> Dict[str, Any]:
        self._log("stop requested")
        self.platform.stop()
        return {"stopped": True}

    def do_passivate(self) -> Dict[str, Any]:
        self._log("passivate requested")
        self.platform.emergency_passivate()
        return {"passivated": True}

    def do_brake(self, action: str) -> Dict[str, Any]:
        action = str(action).lower()
        if action == "status":
            return {name: state.value for name, state in self.platform.brake_states().items()}
        if action in ("engage", "release"):
            return self.platform.set_all_brakes(engaged=(action == "engage"))
        raise ValueError(f"brake action must be 'status', 'engage' or 'release', got {action!r}")

    def do_clear_errors(self) -> Dict[str, int]:
        return self.platform.clear_all_errors()

    def do_set_zero(self, persist: bool = True) -> Dict[str, int]:
        return self.platform.set_zero_here(persist=bool(persist))


class _Handler(socketserver.StreamRequestHandler):
    #: Reject absurd lines rather than buffering without limit.
    max_line_bytes = 64 * 1024

    def handle(self) -> None:
        dispatcher: CommandDispatcher = self.server.dispatcher  # type: ignore[attr-defined]
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        dispatcher._log(f"client connected: {peer}")
        try:
            while True:
                line = self.rfile.readline(self.max_line_bytes)
                if not line:
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    request = json.loads(line.decode("utf-8"))
                    if not isinstance(request, dict):
                        raise ValueError("a request must be a JSON object")
                except (ValueError, UnicodeDecodeError) as exc:
                    response = {"ok": False, "error": f"Malformed request: {exc}"}
                else:
                    response = dispatcher.handle(request)
                self.wfile.write((json.dumps(response) + "\n").encode("utf-8"))
                self.wfile.flush()
        except (ConnectionResetError, BrokenPipeError):
            pass
        finally:
            dispatcher._log(f"client disconnected: {peer}")


class PlatformServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, dispatcher: CommandDispatcher):
        self.dispatcher = dispatcher
        super().__init__(address, _Handler)


def serve(host: str = "127.0.0.1", port: int = 5020,
          config_path: Optional[str] = None, simulate: bool = False,
          platform: Optional[FocalPlanePlatform] = None,
          logger: Optional[Callable[[str], None]] = None) -> int:
    """Run the bridge until interrupted."""
    log = logger or (lambda msg: print(msg, flush=True))
    if platform is None:
        cfg = load_config(config_path)
        platform = FocalPlanePlatform(cfg=cfg, simulate=simulate, logger=log,
                                      config_path=config_path)

    dispatcher = CommandDispatcher(platform, logger=log)
    server = PlatformServer((host, port), dispatcher)

    log(f"psct_motors bridge listening on {host}:{port}"
        + ("  [SIMULATION]" if platform.simulate else ""))
    log("Send one JSON object per line. Try: {\"command\": \"ping\"}")
    if host not in ("127.0.0.1", "localhost", "::1"):
        log("WARNING: bound to a routable address with no authentication. "
            "Anything that can reach this port can move the focal plane.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log("Shutting down. Note this does not stop a move already running.")
    finally:
        server.shutdown()
        server.server_close()
        platform.disconnect()
    return 0


# --------------------------------------------------------------------------
# Client helper
# --------------------------------------------------------------------------

def send_command(command: str, args: Optional[Dict[str, Any]] = None,
                 host: str = "127.0.0.1", port: int = 5020,
                 timeout_s: float = 180.0) -> Dict[str, Any]:
    """One-shot client: connect, send one command, read the reply, close.

    The default timeout is generous because a `move` reply does not arrive
    until the move finishes.
    """
    payload = json.dumps({"command": command, "args": args or {}}) + "\n"
    with socket.create_connection((host, port), timeout=timeout_s) as sock:
        sock.settimeout(timeout_s)
        sock.sendall(payload.encode("utf-8"))
        buffer = b""
        while not buffer.endswith(b"\n"):
            chunk = sock.recv(65536)
            if not chunk:
                raise IOError("The bridge closed the connection without replying.")
            buffer += chunk
    return json.loads(buffer.decode("utf-8"))


__all__ = ["serve", "send_command", "CommandDispatcher", "PlatformServer",
           "PROTOCOL_VERSION"]


if __name__ == "__main__":
    raise SystemExit(serve())
