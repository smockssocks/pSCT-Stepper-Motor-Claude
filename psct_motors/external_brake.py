"""
Brakes that are not on the motor.

What the site actually has
--------------------------
The pSCT motion-control procedure controls the focal-plane brakes from a
separate device with its own web page, not from the motors:

    "Motor brakes ON when power is off = cannot move the focal plane.
     Release these brakes before moving these motors."

That explains something the MacTalk register dump showed and this software
could not otherwise account for: register 179, 'Brake Output', reads 0. It
selects which of the motor's digital outputs drives a brake, and none does,
because the brake is not wired to the motor at all. It is a separate supply,
switched externally.

So a brake control in this software has to reach that device, and this module
is where that goes. `jvl_motor.BrakeStatus` still covers motor-driven brakes
for anyone who wires one that way; this covers the arrangement the pSCT
actually uses.

The device: a ControlByWeb X-432
--------------------------------
The brakes are switched by a ControlByWeb X-432, a web-enabled PLC with 16
relays and 18 digital inputs. `mode = "controlbyweb"` drives it over HTTP,
which is what its own web page uses:

    GET /state.json                   every relay and input, as JSON
    GET /state.json?relay3=1          switch relay 3 on (0 = off)

The tag names are the device's "Local I/O Numbers" -- relay1..relay16,
digitalInput1..digitalInput18 -- and only I/O that has been given one appears
at all. HTTP was chosen over the device's Modbus TCP because the state page
can be opened in a browser, so what this software sees can be checked by eye
in a second, and the relay numbers are the ones printed on the unit rather
than a coil address table that has to be looked up.

How the relays are wired to the brakes is NOT known from here. So nothing
about it is assumed:

  * which relay drives which brake is configured (`relays`), either one relay
    for all three ({"all": 1}) or one per actuator;
  * which way round it is -- whether energising the relay releases the brake,
    which it does for a spring-applied brake powered to release -- is
    configured (`energized_releases`);
  * whether anything reports what the brake is *actually* doing is
    configured (`feedback_inputs`), and when nothing does, the state shown is
    labelled as the relay's state rather than the brake's.

That last point is not a formality. A relay reading "off" says the PLC was
told to turn it off. It does not say the brake clamped: a wiring mistake, a
blown fuse or a reversed polarity would all read the same. EMERGENCY turns
the drives off only when the brakes are *confirmed* holding, and without a
feedback input they cannot be, so the drives stay on. That is deliberate.

Other devices
-------------
"modbus" (a coil per brake) and "http" (fixed URLs to POST to) are kept for a
device that works that way. "none" -- the brakes not under software control
-- is still the default: firing writes at a device nobody has configured is
not a reasonable default for a brake on a suspended camera.

Safety
------
Releasing a brake is the dangerous direction: the load is then held by
whatever the drives are doing. `BrakeController.release` therefore takes an
explicit `drives_holding` argument and refuses when it is False, mirroring the
interlock on the motor-driven path.
"""

from __future__ import annotations

import base64
import json
import re
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from .jvl_motor import BrakeState

#: Brake-controller modes, and what each is called on screen.
MODES = ("none", "controlbyweb", "modbus", "http")


def _plain_reason(reason, timeout: float) -> str:
    """What a network failure means, in words that point at a cause."""
    import socket
    text = str(reason)
    if isinstance(reason, (socket.timeout, TimeoutError)) or "timed out" in text:
        return (f"no answer within {timeout:g} s (timed out: the PLC was slow "
                "or busy, or the packets were lost)")
    if isinstance(reason, ConnectionRefusedError) or "refused" in text.lower():
        return ("connection refused (the PLC is up but would not take the "
                "connection, e.g. too many at once, or it is rebooting)")
    if isinstance(reason, ConnectionResetError) or "reset" in text.lower():
        return "connection reset by the PLC part-way through"
    if "unreachable" in text.lower() or "no route" in text.lower():
        return f"network unreachable ({text}): a cable, switch or address problem"
    return text


class BrakeError(RuntimeError):
    """The brake could not be commanded, or its state could not be read."""


@dataclass
class ExternalBrakeConfig:
    """How to reach the device that switches the brakes.

    mode
        "none"          -- not under software control (the default).
        "controlbyweb"  -- a ControlByWeb X-400-series PLC such as the X-432,
                           over HTTP: relays switch the brakes, and optional
                           digital inputs report what they actually did.
        "modbus"        -- the device answers Modbus TCP; brakes are coils.
        "http"          -- fixed URLs are POSTed to switch the brakes.

    all_or_nothing
        True when one switch controls all three brakes together, which is what
        the procedure's wording implies ("switch on brakes for all motors").
        Set False if each has its own control. Even with separate relays,
        leaving this True switches all three together, which is the safe way
        to run a three-point mount.
    """

    mode: str = "none"
    host: str = ""
    port: int = 502
    unit_id: int = 1
    timeout_s: float = 3.0
    all_or_nothing: bool = True

    # --- modbus mode -------------------------------------------------------
    #: Coil address per actuator name, or a single entry keyed "all".
    coils: Dict[str, int] = field(default_factory=dict)
    #: True when turning the output ON (relay energised, coil 1) releases the
    #: brake. A fail-safe brake is spring-applied and electrically released,
    #: so energising normally releases -- but that depends on how the relay is
    #: wired, which is why it is a setting. Applies to "controlbyweb" too.
    energized_releases: bool = True

    # --- http mode ---------------------------------------------------------
    release_url: str = ""
    engage_url: str = ""
    status_url: str = ""
    #: JSON field in the status response holding the state, if there is one.
    status_field: str = "brakes"

    # --- controlbyweb mode -------------------------------------------------
    #: Which relay switches which brake: {"all": 1} for one relay switching
    #: every brake, or {"Top": 1, "East": 2, "West": 3}. The numbers are the
    #: relays' Local I/O Numbers, as printed in the device's state.json.
    relays: Dict[str, int] = field(default_factory=dict)
    #: Optional digital inputs that report what each brake actually did -- a
    #: limit switch on the brake, or a sense line on the brake supply. With
    #: one configured for every brake, the brake state is a measurement.
    #: Without, it is only the relay's state, and is labelled that way.
    feedback_inputs: Dict[str, int] = field(default_factory=dict)
    #: True when a feedback input reading ON means the brake is released.
    feedback_on_means_released: bool = True
    http_port: int = 80
    use_https: bool = False
    #: The device's login, if it has a password set for its state page.
    username: str = "admin"
    password: str = ""

    def validate(self) -> None:
        if self.mode not in MODES:
            raise ValueError(
                f"external_brake.mode must be one of {', '.join(MODES)}; "
                f"got {self.mode!r}"
            )
        if self.mode == "controlbyweb":
            if not self.host:
                raise ValueError(
                    "external_brake.mode is 'controlbyweb' but no host is set. "
                    "Put the PLC's IP address in external_brake.host.")
            if not self.relays:
                raise ValueError(
                    "external_brake.mode is 'controlbyweb' but no relays are "
                    "mapped. Set relays to {'all': <relay number>} or one "
                    "entry per actuator.")
            for kind, mapping in (("relay", self.relays),
                                  ("feedback input", self.feedback_inputs)):
                for name, number in mapping.items():
                    if not isinstance(number, int) or isinstance(number, bool) \
                            or number < 1:
                        raise ValueError(
                            f"external_brake: {kind} for {name!r} must be a "
                            f"whole number from 1, got {number!r}")
            if not (0 < int(self.http_port) < 65536):
                raise ValueError("external_brake.http_port must be 1..65535")
        if self.mode == "modbus":
            if not self.host:
                raise ValueError("external_brake.mode is 'modbus' but no host is set")
            if not self.coils:
                raise ValueError(
                    "external_brake.mode is 'modbus' but no coils are mapped. "
                    "Set coils to {'all': <address>} or one entry per actuator."
                )
        if self.mode == "http":
            if not (self.release_url and self.engage_url):
                raise ValueError(
                    "external_brake.mode is 'http' but release_url or engage_url "
                    "is empty."
                )

    @property
    def configured(self) -> bool:
        return self.mode != "none"

    @classmethod
    def from_settings(cls, settings) -> "ExternalBrakeConfig":
        """Build from config.ExternalBrakeSettings (same field names)."""
        from dataclasses import fields
        names = {f.name for f in fields(cls)}
        values = {name: getattr(settings, name) for name in names
                  if hasattr(settings, name)}
        for key in ("coils", "relays", "feedback_inputs"):
            if key in values:
                values[key] = dict(values[key])
        return cls(**values)


# --------------------------------------------------------------------------
# Reading a ControlByWeb state page
# --------------------------------------------------------------------------

_RELAY_KEY = re.compile(r"^relay(\d+)(?:state)?$", re.IGNORECASE)
_INPUT_KEY = re.compile(r"^(?:digitalinput|input)(\d+)(?:state)?$", re.IGNORECASE)


def _on_off(value) -> Optional[bool]:
    """A ControlByWeb I/O value as on (True) / off (False), or None."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "on", "true", "closed", "energized", "energised"):
            return True
        if text in ("0", "off", "false", "open", "de-energized", "de-energised"):
            return False
        try:
            return float(text) != 0
        except ValueError:
            return None
    return None


def parse_state(payload: Dict[str, object]) -> Dict[str, Dict[int, bool]]:
    """Pick the relays and digital inputs out of a state.json / state.xml.

    Tolerant on purpose: the X-400 series names them relay1 and
    digitalInput1, older ControlByWeb units relay1state and input1state, and
    values arrive as numbers or strings depending on firmware. Anything that
    is not a relay or a digital input (analog inputs, temperatures, the
    serial number) is ignored here and kept in `raw` by the caller.
    """
    relays: Dict[int, bool] = {}
    inputs: Dict[int, bool] = {}
    for key, value in payload.items():
        state = _on_off(value)
        if state is None:
            continue
        match = _RELAY_KEY.match(str(key))
        if match:
            relays[int(match.group(1))] = state
            continue
        match = _INPUT_KEY.match(str(key))
        if match:
            inputs[int(match.group(1))] = state
    return {"relays": relays, "inputs": inputs}


def _flatten(payload) -> Dict[str, object]:
    """state.json is flat on the X-400, but tolerate one level of nesting."""
    if not isinstance(payload, dict):
        return {}
    flat: Dict[str, object] = {}
    for key, value in payload.items():
        if isinstance(value, dict):
            for inner_key, inner in value.items():
                flat[str(inner_key)] = inner
        else:
            flat[str(key)] = value
    return flat


#: Printed wherever the brakes are asked about and cannot be controlled. Says
#: what is missing and how to find it, rather than only that it is missing.
NOT_CONFIGURED_MESSAGE = (
    "The focal-plane brakes are not under software control.\n\n"
    "On this telescope they are switched by a separate device with its own web "
    "page, not by the motors -- which is why the motor's Brake Output register "
    "(179) reads 0. That device is a ControlByWeb X-432 PLC. To drive it from "
    "here, open Tools > Brake controller (PLC) and enter its IP address and "
    "which relay switches the brakes (or set external_brake.mode to "
    "'controlbyweb' in the configuration). Other devices can be driven over "
    "Modbus TCP coils or fixed HTTP URLs.\n\n"
    "Until then, release and engage the brakes from that web page as the "
    "written procedure describes, and do it BEFORE moving: the brakes are on "
    "when their power is off, and the motors cannot move the focal plane "
    "against them."
)


class BrakeController:
    """Reads and switches brakes that live outside the motors."""

    #: How long a ControlByWeb status read is reused. The GUI polls several
    #: times a second while a move runs; the brakes do not change that fast,
    #: and a small PLC should not be asked that often.
    STATUS_CACHE_S = 0.4
    #: After a failed read, how long to report the same failure without
    #: trying again. An unreachable PLC would otherwise cost every status poll
    #: a full timeout, and the motor readouts would freeze behind it.
    FAILURE_BACKOFF_S = 2.0
    #: A status read that fails is tried once more straight away. One slow
    #: or dropped reply from a small web PLC is common; reporting the brakes
    #: as unreadable for two seconds because of it is not useful.
    STATUS_RETRIES = 1

    def __init__(self, cfg: ExternalBrakeConfig, logger=None,
                 names: Optional[Sequence[str]] = None):
        self.cfg = cfg
        self.cfg.validate()
        self._log = logger or (lambda msg: None)
        self._lock = threading.RLock()
        self._client = None
        #: The actuators' names, so "every brake has feedback" can be judged.
        self.names: List[str] = list(names or [])
        #: Last commanded state, used only to report something in HTTP mode
        #: when the device offers no way to read back. Reported as inferred.
        self._assumed: Optional[BrakeState] = None
        # ControlByWeb status cache.
        self._io: Optional[dict] = None
        self._io_time = 0.0
        self._io_error: Optional[str] = None
        self._io_error_time = 0.0
        #: How the link has been doing, for diagnosing a flaky connection:
        #: reads, reads that needed the retry, reads that failed outright,
        #: the slowest reply, and the last few failure reasons.
        self.link_stats = {"reads": 0, "retried": 0, "failed": 0,
                           "slowest_s": 0.0, "recent_errors": []}
        # Never through a proxy: the PLC is on the local network, and on a
        # Windows machine Python otherwise picks up the system proxy settings.
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        #: Which state page the device answered on, once known.
        self._endpoint = "state.json"

    @property
    def available(self) -> bool:
        return self.cfg.configured

    @property
    def all_or_nothing(self) -> bool:
        """True when one command switches every brake."""
        return (self.cfg.all_or_nothing or "all" in self.cfg.relays
                or "all" in self.cfg.coils)

    def explain_unavailable(self) -> str:
        return NOT_CONFIGURED_MESSAGE

    def describe(self) -> str:
        if self.cfg.mode == "controlbyweb":
            return f"ControlByWeb PLC at {self._base_url()}"
        if self.cfg.mode == "modbus":
            return f"Modbus brake controller at {self.cfg.host}:{self.cfg.port}"
        if self.cfg.mode == "http":
            return "HTTP brake controller"
        return "no brake controller"

    # ---------------------------------------------------------------- state

    def read_state(self, name: str = "all", fresh: bool = False) -> BrakeState:
        """The brake state. `fresh` bypasses the status cache, which an
        interlock that is about to act on the answer must do."""
        if not self.cfg.configured:
            return BrakeState.UNKNOWN
        if self.cfg.mode == "controlbyweb":
            return self._cbw_state(name, self._cbw_io(fresh))
        if self.cfg.mode == "modbus":
            return self._modbus_read(name)
        return self._http_read()

    def state_is_measured(self, name: str = "all") -> bool:
        """Whether read_state reports the brake itself.

        False means it reports only what the device was told to do -- the
        relay's state, or the last command sent. The difference decides
        whether EMERGENCY may turn the drives off: it may only when the brakes
        are confirmed holding, and a relay turned off is not a clamped brake.
        """
        if self.cfg.mode == "controlbyweb":
            return bool(self._cbw_feedback_for(name))
        if self.cfg.mode == "modbus":
            return True
        if self.cfg.mode == "http":
            return bool(self.cfg.status_url)
        return False

    def is_holding(self, name: str) -> bool:
        """Whether this brake is clamping, from the last reading only.

        Asked by a *simulated* motor on every physics step, which is how a
        bench rig with one real motor and a real PLC behaves as if the brakes
        were fitted: a stood-in axis will not turn while the PLC says its
        brake is on. It never touches the network. With no reading yet, or an
        unreadable one, the brake is taken to be holding -- these brakes are
        spring-applied, so "not known to be released" is the honest default.
        """
        if self.cfg.mode != "controlbyweb":
            return False
        with self._lock:
            io = self._io
        if io is None:
            return True
        try:
            return self._cbw_state(name, io) is not BrakeState.RELEASED
        except BrakeError:
            return True

    def read_io(self, fresh: bool = True) -> dict:
        """Every relay and digital input the PLC reports, plus the raw page.

        For the GUI's PLC view and `cli plc`: seeing every I/O point at once
        is how the wiring gets worked out.
        """
        if self.cfg.mode != "controlbyweb":
            raise BrakeError("Only a ControlByWeb brake controller can list its I/O.")
        return self._cbw_io(fresh)

    # -------------------------------------------------------------- control

    def release(self, name: str = "all", drives_holding: bool = False) -> None:
        """Release the brake. Refuses unless the drives are holding.

        The brake is what stops the camera moving when nothing else is. Taking
        it off while the drives are passive leaves the load on friction alone,
        so the caller has to state that the drives are enabled and holding --
        it cannot be inferred from here, since this device knows nothing about
        the motors.
        """
        self._require_configured()
        if not drives_holding:
            raise BrakeError(
                "Refusing to release the brakes: the drives are not confirmed "
                "to be enabled and holding. Enable Position mode on all three "
                "motors first. With the brakes off and the drives passive, "
                "nothing is holding the focal plane."
            )
        self._switch(name, release=True)
        self._log(f"External brake released ({name}).")

    def engage(self, name: str = "all") -> None:
        self._require_configured()
        self._switch(name, release=False)
        self._log(f"External brake engaged ({name}).")

    def _require_configured(self) -> None:
        if not self.cfg.configured:
            raise BrakeError(NOT_CONFIGURED_MESSAGE)

    def _switch(self, name: str, release: bool) -> None:
        if self.cfg.mode == "controlbyweb":
            self._cbw_write(name, release)
        elif self.cfg.mode == "modbus":
            self._modbus_write(name, release)
        else:
            self._http_write(release)
        self._assumed = BrakeState.RELEASED if release else BrakeState.ENGAGED

    # --------------------------------------------------------- controlbyweb

    def _base_url(self) -> str:
        scheme = "https" if self.cfg.use_https else "http"
        default = 443 if self.cfg.use_https else 80
        port = "" if int(self.cfg.http_port) == default else f":{self.cfg.http_port}"
        return f"{scheme}://{self.cfg.host}{port}"

    def _cbw_relays_for(self, name: str) -> List[int]:
        relays = self.cfg.relays
        if "all" in relays:
            return [relays["all"]]
        if name == "all" or self.cfg.all_or_nothing:
            return sorted(set(relays.values()))
        if name not in relays:
            raise BrakeError(
                f"No relay is mapped for the {name!r} brake. Mapped: "
                f"{', '.join(sorted(relays))}.")
        return [relays[name]]

    def _cbw_feedback_for(self, name: str) -> List[int]:
        """Inputs that report the brake(s) `name` refers to -- all of them,
        or none. Partial feedback confirms nothing about the brakes it
        misses, so it counts as none."""
        inputs = self.cfg.feedback_inputs
        if not inputs:
            return []
        if "all" in inputs:
            return [inputs["all"]]
        if name == "all" or self.all_or_nothing:
            wanted = self.names or [n for n in self.cfg.relays if n != "all"]
            if not wanted or any(n not in inputs for n in wanted):
                return []
            return sorted({inputs[n] for n in wanted})
        return [inputs[name]] if name in inputs else []

    def _cbw_state(self, name: str, io: dict) -> BrakeState:
        feedback = self._cbw_feedback_for(name)
        if feedback:
            released = []
            for number in feedback:
                level = io["inputs"].get(number)
                if level is None:
                    raise BrakeError(
                        f"The PLC does not report digital input {number}, which "
                        f"is configured as brake feedback. Give it Local I/O "
                        f"Number {number} on the PLC, or fix feedback_inputs.")
                released.append(level if self.cfg.feedback_on_means_released
                                else not level)
        else:
            released = []
            for number in self._cbw_relays_for(name):
                on = io["relays"].get(number)
                if on is None:
                    raise BrakeError(
                        f"The PLC does not report relay {number}. Give that "
                        f"relay Local I/O Number {number} on the PLC, or fix "
                        f"the relay mapping.")
                released.append(on if self.cfg.energized_releases else not on)
        if all(released):
            return BrakeState.RELEASED
        if not any(released):
            return BrakeState.ENGAGED
        return BrakeState.UNKNOWN

    def _cbw_io(self, fresh: bool) -> dict:
        now = time.monotonic()
        with self._lock:
            if not fresh:
                if (self._io_error is not None
                        and now - self._io_error_time < self.FAILURE_BACKOFF_S):
                    raise BrakeError(self._io_error)
                if self._io is not None and now - self._io_time < self.STATUS_CACHE_S:
                    return self._io
            io = None
            for attempt in range(self.STATUS_RETRIES + 1):
                began = time.monotonic()
                try:
                    io = self._cbw_request(None, timeout=min(self.cfg.timeout_s, 1.5))
                    break
                except BrakeError as exc:
                    self._note_link_error(str(exc))
                    if attempt == self.STATUS_RETRIES:
                        self.link_stats["failed"] += 1
                        if self._io_error is None:
                            self._log(f"Lost the PLC: {exc}")
                        self._io_error, self._io_error_time = str(exc), now
                        raise
                    self.link_stats["retried"] += 1
                    if self._io_error is None:
                        self._log(f"PLC missed one read, trying again: {exc}")
            elapsed = time.monotonic() - began
            if self._io_error is not None:
                self._log("PLC answering again.")
            self.link_stats["reads"] += 1
            self.link_stats["slowest_s"] = max(self.link_stats["slowest_s"], elapsed)
            self._io_error = None
            self._io, self._io_time = io, now
            return io

    def _note_link_error(self, text: str) -> None:
        recent = self.link_stats["recent_errors"]
        recent.append((time.strftime("%H:%M:%S"), text))
        del recent[:-10]

    def _cbw_write(self, name: str, release: bool) -> None:
        energize = release if self.cfg.energized_releases else not release
        numbers = self._cbw_relays_for(name)
        query = {f"relay{n}": 1 if energize else 0 for n in numbers}
        with self._lock:
            io = self._cbw_request(query, timeout=self.cfg.timeout_s)
            wrong = [n for n in numbers if io["relays"].get(n) is not energize]
            if wrong:
                # The reply may predate the change on some firmware: look once
                # more before calling it a failure.
                time.sleep(0.2)
                io = self._cbw_request(None, timeout=self.cfg.timeout_s)
                wrong = [n for n in numbers if io["relays"].get(n) is not energize]
            self._io, self._io_time, self._io_error = io, time.monotonic(), None
        if wrong:
            raise BrakeError(
                f"The PLC was told to turn relay(s) "
                f"{', '.join(str(n) for n in wrong)} "
                f"{'on' if energize else 'off'} but reports "
                + ", ".join(f"relay {n} "
                            f"{'missing' if io['relays'].get(n) is None else ('on' if io['relays'][n] else 'off')}"
                            for n in wrong)
                + ". Check the relay numbers, and that the PLC's own logic "
                "is not overriding them.")

    def _cbw_request(self, query: Optional[Dict[str, int]], timeout: float) -> dict:
        """One GET of the state page, optionally setting relays, parsed."""
        headers = {}
        if self.cfg.password:
            token = base64.b64encode(
                f"{self.cfg.username}:{self.cfg.password}".encode()).decode()
            headers["Authorization"] = f"Basic {token}"
        suffix = ("?" + urllib.parse.urlencode(query)) if query else ""
        endpoints = [self._endpoint] + [e for e in ("state.json", "state.xml")
                                        if e != self._endpoint]
        last_error = ""
        for endpoint in endpoints:
            url = f"{self._base_url()}/{endpoint}{suffix}"
            try:
                request = urllib.request.Request(url, headers=headers)
                with self._opener.open(request, timeout=timeout) as response:
                    body = response.read().decode("utf-8", "replace")
            except urllib.error.HTTPError as exc:
                if exc.code == 401:
                    raise BrakeError(
                        f"The PLC at {self._base_url()} asked for a login. Set "
                        "its user name and password in the brake controller "
                        "settings.") from exc
                if exc.code == 404:
                    last_error = f"{endpoint} not found"
                    continue
                raise BrakeError(
                    f"The PLC at {self._base_url()} answered {exc.code} to "
                    f"{endpoint}.") from exc
            except (urllib.error.URLError, OSError) as exc:
                reason = getattr(exc, "reason", exc)
                raise BrakeError(
                    f"Could not reach the PLC at {self._base_url()}: "
                    f"{_plain_reason(reason, timeout)}"
                ) from exc
            payload = self._parse_body(endpoint, body)
            self._endpoint = endpoint
            io = parse_state(payload)
            io["raw"] = payload
            return io
        raise BrakeError(
            f"The PLC at {self._base_url()} has no state page ({last_error}). "
            "Is this a ControlByWeb X-400-series device?")

    @staticmethod
    def _parse_body(endpoint: str, body: str) -> Dict[str, object]:
        try:
            if endpoint.endswith(".json"):
                return _flatten(json.loads(body))
            root = ElementTree.fromstring(body)
            return {child.tag: (child.text or "").strip() for child in root}
        except (ValueError, ElementTree.ParseError) as exc:
            raise BrakeError(
                f"The PLC's {endpoint} could not be read as "
                f"{'JSON' if endpoint.endswith('.json') else 'XML'}: {exc}"
            ) from exc

    # --------------------------------------------------------------- modbus

    def _coil_for(self, name: str) -> int:
        if self.cfg.all_or_nothing or name == "all":
            if "all" in self.cfg.coils:
                return self.cfg.coils["all"]
            # One switch for everything, but mapped per name: any will do.
            return next(iter(self.cfg.coils.values()))
        try:
            return self.cfg.coils[name]
        except KeyError:
            raise BrakeError(
                f"No brake coil mapped for {name!r}. Known: "
                f"{sorted(self.cfg.coils)}"
            ) from None

    def _modbus_client(self):
        with self._lock:
            if self._client is None:
                try:
                    from pymodbus.client import ModbusTcpClient
                except ImportError as exc:
                    raise BrakeError(
                        "pymodbus is needed to drive the brakes over Modbus."
                    ) from exc
                try:
                    self._client = ModbusTcpClient(
                        self.cfg.host, port=self.cfg.port,
                        timeout=self.cfg.timeout_s, retries=1)
                except TypeError:
                    self._client = ModbusTcpClient(self.cfg.host,
                                                   port=self.cfg.port)
                if not self._client.connect():
                    self._client = None
                    raise BrakeError(
                        f"Could not reach the brake controller at "
                        f"{self.cfg.host}:{self.cfg.port}."
                    )
            return self._client

    def _modbus_call(self, method_name: str, *args):
        client = self._modbus_client()
        method = getattr(client, method_name)
        for keyword in ("device_id", "slave", "unit"):
            try:
                return method(*args, **{keyword: self.cfg.unit_id})
            except TypeError:
                continue
        return method(*args)

    def _modbus_read(self, name: str) -> BrakeState:
        try:
            result = self._modbus_call("read_coils", self._coil_for(name), 1)
        except BrakeError:
            raise
        except Exception as exc:
            raise BrakeError(f"Could not read the brake coil: {exc}") from exc
        if result is None or (hasattr(result, "isError") and result.isError()):
            raise BrakeError(f"Brake coil read returned an error: {result}")
        bits = list(getattr(result, "bits", []) or [])
        if not bits:
            raise BrakeError("Brake coil read returned no data.")
        energized = bool(bits[0])
        released = energized if self.cfg.energized_releases else not energized
        return BrakeState.RELEASED if released else BrakeState.ENGAGED

    def _modbus_write(self, name: str, release: bool) -> None:
        energize = release if self.cfg.energized_releases else not release
        try:
            result = self._modbus_call("write_coil", self._coil_for(name),
                                       bool(energize))
        except BrakeError:
            raise
        except Exception as exc:
            raise BrakeError(f"Could not write the brake coil: {exc}") from exc
        if result is None or (hasattr(result, "isError") and result.isError()):
            raise BrakeError(f"Brake coil write returned an error: {result}")

    # ----------------------------------------------------------------- http

    def _http_write(self, release: bool) -> None:
        url = self.cfg.release_url if release else self.cfg.engage_url
        try:
            request = urllib.request.Request(url, method="POST")
            with urllib.request.urlopen(request, timeout=self.cfg.timeout_s) as response:
                if response.status >= 400:
                    raise BrakeError(
                        f"The brake controller answered {response.status} to "
                        f"{url}")
        except urllib.error.URLError as exc:
            raise BrakeError(f"Could not reach {url}: {exc}") from exc

    def _http_read(self) -> BrakeState:
        if not self.cfg.status_url:
            # No way to ask, so report what we last sent -- and callers are
            # told this is not a measurement via state_is_measured().
            return self._assumed or BrakeState.UNKNOWN
        try:
            with urllib.request.urlopen(self.cfg.status_url,
                                        timeout=self.cfg.timeout_s) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except (urllib.error.URLError, ValueError) as exc:
            raise BrakeError(
                f"Could not read the brake status from {self.cfg.status_url}: "
                f"{exc}") from exc
        value = payload.get(self.cfg.status_field)
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in ("released", "off", "open", "0", "false"):
                return BrakeState.RELEASED
            if lowered in ("engaged", "on", "closed", "1", "true"):
                return BrakeState.ENGAGED
        if isinstance(value, bool):
            return BrakeState.RELEASED if not value else BrakeState.ENGAGED
        return BrakeState.UNKNOWN

    def close(self) -> None:
        with self._lock:
            if self._client is not None:
                try:
                    self._client.close()
                except Exception:
                    pass
                self._client = None


class SimulatedBrakeController:
    """A stand-in for the site's brake device, for `--simulate`.

    The real device is not configured yet -- nobody here knows its protocol --
    so without this, nothing in simulation could exercise the brake interlocks,
    and the interlocks are the part most worth rehearsing: they are what stops
    a released brake dropping the focal plane.

    It answers the same questions as `BrakeController` and additionally holds
    the axes: `is_holding(name)` is consulted by the simulated motors, so a
    move commanded against an engaged brake behaves like the real thing --
    torque climbs and the shaft does not turn -- rather than sailing through.

    It is deliberately obvious about being fake: `describe()` says so, and the
    platform logs it on connect. Nobody should be able to mistake a rehearsal
    for the real brakes being under software control.
    """

    def __init__(self, names, all_or_nothing: bool = True, logger=None):
        self.names = list(names)
        self.all_or_nothing = all_or_nothing
        self._log = logger or (lambda msg: None)
        # Brakes are spring-applied: no power means engaged. Simulation starts
        # in the state the site's procedure describes finding them in.
        self._engaged = {name: True for name in self.names}
        #: Set False to simulate the brake supply being off, which on a
        #: fail-safe brake means the brakes clamp and cannot be released.
        self.powered = True

    # ------------------------------------------------------ same interface

    @property
    def available(self) -> bool:
        return True

    def describe(self) -> str:
        return "simulated brake controller (no real brakes are being switched)"

    def explain_unavailable(self) -> str:
        return ""

    def state_is_measured(self, name: str = "all") -> bool:
        return True

    def read_state(self, name: str = "all", fresh: bool = False) -> BrakeState:
        if name == "all" or self.all_or_nothing:
            if all(self._engaged.values()):
                return BrakeState.ENGAGED
            if not any(self._engaged.values()):
                return BrakeState.RELEASED
            return BrakeState.UNKNOWN
        return (BrakeState.ENGAGED if self._engaged[self._require_name(name)]
                else BrakeState.RELEASED)

    def release(self, name: str = "all", drives_holding: bool = False) -> None:
        if not drives_holding:
            raise BrakeError(
                "Refusing to release the brakes: the drives are not confirmed "
                "to be enabled and holding. Enable Position mode on all three "
                "motors first. With the brakes off and the drives passive, "
                "nothing is holding the focal plane."
            )
        if not self.powered:
            raise BrakeError(
                "The brakes did not release: there is no power to the brake "
                "supply. These brakes are spring-applied and electrically "
                "released, so with the supply off they clamp and the motors "
                "cannot move the focal plane against them."
            )
        self._set(name, engaged=False)
        self._log(f"Simulated brake released ({name}).")

    def engage(self, name: str = "all") -> None:
        self._set(name, engaged=True)
        self._log(f"Simulated brake engaged ({name}).")

    def close(self) -> None:
        return None

    # --------------------------------------------------------- simulation

    def is_holding(self, name: str) -> bool:
        """Whether this actuator's brake is currently clamping the shaft."""
        if not self.powered:
            return True
        return self._engaged.get(name, True)

    def set_powered(self, powered: bool) -> None:
        """Turn the brake supply on or off.

        With it off the brakes clamp, which is the safe direction and the
        state the site's procedure warns about: "motor brakes ON when power is
        off = cannot move the focal plane".
        """
        self.powered = bool(powered)
        self._log(f"Simulated brake supply {'on' if powered else 'OFF'}.")

    def _require_name(self, name: str) -> str:
        if name not in self._engaged:
            raise BrakeError(
                f"No brake named {name!r}. Known: {sorted(self._engaged)}")
        return name

    def _set(self, name: str, engaged: bool) -> None:
        if name == "all" or self.all_or_nothing:
            for key in self._engaged:
                self._engaged[key] = engaged
        else:
            self._engaged[self._require_name(name)] = engaged


__all__ = ["ExternalBrakeConfig", "BrakeController", "BrakeError",
           "SimulatedBrakeController", "NOT_CONFIGURED_MESSAGE", "MODES",
           "parse_state"]
