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

What is still needed before this can drive the real brakes
----------------------------------------------------------
The procedure gives the address of the page but not its protocol, and the two
pages it names are not consistent with each other (172.17.2.14 is called the
PLC on one slide and the 24 V supply on another; 172.17.2.16 is called the
high-voltage GUI on one and the brake page on another). None of that can be
guessed from here. What is needed is one of:

  * the make and model of the device behind that page, or
  * whether it answers Modbus TCP, and on which coil or register, or
  * the URL the page's own buttons POST to, which a browser's network tab
    will show in about a minute.

Until one of those is known, `mode` stays "none": the GUI reports the brakes
as not under software control and says why, rather than showing a state it
cannot actually read. Guessing a URL and firing writes at an unknown device
that controls a brake on a suspended camera is not a reasonable default.

Once known, fill in ExternalBrakeConfig and nothing above this file changes.

Safety
------
Releasing a brake is the dangerous direction: the load is then held by
whatever the drives are doing. `BrakeController.release` therefore takes an
explicit `drives_holding` argument and refuses when it is False, mirroring the
interlock on the motor-driven path.
"""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from typing import Dict, Optional

from .jvl_motor import BrakeState


class BrakeError(RuntimeError):
    """The brake could not be commanded, or its state could not be read."""


@dataclass
class ExternalBrakeConfig:
    """How to reach the device that switches the brakes.

    mode
        "none"    -- not under software control (the honest default today).
        "modbus"  -- the device answers Modbus TCP; brakes are coils or a
                     register bit at `host`.
        "http"    -- the device has an HTTP endpoint; `release_url` and
                     `engage_url` are fetched to switch it.

    all_or_nothing
        True when one switch controls all three brakes together, which is what
        the procedure's wording implies ("switch on brakes for all motors").
        Set False if each has its own control.
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
    #: True when writing 1 releases the brake. A fail-safe brake is
    #: spring-applied and electrically released, so energising normally
    #: releases -- but confirm it rather than assuming.
    energized_releases: bool = True

    # --- http mode ---------------------------------------------------------
    release_url: str = ""
    engage_url: str = ""
    status_url: str = ""
    #: JSON field in the status response holding the state, if there is one.
    status_field: str = "brakes"

    def validate(self) -> None:
        if self.mode not in ("none", "modbus", "http"):
            raise ValueError(
                f"external_brake.mode must be 'none', 'modbus' or 'http', "
                f"got {self.mode!r}"
            )
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


#: Printed wherever the brakes are asked about and cannot be controlled. Says
#: what is missing and how to find it, rather than only that it is missing.
NOT_CONFIGURED_MESSAGE = (
    "The focal-plane brakes are not under software control.\n\n"
    "On this telescope they are switched by a separate device with its own web "
    "page, not by the motors -- which is why the motor's Brake Output register "
    "(179) reads 0. To drive them from here, one of the following is needed:\n"
    "  * the make and model of that device, or\n"
    "  * whether it answers Modbus TCP, and on which coil, or\n"
    "  * the URL its own buttons POST to (a browser's network tab shows this).\n\n"
    "Then fill in external_brake in the configuration. Until then, release and "
    "engage the brakes from that web page as the written procedure describes, "
    "and do it BEFORE moving: the brakes are on when their power is off, and "
    "the motors cannot move the focal plane against them."
)


class BrakeController:
    """Reads and switches brakes that live outside the motors."""

    def __init__(self, cfg: ExternalBrakeConfig,
                 logger=None):
        self.cfg = cfg
        self.cfg.validate()
        self._log = logger or (lambda msg: None)
        self._lock = threading.RLock()
        self._client = None
        #: Last commanded state, used only to report something in HTTP mode
        #: when the device offers no way to read back. Reported as inferred.
        self._assumed: Optional[BrakeState] = None

    @property
    def available(self) -> bool:
        return self.cfg.configured

    def explain_unavailable(self) -> str:
        return NOT_CONFIGURED_MESSAGE

    # ---------------------------------------------------------------- state

    def read_state(self, name: str = "all") -> BrakeState:
        if not self.cfg.configured:
            return BrakeState.UNKNOWN
        if self.cfg.mode == "modbus":
            return self._modbus_read(name)
        return self._http_read()

    def state_is_measured(self, name: str = "all") -> bool:
        """Whether read_state reflects the device or only what we last sent."""
        if self.cfg.mode == "modbus":
            return True
        if self.cfg.mode == "http":
            return bool(self.cfg.status_url)
        return False

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
        if self.cfg.mode == "modbus":
            self._modbus_write(name, release)
        else:
            self._http_write(release)
        self._assumed = BrakeState.RELEASED if release else BrakeState.ENGAGED

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

    def read_state(self, name: str = "all") -> BrakeState:
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
           "SimulatedBrakeController", "NOT_CONFIGURED_MESSAGE"]
