"""
Configuration model for the pSCT focal-plane actuator system.

Everything the code needs to know about the hardware lives here, is loaded
from a single JSON file, and is round-trippable. There are no magic numbers
buried in the driver, the GUI or the kinematics.

The one number you are most likely to need to change is `counts_per_mm`.
Read the note on `ActuatorConfig` below before you touch it.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, asdict, replace
from typing import Any, Dict, List, Optional

from .registers import WordOrder

DEFAULT_CONFIG_FILENAME = "psct_motors.json"


# --------------------------------------------------------------------------
# Brake
# --------------------------------------------------------------------------

@dataclass
class BrakeConfig:
    """How this actuator's holding brake is wired and controlled.

    The pSCT motors are ordered with a brake, but *how the brake is driven* is
    an installation choice, so this is configurable rather than assumed. Three
    modes are supported:

    "none"
        No brake, or the brake is not under software control. The GUI shows
        the brake state as unknown and the control is disabled.

    "auto"
        The brake is wired so that the motor's own firmware releases it. On a
        JVL drive this means the brake follows the drive-enable state: the
        brake releases when the motor leaves Passive mode and re-engages when
        it returns to Passive. Software cannot command it independently, so
        the GUI shows an *inferred* state derived from MODE_REG and labels it
        as inferred. This is the safest assumption and is the default.

    "output"
        The brake is wired to one of the motor's digital outputs, and software
        releases it by setting that output. Set `output_register` to the JVL
        register holding the outputs, `output_bit` to the bit index, and
        `energized_releases` to describe the polarity.

    Polarity
    --------
    A fail-safe holding brake is spring-applied and electrically released: no
    power means the brake is ON (holding). With `energized_releases = True`
    (the default, and the normal wiring), driving the output HIGH releases the
    brake. If your brake is wired the other way round, set it to False.

    What the pSCT motor says today
    ------------------------------
    MacTalk's register list has register 179, 'Brake Output', which selects
    WHICH digital output drives the brake -- the same pattern as 137 ('In
    Position' Output) and 138 ('Error' Output). On the pSCT motor it reads 0,
    meaning **no output is assigned to the brake function**.

    So the default here is "none". Setting it to "auto" would have the GUI
    infer a brake state from the drive mode with nothing behind the inference,
    and "output" would toggle an output the brake is not wired to. Neither is
    honest until register 179 is set, or until you confirm the brake is wired
    some other way.

    `check_against_motor` reads register 179 and reports whether the
    configuration matches what the drive is set up to do.

    Verifying
    ---------
    `python -m psct_motors.cli probe-brake --motor A` toggles the configured
    output with the motor passive and prompts you to confirm you heard the
    brake click. Do that once per motor before trusting the indicator.
    """

    mode: str = "none"                    # "none" | "auto" | "output"
    #: JVL register holding the digital output states (register 19, 'Digital
    #: Outputs'). This is the register software toggles in "output" mode.
    output_register: int = 19
    output_bit: int = 0
    energized_releases: bool = True
    #: Seconds to wait after commanding the brake before moving. A mechanical
    #: brake takes tens of milliseconds to physically release; commanding a
    #: move before then grinds the brake disc.
    settle_s: float = 0.35

    def validate(self) -> None:
        if self.mode not in ("none", "auto", "output"):
            raise ValueError(
                f"brake.mode must be 'none', 'auto' or 'output', got {self.mode!r}"
            )
        if not (0 <= self.output_bit <= 31):
            raise ValueError(f"brake.output_bit must be 0..31, got {self.output_bit}")
        if self.settle_s < 0:
            raise ValueError("brake.settle_s must be >= 0")


# --------------------------------------------------------------------------
# One actuator (motor + screw + its place in the triangle)
# --------------------------------------------------------------------------

@dataclass
class ActuatorConfig:
    """One of the three focal-plane actuators.

    Scaling: counts -> millimetres
    ------------------------------
    The motor reports position in encoder counts. What the kinematics need is
    millimetres of travel along z. The conversion is::

        mm = counts / counts_per_mm

    `counts_per_mm` can be supplied two ways:

    1. Measured (preferred). Run
       `python -m psct_motors.cli calibrate --motor A`, which moves the
       actuator a known number of counts and asks you for the displacement
       you measured with a dial indicator. It writes the result here as
       `counts_per_mm`. This is the number to trust, because it absorbs the
       gearbox ratio, screw lead and any fixed error in one measurement --
       you do not need to know the gear ratio to use it.

    2. Derived from the drivetrain, if you leave `counts_per_mm` as null::

           counts_per_mm = counts_per_rev * gear_ratio / screw_lead_mm

       where `gear_ratio` is motor revolutions per screw revolution and
       `screw_lead_mm` is the axial travel per screw revolution.

    Defaults below encode what is known about the pSCT actuators today:
    `counts_per_rev = 409600` was read from MacTalk's encoder panel on these
    motors. The published pSCT camera description gives 12.7 um of z travel
    per motor step and 5.08 cm (2 in) of total travel; 12.7 um per 1.8-degree
    full step implies a 2.54 mm/rev lead with no reduction, which is the
    default here. That inference has NOT been checked against the actual
    hardware, so `gear_ratio` and `screw_lead_mm` are a starting point only.
    Measure `counts_per_mm` and the inference stops mattering.

    Sign
    ----
    `direction` is +1 or -1 and answers "does increasing counts move the
    focal plane towards +z?". Get it wrong on one actuator and a pure focus
    move turns into a tilt, so check it with the CLI `jog` command and a
    dial indicator before commissioning.
    """

    name: str = "A"
    ip: str = "192.168.0.52"
    port: int = 502
    unit_id: int = 1
    word_order: str = WordOrder.LOW_HIGH.value

    # --- scaling -----------------------------------------------------------
    counts_per_mm: Optional[float] = None   # measured; overrides the chain below
    counts_per_rev: float = 409600.0        # CONFIRMED from MacTalk on these motors
    gear_ratio: float = 1.0                 # motor revs per screw rev -- VERIFY
    screw_lead_mm: float = 2.54             # mm per screw rev -- inferred, VERIFY
    direction: int = 1                      # +1 or -1

    # --- geometry ----------------------------------------------------------
    #: Azimuth of this actuator around the optical axis, degrees CCW from +x
    #: looking along -z (i.e. standard math convention in the focal plane).
    azimuth_deg: float = 90.0
    #: Distance of the ball joint from the optical axis, millimetres.
    radius_mm: float = 500.0                # VERIFY against the camera drawing

    # --- travel limits -----------------------------------------------------
    #: Soft limits in millimetres of actuator travel, relative to the zero set
    #: by `set-zero`. The published total range is 5.08 cm; the default keeps a
    #: 1 mm buffer at each end so a move never parks against a hard stop.
    min_travel_mm: float = 1.0
    max_travel_mm: float = 49.8
    #: Counts reading that corresponds to 0 mm of travel. Written by `set-zero`.
    zero_counts: int = 0

    # --- motion defaults ---------------------------------------------------
    velocity_raw: int = 1000                # V_SOLL for normal moves
    acceleration_raw: int = 1000            # A_SOLL
    #: Position tolerance for "the move finished", in millimetres. Applied to
    #: the profile generator's output, so it answers "has the commanded ramp
    #: completed".
    in_position_tol_mm: float = 0.005
    #: How far the shaft may lag the profile and still count as arrived, in
    #: motor counts. This is the condition the projected position cannot
    #: express: the ramp can finish while the shaft is short of the target.
    #:
    #: A settled pSCT motor reads a following error of 231 counts, so a
    #: standing value is normal and the window has to be comfortably above it.
    #: The motor's own 'In Position' Window (register 33) is 20000 counts,
    #: which is far looser than anything a focal plane wants; 2000 counts is
    #: about 1.8 degrees of shaft, tight enough to catch a stall and loose
    #: enough not to reject a normal settle.
    follow_error_window_counts: int = 2000
    #: Maximum time to wait for a move to finish, seconds.
    move_timeout_s: float = 120.0

    brake: BrakeConfig = field(default_factory=BrakeConfig)

    # ---------------------------------------------------------------- derived

    @property
    def resolved_counts_per_mm(self) -> float:
        """The scale factor actually used, measured value winning."""
        if self.counts_per_mm is not None:
            value = float(self.counts_per_mm)
        else:
            if self.screw_lead_mm == 0:
                raise ValueError(
                    f"Actuator {self.name}: screw_lead_mm is 0 and counts_per_mm is "
                    "not set, so counts cannot be converted to millimetres."
                )
            value = self.counts_per_rev * self.gear_ratio / self.screw_lead_mm
        if value <= 0:
            raise ValueError(
                f"Actuator {self.name}: counts_per_mm resolved to {value}, which "
                "must be positive. Use `direction` to express travel sense."
            )
        return value

    @property
    def scale_is_measured(self) -> bool:
        return self.counts_per_mm is not None

    def counts_to_mm(self, counts: float) -> float:
        """Raw motor counts -> millimetres of travel from the actuator zero."""
        return (counts - self.zero_counts) * self.direction / self.resolved_counts_per_mm

    def mm_to_counts(self, mm: float) -> int:
        """Millimetres of travel from the actuator zero -> raw motor counts."""
        return int(round(mm * self.resolved_counts_per_mm * self.direction)) + self.zero_counts

    @property
    def position_xy_mm(self) -> tuple:
        """(x, y) of this actuator's ball joint in the focal plane, mm."""
        a = math.radians(self.azimuth_deg)
        return (self.radius_mm * math.cos(a), self.radius_mm * math.sin(a))

    def validate(self) -> None:
        if self.direction not in (1, -1):
            raise ValueError(
                f"Actuator {self.name}: direction must be +1 or -1, got {self.direction}"
            )
        if self.min_travel_mm >= self.max_travel_mm:
            raise ValueError(
                f"Actuator {self.name}: min_travel_mm ({self.min_travel_mm}) must be "
                f"below max_travel_mm ({self.max_travel_mm})"
            )
        if self.radius_mm <= 0:
            raise ValueError(f"Actuator {self.name}: radius_mm must be positive")
        if self.in_position_tol_mm <= 0:
            raise ValueError(f"Actuator {self.name}: in_position_tol_mm must be positive")
        if self.follow_error_window_counts <= 0:
            raise ValueError(
                f"Actuator {self.name}: follow_error_window_counts must be positive"
            )
        if not (0 < self.velocity_raw <= 32767):
            raise ValueError(
                f"Actuator {self.name}: velocity_raw must be 1..32767, got {self.velocity_raw}"
            )
        WordOrder.parse(self.word_order)
        self.brake.validate()
        self.resolved_counts_per_mm  # raises if unusable


# --------------------------------------------------------------------------
# Platform-wide settings
# --------------------------------------------------------------------------

@dataclass
class PlatformLimits:
    """Soft limits on the *commanded orientation*, checked before any motion.

    These are a second, independent guard on top of the per-actuator travel
    limits. Actuator limits stop a single axis from running out of travel;
    these stop you from asking for an orientation that is physically silly
    even if all three actuators could reach it.
    """

    min_focus_mm: float = 1.0
    max_focus_mm: float = 49.8
    max_tilt_deg: float = 1.0       # magnitude of total tilt from the z axis
    #: Largest single commanded step, as a guard against a typo or a bad unit
    #: conversion sending an actuator across its whole range at once.
    max_step_mm: float = 10.0
    max_tilt_step_deg: float = 0.5

    def validate(self) -> None:
        if self.min_focus_mm >= self.max_focus_mm:
            raise ValueError("limits.min_focus_mm must be below limits.max_focus_mm")
        for name in ("max_tilt_deg", "max_step_mm", "max_tilt_step_deg"):
            if getattr(self, name) <= 0:
                raise ValueError(f"limits.{name} must be positive")


@dataclass
class PlatformConfig:
    actuators: List[ActuatorConfig] = field(default_factory=list)
    limits: PlatformLimits = field(default_factory=PlatformLimits)

    #: Scale the three actuators' velocities so they all finish together.
    #: Without this the shortest move finishes first and the plate is
    #: transiently racked about its ball joints on every combined move.
    synchronize_moves: bool = True
    #: Velocity floor when synchronising; below this the motor may stall.
    min_velocity_raw: int = 20

    #: Seconds between live status polls.
    poll_interval_s: float = 0.5
    #: Modbus socket timeout, seconds.
    modbus_timeout_s: float = 2.0
    #: Attempts per Modbus transaction. Keep this at 1 unless you have a
    #: specific reason: pymodbus defaults to 3, which turns one failed read
    #: into a multi-second stall that presents as the application hanging
    #: rather than as an error. Worst-case block is
    #: modbus_timeout_s x modbus_retries.
    modbus_retries: int = 1

    #: If true, connecting checks each motor's word order against PROG_VERSION
    #: and complains rather than silently reading garbage.
    verify_word_order_on_connect: bool = True

    def validate(self) -> None:
        if len(self.actuators) != 3:
            raise ValueError(
                f"The focal plane is a three-point mount: expected 3 actuators, "
                f"got {len(self.actuators)}"
            )
        names = [a.name for a in self.actuators]
        if len(set(names)) != 3:
            raise ValueError(f"Actuator names must be unique, got {names}")
        for a in self.actuators:
            a.validate()
        self.limits.validate()
        if self.min_velocity_raw < 1:
            raise ValueError("min_velocity_raw must be >= 1")
        if self.poll_interval_s <= 0:
            raise ValueError("poll_interval_s must be positive")

        # Three actuators on the same point, or on one line, cannot define a
        # plane -- the kinematics would be singular. Catch it here, in the
        # config, rather than as a division by zero mid-move.
        pts = [a.position_xy_mm for a in self.actuators]
        area2 = abs(
            (pts[1][0] - pts[0][0]) * (pts[2][1] - pts[0][1])
            - (pts[2][0] - pts[0][0]) * (pts[1][1] - pts[0][1])
        )
        if area2 < 1e-6:
            raise ValueError(
                "The three actuators are collinear (or coincident), so they do not "
                "define a plane. Check azimuth_deg and radius_mm for each actuator."
            )

    def actuator(self, name: str) -> ActuatorConfig:
        for a in self.actuators:
            if a.name.lower() == str(name).lower():
                return a
        raise KeyError(
            f"No actuator named {name!r}. Known: {[a.name for a in self.actuators]}"
        )


# --------------------------------------------------------------------------
# Defaults, load and save
# --------------------------------------------------------------------------

def default_config() -> PlatformConfig:
    """A three-actuator platform with the actuators 120 degrees apart.

    IP addresses are placeholders apart from motor A, which is the address the
    existing single-motor test setup uses. Everything geometric is a starting
    point to be replaced with real values from the camera drawings.
    """
    return PlatformConfig(
        actuators=[
            ActuatorConfig(name="A", ip="192.168.0.52", azimuth_deg=90.0),
            ActuatorConfig(name="B", ip="192.168.0.53", azimuth_deg=210.0),
            ActuatorConfig(name="C", ip="192.168.0.54", azimuth_deg=330.0),
        ]
    )


def _from_dict(cls, data: Dict[str, Any]):
    """Build a dataclass from a dict, ignoring unknown keys but reporting them."""
    known = {f.name for f in cls.__dataclass_fields__.values()}
    unknown = set(data) - known
    if unknown:
        raise ValueError(
            f"Unknown key(s) in {cls.__name__} config: {sorted(unknown)}. "
            f"Valid keys: {sorted(known)}"
        )
    return cls(**{k: v for k, v in data.items() if k in known})


def config_from_dict(data: Dict[str, Any]) -> PlatformConfig:
    data = dict(data)
    data.pop("_comment", None)
    actuators_raw = data.pop("actuators", None)
    limits_raw = data.pop("limits", None)

    actuators: List[ActuatorConfig] = []
    for entry in actuators_raw or []:
        entry = dict(entry)
        brake_raw = entry.pop("brake", None)
        act = _from_dict(ActuatorConfig, entry)
        if brake_raw is not None:
            act.brake = _from_dict(BrakeConfig, brake_raw)
        actuators.append(act)

    cfg = _from_dict(PlatformConfig, data)
    cfg.actuators = actuators or default_config().actuators
    if limits_raw is not None:
        cfg.limits = _from_dict(PlatformLimits, limits_raw)
    return cfg


def config_to_dict(cfg: PlatformConfig) -> Dict[str, Any]:
    return asdict(cfg)


def load_config(path: Optional[str] = None) -> PlatformConfig:
    """Load configuration, falling back to defaults when the file is absent."""
    path = path or default_config_path()
    if not os.path.exists(path):
        return default_config()
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    cfg = config_from_dict(data)
    cfg.validate()
    return cfg


def save_config(cfg: PlatformConfig, path: Optional[str] = None) -> str:
    path = path or default_config_path()
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    payload = config_to_dict(cfg)
    payload["_comment"] = (
        "pSCT focal-plane actuator configuration. Values marked VERIFY in the "
        "source docstrings must be checked against the hardware before use."
    )
    # Write via a temporary file so a crash mid-write cannot leave the only
    # copy of the machine's calibration truncated.
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, sort_keys=False)
    os.replace(tmp, path)
    return path


def default_config_path() -> str:
    """Where the config lives if no path is given.

    Honours PSCT_MOTORS_CONFIG so a test bench and the telescope can run the
    same code against different machines.
    """
    env = os.environ.get("PSCT_MOTORS_CONFIG")
    if env:
        return env
    return os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config",
        DEFAULT_CONFIG_FILENAME,
    )


__all__ = [
    "BrakeConfig", "ActuatorConfig", "PlatformLimits", "PlatformConfig",
    "default_config", "load_config", "save_config", "default_config_path",
    "config_from_dict", "config_to_dict", "replace",
]
