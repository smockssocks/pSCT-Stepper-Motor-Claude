"""
The three actuators driven as one focal-plane positioner.

This is the layer that answers the actual request: point the focal plane at an
orientation, and let the software work out which motors move and by how much.
Nobody has to reason about screws, triangles or which motor is "the one on the
left" any more.

Order of operations for every move
----------------------------------
The sequence below is not arbitrary; each step exists to remove a specific way
a three-actuator mount can be damaged.

1. Solve the kinematics for all three actuator targets.
2. Check the *commanded orientation* against the platform limits, and each
   actuator target against that actuator's soft travel limits, and the size of
   each step against the step limits. All of this happens before a single
   register is written, so a move that cannot complete is never started. A
   partially executed combined move is exactly the state that racks the ball
   joints.
3. Put every motor in Position mode and confirm it took.
4. Release the brakes, if they are under software control, and wait out the
   mechanical release time.
5. Scale the three velocities so all three axes arrive together.
6. Write all three targets.
7. Wait for all three to report in position.

Every one of those steps can abort the move cleanly, and none of them leaves
the plate held at a half-applied orientation.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

from .config import PlatformConfig, load_config, save_config
from .history import MoveRecord, PositionHistory
from .jvl_motor import BrakeState, JVLMotor, MotorFault, MotorStatus
from .kinematics import Orientation, ThreePointPlatform, platform_from_config
from .saved_positions import SavedPosition, SavedPositions, make_saved_position
from .transport import ModbusError


#: How far beyond the soft focus limits the *simulated* end stops sit.
#:
#: The published pSCT travel is 5.08 cm, i.e. +/-25.4 mm, against soft limits
#: of +/-24 mm -- so about a millimetre and a half of margin at each end, which
#: is what this reproduces. Small enough that the search finds a stop inside a
#: sensible budget, and large enough that the soft limit is what stops an
#: ordinary move first.
SIMULATED_STOP_MARGIN_MM = 1.4

#: In bench mode the stand-ins copy the real motor, so no tilt can be made.
#: A target is "flat" below this: a saved position or a log entry from the
#: bench carries a few counts of rounding, which is far below anything real.
COPY_TILT_TOLERANCE_DEG = 1e-3


class PlatformError(RuntimeError):
    """A move was refused, or the platform is not in a state to move."""


@dataclass(frozen=True)
class EmergencyResult:
    """What the EMERGENCY control actually did.

    Every field is read back or observed, not assumed. The whole point of this
    type is that an operator who pressed the button can be told the truth
    about the state the machine is now in -- including, in particular, that
    the drives were deliberately left on.
    """

    stop_problems: List[str]
    brakes_engaged: bool
    brake_message: str
    drives_off: bool
    drive_problems: List[str]
    why_drives_are_still_on: str
    positions_mm: Dict[str, float]
    #: Axes that were passive with no confirmed brake, and were re-enabled so
    #: that something is holding them.
    took_hold: List[str] = field(default_factory=list)

    @property
    def holding(self) -> bool:
        """True when something is holding the focal plane: brakes, or drives."""
        return self.brakes_engaged or not self.drives_off

    @property
    def stopped(self) -> bool:
        return not self.stop_problems

    def summary(self) -> str:
        lines = []
        if self.stop_problems:
            lines.append("EMERGENCY: motion halted, EXCEPT on "
                         + "; ".join(self.stop_problems))
        else:
            lines.append("EMERGENCY: all three actuators halted and holding.")
        for name, mm in self.positions_mm.items():
            lines.append(f"  {name:<5} stopped at {mm:+9.4f} mm")
        lines.append(f"  brakes: {self.brake_message}")
        if self.drives_off:
            lines.append("  drives: OFF. The brakes are holding the focal plane.")
        else:
            lines.append("  drives: ON and holding position. "
                         + (self.why_drives_are_still_on or
                            "The brakes are not confirmed, so the drives keep "
                            "holding the load."))
        if self.took_hold:
            lines.append("  took hold of " + ", ".join(self.took_hold)
                         + ": they were passive with no confirmed brake, so "
                         "the drives were enabled to hold them where they are.")
        if self.drive_problems:
            lines.append("  could not passivate: " + "; ".join(self.drive_problems))
        if not self.holding:
            lines.append("  ** NOTHING IS CONFIRMED HOLDING THE FOCAL PLANE. "
                         "Check it physically. **")
        return "\n".join(lines)

    def as_dict(self) -> dict:
        return {
            "stopped": self.stopped,
            "stop_problems": list(self.stop_problems),
            "brakes_engaged": self.brakes_engaged,
            "brake_message": self.brake_message,
            "drives_off": self.drives_off,
            "drive_problems": list(self.drive_problems),
            "why_drives_are_still_on": self.why_drives_are_still_on,
            "positions_mm": dict(self.positions_mm),
            "took_hold": list(self.took_hold),
            "holding": self.holding,
        }


@dataclass(frozen=True)
class HardStopProgress:
    """One step's worth of a coordinated hard-stop search, for a live display."""

    positions_mm: Dict[str, float]
    travelled_mm: float
    spread_mm: float
    torque_percent: Dict[str, float]


@dataclass(frozen=True)
class HardStopResult:
    """Where the coordinated hard-stop search ended, and why."""

    direction: int
    #: The actuator(s) that reported reaching a stop. The others were halted
    #: with them, so their positions are where they happened to be, not their
    #: own end of travel.
    stopped_by: List[str]
    reasons: Dict[str, str]
    start_mm: Dict[str, float]
    positions_mm: Dict[str, float]
    positions_counts: Dict[str, int]
    travelled_mm: Dict[str, float]
    #: Difference in travel between the highest and lowest actuator when the
    #: search finished, i.e. the tilt left in the plate. Small after levelling.
    spread_mm: float
    #: The worst that difference got at any point during the search.
    worst_spread_mm: float
    peak_torque_percent: Dict[str, float]
    levelled: bool = False
    #: Where each actuator was when the travel ended -- which is not where it
    #: is now, because the search backs off afterwards rather than leaving the
    #: mechanism resting on its stop.
    stop_mm: Dict[str, float] = field(default_factory=dict)
    backed_off_mm: float = 0.0

    def summary(self) -> str:
        towards = "M1 (primary)" if self.direction > 0 else "M2 (secondary)"
        lines = [
            f"Hard stop found while running towards {towards}. "
            f"Stopped by: {', '.join(self.stopped_by) or 'nothing'}."
        ]
        for name, mm in self.positions_mm.items():
            why = self.reasons.get(name, "halted with the others")
            lines.append(
                f"  {name:<5} {mm:+9.4f} mm  ({self.travelled_mm[name]:+.4f} mm "
                f"travelled, peak torque "
                f"{self.peak_torque_percent.get(name, 0.0):.0f}%)  -- {why}"
            )
        lines.append(
            f"  tilt: {self.spread_mm:.4f} mm between the highest and lowest "
            f"actuator now, {self.worst_spread_mm:.4f} mm at its worst during "
            f"the search."
        )
        if self.levelled:
            lines.append("  The other actuators were backed off to match the one "
                         "that stopped, so the plate is flat again.")
        if self.backed_off_mm:
            lines.append(f"  Then all three retreated {self.backed_off_mm:.3f} mm "
                         f"from the stop, so nothing is left resting on it.")
        if self.stop_mm:
            ends = ", ".join(f"{n} {v:+.4f}" for n, v in self.stop_mm.items())
            lines.append(f"  The travel ends at: {ends} mm")
        return "\n".join(lines)

    def as_dict(self) -> dict:
        return {
            "direction": self.direction,
            "stopped_by": list(self.stopped_by),
            "reasons": dict(self.reasons),
            "start_mm": dict(self.start_mm),
            "positions_mm": dict(self.positions_mm),
            "positions_counts": dict(self.positions_counts),
            "travelled_mm": dict(self.travelled_mm),
            "spread_mm": self.spread_mm,
            "worst_spread_mm": self.worst_spread_mm,
            "peak_torque_percent": dict(self.peak_torque_percent),
            "levelled": self.levelled,
            "stop_mm": dict(self.stop_mm),
            "backed_off_mm": self.backed_off_mm,
        }


@dataclass
class PlatformState:
    """One consistent snapshot of the whole positioner."""

    motors: List[MotorStatus] = field(default_factory=list)
    orientation: Optional[Orientation] = None
    #: False when at least one actuator could not be read, in which case the
    #: orientation is missing rather than computed from stale numbers. A tilt
    #: derived from two live positions and one stale one is worse than none.
    orientation_valid: bool = False
    message: str = ""
    #: One line about the brake controller: which device, what it reports,
    #: and whether that is the brake itself or only its relay. Empty when no
    #: brake controller is configured.
    brake_summary: str = ""

    @property
    def all_connected(self) -> bool:
        return bool(self.motors) and all(m.connected and not m.comms_error for m in self.motors)

    @property
    def any_error(self) -> bool:
        return any(m.error_bits for m in self.motors)

    @property
    def moving(self) -> bool:
        """True while an enabled drive has not reached its target.

        A passive drive is excluded. Its P_SOLL is whatever was last written,
        which after a power cycle or a passivate need not match where the
        shaft sits -- and a machine with the drives off is not moving, however
        far apart those two numbers are. Counting it kept the fast poll rate
        and the MOVING flag on for ever on a parked, unpowered axis.
        """
        return any(not m.in_position for m in self.motors
                   if not m.comms_error and m.mode != 0)

    def as_dict(self) -> dict:
        return {
            "motors": [m.as_dict() for m in self.motors],
            "orientation": self.orientation.as_dict() if self.orientation else None,
            "orientation_valid": self.orientation_valid,
            "all_connected": self.all_connected,
            "any_error": self.any_error,
            "moving": self.moving,
            "message": self.message,
            "brake_summary": self.brake_summary,
        }


class _InStep:
    """Where each actuator is along its own move, for keeping them together.

    Each actuator's progress is how far it has gone as a fraction of its own
    move. If the plate is to stay on the straight line between the two
    orientations, those fractions must stay equal; the smallest says where the
    plate should be, and anything further along is "ahead" by that much.
    """

    #: Moves shorter than this have no meaningful progress to compare.
    MIN_MOVE_MM = 0.005

    def __init__(self, motors, starts, targets, velocities):
        self.names = [m.name for m in motors]
        self.index = {name: i for i, name in enumerate(self.names)}
        self.starts = starts
        self.targets = targets
        self.velocities = velocities
        #: name -> when it was held.
        self.paused: Dict[str, float] = {}
        #: Motors whose move has finished. They count as all the way there:
        #: one with a standing lag would otherwise look behind for ever, and
        #: the others would be held waiting for it. Settling deals with it.
        self.finished = set()
        #: Sent on again in this round of checks.
        self.resumed = set()

    def _moving(self):
        return [i for i in range(len(self.names))
                if abs(self.targets[i] - self.starts[i]) > self.MIN_MOVE_MM]

    def _progress(self, positions):
        out = {}
        for i in self._moving():
            if self.names[i] in self.finished:
                out[i] = 1.0
                continue
            delta = self.targets[i] - self.starts[i]
            out[i] = min(1.0, max(0.0, (positions[i] - self.starts[i]) / delta))
        return out

    def leads(self, positions) -> Dict[str, float]:
        """How far, in mm, each actuator is ahead of the slowest."""
        progress = self._progress(positions)
        if len(progress) < 2:
            return {}
        slowest = min(progress.values())
        return {self.names[i]: (p - slowest) * abs(self.targets[i] - self.starts[i])
                for i, p in progress.items()}

    def slowest(self, positions) -> str:
        progress = self._progress(positions)
        return self.names[min(progress, key=progress.get)] if progress else ""

    def arrived(self, motor, positions) -> bool:
        i = self.index[motor.name]
        return abs(positions[i] - self.targets[i]) <= self.MIN_MOVE_MM


class FocalPlanePlatform:
    """Coordinated control of the three focal-plane actuators."""

    def __init__(self, cfg: Optional[PlatformConfig] = None,
                 simulate: bool = False,
                 logger: Optional[Callable[[str], None]] = None,
                 config_path: Optional[str] = None,
                 history_path: Optional[str] = None,
                 on_history_change: Optional[Callable[[], None]] = None,
                 use_real_brakes: bool = False,
                 saved_positions_path: Optional[str] = None):
        self.cfg = cfg or load_config(config_path)
        self.cfg.validate()
        self.config_path = config_path
        self.simulate = simulate
        #: Drive the configured brake controller even in full simulation. Off
        #: by default: `--simulate` promises that no hardware is touched, and a
        #: PLC that switches brakes is hardware. On, it lets the PLC be bench
        #: tested with no motors connected at all.
        self.use_real_brakes = use_real_brakes
        self._log = logger or (lambda msg: None)
        self.geometry: ThreePointPlatform = platform_from_config(self.cfg)
        self.brake_summary = ""
        self.external_brake = self._build_external_brake()
        self._move_lock = threading.RLock()
        self._abort = threading.Event()
        #: Every move, with where the plane was before and after it. Kept in
        #: memory always; written to `history_path` as well when one is given,
        #: which the GUI and CLI do and the tests and drills do not.
        self.history = PositionHistory(path=history_path or None,
                                       on_change=on_history_change,
                                       logger=self._log)
        #: Named positions ("Default", "Window open" ...). Written to
        #: `saved_positions_path` when one is given, like the history.
        self.saved_positions = SavedPositions(path=saved_positions_path or None,
                                              logger=self._log)
        #: Rest on the brakes (brakes on, drives off) after every move. Set
        #: from the configuration, or by the GUI's checkbox.
        self.rest_after_moves = bool(self.cfg.rest_on_brakes_after_moves)
        #: Falling watch: where each axis was when the drive stopped driving
        #: it, and the last alarm raised, for the GUI to show.
        self._fall_ref: Dict[str, tuple] = {}
        self._fall_alarm: Optional[str] = None
        #: How the GUI shows positions (see its position reference): added to
        #: positions in log lines, with words saying where 0 is. Nothing else
        #: here uses them.
        self.shown_offset_mm = 0.0
        self.shown_from = ""
        #: Set by the GUI once a person has confirmed (with the password) that
        #: moving is fine while the brake controller cannot be read: the move
        #: then goes ahead without touching the brakes.
        self.allow_unknown_brakes = False
        #: Whether a person has accepted the brake relay's word for "engaged"
        #: when nothing senses the brakes. Never set by this module.
        self.trust_relay_brakes = False
        #: Said once, not on every move: there is no healthy supply reading to
        #: compare against. Repeating it every time would train people to
        #: ignore it.
        self._warned_no_supply_baseline = False

        # An actuator is simulated when the whole platform is, or when that
        # one actuator asks to be. The mixed case is the point: one real motor
        # on a bench, two stood in, so everything above the driver can be
        # exercised before all three are wired.
        self._stand_ins = [a.name for a in self.cfg.actuators
                           if simulate or a.simulated]
        self.motors: List[JVLMotor] = [
            self._build_motor(a, a.name in self._stand_ins)
            for a in self.cfg.actuators
        ]
        #: Stand-ins that copy a real motor (bench mode), and which motor.
        self.copied_names: List[str] = []
        self.copy_source: Optional[str] = None
        self._attach_copies()
        self.sync_travel_to_stops()

    def _build_real_motor(self, a) -> JVLMotor:
        return JVLMotor(a, timeout_s=self.cfg.modbus_timeout_s,
                        retries=self.cfg.modbus_retries, logger=self._log)

    def _attach_copies(self) -> None:
        """Bench mode: make each stand-in a copy of the real motor.

        With one real motor and two independent simulated ones, the plane
        tilted as soon as they disagreed, and with the drives off and no brake
        the simulated two sagged under gravity without end. As copies they sit
        wherever the real motor is and go wherever it is sent, so the plane is
        flat and moves only in focus, which is all one motor can show anyway.
        """
        if not self.cfg.bench_stand_ins_copy_real:
            return
        real = [m for m in self.motors if m.name not in self._stand_ins]
        if not real or len(real) == len(self.motors):
            return
        source = real[0]
        cache = {"at": 0.0, "reading": None}
        lock = threading.Lock()

        def source_reading():
            """(encoder, projected, target) of the real motor, in its counts,
            read at most every 0.1 s however many copies ask."""
            with lock:
                now = time.monotonic()
                if cache["reading"] is None or now - cache["at"] > 0.1:
                    try:
                        cache["reading"] = (source.get_position_counts(),
                                            source.get_projected_position_counts(),
                                            source.get_target_counts())
                        cache["at"] = now
                    except Exception:  # noqa: BLE001 -- not connected yet, say
                        return None
                return cache["reading"]

        for motor in self.motors:
            if motor.name not in self._stand_ins:
                continue

            def follow(cfg=motor.cfg):
                reading = source_reading()
                if reading is None:
                    return None
                return tuple(cfg.mm_to_counts(source.cfg.counts_to_mm(c))
                             for c in reading)
            motor._transport.follow = follow
            self.copied_names.append(motor.name)
        self.copy_source = source.name

    def _build_motor(self, a, simulated: bool) -> JVLMotor:
        """One motor: real Modbus, or a stand-in."""
        if not simulated:
            return self._build_real_motor(a)

        from .simulator import simulated_motor
        # Start the simulated actuators mid-travel so relative moves in both
        # directions are possible straight away.
        mid = (self.cfg.limits.min_focus_mm + self.cfg.limits.max_focus_mm) / 2.0
        low_stop, high_stop = self._simulated_stops()
        return simulated_motor(
            a, start_mm=mid,
            # What "full speed" means for a stand-in axis. Chosen by a person,
            # in the units a person watches, because the drive's own velocity
            # units have never been measured against millimetres.
            full_speed_mm_per_s=self.cfg.simulated_speed_mm_per_s,
            # Mechanical end stops just outside the *focus* limits, which is
            # the range the operator and the gauge think in.
            #
            # They used to be placed from the actuator travel limits instead,
            # which on a configuration whose actuator limits are wider than its
            # focus limits put the simulated end of travel far outside
            # everything: a search that ran past its budget without finding
            # anything, and, when it did find something, a plate parked well
            # outside the limits so every ordinary move afterwards was refused.
            # Soft limits sit inside the mechanism; the simulation has to agree.
            hard_stop_low=a.mm_to_counts(low_stop),
            hard_stop_high=a.mm_to_counts(high_stop),
            # The load the brakes exist to hold. With the brakes off and the
            # drives passive, a simulated axis falls -- which is the failure the
            # interlocks are there to prevent, and it cannot be rehearsed if
            # the simulation ignores gravity.
            gravity_counts_per_s=a.resolved_counts_per_mm * 2.0,
            brake_held=self._make_brake_hook(a.name),
        )

    def _simulated_stops(self):
        """Where the simulated mechanism physically ends, in focus mm.

        The span comes from the total travel, not from the soft limits.
        Deriving the stops from the limits was circular: `find-stop` adopts the
        stop it finds as the new limit, so the next simulated run put its stop
        further out again, and repeated rehearsals walked the machine off into
        the distance.

        They are centred on the middle of the configured focus range rather
        than on zero, because the zero reference is wherever `set-zero` put it
        and need not be mid-travel. That centre is stable under adoption: the
        limits become the stops less a margin at each end, which leaves the
        midpoint exactly where it was.
        """
        limits = self.cfg.limits
        centre = (limits.min_focus_mm + limits.max_focus_mm) / 2.0
        if limits.total_travel_mm:
            half = limits.total_travel_mm / 2.0
            return centre - half, centre + half
        return (limits.min_focus_mm - SIMULATED_STOP_MARGIN_MM,
                limits.max_focus_mm + SIMULATED_STOP_MARGIN_MM)

    @property
    def simulated_names(self) -> List[str]:
        """Which actuators are stood in rather than real."""
        return list(self._stand_ins)

    @property
    def is_mixed(self) -> bool:
        """True when some motors are real and some are simulated."""
        simulated = set(self.simulated_names)
        return bool(simulated) and len(simulated) != len(self.motors)

    def _make_brake_hook(self, name: str):
        """Let a simulated motor ask whether its brake is clamping the shaft."""
        def held() -> bool:
            controller = getattr(self, "external_brake", None)
            hook = getattr(controller, "is_holding", None)
            return bool(hook(name)) if hook else False
        return held

    def _build_external_brake(self):
        """The device that switches the focal-plane brakes, if there is one.

        On the pSCT the brakes are not on the motors, so brake control has to
        go somewhere else. Built unconditionally: when it is unconfigured it
        still answers "not available, and here is why", which is more use than
        an attribute that does not exist.

        In full simulation a fake one stands in, so the brake interlocks -- the
        checks that stop a released brake dropping the camera -- can be
        rehearsed. That is true even when a real device IS configured: it used
        to be that configuring the PLC made `--simulate` switch its relays,
        which broke the one promise `--simulate` makes. The real device is used
        in simulation only when asked for (`use_real_brakes`, `--real-brakes`).

        In bench mode (`--bench`) the platform is not simulated -- one motor is
        real -- so the configured device is used, and the stood-in motors obey
        it: a simulated axis will not turn while the PLC says its brake is on.
        """
        from .external_brake import BrakeController, ExternalBrakeConfig
        settings = self.cfg.external_brake
        if self.simulate and not (self.use_real_brakes and settings.mode != "none"):
            from .external_brake import SimulatedBrakeController
            if settings.mode != "none":
                self._log(
                    f"Simulation: the configured brake controller "
                    f"({settings.mode} at {settings.host or 'no host'}) is NOT "
                    f"being used; a simulated one stands in. Pass --real-brakes "
                    f"to switch the real one from a simulation.")
            return SimulatedBrakeController(
                [a.name for a in self.cfg.actuators],
                all_or_nothing=settings.all_or_nothing,
                logger=self._log,
            )
        return BrakeController(
            ExternalBrakeConfig.from_settings(settings),
            logger=self._log,
            names=[a.name for a in self.cfg.actuators],
        )

    def reload_external_brake(self) -> None:
        """Rebuild the brake controller from the current configuration.

        For the GUI's brake settings: the motors stay connected, and the
        stood-in axes pick up the new controller on their next physics step,
        because they ask for `self.external_brake` by name each time.
        """
        old = self.external_brake
        self.external_brake = self._build_external_brake()
        self.brake_summary = ""
        try:
            old.close()
        except Exception:  # noqa: BLE001 -- closing the old one is best effort
            pass

    @property
    def drives_holding(self) -> bool:
        """True when every motor is enabled and actively holding position.

        The precondition for releasing a brake. Checked against the motors
        rather than assumed, because the brake controller cannot see them.
        """
        return self.holding(None)

    def holding(self, name: Optional[str]) -> bool:
        """Whether the drive(s) a brake holds are enabled and holding.

        `name` None or "all" means every drive. With separate brakes, one
        brake only needs its own drive: the other two are still clamped.
        """
        from .registers import MotorMode
        motors = (self.motors if name in (None, "all")
                  else [self.motor(name)])
        try:
            return all(m.get_mode() == int(MotorMode.POSITION) for m in motors)
        except (ModbusError, MotorFault):
            return False

    def _brake_target(self, name: Optional[str]) -> str:
        """What a request for one brake actually switches: that brake when
        they are separate, all of them when one switch holds every brake."""
        if name in (None, "all"):
            return "all"
        return "all" if getattr(self.external_brake, "all_or_nothing", True) else name

    # ------------------------------------------------------------ accessors

    def motor(self, name: str) -> JVLMotor:
        for m in self.motors:
            if m.name.lower() == str(name).lower():
                return m
        raise KeyError(
            f"No actuator named {name!r}. Known: {[m.name for m in self.motors]}"
        )

    @property
    def connected(self) -> bool:
        return all(m.connected for m in self.motors)

    @property
    def any_connected(self) -> bool:
        """True while at least one motor is still reachable.

        The safety controls key off this rather than `connected`. If one motor
        drops off the network mid-move, `connected` goes false -- and a STOP
        button that refuses to act because one of three motors is unreachable
        would leave the other two running.
        """
        return any(m.connected for m in self.motors)

    @property
    def names(self) -> List[str]:
        return [m.name for m in self.motors]

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> None:
        """Connect to all three motors, reporting every failure at once.

        Deliberately not fail-fast: if all three are unplugged you want to be
        told that once, not one motor per attempt.
        """
        failures: List[str] = []
        for m in self.motors:
            try:
                m.connect(verify_word_order=self.cfg.verify_word_order_on_connect)
                self._log(f"Connected to {m.describe()}")
            except (ModbusError, MotorFault) as exc:
                failures.append(f"{m.name}: {exc}")
        if failures:
            raise PlatformError(
                "Could not bring up every actuator:\n  " + "\n  ".join(failures)
            )
        self._abort.clear()

    def disconnect(self) -> None:
        for m in self.motors:
            m.disconnect()
        pool, self._pool = getattr(self, "_pool", None), None
        if pool is not None:
            pool.shutdown(wait=False)
        close = getattr(self.external_brake, "close", None)
        if close is not None:
            close()          # stops its background reader; restarts on demand

    def __enter__(self) -> "FocalPlanePlatform":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.disconnect()

    # ---------------------------------------------------------------- state

    def read_actuator_positions_mm(self) -> List[float]:
        return [m.get_position_mm() for m in self.motors]

    def read_orientation(self) -> Orientation:
        """Current orientation, computed from the three live positions."""
        return self.geometry.orientation_from_actuators(self.read_actuator_positions_mm())

    def read_state(self, include_slow: bool = True) -> PlatformState:
        """Poll everything. Never raises -- suitable for a UI timer.

        `include_slow` is passed to each motor: False skips temperature and bus
        voltage and reuses the last ones, which is what a fast poll loop wants.
        """
        brake = self._external_brake_reading()
        # In parallel: each motor has its own connection, so reading them one
        # after another made every poll three times as long as it needed to be.
        raw = list(self._poll_pool().map(
            lambda m: m.read_status(include_slow), self.motors))
        statuses = [self._with_external_brake(s, brake) for s in raw]
        self._watch_for_falling(statuses)
        valid = all(s.connected and not s.comms_error for s in statuses)
        orientation = None
        message = ""
        if valid:
            try:
                orientation = self.geometry.orientation_from_actuators(
                    [s.position_mm for s in statuses]
                )
            except Exception as exc:  # geometry is validated, but never let a
                valid = False         # UI poll loop die on an unexpected value
                message = f"Could not compute orientation: {exc}"
        else:
            offline = [s.name for s in statuses if s.comms_error or not s.connected]
            message = f"Orientation unavailable: no reading from {', '.join(offline)}"
        return PlatformState(motors=statuses, orientation=orientation,
                             orientation_valid=valid, message=message,
                             brake_summary=self.brake_summary)

    def _watch_for_falling(self, statuses) -> None:
        """Brakes on at once if an axis nobody is driving moves on its own.

        "Not being driven" is a passive drive (or any mode but Position), or
        one in Position mode whose move has finished -- its profile output is
        at its target. While that holds, the shaft should stay put. If its
        encoder moves more than `fall_limit_mm` from where it was when that
        began, the axis is falling (or slipping under the drive): every brake
        is engaged, every passive drive takes hold where it now is, and an
        alarm is raised. The reference resets whenever the drive is driving,
        so a move, a STOP or a hard-stop search never looks like a fall.
        Never raises: this runs inside the status poll.
        """
        from .registers import MotorMode
        cfg = self.cfg
        if not cfg.fall_watch:
            return
        falling = []
        for status in statuses:
            if status.comms_error or not status.connected:
                self._fall_ref.pop(status.name, None)
                continue
            idle = (status.mode != int(MotorMode.POSITION)
                    or abs(status.projected_counts - status.target_counts) <= 2)
            if not idle:
                self._fall_ref.pop(status.name, None)
                continue
            try:
                motor = self.motor(status.name)
                per_mm = motor.cfg.resolved_counts_per_mm
            except Exception:  # noqa: BLE001 -- no scale, no judgement
                continue
            # A reference taken before the drive was last commanded is stale:
            # a whole move can finish between two polls.
            sent = motor.commands_sent
            if status.name in self.copied_names:
                # A bench copy moves when the motor it copies is commanded.
                sent += self.motor(self.copy_source).commands_sent
            ref = self._fall_ref.get(status.name)
            if ref is None or ref[1] != sent:
                self._fall_ref[status.name] = (status.position_counts, sent)
                continue
            moved = abs(status.position_counts - ref[0]) / per_mm
            if moved > cfg.fall_limit_mm:
                falling.append((status.name, moved,
                                "drive off" if status.mode != int(MotorMode.POSITION)
                                else "drive holding"))
        if falling:
            self._respond_to_fall(falling)

    def _respond_to_fall(self, falling) -> None:
        """Brakes on, drives take hold, alarm. Never raises."""
        what = ", ".join(f"{name} moved {mm:.3f} mm with the {state}"
                         for name, mm, state in falling)
        try:
            results = self.set_all_brakes(engaged=True)
            failed = {k: v for k, v in results.items() if not str(v).startswith("ok")}
            brakes = ("all brakes engaged" if not failed else
                      "the brakes could NOT be engaged: "
                      + "; ".join(f"{k}: {v}" for k, v in failed.items()))
        except Exception as exc:  # noqa: BLE001 -- keep going to take hold
            brakes = f"the brakes could NOT be engaged: {exc}"
        # Anything still being driven stops too: nothing should push against
        # brakes that have just come on.
        self._halt_all_quietly("an axis was falling")
        try:
            taken = self._take_hold()
        except Exception as exc:  # noqa: BLE001
            taken = []
            self._log(f"Could not take hold after a fall: {exc}")
        alarm = (f"FALLING: {what}. {brakes[0].upper()}{brakes[1:]}"
                 + (f"; {', '.join(taken)} turned on and holding" if taken else "")
                 + ". Find out why before moving again.")
        self._log(alarm)
        self._fall_alarm = alarm
        # Start again from here: if it keeps going, that is said again.
        self._fall_ref.clear()

    def pop_fall_alarm(self) -> Optional[str]:
        """The last falling alarm, once, for the GUI to show."""
        alarm, self._fall_alarm = self._fall_alarm, None
        return alarm

    def _poll_pool(self):
        """Worker threads for reading the motors side by side."""
        pool = getattr(self, "_pool", None)
        if pool is None:
            from concurrent.futures import ThreadPoolExecutor
            pool = self._pool = ThreadPoolExecutor(
                max_workers=max(1, len(self.motors)), thread_name_prefix="read")
        return pool

    def _external_brake_reading(self):
        """One read of the brake device per poll, shared by all three axes.

        Returns {name: BrakeStatus}, or None when there is no device. The
        site's brakes are one switch for all three, so asking the device
        three times per poll -- which at the moving poll rate would be twenty
        HTTP requests a second to a PLC -- told us nothing the first answer
        did not.
        """
        from .external_brake import BrakeError
        from .jvl_motor import BrakeStatus

        controller = self.external_brake
        if not controller.available:
            self.brake_summary = ""
            return None
        detail = getattr(controller, "describe", lambda: "external brake")()
        readings = {}
        # The simulated controller carries the flag itself; the real one
        # carries it on its config. Either way, one switch means one read.
        all_or_nothing = getattr(controller, "all_or_nothing", None)
        if all_or_nothing is None:
            all_or_nothing = getattr(getattr(controller, "cfg", None),
                                     "all_or_nothing", True)
        names_to_read = ["all"] if all_or_nothing else [m.name for m in self.motors]
        try:
            for name in names_to_read:
                state = controller.read_state(name)
                measured = controller.state_is_measured(name)
                readings[name] = BrakeStatus(state, not measured, detail)
        except BrakeError as exc:
            self.brake_summary = f"{detail}: NOT READABLE -- {exc}"
            failed = BrakeStatus(BrakeState.UNKNOWN, True, str(exc))
            return {m.name: failed for m in self.motors}

        # The same words the GUI's actuator table uses.
        spoken = {BrakeState.ENGAGED: "engaged",
                  BrakeState.RELEASED: "disengaged",
                  BrakeState.UNKNOWN: "unknown"}
        words = sorted({spoken[r.state] for r in readings.values()})
        measured_all = all(not r.inferred for r in readings.values())
        self.brake_summary = (
            f"{detail}: brakes {'/'.join(words)}"
            + ("" if measured_all else
               " (relay state; nothing measures the brake itself)"))
        if "all" in readings:
            return {m.name: readings["all"] for m in self.motors}
        return readings

    def _with_external_brake(self, status: MotorStatus, brake=None) -> MotorStatus:
        """Show the brake that actually holds this axis.

        `MotorStatus.brake` describes the motor's own brake output, which on
        the pSCT is unassigned -- the brakes are on a separate device. Where
        that device is reachable, its state is the true one, and displaying the
        motor's instead would be showing an indicator that cannot change.

        `brake` is the reading from `_external_brake_reading`; when None it is
        taken now, for callers outside the poll.
        """
        from dataclasses import replace

        if not self.external_brake.available or status.comms_error:
            return status
        if brake is None:
            brake = self._external_brake_reading()
        if not brake or status.name not in brake:
            return status
        return replace(status, brake=brake[status.name])

    # --------------------------------------------------------------- limits

    def check_orientation(self, orientation: Orientation,
                          current: Optional[Orientation] = None) -> List[float]:
        """Validate a target orientation and return the actuator targets in mm.

        Raises PlatformError describing every problem found, rather than the
        first one, so a bad command is fixed in one pass.
        """
        limits = self.cfg.limits
        problems: List[str] = []

        if not (limits.min_focus_mm <= orientation.focus_mm <= limits.max_focus_mm):
            problems.append(
                f"focus {orientation.focus_mm:.4f} mm is outside the allowed "
                f"{limits.min_focus_mm:.3f}..{limits.max_focus_mm:.3f} mm"
            )
        total_tilt = orientation.total_tilt_deg
        if total_tilt > limits.max_tilt_deg:
            problems.append(
                f"total tilt {total_tilt:.4f} deg exceeds the {limits.max_tilt_deg:.3f} "
                f"deg limit (tip {orientation.tip_deg:+.4f}, tilt {orientation.tilt_deg:+.4f})"
            )
        if self.copied_names and total_tilt > COPY_TILT_TOLERANCE_DEG:
            problems.append(
                f"tip/tilt is not possible in bench mode: "
                f"{' and '.join(self.copied_names)} copy {self.copy_source}, "
                "so the plane can only move in focus"
            )

        targets = self.geometry.actuators_from_orientation(orientation)
        for motor, target in zip(self.motors, targets):
            lo, hi = motor.cfg.min_travel_mm, motor.cfg.max_travel_mm
            if not (lo <= target <= hi):
                problems.append(
                    f"actuator {motor.name} would need {target:.4f} mm, outside its "
                    f"travel limits {lo:.3f}..{hi:.3f} mm"
                )

        if current is not None:
            # A move that brings the focal plane back inside the limits is
            # never blocked for being too large.
            #
            # Without this the software can strand itself, and did: a
            # hard-stop search leaves the plate just outside the soft limit by
            # construction, and every move back to the middle is then a step
            # bigger than the single-step limit. The step limit exists to catch
            # a typed mistake, and "return to somewhere legal" is not one --
            # refusing it leaves the operator with no way back except editing
            # the configuration.
            recovering = (
                not self._focus_within_limits(current.focus_mm)
                and self._focus_within_limits(orientation.focus_mm)
                and abs(orientation.focus_mm - self._focus_centre())
                < abs(current.focus_mm - self._focus_centre())
            )
            if recovering:
                self._log(
                    f"Focus is at {current.focus_mm:+.4f} mm, outside the "
                    f"{limits.min_focus_mm:.3f}..{limits.max_focus_mm:.3f} mm "
                    f"limits. Allowing a "
                    f"{abs(orientation.focus_mm - current.focus_mm):.3f} mm move "
                    f"back inside them despite the single-step limit."
                )

            d_focus = abs(orientation.focus_mm - current.focus_mm)
            if d_focus > limits.max_step_mm and not recovering:
                problems.append(
                    f"this move changes focus by {d_focus:.3f} mm, more than the "
                    f"{limits.max_step_mm:.3f} mm single-step limit"
                )
            d_tip = abs(orientation.tip_deg - current.tip_deg)
            d_tilt = abs(orientation.tilt_deg - current.tilt_deg)
            worst = max(d_tip, d_tilt)
            if worst > limits.max_tilt_step_deg and not recovering:
                problems.append(
                    f"this move changes an angle by {worst:.4f} deg, more than the "
                    f"{limits.max_tilt_step_deg:.3f} deg single-step limit"
                )

        if problems:
            raise PlatformError(
                "Move refused, nothing was commanded:\n  - " + "\n  - ".join(problems)
            )
        return targets

    def adopt_hard_stop(self, direction: int, stop_mm: float) -> List[str]:
        """Record an end of travel and make the soft limit follow it.

        The soft limits ship as a guess. A hard stop is a measurement, so once
        one has been found it is the better number -- the limit becomes the
        stop, less `safety_margin_mm`, rather than staying wherever it was set
        before anybody knew where the travel ended.

        If the total travel is known and only one end has been found, the other
        end follows from it. That saves running the search a second time, which
        matters because the second run is the one that drives towards M2 with
        the camera's weight behind it.

        Returns the lines describing what changed, for the log.
        """
        limits = self.cfg.limits
        margin = limits.safety_margin_mm
        notes: List[str] = []

        if direction > 0:
            limits.hard_stop_high_mm = stop_mm
            limits.max_focus_mm = stop_mm - margin
            notes.append(f"Upper end of travel: {stop_mm:+.4f} mm. Upper focus "
                         f"limit set to {limits.max_focus_mm:+.4f} mm "
                         f"({margin:.3f} mm inside it).")
        else:
            limits.hard_stop_low_mm = stop_mm
            limits.min_focus_mm = stop_mm + margin
            notes.append(f"Lower end of travel: {stop_mm:+.4f} mm. Lower focus "
                         f"limit set to {limits.min_focus_mm:+.4f} mm "
                         f"({margin:.3f} mm inside it).")

        travel = limits.total_travel_mm
        if travel:
            if direction > 0 and limits.hard_stop_low_mm is None:
                limits.hard_stop_low_mm = stop_mm - travel
                limits.min_focus_mm = limits.hard_stop_low_mm + margin
                notes.append(
                    f"The other end follows from the {travel:.2f} mm published "
                    f"travel: {limits.hard_stop_low_mm:+.4f} mm, lower limit "
                    f"{limits.min_focus_mm:+.4f} mm. That end is DERIVED, not "
                    f"measured -- run the search downwards to confirm it."
                )
            elif direction < 0 and limits.hard_stop_high_mm is None:
                limits.hard_stop_high_mm = stop_mm + travel
                limits.max_focus_mm = limits.hard_stop_high_mm - margin
                notes.append(
                    f"The other end follows from the {travel:.2f} mm published "
                    f"travel: {limits.hard_stop_high_mm:+.4f} mm, upper limit "
                    f"{limits.max_focus_mm:+.4f} mm. That end is DERIVED, not "
                    f"measured -- run the search upwards to confirm it."
                )

        if limits.min_focus_mm >= limits.max_focus_mm:
            raise PlatformError(
                f"Those ends of travel leave no room: the limits would be "
                f"{limits.min_focus_mm:+.4f}..{limits.max_focus_mm:+.4f} mm. "
                f"Check limits.total_travel_mm and the zero reference."
            )
        notes.extend(self.sync_travel_to_stops())
        return notes

    def sync_travel_to_stops(self) -> List[str]:
        """One set of limits: pin them to the ends of travel, and make every
        actuator use them.

        There used to be three overlapping sets -- focus limits, a travel
        limit per actuator (shipped at -24..+24 mm and never updated), and the
        ends of travel -- and they disagreed: with the stop found further out,
        a move near it passed one check and failed another. Now:

        * where an end of travel is known, the limit on that side is the stop
          less `safety_margin_mm`;
        * where it is not known yet, the configured focus limit stands;
        * each actuator's travel limit is that same range.

        Returns lines for the log (empty if nothing changed).
        """
        limits = self.cfg.limits
        margin = limits.safety_margin_mm
        before = (limits.min_focus_mm, limits.max_focus_mm,
                  [(a.min_travel_mm, a.max_travel_mm) for a in self.cfg.actuators])
        if limits.hard_stop_low_mm is not None:
            limits.min_focus_mm = limits.hard_stop_low_mm + margin
        if limits.hard_stop_high_mm is not None:
            limits.max_focus_mm = limits.hard_stop_high_mm - margin
        if limits.min_focus_mm >= limits.max_focus_mm:
            raise PlatformError(
                f"The ends of travel and the {margin:g} mm margin leave no room: "
                f"{limits.min_focus_mm:+.4f}..{limits.max_focus_mm:+.4f} mm.")
        for actuator in self.cfg.actuators:
            actuator.min_travel_mm = limits.min_focus_mm
            actuator.max_travel_mm = limits.max_focus_mm
        after = (limits.min_focus_mm, limits.max_focus_mm,
                 [(a.min_travel_mm, a.max_travel_mm) for a in self.cfg.actuators])
        if after == before:
            return []
        return [f"Limits: {limits.min_focus_mm:+.4f} to {limits.max_focus_mm:+.4f} mm "
                f"(motor zero), for the focus and every actuator."]

    def _focus_within_limits(self, focus_mm: float) -> bool:
        limits = self.cfg.limits
        return limits.min_focus_mm <= focus_mm <= limits.max_focus_mm

    def _focus_centre(self) -> float:
        limits = self.cfg.limits
        return (limits.min_focus_mm + limits.max_focus_mm) / 2.0

    def preview(self, orientation: Orientation) -> Dict[str, float]:
        """Actuator targets for an orientation, without checking or moving.

        Handy for showing an operator what a command would do before they
        commit to it.
        """
        targets = self.geometry.actuators_from_orientation(orientation)
        return {m.name: t for m, t in zip(self.motors, targets)}

    # ----------------------------------------------------------------- moves

    def move_to_orientation(self, orientation: Orientation, wait: bool = True,
                            check_step: bool = True, kind: str = "move",
                            note: str = "") -> PlatformState:
        """Drive the focal plane to an absolute orientation.

        `kind` and `note` are for the position history: what sort of command
        this was ("move", "fine adjust", "go-back" ...) and anything worth writing
        beside it. They change nothing about the motion.
        """
        with self._move_lock:
            self._require_connected()
            self._abort.clear()
            for m in self.motors:
                m.clear_cancel()

            current = self.read_orientation() if check_step else None
            targets = self.check_orientation(orientation, current=current)

            self._log(
                f"Move to {self._shown_describe(orientation)} -> "
                + ", ".join(f"{m.name} {t + self.shown_offset_mm:.4f} mm"
                            for m, t in zip(self.motors, targets))
            )

            # Nothing has moved yet. From here on, whatever happens is
            # recorded -- a halted move changes the position just as much as
            # a completed one, and "where was it before" is exactly what gets
            # asked after a halt.
            started = time.time()
            before = current if current is not None else self._orientation_or_none()
            actuators_before = self._actuator_positions_or_empty()
            outcome = "done"
            commanded = False
            try:
                self._prepare_for_motion()
                starts = [m.get_position_mm() for m in self.motors]
                velocities = self._apply_synchronised_velocities(targets)

                for motor, target in zip(self.motors, targets):
                    motor.command_position_mm(target)
                    commanded = True

                if wait:
                    self._wait_for_all(targets, starts, velocities)
                    self._settle(list(zip(self.motors, targets)))
                    self._rest_after_move()
            except Exception as exc:  # noqa: BLE001 -- recorded, then re-raised
                outcome = "failed: " + str(exc).splitlines()[0][:160]
                raise
            finally:
                # A refusal before the first target was written moved nothing
                # and is not a position; it is in the log, not the history.
                if commanded:
                    self._record_move(kind, before, orientation, actuators_before,
                                      outcome, note if wait else
                                      (note + " (commanded, not waited for)").strip(),
                                      started)
            return self.read_state()

    def _shown_describe(self, o: Orientation) -> str:
        if not self.shown_offset_mm and not self.shown_from:
            return o.describe()
        return (f"focus {o.focus_mm + self.shown_offset_mm:+.4f} mm{self.shown_from}, "
                f"tip {o.tip_deg:+.5f} deg, tilt {o.tilt_deg:+.5f} deg")

    def move_relative(self, d_focus_mm: float = 0.0, d_tip_deg: float = 0.0,
                      d_tilt_deg: float = 0.0, wait: bool = True) -> PlatformState:
        """Fine adjust the focal plane relative to where it is now."""
        with self._move_lock:
            self._require_connected()
            current = self.read_orientation()
            target = current.offset_by(d_focus_mm, d_tip_deg, d_tilt_deg)
            parts = []
            if d_focus_mm:
                parts.append(f"focus {d_focus_mm:+g} mm")
            if d_tip_deg:
                parts.append(f"tip {d_tip_deg:+g} deg")
            if d_tilt_deg:
                parts.append(f"tilt {d_tilt_deg:+g} deg")
            return self.move_to_orientation(target, wait=wait, kind="fine adjust",
                                            note=", ".join(parts))

    # ------------------------------------------------------------- history

    def _orientation_or_none(self) -> Optional[Orientation]:
        """The orientation now, or None if it cannot be read. Never raises."""
        try:
            return self.read_orientation()
        except Exception:  # noqa: BLE001 -- a record, not a command
            return None

    def _actuator_positions_or_empty(self) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for motor in self.motors:
            try:
                out[motor.name] = motor.get_position_mm()
            except (ModbusError, MotorFault):
                pass
        return out

    def _record_move(self, kind: str, before: Optional[Orientation],
                     commanded: Optional[Orientation],
                     actuators_before: Dict[str, float], outcome: str,
                     note: str, started: float) -> MoveRecord:
        """Write one entry in the position history, reading back where the
        plane ended up rather than assuming it went where it was sent."""
        return self.history.record(
            kind=kind, before=before, commanded=commanded,
            after=self._orientation_or_none(),
            actuators_before=actuators_before,
            actuators_after=self._actuator_positions_or_empty(),
            outcome=outcome, note=note, timestamp=started,
        )

    def go_back(self, wait: bool = True) -> PlatformState:
        """Return to where the focal plane was before the most recent move.

        An ordinary move with an ordinary set of checks: the previous
        orientation is handed to `move_to_orientation`, which will refuse it
        if it is outside the limits or too big a step, exactly as it would a
        typed one. Raises PlatformError when there is nothing to go back to.
        """
        record = self.history.last_with_before()
        if record is None:
            raise PlatformError(
                "There is no previous position to go back to: nothing has "
                "been moved since this record began.")
        return self.move_to_orientation(
            record.before, wait=wait, kind="go-back",
            note=f"back to where it was before the {record.kind} at {record.when}")

    # ------------------------------------------------------ saved positions

    def save_position(self, name: str, note: str = "") -> tuple:
        """Remember where the focal plane is now, under `name`.

        Returns (the saved position, whether it replaced one of that name).
        Reads every actuator's encoder: a saved position is somewhere the
        plane has actually been, never a number somebody typed.
        """
        self._require_connected()
        counts = {m.name: m.get_position_counts() for m in self.motors}
        mm = {m.name: m.cfg.counts_to_mm(counts[m.name]) for m in self.motors}
        orientation = self.geometry.orientation_from_actuators(
            [mm[m.name] for m in self.motors])
        position = make_saved_position(name, orientation, counts, mm, note)
        replaced = self.saved_positions.put(position)
        self._log(f"Saved position {position.name!r}: {orientation.describe()}")
        return position, replaced

    def saved_position_target(self, position: SavedPosition) -> Orientation:
        """Where going to `position` means, measured from today's zero.

        Worked out from the stored encoder counts, so the answer is the same
        physical place even if the zero has been set again since it was saved.
        """
        missing = [m.name for m in self.motors
                   if m.name not in position.actuator_counts]
        if missing:
            raise PlatformError(
                f"Saved position {position.name!r} has no reading for "
                f"{', '.join(missing)}, so it cannot be gone back to safely.")
        return self.geometry.orientation_from_actuators(
            [m.cfg.counts_to_mm(position.actuator_counts[m.name])
             for m in self.motors])

    def go_to_saved_position(self, name: str, wait: bool = True) -> PlatformState:
        """An ordinary checked move to a saved position."""
        position = self.saved_positions.get(name)
        if position is None:
            raise PlatformError(f"There is no saved position called {name!r}.")
        return self.move_to_orientation(
            self.saved_position_target(position), wait=wait,
            kind="saved position", note=position.name)

    def move_to_polar_tilt(self, focus_mm: float, total_tilt_deg: float,
                           azimuth_deg: float, wait: bool = True) -> PlatformState:
        """Absolute move expressed as 'tilt this much, uphill towards there'."""
        return self.move_to_orientation(
            Orientation.from_polar_tilt(focus_mm, total_tilt_deg, azimuth_deg),
            wait=wait,
        )

    def move_actuator_mm(self, name: str, target_mm: float, relative: bool = False,
                         wait: bool = True) -> MotorStatus:
        """Move one actuator on its own.

        For commissioning: checking a direction sign, measuring the scale
        factor, or backing a single axis off a limit. It intentionally
        bypasses the orientation limits, because during commissioning the
        orientation is not yet meaningful -- but it still enforces that
        actuator's own travel limits.
        """
        if name in self.copied_names:
            raise PlatformError(
                f"{name} is standing in as a copy of {self.copy_source} in bench "
                f"mode, so it cannot be moved on its own. Jog {self.copy_source}, "
                "and the copies follow it.")
        with self._move_lock:
            motor = self.motor(name)
            if not motor.connected:
                raise PlatformError(f"{motor.name} is not connected.")
            motor.clear_cancel()
            target = (motor.get_position_mm() + target_mm) if relative else target_mm
            motor.check_travel_limit(target)

            step = abs(target - motor.get_position_mm())
            if step > self.cfg.limits.max_step_mm:
                raise PlatformError(
                    f"{motor.name}: a {step:.3f} mm step exceeds the "
                    f"{self.cfg.limits.max_step_mm:.3f} mm single-step limit. "
                    "Raise limits.max_step_mm if this is genuinely intended."
                )

            # The same checks a coordinated move makes before any brake comes
            # off. A jog used to skip them: it enabled the drives and released
            # the brakes without looking at the drive's error bits or its
            # supply -- and a JVL with no main supply still answers Modbus and
            # can report Position mode while holding nothing at all.
            for other in self.motors:
                if not other.connected:
                    continue
                errors = other.get_errors()
                if errors:
                    raise PlatformError(
                        f"Not jogging: {other.name} has an active error "
                        f"({other.error_text()}). Clear it first; the brakes "
                        "have not been touched.")
            self._check_drive_power()

            # Every drive is enabled, not just the one that moves: a jog tilts
            # the plate about the other two, and they should be holding while
            # it does. Each is read back as being in Position mode.
            for other in self.motors:
                if other.connected:
                    other.ensure_position_mode()
            self._refuse_if_straining(self.motors)
            self._log(f"{motor.name}: drive on and holding (read back); "
                      "brakes may now be released.")
            self._release_brake_if_controlled(motor)
            # Only this actuator's brake when the brakes are separate; all of
            # them when one switch holds all three.
            self._release_external_brakes(motor.name)
            motor.set_velocity(motor.cfg.velocity_raw)

            started = time.time()
            before = self._orientation_or_none()
            actuators_before = self._actuator_positions_or_empty()
            outcome = "done"
            commanded = False
            try:
                motor.command_position_mm(target)
                commanded = True
                self._log(f"{motor.name}: single-axis move to {target:.4f} mm.")
                if wait and motor.wait_for_in_position():
                    self._settle([(motor, target)])
                    self._rest_after_move()
                elif wait:
                    # wait_for_in_position has already halted this axis.
                    outcome = "failed: timed out"
                    raise PlatformError(
                        f"{motor.name} did not reach {target:.4f} mm within "
                        f"{motor.cfg.move_timeout_s:.0f} s, and has been halted where "
                        "it got to."
                    )
            except Exception as exc:  # noqa: BLE001 -- recorded, then re-raised
                if outcome == "done":
                    outcome = "failed: " + str(exc).splitlines()[0][:160]
                raise
            finally:
                if commanded:
                    self._record_move(
                        "jog", before, None, actuators_before, outcome,
                        f"{motor.name} to {target:+.4f} mm"
                        + (f" ({target_mm:+g} mm relative)" if relative else ""),
                        started)
            return motor.read_status()

    # --------------------------------------------------------- move internals

    def _require_connected(self) -> None:
        missing = [m.name for m in self.motors if not m.connected]
        if missing:
            raise PlatformError(
                f"Not connected to actuator(s) {', '.join(missing)}. "
                "A coordinated move needs all three."
            )

    def _prepare_for_motion(self) -> None:
        """Position mode and brakes released on all three, or nothing moves.

        The order matters. Power is checked before anything is commanded,
        because a drive with no supply accepts writes and ignores them. The
        drives are enabled before the brakes come off, because a brake released
        over a passive drive leaves the focal plane held by nothing.
        """
        for motor in self.motors:
            errors = motor.get_errors()
            if errors:
                raise PlatformError(
                    f"{motor.name} has an active error ({motor.error_text()}). "
                    "Clear it before moving."
                )
        self._check_drive_power()
        for motor in self.motors:
            motor.ensure_position_mode()
        self._refuse_if_straining(self.motors)
        for motor in self.motors:
            self._release_brake_if_controlled(motor)
        self._release_external_brakes()

    def _refuse_if_straining(self, motors) -> None:
        """No move starts while a motor is already over its torque limit.

        A motor working that hard before it has been asked to go anywhere is
        pushing against something. Moving would force it, and the limit check
        during a move only stops it after it has pushed for a few readings.
        Checked with the drives on and holding, before any brake comes off.
        """
        straining = []
        for motor in motors:
            if not motor.connected or not motor.cfg.stall_protection:
                continue
            percent = motor.get_torque_percent()
            if percent is not None and percent >= motor.cfg.stall_torque_percent:
                straining.append(f"{motor.name} at {percent:.0f}% "
                                 f"(limit {motor.cfg.stall_torque_percent:.0f}%)")
        if straining:
            raise PlatformError(
                "Not moving: " + ", ".join(straining) + " is already over the "
                "torque limit while standing still, so something is pushing "
                "against it. Forcing a move could damage the telescope. Nothing "
                "was commanded and the brakes were not touched. Find out what "
                "it is pressing on first (an end stop, an obstruction, a brake "
                "that is on).")

    def _check_drive_power(self) -> None:
        """Refuse to move if a drive's supply has failed.

        A JVL with no main supply still answers Modbus from its control
        supply: the target is accepted, the mode reads back, and nothing
        turns. That looks exactly like the software being broken, so it is
        worth naming.

        The comparison is register 97 against its *own* recorded healthy value,
        not against register 139 ('Acceptance Voltage'). Those two are both in
        the drive's raw units, but nothing establishes that they share a scale,
        and on the pSCT bench motor they read 1794 and 2054 -- which under the
        old check read as "below acceptance" and would have refused every move
        on a motor running perfectly well at 48 V.

        Register 97 is judged on its own scale: 1804 raw = 48.0 V, measured
        against MacTalk on the pSCT motor, or a reading recorded for that
        motor. A supply that cannot be read at all is said once and does not
        block, since the motor answering Modbus is itself evidence of power.
        """
        dead = []
        unknown = []
        for motor in self.motors:
            verdict, explanation = motor.supply_is_healthy()
            if verdict is False:
                dead.append(f"{motor.name}: {explanation}")
            elif verdict is None:
                unknown.append(motor.name)

        if dead:
            raise PlatformError(
                "Not moving: the supply has failed on "
                + "; ".join(dead)
                + ". The motors are reachable -- they answer Modbus from their "
                "control supply -- but with the main supply down they will "
                "accept a target and not move. Check the supply and its "
                "breaker before commanding anything else."
            )
        if unknown and not self._warned_no_supply_baseline:
            self._warned_no_supply_baseline = True
            self._log(
                "Note: the supply voltage could not be read on "
                + ", ".join(unknown)
                + ", so a failed supply cannot be detected there. A motor that "
                "silently ignores its targets would look like a software fault."
            )

    def _release_external_brakes(self, name: Optional[str] = None) -> None:
        """Take the site's brakes off, or say why the move cannot go ahead.

        `name` is for a single-axis move: with separate brakes only that
        actuator's comes off, and only its own drive has to be holding.
        """
        target = self._brake_target(name)
        who = "the brakes" if target == "all" else f"the {target} brake"
        controller = self.external_brake
        if not controller.available:
            # Nothing to command and nothing to read. Say so once per move
            # rather than pretending the brakes are off.
            self._log(
                "Note: the focal-plane brakes are not under software control, "
                "so this move assumes they have already been released from the "
                "brake page. If nothing moves, that is the first thing to check."
            )
            return

        from .external_brake import BrakeError
        try:
            # Fresh, not the poll's cached reading: this answer decides
            # whether the motors are about to push against a clamped brake.
            state = controller.read_state(target, fresh=True)
        except BrakeError as exc:
            if self.allow_unknown_brakes:
                self._log(f"The brake controller could not be read ({exc}); "
                          "moving without touching the brakes, as confirmed.")
                return
            raise PlatformError(
                f"Not moving: the brake controller could not be read ({exc}). "
                "Moving without knowing whether the brakes are off risks driving "
                "the motors against them."
            ) from exc

        if state is BrakeState.RELEASED:
            return

        if not self.holding(target):
            raise PlatformError(
                f"Not moving: {who} are engaged and the drive(s) they hold are "
                "not holding position, so releasing now would leave the focal "
                "plane held by nothing. Enable the drives first."
            )
        try:
            controller.release(target, drives_holding=True)
        except BrakeError as exc:
            raise PlatformError(
                f"Not moving: {who} did not release ({exc}). Driving the "
                "motors against an engaged brake is how a lead screw or a "
                "coupling gets damaged."
            ) from exc

        try:
            after = controller.read_state(target, fresh=True)
        except BrakeError:
            return          # commanded, but unreadable; already logged
        if after is not BrakeState.RELEASED:
            raise PlatformError(
                f"Not moving: {who} were commanded to release but still "
                f"read back as {after.value}. Check the brake supply -- these "
                "brakes are spring-applied, so with no power to them they clamp."
            )

    def _release_brake_if_controlled(self, motor: JVLMotor) -> None:
        if not motor.brake_is_software_controlled:
            return
        status = motor.get_brake_status()
        if status.state is BrakeState.RELEASED:
            return
        motor.release_brake()

    def _apply_synchronised_velocities(self, targets_mm: Sequence[float]) -> Dict[str, int]:
        """Scale each axis' speed so all three finish at the same moment.

        Without this the shortest of the three moves finishes first, and until
        the last one lands the plate sits at an orientation nobody asked for,
        pivoting on its ball joints. Scaling by distance keeps the plate on a
        straight line between the two orientations.
        """
        velocities = {m.name: int(m.cfg.velocity_raw) for m in self.motors}
        if not self.cfg.synchronize_moves:
            for motor in self.motors:
                motor.set_velocity(motor.cfg.velocity_raw)
            return velocities

        deltas = [
            abs(target - motor.get_position_mm())
            for motor, target in zip(self.motors, targets_mm)
        ]
        longest = max(deltas)
        if longest <= 0:
            return velocities
        for motor, delta in zip(self.motors, deltas):
            scaled = motor.cfg.velocity_raw * (delta / longest)
            velocities[motor.name] = max(self.cfg.min_velocity_raw, int(round(scaled)))
            motor.set_velocity(velocities[motor.name])
        return velocities

    def _wait_for_all(self, targets_mm: Optional[Sequence[float]] = None,
                      starts_mm: Optional[Sequence[float]] = None,
                      velocities: Optional[Dict[str, int]] = None) -> None:
        """Wait for every axis, then report all stragglers together.

        Given where each started and where each is going, it also keeps them
        together on the way (see `_keep_in_step`).
        """
        deadline = time.monotonic() + max(m.cfg.move_timeout_s for m in self.motors)
        pending = list(self.motors)
        sync = None
        if targets_mm is not None and starts_mm is not None:
            sync = _InStep(self.motors, list(starts_mm), list(targets_mm),
                           dict(velocities or {}))
        while (pending or (sync and sync.paused)) and time.monotonic() < deadline:
            if self._abort.is_set():
                raise PlatformError(
                    "Move did not finish: STOP (or EMERGENCY) was used while it "
                    "was running. The actuators are wherever they were halted, "
                    "so the focal plane is at neither the old orientation nor "
                    "the requested one -- read the current orientation before "
                    "commanding anything else."
                )
            still_pending = []
            for motor in pending:
                if motor.cancelled:
                    raise PlatformError(
                        f"Move did not finish: {motor.name} was stopped while it "
                        "was running."
                    )
                try:
                    self._poll_during_move(motor, still_pending)
                except ModbusError as exc:
                    # The cable came out, or the drive stopped answering. The
                    # other two are still moving towards a target this one will
                    # never reach, which is how the plate gets racked about its
                    # ball joints -- so they stop too, before anything is
                    # reported.
                    self._halt_all_quietly(f"lost contact with {motor.name}")
                    raise PlatformError(
                        f"Lost contact with {motor.name} during the move: {exc}. "
                        "The other actuators have been halted where they were, so "
                        "the focal plane is at neither the old orientation nor the "
                        "requested one. Check that motor's cable and power, then "
                        "read the current orientation before commanding anything "
                        "else."
                    ) from exc
            if sync is not None:
                sync.finished = {m.name for m in self.motors
                                 if m not in still_pending and m.name not in sync.paused}
                sync.resumed = set()
                self._keep_in_step(sync)
                # A held motor reads as arrived -- it is where it was told to
                # be -- but it is not where it is going; nor is one that was
                # sent on again just now.
                for motor in self.motors:
                    if ((motor.name in sync.paused or motor.name in sync.resumed)
                            and motor not in still_pending):
                        still_pending.append(motor)
            if not still_pending:
                return
            pending = still_pending
            time.sleep(0.1)
        if pending:
            names = ", ".join(m.name for m in pending)
            self._halt_all_quietly(f"{names} did not reach position in time")
            raise PlatformError(
                f"Timed out waiting for actuator(s) {names} to reach position. "
                "All three actuators have been halted. Check for a mechanical "
                "obstruction, a brake that did not release, or a velocity set "
                "so low the move could not finish inside the timeout."
            )

    def _poll_during_move(self, motor: JVLMotor,
                          still_pending: List[JVLMotor]) -> None:
        """One motor's mid-move health check.

        Appends the motor to `still_pending` if it has not arrived yet. Raises
        rather than returning a verdict, because every problem it can find
        means the whole move stops.
        """
        errors = motor.get_errors()
        if errors:
            # One axis faulting does not stop the other two, and two actuators
            # continuing to a target the third will never reach is precisely
            # how the plate gets racked about its ball joints. Halt
            # everything, then report.
            text = motor.error_text()
            self._halt_all_quietly(f"{motor.name} faulted mid-move")
            raise PlatformError(
                f"{motor.name} faulted during the move: {text}. All three "
                "actuators have been halted where they were, so the focal "
                "plane is at neither the old orientation nor the requested "
                "one -- read the current orientation before continuing."
            )

        torque = motor.check_stall()
        if torque is not None:
            # Something is resisting. Waiting out the timeout would mean
            # pushing against it for the rest of the move, with the other two
            # still travelling.
            self._halt_all_quietly(f"{motor.name} is stalling")
            raise PlatformError(
                f"{motor.name} was resisting at {torque:.0f}% torque, so the "
                f"move was stopped and all three actuators halted. Something "
                f"is in the way, an axis has reached the end of its travel, or "
                f"a brake did not release. The focal plane is at neither the "
                f"old orientation nor the requested one -- read the current "
                f"orientation before continuing."
            )

        try:
            lag = motor.get_follow_error()
        except ModbusError:
            lag = 0
        limit = self._big_error_counts(motor)
        if abs(lag) > limit:
            brakes = self._halt_and_brake(f"{motor.name} fell too far behind")
            raise PlatformError(
                f"{motor.name} fell {abs(lag) / motor.cfg.resolved_counts_per_mm:.3f} mm "
                f"behind where it was being driven (the limit is "
                f"{self.cfg.max_position_error_mm:g} mm), so it has slipped or is "
                f"blocked. All three were halted and {brakes}. Read the current "
                "orientation before continuing.")

        if not motor.is_in_position():
            still_pending.append(motor)

    # ------------------------------------------------- staying together

    def _big_error_counts(self, motor: JVLMotor) -> float:
        return self.cfg.max_position_error_mm * motor.cfg.resolved_counts_per_mm

    def _halt_and_brake(self, reason: str) -> str:
        """Stop all three, holding, then apply the brakes. Says how that went.

        The drives stay on and holding: with the brakes also on, nothing can
        drop. Never raises, because it runs on the way to reporting something
        else.
        """
        self._halt_all_quietly(reason)
        try:
            results = self.set_all_brakes(engaged=True)
        except Exception as exc:  # noqa: BLE001 -- reported, not raised
            results = {"all": str(exc)}
        failed = {k: v for k, v in results.items() if not str(v).startswith("ok")}
        if not failed:
            self._log(f"Stopped and brakes applied: {reason}.")
            return "the brakes were applied (drives still holding)"
        detail = "; ".join(f"{k}: {v}" for k, v in failed.items())
        self._log(f"Stopped ({reason}); the brakes could not be applied: {detail}")
        return (f"the brakes could NOT be applied ({detail}), so the drives are "
                "holding it on their own")

    def _keep_in_step(self, sync: "_InStep") -> None:
        """Hold any actuator that gets ahead, so the plate does not tilt.

        Progress along each actuator's own move is compared. The slowest one
        says where the plate should be; one more than `sync_pause_mm` ahead of
        that is held where it is, and sent on again -- a fifth slower -- once
        the slowest has caught up to half that. Out of step by
        `sync_abort_mm`, or the others held for `sync_max_wait_s` without the
        slow one catching up, and the move is stopped with the brakes applied.
        """
        cfg = self.cfg
        positions = [m.get_position_mm() for m in self.motors]
        leads = sync.leads(positions)
        if not leads:
            return
        worst_name = max(leads, key=leads.get)
        slowest = sync.slowest(positions)
        if leads[worst_name] > cfg.sync_abort_mm:
            load = self._torque_note(slowest)
            brakes = self._halt_and_brake("the actuators got out of step")
            raise PlatformError(
                f"The actuators got out of step: {worst_name} was "
                f"{leads[worst_name]:.3f} mm ahead of {slowest} (the limit is "
                f"{cfg.sync_abort_mm:g} mm), which tilts the plate. All three "
                f"were halted and {brakes}. {slowest} may be under more load "
                f"or obstructed{load}. Read the current orientation before "
                "continuing.")

        now = time.monotonic()
        for motor in self.motors:
            name = motor.name
            lead = leads.get(name)
            if lead is None:
                continue
            if name in sync.paused:
                if lead <= cfg.sync_pause_mm / 2:
                    sync.velocities[name] = max(
                        cfg.min_velocity_raw,
                        int(sync.velocities.get(name, motor.cfg.velocity_raw) * 0.8))
                    motor.set_velocity(sync.velocities[name])
                    motor.command_position_mm(sync.targets[sync.index[name]])
                    del sync.paused[name]
                    sync.resumed.add(name)
                    self._log(f"{name}: {slowest} caught up; going on a little slower.")
                elif now - sync.paused[name] > cfg.sync_max_wait_s:
                    load = self._torque_note(slowest)
                    brakes = self._halt_and_brake(f"{slowest} is not keeping up")
                    raise PlatformError(
                        f"{slowest} is not keeping up: {name} was held for "
                        f"{cfg.sync_max_wait_s:g} s waiting for it and it did not "
                        f"catch up. All three were halted and {brakes}. "
                        f"{slowest} may be blocked or straining{load}. Read the "
                        "current orientation before continuing.")
            elif lead > cfg.sync_pause_mm and not sync.arrived(motor, positions):
                motor.command_position_counts(motor.get_projected_position_counts())
                sync.paused[name] = now
                self._log(f"{name}: {lead:.3f} mm ahead of {slowest}; holding "
                          "it until the others catch up.")

    def _torque_note(self, name: str) -> str:
        """", resisting at 62% torque" for the error message, or nothing."""
        try:
            percent = self.motor(name).get_torque_percent()
        except Exception:  # noqa: BLE001 -- extra detail only
            return ""
        return "" if percent is None else f" (it was resisting at {percent:.0f}% torque)"

    def _settle(self, pairs) -> None:
        """Nudge each motor until its encoder is on its target.

        Under load a stepper sits slightly behind its command. Asking for the
        difference on top puts the shaft where it was meant to be. Tried at
        most `settle_max_tries` times, for differences bigger than
        `settle_deadband_counts`; anything still off is reported, not chased.
        A difference beyond `max_position_error_mm` is not a settling matter:
        the move is stopped and the brakes applied.
        """
        cfg = self.cfg
        pairs = [(m, cfg_target) for m, cfg_target in pairs
                 if m.name not in self.copied_names]
        if not pairs:
            return
        start_errors = {}
        corrections = 0
        for attempt in range(cfg.settle_max_tries + 1):
            off = []
            for motor, target_mm in pairs:
                target = motor.cfg.mm_to_counts(target_mm)
                error = target - motor.get_position_counts()
                start_errors.setdefault(motor.name, error)
                if abs(error) > self._big_error_counts(motor):
                    brakes = self._halt_and_brake(f"{motor.name} is far off its target")
                    raise PlatformError(
                        f"{motor.name} ended {abs(error) / motor.cfg.resolved_counts_per_mm:.3f} mm "
                        f"from its target (the limit is {cfg.max_position_error_mm:g} "
                        f"mm). All three were halted and {brakes}. Read the "
                        "current orientation before continuing.")
                if abs(error) > cfg.settle_deadband_counts:
                    off.append((motor, error))
            if not off or not cfg.settle_enabled or attempt == cfg.settle_max_tries:
                break
            corrections += 1
            for motor, error in off:
                motor.command_position_counts(motor.get_target_counts() + error)
            for motor, _error in off:
                if not motor.wait_for_in_position(timeout_s=10.0):
                    raise PlatformError(
                        f"Settling did not finish: {motor.name} was stopped or "
                        "did not arrive.")
        if not corrections:
            return
        final = {m.name: m.cfg.mm_to_counts(t) - m.get_position_counts()
                 for m, t in pairs}
        self._log("Settled at the target ("
                  + ", ".join(f"{n} {start_errors[n]:+d} -> {final[n]:+d} counts"
                              for n in final)
                  + f", {corrections} correction{'s' if corrections > 1 else ''}).")

    # ----------------------------------------------------- hard-stop seeking

    def seek_hard_stop_together(
        self,
        direction: int,
        budget_mm: float = 30.0,
        max_spread_mm: Optional[float] = None,
        speed_fraction: float = 0.25,
        no_progress_s: float = 0.6,
        no_progress_mm: float = 0.003,
        poll_s: float = 0.05,
        level_after: bool = True,
        back_off_mm: float = 0.5,
        progress: Optional[Callable[["HardStopProgress"], None]] = None,
    ) -> "HardStopResult":
        """Run all three actuators out together until the travel ends.

        This is the site's calibration procedure -- drive to the end and let it
        stop. It is done to all three at once because that is the only safe way:
        sending one actuator to its end stop on its own tilts the focal plane
        about the other two ball joints, and the site's experience is that this
        can break something. There is deliberately no way to ask this for a
        single axis.

        The three move **continuously and together**, not in steps. One target
        is written to each -- the same distance in millimetres -- at a speed
        scaled so they all travel at the same millimetres per second even if
        their calibrations differ. They then run smoothly to the end while this
        watches, every `poll_s`:

        * torque on each axis, which climbs when something resists;
        * whether each axis is still making progress, because at a stop it
          is not;
        * how far apart the three have drifted, because divergence *is* tilt.

        The first axis to stop ends the run for all three: every motor is
        commanded to hold where it is, in the same poll, so no axis keeps
        pushing and none keeps travelling past the others. The two that did not
        stop are then backed off to match the one that did (`level_after`),
        because they carry on for a fraction of a second before the halt lands
        and that difference is a tilt.

        `direction` is +1 or -1: + drives the camera towards M1, - towards M2.
        Raises PlatformError if the axes diverge past `max_spread_mm`, or if
        the budget is used up without anything stopping.
        """
        if direction not in (1, -1):
            raise ValueError(f"direction must be +1 or -1, got {direction}")
        if budget_mm <= 0:
            raise ValueError("budget_mm must be positive")

        limit_spread = (self.cfg.limits.max_hard_stop_spread_mm
                        if max_spread_mm is None else max_spread_mm)

        self._require_connected()
        self._abort.clear()
        for motor in self.motors:
            motor.clear_cancel()
        self._prepare_for_motion()

        start_mm = {m.name: m.get_position_mm() for m in self.motors}
        original_velocity = {m.name: m.read_register("V_SOLL") for m in self.motors}
        for motor in self.motors:
            motor.peak_torque_percent = None

        worst_spread = 0.0
        stopped_by: List[str] = []
        reasons: Dict[str, str] = {}
        started = time.time()
        before = self._orientation_or_none()
        outcome = "done"
        commanded = False

        try:
            self._set_synchronised_seek_speed(original_velocity, speed_fraction)

            # One target each, the same distance, written as close together as
            # three Modbus writes allow. From here they simply run.
            for motor in self.motors:
                target = start_mm[motor.name] + direction * budget_mm
                motor.command_position_counts(motor.cfg.mm_to_counts(target))
                commanded = True

            last_movement = {m.name: time.monotonic() for m in self.motors}
            last_position = dict(start_mm)
            #: Largest distance any axis covered between two polls. Used to
            #: size the divergence guard's allowance for detection latency.
            per_poll = 0.0
            deadline = time.monotonic() + self._seek_timeout_s(budget_mm)

            while True:
                if self._abort.is_set():
                    self._settle_all_where_they_are()
                    raise PlatformError(
                        "Hard-stop search stopped by the operator. All three "
                        "actuators are holding where they were halted."
                    )
                if time.monotonic() > deadline:
                    self._settle_all_where_they_are()
                    raise PlatformError(
                        f"Hard-stop search gave up after "
                        f"{self._seek_timeout_s(budget_mm):.0f} s without any "
                        "actuator reaching a stop. All three are holding where "
                        "they are. Either the axes are moving far slower than "
                        "their configured velocity, or something is not moving "
                        "at all."
                    )

                now = time.monotonic()
                here = {m.name: m.get_position_mm() for m in self.motors}
                # Snapshot before the per-motor loop below updates it, or the
                # per-poll distance measured afterwards is always zero.
                previous = dict(last_position)

                for motor in self.motors:
                    name = motor.name
                    # --- torque ---
                    torque = motor.check_stall()
                    if torque is not None and name not in reasons:
                        stopped_by.append(name)
                        reasons[name] = f"torque reached {torque:.0f}%"
                        continue
                    # --- a drive fault ---
                    errors = motor.get_errors()
                    if errors and name not in reasons:
                        stopped_by.append(name)
                        reasons[name] = f"drive faulted: {motor.error_text()}"
                        continue
                    # --- progress ---
                    if abs(here[name] - last_position[name]) >= no_progress_mm:
                        last_position[name] = here[name]
                        last_movement[name] = now
                    elif (now - last_movement[name] > no_progress_s
                            and name not in reasons):
                        travelled_so_far = abs(here[name] - start_mm[name])
                        if travelled_so_far < budget_mm - 0.05:
                            stopped_by.append(name)
                            reasons[name] = (
                                f"stopped moving {no_progress_s:.1f} s after "
                                f"{travelled_so_far:.3f} mm, with "
                                f"{budget_mm - travelled_so_far:.3f} mm still "
                                f"commanded")

                travelled = {n: here[n] - start_mm[n] for n in here}
                spread = max(travelled.values()) - min(travelled.values())
                worst_spread = max(worst_spread, spread)

                # How far an axis covers between polls, measured rather than
                # assumed. The divergence guard has to allow for it: when one
                # axis meets its stop, confirming the stall takes
                # `stall_persist_samples` consecutive readings, and the other
                # two keep travelling throughout. That is detection latency,
                # not a mechanism tilting, and a guard that cannot tell them
                # apart aborts every successful search.
                moved_this_poll = max(
                    (abs(here[n] - previous[n]) for n in here), default=0.0)
                per_poll = max(per_poll, moved_this_poll)
                latency_allowance = per_poll * (
                    max(m.cfg.stall_persist_samples for m in self.motors) + 2)

                if progress is not None:
                    progress(HardStopProgress(
                        positions_mm=dict(here),
                        travelled_mm=max(abs(v) for v in travelled.values()),
                        spread_mm=spread,
                        torque_percent={m.name: (m.get_torque_percent() or 0.0)
                                        for m in self.motors},
                    ))

                if stopped_by:
                    # In this same poll, before anything else moves further.
                    self._settle_all_where_they_are()
                    break

                if spread > limit_spread + latency_allowance:
                    self._settle_all_where_they_are()
                    lagging = min(travelled, key=travelled.get)
                    leading = max(travelled, key=travelled.get)
                    raise PlatformError(
                        f"Hard-stop search abandoned: the actuators drifted "
                        f"{spread:.4f} mm apart (limit {limit_spread:.4f} mm "
                        f"plus {latency_allowance:.4f} mm allowed for stall "
                        f"detection latency), "
                        f"with {leading} ahead of {lagging}. That difference is "
                        f"tilt in the focal plane, which is what running all "
                        f"three together is meant to avoid. All three have been "
                        f"halted where they were. Check for a binding axis or a "
                        f"wrong counts_per_mm before trying again."
                    )

                if all(abs(travelled[m.name]) >= budget_mm - 0.05
                       for m in self.motors):
                    self._settle_all_where_they_are()
                    raise PlatformError(
                        f"Travelled the whole {budget_mm:.1f} mm budget without "
                        "any actuator finding a stop. Either the travel is "
                        "longer than expected, or the stall threshold is too "
                        "high for the end stop to register. All three actuators "
                        "are holding where they are."
                    )

                time.sleep(poll_s)
        except Exception as exc:  # noqa: BLE001 -- recorded, then re-raised
            outcome = "failed: " + str(exc).splitlines()[0][:160]
            raise
        finally:
            for motor in self.motors:
                try:
                    motor.set_velocity(original_velocity[motor.name])
                except (ModbusError, MotorFault):
                    pass
            if outcome != "done" and commanded:
                self._record_move("find-stop", before, None, start_mm, outcome,
                                  f"direction {direction:+d}", started)

        stop_mm = {m.name: m.get_position_mm() for m in self.motors}
        levelled = self._level_after_stop(start_mm, direction) if level_after else False
        backed_off = self._back_off_from_stop(direction, back_off_mm)

        end_mm = {m.name: m.get_position_mm() for m in self.motors}
        travelled = {name: end_mm[name] - start_mm[name] for name in end_mm}
        result = HardStopResult(
            direction=direction,
            stopped_by=list(dict.fromkeys(stopped_by)),
            reasons=reasons,
            start_mm=start_mm,
            positions_mm=end_mm,
            positions_counts={m.name: m.get_position_counts() for m in self.motors},
            travelled_mm=travelled,
            spread_mm=max(travelled.values()) - min(travelled.values()),
            worst_spread_mm=worst_spread,
            peak_torque_percent={m.name: (m.peak_torque_percent or 0.0)
                                 for m in self.motors},
            levelled=levelled,
            stop_mm=stop_mm,
            backed_off_mm=backed_off,
        )
        self._log(result.summary())
        self._record_move(
            "find-stop", before, None, start_mm, "done",
            f"direction {direction:+d}, stopped by "
            f"{', '.join(result.stopped_by) or 'nothing'}, travel ends at "
            f"{sum(stop_mm.values()) / len(stop_mm):+.4f} mm", started)
        return result

    def _back_off_from_stop(self, direction: int, back_off_mm: float) -> float:
        """Retreat a little from the end of travel.

        Leaving the mechanism resting against its stop is how a lead screw
        gets damaged over time, and it also leaves the focal plane parked
        outside the soft limits, so the next ordinary move is refused. Backing
        off is always a move away from the stop, so it cannot press anything
        harder.
        """
        if back_off_mm <= 0:
            return 0.0
        targets = {}
        for motor in self.motors:
            targets[motor.name] = motor.get_position_mm() - direction * back_off_mm
        self._log(f"Backing off {back_off_mm:.3f} mm from the stop.")
        for motor in self.motors:
            motor.command_position_mm(targets[motor.name])
        deadline = time.monotonic() + max(m.cfg.move_timeout_s for m in self.motors)
        while time.monotonic() < deadline:
            if all(m.is_in_position() for m in self.motors):
                return back_off_mm
            if self._abort.is_set():
                break
            time.sleep(0.05)
        self._log("The retreat from the stop did not complete; the actuators may "
                  "still be resting against it.")
        return 0.0

    def _set_synchronised_seek_speed(self, original: Dict[str, int],
                                     fraction: float) -> None:
        """Set a slow speed that is the same in millimetres per second.

        Equal raw velocity only means equal speed when the actuators have the
        same counts per millimetre. Scaling by that ratio is what keeps them
        together in the units that matter -- and staying together is the whole
        point of running all three at once.

        Deliberately slow. Meeting a mechanical stop at speed is how a lead
        screw gets damaged, and a slow approach makes the torque rise easy to
        tell apart from an acceleration transient.
        """
        reference = max(m.cfg.resolved_counts_per_mm for m in self.motors)
        for motor in self.motors:
            scale = motor.cfg.resolved_counts_per_mm / reference
            raw = int(round(original[motor.name] * fraction * scale))
            motor.set_velocity(max(self.cfg.min_velocity_raw, raw))

    def _seek_timeout_s(self, budget_mm: float) -> float:
        """How long the whole run may take before it is abandoned.

        Generous: this is a backstop against a poll loop that would otherwise
        run for ever, not a performance target. The real endings are a stop
        being found, the budget being covered, or the axes diverging.
        """
        return max(m.cfg.move_timeout_s for m in self.motors) + budget_mm * 20.0


    def _level_after_stop(self, start_mm: Dict[str, float], direction: int) -> bool:
        """Bring the axes that did not stop back to match the one that did.

        The search halts everything the moment the first axis stops, but the
        other two were mid-step when that happened, so they sit up to one step
        further out. That difference is tilt. Backing them off is always a move
        *away* from the stop, so it cannot press anything harder.
        """
        travelled = {m.name: m.get_position_mm() - start_mm[m.name]
                     for m in self.motors}
        # The axis that stopped is the least-travelled one in the direction of
        # travel; matching it is what makes the plate flat again.
        reference = min(travelled.values()) if direction > 0 else max(travelled.values())
        moving = [m for m in self.motors
                  if abs(travelled[m.name] - reference) > m.cfg.in_position_tol_mm]
        if not moving:
            return False

        self._log(
            f"Levelling: backing off {', '.join(m.name for m in moving)} to match "
            f"the actuator that stopped, so the plate does not stay tilted."
        )
        for motor in moving:
            motor.command_position_mm(start_mm[motor.name] + reference)

        deadline = time.monotonic() + max(m.cfg.move_timeout_s for m in moving)
        while time.monotonic() < deadline:
            if all(m.is_in_position() for m in moving):
                return True
            if self._abort.is_set():
                break
            time.sleep(0.05)
        self._log("Levelling did not complete -- the plate may still be tilted. "
                  "Check the actuator positions before commanding a move.")
        return False

    def _settle_all_where_they_are(self) -> None:
        """Command every axis to hold its present position.

        Without this the motors are left commanded past where they got to and
        keep pushing -- against a stop, or against each other through the
        plate.
        """
        for motor in self.motors:
            try:
                motor.settle_at_stop(motor.get_position_counts())
            except (ModbusError, MotorFault) as exc:
                self._log(f"{motor.name}: could not release against the stop: {exc}")

    def _halt_all_quietly(self, reason: str) -> None:
        """Stop every axis on an error path, without masking the original fault."""
        self._abort.set()
        for motor in self.motors:
            motor.stop_quietly(reason)

    # -------------------------------------------------------------- stopping

    def stop(self) -> List[str]:
        """Controlled stop of all three axes: decelerate and hold.

        The motors keep their drive current and keep holding position, which
        is what you want for a loaded vertical axis. This is the big red
        button's action.

        Returns the motors it could not stop, one string each, so a caller can
        report what actually happened. An empty list means all three stopped.
        """
        self._abort.set()
        problems = []
        for motor in self.motors:
            try:
                motor.stop()
            except (ModbusError, MotorFault) as exc:
                problems.append(f"{motor.name}: {exc}")
        if problems:
            self._log("STOP had trouble on: " + "; ".join(problems))
        else:
            self._log("STOP: all three actuators holding position.")
        return problems

    def emergency_stop(self, force_drives_off: bool = False) -> "EmergencyResult":
        """The EMERGENCY control: stop, engage the brakes, and only then --
        if the brakes are confirmed holding -- cut drive power.

        This used to write MODE_REG = 0 straight away, and on this telescope
        that was the worst thing it could do. The focal plane hangs on three
        screws; the brakes are on a separate device this software cannot
        command; so removing drive power left the camera held by nothing but
        screw friction, and it sank -- back-driving the screws, with the
        encoder count running down, for as long as there was travel left.
        The drives were the only thing holding it, and EMERGENCY turned them
        off.

        So the order is now: hold first, give up holding only if something
        else has taken over.

        1. Stop every axis and keep it powered and holding. Always. This is
           the part that is unconditionally safe and it happens first.
        2. Engage the brakes, if there is anything here that can engage them.
        3. Read the brakes back. Only if every one of them is *confirmed*
           engaged are the drives passivated.

        If the brakes cannot be confirmed, the drives stay on and holding and
        the result says so in as many words. That is not a failure: a powered
        drive holding position is a safe state, and it is a far better one
        than an unpowered drive over a falling camera.

        `force_drives_off` overrides step 3 for the case where somebody has
        engaged the brakes by hand and genuinely wants the drives off. It is
        never set by the EMERGENCY button.
        """
        self._abort.set()
        stamp_positions: Dict[str, float] = {}

        # --- 1. stop, under power -----------------------------------------
        stop_problems = self.stop()

        # --- 2. brakes -----------------------------------------------------
        brakes_engaged, brake_message = self._engage_brakes_for_emergency()

        # --- 3. drives off only if something else is holding ---------------
        drives_off = False
        drive_problems: List[str] = []
        why_still_on = ""
        took_hold: List[str] = []
        if brakes_engaged or force_drives_off:
            for motor in self.motors:
                try:
                    # Already stopped above; passivate must not re-stop and
                    # must not wait, because this is the emergency path.
                    motor.passivate(engage_brake_first=True, stop_first=False)
                except (ModbusError, MotorFault) as exc:
                    drive_problems.append(f"{motor.name}: {exc}")
            drives_off = not drive_problems
        else:
            took_hold = self._take_hold()
            why_still_on = (
                "The drives are still ON and holding position, deliberately. "
                f"{brake_message} With the drives off and nothing holding the "
                "brakes, the focal plane would be supported only by screw "
                "friction and could sink. Engage the brakes from the brake "
                "page, then use `cli passivate --force` if you need the drives "
                "electrically off."
            )

        for motor in self.motors:
            try:
                stamp_positions[motor.name] = motor.get_position_mm()
            except (ModbusError, MotorFault):
                pass

        result = EmergencyResult(
            stop_problems=stop_problems,
            brakes_engaged=brakes_engaged,
            brake_message=brake_message,
            drives_off=drives_off,
            drive_problems=drive_problems,
            why_drives_are_still_on=why_still_on,
            positions_mm=stamp_positions,
            took_hold=took_hold,
        )
        self._log(result.summary())
        return result

    def _take_hold(self) -> List[str]:
        """Enable any drive that is passive, so it holds where it is.

        If EMERGENCY is pressed while a drive is already passive and the
        brakes are not confirmed, nothing at all is holding that axis -- the
        camera is on screw friction and may already be sinking. Enabling the
        drive is what stops that, so the emergency control does it rather than
        reporting an unsafe state it could have fixed.

        The target is written *before* the mode change, so the motor holds the
        position it is at instead of jumping to whatever stale P_SOLL it was
        left with. Returns the axes it took hold of.

        The target is the ENCODER position, not the projected one. On an
        enabled drive `stop()` freezes the profile output, because writing the
        encoder reading there would command a step of the standing following
        error. On a passive drive the profile output is stale: it says where
        the generator was when the drive went off, and if the load has sunk
        since -- which is exactly the situation this exists for -- enabling
        the drive at that target would haul the camera back up to it. The
        encoder is where the shaft is now, and that is what to hold.
        """
        from .registers import MotorMode

        taken = []
        for motor in self.motors:
            try:
                if motor.get_mode() == int(MotorMode.POSITION):
                    continue
                motor.command_position_counts(motor.get_position_counts())
                motor.ensure_position_mode()
                taken.append(motor.name)
            except (ModbusError, MotorFault) as exc:
                self._log(
                    f"{motor.name}: could not take hold of a passive axis: {exc}. "
                    "If the brakes are also off, this actuator is held by "
                    "nothing -- check the focal plane physically."
                )
        if taken:
            self._log(
                "EMERGENCY: " + ", ".join(taken) + " were passive with no "
                "confirmed brake, so the drives were enabled to hold them "
                "where they are."
            )
        return taken

    #: An older name for the same safe sequence, kept so that any caller
    #: written against it gets the interlocked behaviour rather than a missing
    #: attribute. There is deliberately no way to reach the old "cut power
    #: immediately" behaviour through an emergency control.
    emergency_passivate = emergency_stop

    def _engage_brakes_for_emergency(self):
        """Try to engage the brakes. Returns (confirmed_engaged, message)."""
        from .external_brake import BrakeError

        if self.external_brake.available:
            try:
                self.external_brake.engage()
            except BrakeError as exc:
                return False, f"The brakes could not be engaged ({exc})."
            try:
                state = self.external_brake.read_state(fresh=True)
            except BrakeError as exc:
                return False, f"The brakes were commanded on but cannot be read back ({exc})."
            if state is not BrakeState.ENGAGED:
                return False, f"The brakes were commanded on but read back {state.value}."
            # Engaged -- but engaged according to what? A relay reading off,
            # or an HTTP device that only echoes the last command, says the
            # brake was TOLD to clamp. Turning the drives off on that basis
            # would bet the camera on the wiring being right. Only a reading
            # of the brake itself counts as confirmation.
            if not self.external_brake.state_is_measured("all"):
                return False, (
                    "The brakes were commanded on and the controller reports "
                    "its output off, but nothing measures the brakes "
                    "themselves, so they are not confirmed holding.")
            return True, "The brakes are engaged and read back engaged."

        controllable = [m for m in self.motors if m.brake_is_software_controlled]
        if not controllable:
            return False, (
                "There are no brakes under software control on this "
                "installation -- they are switched by a separate device."
            )

        problems = []
        for motor in controllable:
            try:
                motor.engage_brake()
            except (ModbusError, MotorFault) as exc:
                problems.append(f"{motor.name}: {exc}")
        if problems:
            return False, "Some brakes did not engage: " + "; ".join(problems) + "."
        if len(controllable) != len(self.motors):
            missing = [m.name for m in self.motors if not m.brake_is_software_controlled]
            return False, (
                f"Only some actuators have a software-controlled brake; "
                f"{', '.join(missing)} do not."
            )
        return True, "The brakes are engaged and read back engaged."

    def rest_on_brakes(self, trust_relay: bool = False) -> str:
        """Brakes on, then drives off, then make sure the brakes are holding.

        For a parked focal plane: with the drives off the motors make no small
        corrections, so the only thing holding the camera is the brakes, which
        is what people want to rely on once it is in position.

        1. The brakes are engaged and read back. If the reading is a sensor
           on the brake, that is confirmation. If it is only the relay (the
           PLC was told to clamp), it is accepted only with `trust_relay`,
           which the GUI asks a person for.
        2. The drives are turned off.
        3. The encoders are watched for `rest_watch_s`. Any motor that moves
           more than `rest_sink_limit_mm` means the brakes are not holding:
           the drives are turned straight back on, holding where they are,
           and this raises.

        Anything short of that leaves the drives on and holding. Returns a
        sentence saying what was done.
        """
        from .external_brake import BrakeError
        cfg = self.cfg
        with self._move_lock:
            self._require_connected()
            engaged, message = self._engage_brakes_for_emergency()
            if not engaged:
                relay_only = False
                if self.external_brake.available:
                    try:
                        relay_only = (
                            self.external_brake.read_state(fresh=True) is BrakeState.ENGAGED
                            and not self.external_brake.state_is_measured("all"))
                    except BrakeError:
                        relay_only = False
                if not (relay_only and trust_relay):
                    raise PlatformError(
                        f"The drives were left on and holding. {message} Turning "
                        "them off with nothing confirmed holding would leave the "
                        "focal plane on screw friction alone.")
                message = ("The brakes read back engaged from the relay (nothing "
                           "senses the brake itself); accepted on the operator's "
                           "say-so, and checked below by watching the encoders.")
            time.sleep(cfg.actuators[0].brake.settle_s)

            before = {m.name: m.get_position_counts() for m in self.motors}
            problems = []
            for motor in self.motors:
                try:
                    motor.passivate(engage_brake_first=True, stop_first=False)
                except (ModbusError, MotorFault) as exc:
                    problems.append(f"{motor.name}: {exc}")
            if problems:
                taken = self._take_hold()
                raise PlatformError(
                    "Not every drive turned off (" + "; ".join(problems) + "). "
                    + (f"{', '.join(taken)} turned back on and holding. " if taken else "")
                    + "The brakes are on.")

            deadline = time.monotonic() + cfg.rest_watch_s
            while True:
                for motor in self.motors:
                    try:
                        moved = abs(motor.get_position_counts() - before[motor.name])
                    except (ModbusError, MotorFault):
                        continue
                    moved_mm = moved / motor.cfg.resolved_counts_per_mm
                    if moved_mm > cfg.rest_sink_limit_mm:
                        taken = self._take_hold()
                        self._log(f"{motor.name} moved {moved_mm:.4f} mm with the "
                                  "drives off: the brakes are not holding. Drives "
                                  "back on.")
                        raise PlatformError(
                            f"The brakes are not holding: {motor.name} moved "
                            f"{moved_mm:.4f} mm within {cfg.rest_watch_s:g} s of "
                            "the drives going off. The drives were turned straight "
                            f"back on ({', '.join(taken) or 'none needed'}) and are "
                            "holding where they are. Check the brakes before "
                            "trying again.")
                if time.monotonic() >= deadline:
                    break
                time.sleep(0.1)

            done = (f"Resting on the brakes: drives off, nothing moved more than "
                    f"{cfg.rest_sink_limit_mm:g} mm in {cfg.rest_watch_s:g} s.")
            self._log(done)
            return message + " " + done

    def _rest_after_move(self) -> None:
        """If asked for, rest on the brakes once a move has finished.

        A failure here does not make the move a failure: the plane is where it
        was sent. It is said loudly, and the drives are left holding.
        """
        if not self.rest_after_moves:
            return
        try:
            self.rest_on_brakes(trust_relay=self.trust_relay_brakes)
        except PlatformError as exc:
            self._log(f"Move finished, but did NOT rest on the brakes: {exc}")

    def passivate_all(self, force: bool = False) -> List[str]:
        """Deliberately remove drive power from all three motors.

        For maintenance, not for an emergency -- `emergency_stop` is the
        button. Refuses unless the brakes are confirmed engaged, because with
        the drives off and the brakes off the focal plane is held by nothing.
        `force=True` overrides that, for somebody who has checked the brakes
        by hand.
        """
        if not force:
            engaged, message = self._engage_brakes_for_emergency()
            if not engaged:
                raise PlatformError(
                    f"Refusing to turn the drives off. {message} With the "
                    "drives off and the brakes not holding, the focal plane "
                    "rests on screw friction alone and can sink. Engage the "
                    "brakes first, or pass --force if you have checked them "
                    "by hand."
                )
        self._abort.set()
        problems = []
        for motor in self.motors:
            try:
                motor.passivate(engage_brake_first=True)
            except (ModbusError, MotorFault) as exc:
                problems.append(f"{motor.name}: {exc}")
        return problems

    # ---------------------------------------------------------------- brakes

    def set_all_brakes(self, engaged: bool) -> Dict[str, str]:
        """Engage or release the brakes.

        Prefers the external controller when one is configured, because on the
        pSCT that is where the brakes actually are. Falls back to per-motor
        outputs for an installation that wires them to the drives instead.
        """
        from .external_brake import BrakeError
        if self.external_brake.available:
            try:
                if engaged:
                    self.external_brake.engage()
                else:
                    self.external_brake.release(
                        drives_holding=self.drives_holding)
                return {"all": "ok"}
            except BrakeError as exc:
                return {"all": str(exc)}

        results: Dict[str, str] = {}
        for motor in self.motors:
            try:
                if engaged:
                    motor.engage_brake()
                else:
                    motor.release_brake()
                results[motor.name] = "ok"
            except (ModbusError, MotorFault) as exc:
                results[motor.name] = str(exc)
        return results

    def set_brake(self, name: str, engaged: bool) -> Dict[str, str]:
        """One actuator's brake, from its row in the GUI.

        When one switch holds every brake -- one relay, or `all_or_nothing` --
        there is no such thing as one brake: the request becomes all three,
        and the result says so rather than pretending only one changed.
        Otherwise the per-actuator relay or output is switched.
        """
        from .external_brake import BrakeError
        controller = self.external_brake
        if controller.available:
            target = self._brake_target(name)
            try:
                if engaged:
                    controller.engage(target)
                else:
                    # With separate brakes only this actuator's drive has to
                    # be holding: the other two are still clamped.
                    controller.release(target, drives_holding=self.holding(target))
            except BrakeError as exc:
                return {target: str(exc)}
            return {target: "ok" if target != "all" else
                    "ok (one switch holds all three brakes, so all three changed)"}

        motor = self.motor(name)
        try:
            if engaged:
                motor.engage_brake()
            else:
                motor.release_brake()
            return {motor.name: "ok"}
        except (ModbusError, MotorFault) as exc:
            return {motor.name: str(exc)}

    def enable_drives(self) -> List[str]:
        """Enable every passive drive, holding exactly where its shaft is.

        The step before releasing a brake: the interlock refuses to take the
        brakes off unless the drives are holding. Returns the axes it enabled.
        `JVLMotor.ensure_position_mode` writes the encoder position as the
        target before enabling, so this cannot move anything.
        """
        from .registers import MotorMode
        enabled = []
        for motor in self.motors:
            if not motor.connected:
                continue
            if motor.get_mode() == int(MotorMode.POSITION):
                continue
            motor.ensure_position_mode()
            enabled.append(motor.name)
        if enabled:
            self._log("Drives enabled and holding position: " + ", ".join(enabled))
        return enabled

    def brake_states(self) -> Dict[str, BrakeState]:
        from .external_brake import BrakeError
        if self.external_brake.available:
            try:
                state = self.external_brake.read_state(fresh=True)
            except BrakeError:
                state = BrakeState.UNKNOWN
            return {m.name: state for m in self.motors}

        out: Dict[str, BrakeState] = {}
        for motor in self.motors:
            try:
                out[motor.name] = motor.get_brake_status().state
            except (ModbusError, MotorFault):
                out[motor.name] = BrakeState.UNKNOWN
        return out

    # ---------------------------------------------------------------- errors

    def clear_all_errors(self) -> Dict[str, int]:
        out: Dict[str, int] = {}
        for motor in self.motors:
            try:
                out[motor.name] = motor.clear_errors()
            except (ModbusError, MotorFault) as exc:
                self._log(f"{motor.name}: clear errors failed: {exc}")
                out[motor.name] = -1
        return out

    # --------------------------------------------------------------- zeroing

    def set_zero_here(self, persist: bool = True) -> Dict[str, int]:
        """Define the current mechanical position as the reference.

        Do this once, with the focal plane at a known-good orientation
        (typically the one established by a survey), and every subsequent
        command is relative to that reference. Persisting is on by default
        because a zero that is lost when the program closes is worse than no
        zero at all.
        """
        out: Dict[str, int] = {}
        for motor in self.motors:
            out[motor.name] = motor.set_zero_here()
        if persist:
            path = save_config(self.cfg, self.config_path)
            self._log(f"Zero saved to {path}")
        return out

    def save(self) -> str:
        return save_config(self.cfg, self.config_path)


__all__ = ["FocalPlanePlatform", "PlatformState", "PlatformError", "MoveRecord"]
