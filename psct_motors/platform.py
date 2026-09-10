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
from .jvl_motor import BrakeState, JVLMotor, MotorFault, MotorStatus
from .kinematics import Orientation, ThreePointPlatform, platform_from_config
from .transport import ModbusError


class PlatformError(RuntimeError):
    """A move was refused, or the platform is not in a state to move."""


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

    @property
    def all_connected(self) -> bool:
        return bool(self.motors) and all(m.connected and not m.comms_error for m in self.motors)

    @property
    def any_error(self) -> bool:
        return any(m.error_bits for m in self.motors)

    @property
    def moving(self) -> bool:
        return any(not m.in_position for m in self.motors if not m.comms_error)

    def as_dict(self) -> dict:
        return {
            "motors": [m.as_dict() for m in self.motors],
            "orientation": self.orientation.as_dict() if self.orientation else None,
            "orientation_valid": self.orientation_valid,
            "all_connected": self.all_connected,
            "any_error": self.any_error,
            "moving": self.moving,
            "message": self.message,
        }


class FocalPlanePlatform:
    """Coordinated control of the three focal-plane actuators."""

    def __init__(self, cfg: Optional[PlatformConfig] = None,
                 simulate: bool = False,
                 logger: Optional[Callable[[str], None]] = None,
                 config_path: Optional[str] = None):
        self.cfg = cfg or load_config(config_path)
        self.cfg.validate()
        self.config_path = config_path
        self.simulate = simulate
        self._log = logger or (lambda msg: None)
        self.geometry: ThreePointPlatform = platform_from_config(self.cfg)
        self.external_brake = self._build_external_brake()
        self._move_lock = threading.RLock()
        self._abort = threading.Event()

        if simulate:
            from .simulator import simulated_motor
            # Start the simulated actuators mid-travel so relative moves in
            # both directions are possible straight away.
            mid = (self.cfg.limits.min_focus_mm + self.cfg.limits.max_focus_mm) / 2.0
            self.motors: List[JVLMotor] = [
                simulated_motor(
                    a, start_mm=mid,
                    # Mechanical end stops a little beyond the soft limits, so
                    # `find-stop` has something to find in simulation and the
                    # soft limits are still what stops an ordinary move first.
                    # Without these, rehearsing the calibration would only ever
                    # show the "no stop found" path.
                    hard_stop_low=a.mm_to_counts(a.min_travel_mm - 2.0),
                    hard_stop_high=a.mm_to_counts(a.max_travel_mm + 2.0),
                    # The load the brakes exist to hold. With the brakes off
                    # and the drives passive, a simulated axis falls -- which
                    # is the failure the interlocks are there to prevent, and
                    # it cannot be rehearsed if the simulation ignores gravity.
                    gravity_counts_per_s=a.resolved_counts_per_mm * 2.0,
                    brake_held=self._make_brake_hook(a.name),
                )
                for a in self.cfg.actuators
            ]
        else:
            self.motors = [
                JVLMotor(a, timeout_s=self.cfg.modbus_timeout_s,
                         retries=self.cfg.modbus_retries, logger=self._log)
                for a in self.cfg.actuators
            ]

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

        In simulation, and only when no real device is configured, a fake one
        stands in. Otherwise the brake interlocks -- the checks that stop a
        released brake dropping the camera -- could not be rehearsed at all,
        since the real device's protocol is still unknown.
        """
        from .external_brake import BrakeController, ExternalBrakeConfig
        settings = self.cfg.external_brake
        if self.simulate and settings.mode == "none":
            from .external_brake import SimulatedBrakeController
            return SimulatedBrakeController(
                [a.name for a in self.cfg.actuators],
                all_or_nothing=settings.all_or_nothing,
                logger=self._log,
            )
        return BrakeController(
            ExternalBrakeConfig(
                mode=settings.mode, host=settings.host, port=settings.port,
                unit_id=settings.unit_id, timeout_s=settings.timeout_s,
                all_or_nothing=settings.all_or_nothing,
                coils=dict(settings.coils),
                energized_releases=settings.energized_releases,
                release_url=settings.release_url,
                engage_url=settings.engage_url,
                status_url=settings.status_url,
                status_field=settings.status_field,
            ),
            logger=self._log,
        )

    @property
    def drives_holding(self) -> bool:
        """True when every motor is enabled and actively holding position.

        The precondition for releasing a brake. Checked against the motors
        rather than assumed, because the brake controller cannot see them.
        """
        from .registers import MotorMode
        try:
            return all(m.get_mode() == int(MotorMode.POSITION) for m in self.motors)
        except (ModbusError, MotorFault):
            return False

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

    def read_state(self) -> PlatformState:
        """Poll everything. Never raises -- suitable for a UI timer."""
        statuses = [self._with_external_brake(m.read_status()) for m in self.motors]
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
                             orientation_valid=valid, message=message)

    def _with_external_brake(self, status: MotorStatus) -> MotorStatus:
        """Show the brake that actually holds this axis.

        `MotorStatus.brake` describes the motor's own brake output, which on
        the pSCT is unassigned -- the brakes are on a separate device. Where
        that device is reachable, its state is the true one, and displaying the
        motor's instead would be showing an indicator that cannot change.
        """
        from dataclasses import replace
        from .external_brake import BrakeError
        from .jvl_motor import BrakeStatus

        controller = self.external_brake
        if not controller.available or status.comms_error:
            return status
        try:
            state = controller.read_state(status.name)
            measured = controller.state_is_measured(status.name)
            detail = getattr(controller, "describe", lambda: "external brake")()
        except BrakeError as exc:
            state, measured, detail = BrakeState.UNKNOWN, False, str(exc)
        return replace(status, brake=BrakeStatus(state, not measured, detail))

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

        targets = self.geometry.actuators_from_orientation(orientation)
        for motor, target in zip(self.motors, targets):
            lo, hi = motor.cfg.min_travel_mm, motor.cfg.max_travel_mm
            if not (lo <= target <= hi):
                problems.append(
                    f"actuator {motor.name} would need {target:.4f} mm, outside its "
                    f"travel limits {lo:.3f}..{hi:.3f} mm"
                )

        if current is not None:
            d_focus = abs(orientation.focus_mm - current.focus_mm)
            if d_focus > limits.max_step_mm:
                problems.append(
                    f"this move changes focus by {d_focus:.3f} mm, more than the "
                    f"{limits.max_step_mm:.3f} mm single-step limit"
                )
            d_tip = abs(orientation.tip_deg - current.tip_deg)
            d_tilt = abs(orientation.tilt_deg - current.tilt_deg)
            worst = max(d_tip, d_tilt)
            if worst > limits.max_tilt_step_deg:
                problems.append(
                    f"this move changes an angle by {worst:.4f} deg, more than the "
                    f"{limits.max_tilt_step_deg:.3f} deg single-step limit"
                )

        if problems:
            raise PlatformError(
                "Move refused, nothing was commanded:\n  - " + "\n  - ".join(problems)
            )
        return targets

    def preview(self, orientation: Orientation) -> Dict[str, float]:
        """Actuator targets for an orientation, without checking or moving.

        Handy for showing an operator what a command would do before they
        commit to it.
        """
        targets = self.geometry.actuators_from_orientation(orientation)
        return {m.name: t for m, t in zip(self.motors, targets)}

    # ----------------------------------------------------------------- moves

    def move_to_orientation(self, orientation: Orientation, wait: bool = True,
                            check_step: bool = True) -> PlatformState:
        """Drive the focal plane to an absolute orientation."""
        with self._move_lock:
            self._require_connected()
            self._abort.clear()
            for m in self.motors:
                m.clear_cancel()

            current = self.read_orientation() if check_step else None
            targets = self.check_orientation(orientation, current=current)

            self._log(
                f"Move to {orientation.describe()} -> "
                + ", ".join(f"{m.name} {t:.4f} mm" for m, t in zip(self.motors, targets))
            )

            self._prepare_for_motion()
            self._apply_synchronised_velocities(targets)

            for motor, target in zip(self.motors, targets):
                motor.command_position_mm(target)

            if wait:
                self._wait_for_all()
            return self.read_state()

    def move_relative(self, d_focus_mm: float = 0.0, d_tip_deg: float = 0.0,
                      d_tilt_deg: float = 0.0, wait: bool = True) -> PlatformState:
        """Nudge the focal plane relative to where it is now."""
        with self._move_lock:
            self._require_connected()
            current = self.read_orientation()
            target = current.offset_by(d_focus_mm, d_tip_deg, d_tilt_deg)
            return self.move_to_orientation(target, wait=wait)

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

            # Every drive is enabled, not just the one that moves. The brakes
            # are one switch for all three actuators, so taking them off with
            # two drives passive would leave most of the plate held by nothing
            # -- and the release interlock would refuse anyway.
            for other in self.motors:
                if other.connected:
                    other.ensure_position_mode()
            self._release_brake_if_controlled(motor)
            self._release_external_brakes()
            motor.set_velocity(motor.cfg.velocity_raw)
            motor.command_position_mm(target)
            self._log(f"{motor.name}: single-axis move to {target:.4f} mm.")
            if wait and not motor.wait_for_in_position():
                # wait_for_in_position has already halted this axis.
                raise PlatformError(
                    f"{motor.name} did not reach {target:.4f} mm within "
                    f"{motor.cfg.move_timeout_s:.0f} s, and has been halted where "
                    "it got to."
                )
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
        for motor in self.motors:
            self._release_brake_if_controlled(motor)
        self._release_external_brakes()

    def _check_drive_power(self) -> None:
        """Refuse to move if a drive's supply is below its own threshold.

        A JVL with no 60 V still answers Modbus from its control supply: the
        target is accepted, the mode reads back, and nothing turns. That looks
        exactly like the software being broken, so it is worth naming.
        """
        dead = []
        for motor in self.motors:
            try:
                bus = motor.read_register("BUS_VOLTAGE")
                acceptance = motor.read_register("ACCEPTANCE_VOLTAGE")
            except (ModbusError, MotorFault):
                continue          # a comms problem is reported elsewhere
            if acceptance > 0 and bus < acceptance:
                dead.append(f"{motor.name} (bus {bus}, needs {acceptance})")
        if dead:
            raise PlatformError(
                "Not moving: the drive supply is below the acceptance voltage "
                f"on {', '.join(dead)}. The motors are reachable -- they answer "
                "Modbus from their control supply -- but with the main supply "
                "off they will accept a target and not move. Check the 60 V "
                "supply and its breaker before commanding anything else."
            )

    def _release_external_brakes(self) -> None:
        """Take the site's brakes off, or say why the move cannot go ahead."""
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
            state = controller.read_state()
        except BrakeError as exc:
            raise PlatformError(
                f"Not moving: the brake controller could not be read ({exc}). "
                "Moving without knowing whether the brakes are off risks driving "
                "the motors against them."
            ) from exc

        if state is BrakeState.RELEASED:
            return

        if not self.drives_holding:
            raise PlatformError(
                "Not moving: the brakes are engaged and the drives are not "
                "holding position, so releasing them now would leave the focal "
                "plane held by nothing. Enable the drives first."
            )
        try:
            controller.release("all", drives_holding=True)
        except BrakeError as exc:
            raise PlatformError(
                f"Not moving: the brakes did not release ({exc}). Driving the "
                "motors against an engaged brake is how a lead screw or a "
                "coupling gets damaged."
            ) from exc

        try:
            after = controller.read_state()
        except BrakeError:
            return          # commanded, but unreadable; already logged
        if after is not BrakeState.RELEASED:
            raise PlatformError(
                "Not moving: the brakes were commanded to release but still "
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

    def _apply_synchronised_velocities(self, targets_mm: Sequence[float]) -> None:
        """Scale each axis' speed so all three finish at the same moment.

        Without this the shortest of the three moves finishes first, and until
        the last one lands the plate sits at an orientation nobody asked for,
        pivoting on its ball joints. Scaling by distance keeps the plate on a
        straight line between the two orientations.
        """
        if not self.cfg.synchronize_moves:
            for motor in self.motors:
                motor.set_velocity(motor.cfg.velocity_raw)
            return

        deltas = [
            abs(target - motor.get_position_mm())
            for motor, target in zip(self.motors, targets_mm)
        ]
        longest = max(deltas)
        if longest <= 0:
            return
        for motor, delta in zip(self.motors, deltas):
            scaled = motor.cfg.velocity_raw * (delta / longest)
            motor.set_velocity(max(self.cfg.min_velocity_raw, int(round(scaled))))

    def _wait_for_all(self) -> None:
        """Wait for every axis, then report all stragglers together."""
        deadline = time.monotonic() + max(m.cfg.move_timeout_s for m in self.motors)
        pending = list(self.motors)
        while pending and time.monotonic() < deadline:
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

        if not motor.is_in_position():
            still_pending.append(motor)

    # ----------------------------------------------------- hard-stop seeking

    def seek_hard_stop_together(
        self,
        direction: int,
        step_mm: float = 0.2,
        budget_mm: float = 30.0,
        max_spread_mm: Optional[float] = None,
        settle_s: float = 0.4,
        step_timeout_s: float = 5.0,
        level_after: bool = True,
        progress: Optional[Callable[["HardStopProgress"], None]] = None,
    ) -> "HardStopResult":
        """Run all three actuators out together until the travel ends.

        This is the site's calibration procedure -- drive to the end and let it
        stop -- done to all three axes at once, which is the only safe way to
        do it. Sending one actuator to its end stop on its own tilts the focal
        plane about the other two ball joints, and the site's own experience is
        that this can break something.

        So the three are walked out in lockstep, in millimetres rather than
        counts, so they cover the same distance even if their calibrations
        differ. After every step three things are checked:

        * torque on each axis, which climbs when something starts to resist;
        * whether each shaft actually moved, because at a stop it will not;
        * how far apart the three have drifted, because divergence *is* tilt.

        The first axis to reach its stop ends the search for all three: every
        motor is immediately commanded to hold where it is, so no axis keeps
        pushing and no axis keeps travelling past the others. The two that did
        not stop are then backed off to match the one that did (`level_after`),
        because the step in which the first axis stopped left them up to one
        step ahead of it -- which is a tilt, and the whole point is to not
        leave one in the plate.

        `direction` is +1 or -1: + drives the camera towards M1, - towards M2.
        Returns a HardStopResult describing where the plate ended up. Raises
        PlatformError if the axes diverge past `max_spread_mm`, or if the
        budget is exhausted without finding a stop.
        """
        if direction not in (1, -1):
            raise ValueError(f"direction must be +1 or -1, got {direction}")
        if step_mm <= 0:
            raise ValueError("step_mm must be positive")
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

        travelled_mm = 0.0
        worst_spread = 0.0
        stopped_by: List[str] = []
        reasons: Dict[str, str] = {}

        try:
            # Deliberately slow. Meeting a mechanical stop at speed is how a
            # lead screw gets damaged, and a slow approach makes the torque
            # rise easy to tell apart from an acceleration transient.
            for motor in self.motors:
                motor.set_velocity(max(1, original_velocity[motor.name] // 4))

            while travelled_mm < budget_mm:
                if self._abort.is_set():
                    self._settle_all_where_they_are()
                    raise PlatformError(
                        "Hard-stop search stopped by the operator. All three "
                        "actuators are holding where they were halted."
                    )

                before = {m.name: m.get_position_mm() for m in self.motors}
                for motor in self.motors:
                    target = before[motor.name] + direction * step_mm
                    motor.command_position_counts(motor.cfg.mm_to_counts(target))

                stopped_by, reasons = self._wait_out_hard_stop_step(
                    settle_s + step_timeout_s)

                after = {m.name: m.get_position_mm() for m in self.motors}
                moved = {name: abs(after[name] - before[name]) for name in after}

                # An axis that was told to move and barely did has reached its
                # stop, whether or not the torque reading noticed.
                for motor in self.motors:
                    if (motor.name not in reasons
                            and moved[motor.name] < step_mm * 0.25):
                        stopped_by.append(motor.name)
                        reasons[motor.name] = (
                            f"commanded {step_mm:.3f} mm, moved "
                            f"{moved[motor.name]:.4f} mm")

                travelled = {name: after[name] - start_mm[name] for name in after}
                spread = max(travelled.values()) - min(travelled.values())
                worst_spread = max(worst_spread, spread)

                if progress is not None:
                    progress(HardStopProgress(
                        positions_mm=dict(after),
                        travelled_mm=max(abs(v) for v in travelled.values()),
                        spread_mm=spread,
                        torque_percent={m.name: (m.peak_torque_percent or 0.0)
                                        for m in self.motors},
                    ))

                if stopped_by:
                    self._settle_all_where_they_are()
                    break

                if spread > limit_spread:
                    self._settle_all_where_they_are()
                    lagging = min(travelled, key=travelled.get)
                    leading = max(travelled, key=travelled.get)
                    raise PlatformError(
                        f"Hard-stop search abandoned: the actuators drifted "
                        f"{spread:.4f} mm apart (limit {limit_spread:.4f} mm), "
                        f"with {leading} ahead of {lagging}. That difference is "
                        f"tilt in the focal plane, which is what running all "
                        f"three together is meant to avoid. All three have been "
                        f"halted where they were. Check for a binding axis or a "
                        f"wrong counts_per_mm before trying again."
                    )

                travelled_mm = max(abs(v) for v in travelled.values())
            else:
                self._settle_all_where_they_are()
                raise PlatformError(
                    f"Travelled {travelled_mm:.2f} mm without any actuator "
                    f"finding a stop, and gave up at the {budget_mm:.1f} mm "
                    "budget. Either the travel is longer than expected, or the "
                    "stall torque threshold is too high for the end stop to "
                    "register. All three actuators are holding where they are."
                )
        finally:
            for motor in self.motors:
                try:
                    motor.set_velocity(original_velocity[motor.name])
                except (ModbusError, MotorFault):
                    pass

        levelled = self._level_after_stop(start_mm, direction) if level_after else False

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
        )
        self._log(result.summary())
        return result

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

    def _wait_out_hard_stop_step(self, timeout_s: float):
        """Wait for one step of the coordinated search to finish or stop.

        Returns (names that stopped, why). Deliberately not `_wait_for_all`:
        there, an axis that does not reach its target is a failure, while here
        it is the thing being looked for.
        """
        stopped: List[str] = []
        reasons: Dict[str, str] = {}
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._abort.is_set():
                return stopped, reasons
            pending = False
            for motor in self.motors:
                if motor.name in reasons:
                    continue
                torque = motor.check_stall()
                if torque is not None:
                    stopped.append(motor.name)
                    reasons[motor.name] = f"torque reached {torque:.0f}%"
                    continue
                errors = motor.get_errors()
                if errors:
                    stopped.append(motor.name)
                    reasons[motor.name] = f"drive faulted: {motor.error_text()}"
                    continue
                if not motor.is_in_position():
                    pending = True
            if stopped:
                # One axis has finished travelling. Every other axis must stop
                # now, in this poll, or the plane tilts by however far they get
                # before the loop next comes round.
                return stopped, reasons
            if not pending:
                return stopped, reasons
            time.sleep(0.05)
        return stopped, reasons

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

    def emergency_passivate(self) -> List[str]:
        """Brakes on, drive off. The last resort.

        Note what this gives up: with the drive passive the motor is not
        holding anything. If the brakes are not actually wired and working,
        the load is then held only by screw friction. Prefer `stop()`.

        Returns the motors it could not passivate, one string each, so a
        caller can report what actually happened rather than assuming it
        worked. An empty list means every motor was passivated.
        """
        self._abort.set()
        problems = []
        for motor in self.motors:
            try:
                motor.passivate(engage_brake_first=True)
            except (ModbusError, MotorFault) as exc:
                problems.append(f"{motor.name}: {exc}")
        if problems:
            self._log("EMERGENCY PASSIVATE had trouble on: " + "; ".join(problems))
        else:
            self._log("EMERGENCY PASSIVATE: brakes engaged, drives off.")
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

    def brake_states(self) -> Dict[str, BrakeState]:
        from .external_brake import BrakeError
        if self.external_brake.available:
            try:
                state = self.external_brake.read_state()
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


__all__ = ["FocalPlanePlatform", "PlatformState", "PlatformError"]
