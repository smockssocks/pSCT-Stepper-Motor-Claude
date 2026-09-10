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
                )
                for a in self.cfg.actuators
            ]
        else:
            self.motors = [
                JVLMotor(a, timeout_s=self.cfg.modbus_timeout_s,
                         retries=self.cfg.modbus_retries, logger=self._log)
                for a in self.cfg.actuators
            ]

    def _build_external_brake(self):
        """The device that switches the focal-plane brakes, if there is one.

        On the pSCT the brakes are not on the motors, so brake control has to
        go somewhere else. Built unconditionally: when it is unconfigured it
        still answers "not available, and here is why", which is more use than
        an attribute that does not exist.
        """
        from .external_brake import BrakeController, ExternalBrakeConfig
        settings = self.cfg.external_brake
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
        statuses = [m.read_status() for m in self.motors]
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

            motor.ensure_position_mode()
            self._release_brake_if_controlled(motor)
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
        """Position mode and brakes released on all three, or nothing moves."""
        for motor in self.motors:
            errors = motor.get_errors()
            if errors:
                raise PlatformError(
                    f"{motor.name} has an active error ({motor.error_text()}). "
                    "Clear it before moving."
                )
        for motor in self.motors:
            motor.ensure_position_mode()
        for motor in self.motors:
            self._release_brake_if_controlled(motor)

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
                errors = motor.get_errors()
                if errors:
                    # One axis faulting does not stop the other two, and two
                    # actuators continuing to a target the third will never
                    # reach is precisely how the plate gets racked about its
                    # ball joints. Halt everything, then report.
                    text = motor.error_text()
                    self._halt_all_quietly(f"{motor.name} faulted mid-move")
                    raise PlatformError(
                        f"{motor.name} faulted during the move: {text}. All three "
                        "actuators have been halted where they were, so the focal "
                        "plane is at neither the old orientation nor the requested "
                        "one -- read the current orientation before continuing."
                    )
                if not motor.is_in_position():
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

    def _halt_all_quietly(self, reason: str) -> None:
        """Stop every axis on an error path, without masking the original fault."""
        self._abort.set()
        for motor in self.motors:
            motor.stop_quietly(reason)

    # -------------------------------------------------------------- stopping

    def stop(self) -> None:
        """Controlled stop of all three axes: decelerate and hold.

        The motors keep their drive current and keep holding position, which
        is what you want for a loaded vertical axis. This is the big red
        button's action.
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

    def emergency_passivate(self) -> None:
        """Brakes on, drive off. The last resort.

        Note what this gives up: with the drive passive the motor is not
        holding anything. If the brakes are not actually wired and working,
        the load is then held only by screw friction. Prefer `stop()`.
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
