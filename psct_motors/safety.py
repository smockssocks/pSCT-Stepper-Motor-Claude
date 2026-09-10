"""
Safety drills: provoke each dangerous situation and check it is refused.

Every check in this software exists because of a specific way the focal plane
can be damaged or dropped. A check nobody has seen fire is a check nobody
should trust, so this module sets up each situation in simulation and reports
whether the software actually refused, and with what words.

    python -m psct_motors.cli safety-check

Nothing here touches hardware: each drill builds its own simulated platform.
That is deliberate. Several of these drills work by turning off the 60 V
supply or dropping the load, and the point is to see the refusal, not to find
out what the telescope does when the refusal is missing.

What is covered
---------------
brakes          moving with the brakes on, a brake supply that is off, and
                releasing a brake with nothing holding the load
power           moving with no drive supply, which a motor accepts silently
falling         what actually happens when the brakes come off an unpowered
                axis -- the reason the brake interlock exists
limits          focus, tilt and step-size limits
mechanics       running into a hard stop, and one motor lost mid-move
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from .config import PlatformConfig, default_config
from .external_brake import BrakeError
from .jvl_motor import MotorFault
from .kinematics import Orientation
from .platform import FocalPlanePlatform, PlatformError

#: Everything a guard can raise. A refusal is a refusal whichever layer says
#: it, and a drill that only caught one class would report an error instead of
#: a verdict the first time a different layer got there first.
REFUSALS = (PlatformError, BrakeError, MotorFault)


@dataclass
class DrillResult:
    name: str
    passed: bool
    what_was_done: str
    what_happened: str
    expected: str = ""

    @property
    def verdict(self) -> str:
        return "PASS" if self.passed else "FAIL"


@dataclass
class DrillReport:
    results: List[DrillResult] = field(default_factory=list)

    @property
    def failures(self) -> List[DrillResult]:
        return [r for r in self.results if not r.passed]

    @property
    def passed(self) -> bool:
        return not self.failures

    def as_dict(self) -> dict:
        return {
            "passed": self.passed,
            "drills": [
                {"name": r.name, "verdict": r.verdict,
                 "did": r.what_was_done, "result": r.what_happened,
                 "expected": r.expected}
                for r in self.results
            ],
        }


def bench_config() -> PlatformConfig:
    """A platform scaled so drills run in seconds rather than minutes."""
    cfg = default_config()
    for actuator in cfg.actuators:
        actuator.counts_per_mm = 1000.0
        actuator.velocity_raw = 4000
        actuator.min_travel_mm = -40.0
        actuator.max_travel_mm = 40.0
        actuator.in_position_tol_mm = 0.01
        actuator.move_timeout_s = 15.0
    cfg.limits.min_focus_mm = -24.0
    cfg.limits.max_focus_mm = 24.0
    cfg.poll_interval_s = 0.1
    cfg.validate()
    return cfg


def _platform(cfg: Optional[PlatformConfig] = None) -> FocalPlanePlatform:
    platform = FocalPlanePlatform(cfg=cfg or bench_config(), simulate=True)
    platform.connect()
    return platform


def _expect_refusal(name: str, what: str, expected: str,
                    action: Callable[[], None],
                    must_mention: List[str]) -> DrillResult:
    """Run `action`, which must raise, and check the message is useful.

    A refusal that does not say why is barely better than a hang, so the
    wording is part of what is being tested: each drill names the phrases an
    operator needs to see to know what to do next.
    """
    try:
        action()
    except REFUSALS as exc:
        message = str(exc)
        missing = [phrase for phrase in must_mention
                   if phrase.lower() not in message.lower()]
        if missing:
            return DrillResult(
                name, False, what,
                f"refused, but the message never mentions {missing}: {message}",
                expected)
        return DrillResult(name, True, what, f"refused: {message}", expected)
    return DrillResult(name, False, what,
                       "IT WENT AHEAD. Nothing refused it.", expected)


# --------------------------------------------------------------------------
# Brakes
# --------------------------------------------------------------------------

def drill_move_releases_the_brakes() -> DrillResult:
    platform = _platform()
    try:
        before = platform.brake_states()
        platform.move_to_orientation(Orientation(1.0, 0.0, 0.0))
        after = platform.brake_states()
        engaged_before = all(s.value == "engaged" for s in before.values())
        released_after = all(s.value == "released" for s in after.values())
        moved = abs(platform.read_orientation().focus_mm - 1.0) < 0.05
        passed = engaged_before and released_after and moved
        return DrillResult(
            "brakes released before a move", passed,
            "commanded a 1 mm focus move with the brakes engaged",
            f"brakes were {'engaged' if engaged_before else 'NOT engaged'} "
            f"beforehand, {'released' if released_after else 'NOT released'} "
            f"afterwards, focus reached "
            f"{platform.read_orientation().focus_mm:+.4f} mm",
            "the software releases the brakes itself, then moves")
    finally:
        platform.disconnect()


def drill_move_refused_when_brakes_will_not_release() -> DrillResult:
    platform = _platform()
    try:
        platform.external_brake.engage()
        platform.external_brake.set_powered(False)
        return _expect_refusal(
            "move refused when the brakes cannot release",
            "turned off the brake supply, leaving the brakes clamped, then "
            "commanded a move",
            "refuse, rather than drive the motors against a clamped brake",
            lambda: platform.move_to_orientation(Orientation(2.0, 0.0, 0.0)),
            ["brake", "did not release"])
    finally:
        platform.disconnect()


def drill_release_refused_when_drives_are_passive() -> DrillResult:
    platform = _platform()
    try:
        results = platform.set_all_brakes(engaged=False)
        message = " ".join(results.values())
        refused = "refusing" in message.lower()
        mentions_why = "nothing is holding" in message.lower()
        return DrillResult(
            "brake release refused with the drives passive",
            refused and mentions_why,
            "asked to release the brakes while every drive was passive",
            message,
            "refuse: with the brakes off and the drives passive, nothing holds "
            "the focal plane")
    finally:
        platform.disconnect()


def drill_the_focal_plane_falls_without_a_brake() -> DrillResult:
    """The reason the interlock above exists, shown rather than asserted."""
    platform = _platform()
    try:
        # Force the situation the interlock prevents: brakes off, drives off.
        platform.external_brake._set("all", engaged=False)
        start = platform.read_actuator_positions_mm()
        time.sleep(0.6)
        end = platform.read_actuator_positions_mm()
        dropped = max(abs(a - b) for a, b in zip(start, end))
        return DrillResult(
            "an unbraked, unpowered axis falls", dropped > 0.05,
            "released the brakes with the drives passive, then watched for "
            "0.6 s without commanding anything",
            f"the actuators moved {dropped:.3f} mm on their own",
            "the load moves, which is what the interlock above prevents")
    finally:
        platform.disconnect()


# --------------------------------------------------------------------------
# Power
# --------------------------------------------------------------------------

def drill_move_refused_without_drive_power() -> DrillResult:
    platform = _platform()
    try:
        for motor in platform.motors:
            motor._transport.set_powered(False)
        return _expect_refusal(
            "move refused with no drive supply",
            "turned off the 60 V supply, leaving the motors answering Modbus "
            "but unable to move, then commanded a move",
            "refuse and name the supply, rather than time out looking like a "
            "software fault",
            lambda: platform.move_to_orientation(Orientation(2.0, 0.0, 0.0)),
            ["acceptance voltage", "60 V"])
    finally:
        platform.disconnect()


# --------------------------------------------------------------------------
# Limits
# --------------------------------------------------------------------------

def drill_focus_limit() -> DrillResult:
    platform = _platform()
    try:
        return _expect_refusal(
            "focus beyond the limit is refused",
            "commanded focus far outside the configured range",
            "refuse before writing anything to a motor",
            lambda: platform.move_to_orientation(Orientation(900.0, 0.0, 0.0)),
            ["focus"])
    finally:
        platform.disconnect()


def drill_tilt_limit() -> DrillResult:
    platform = _platform()
    try:
        return _expect_refusal(
            "tilt beyond the limit is refused",
            "commanded a tilt far beyond the configured maximum",
            "refuse before writing anything to a motor",
            lambda: platform.move_to_orientation(Orientation(0.0, 45.0, 0.0)),
            ["tilt"])
    finally:
        platform.disconnect()


def drill_step_limit() -> DrillResult:
    cfg = bench_config()
    cfg.limits.max_step_mm = 1.0
    platform = _platform(cfg)
    try:
        platform.move_to_orientation(Orientation(0.0, 0.0, 0.0))
        return _expect_refusal(
            "an oversized single step is refused",
            f"commanded a jump larger than the {cfg.limits.max_step_mm} mm "
            f"step limit",
            "refuse: a step that big is usually a typo or a unit mistake",
            lambda: platform.move_to_orientation(Orientation(20.0, 0.0, 0.0)),
            ["step"])
    finally:
        platform.disconnect()


# --------------------------------------------------------------------------
# Mechanics
# --------------------------------------------------------------------------

def drill_hard_stop_keeps_the_plate_flat() -> DrillResult:
    platform = _platform()
    try:
        # One axis reaches its stop well before the others.
        platform.motors[0]._transport.hard_stop_high = (
            platform.motors[0].cfg.mm_to_counts(3.0))
        result = platform.seek_hard_stop_together(+1, step_mm=0.2, budget_mm=10.0)
        flat = result.spread_mm < 0.05
        stopped_together = all(
            abs(result.positions_mm[name] - result.positions_mm["Top"]) < 0.05
            for name in result.positions_mm)
        return DrillResult(
            "the hard-stop search keeps all three together",
            flat and stopped_together,
            "gave the Top actuator an end stop 3 mm out and ran the "
            "calibration search",
            f"stopped by {', '.join(result.stopped_by)}; the three ended "
            f"{result.spread_mm:.4f} mm apart (worst during the search "
            f"{result.worst_spread_mm:.4f} mm)",
            "every axis halts with the first one to stop, and the plate is "
            "levelled afterwards")
    finally:
        platform.disconnect()


def drill_stall_protection_stops_pushing() -> DrillResult:
    platform = _platform()
    try:
        motor = platform.motors[0]
        motor._transport.hard_stop_high = motor.cfg.mm_to_counts(1.0)
        platform.motors[1]._transport.hard_stop_high = None
        platform.motors[2]._transport.hard_stop_high = None
        result = _expect_refusal(
            "a move into an obstruction is stopped",
            "put an obstruction 1 mm out and commanded a 5 mm move",
            "stop pushing and say the axis was resisting",
            lambda: platform.move_to_orientation(Orientation(5.0, 0.0, 0.0)),
            ["resisting", "torque", "halted"])
        if result.passed:
            torque = motor.get_torque_percent() or 0.0
            if torque > 40.0:
                result.passed = False
                result.what_happened += (
                    f" -- but the motor is still pushing at {torque:.0f}% torque")
        return result
    finally:
        platform.disconnect()


def drill_a_lost_motor_halts_the_others() -> DrillResult:
    """Two actuators continuing to a target the third will never reach is
    exactly how the plate gets racked about its ball joints."""
    cfg = bench_config()
    # The move has to be long enough to interrupt part-way through, so the
    # single-step guard is widened for this drill only -- otherwise it refuses
    # the move first and the drill proves nothing.
    cfg.limits.max_step_mm = 30.0
    platform = _platform(cfg)
    try:
        for actuator in platform.cfg.actuators:
            actuator.velocity_raw = 30           # slow enough to interrupt
        platform.move_to_orientation(Orientation(0.0, 0.0, 0.0))

        import threading
        outcome = {}

        def mover():
            try:
                platform.move_to_orientation(Orientation(20.0, 0.0, 0.0))
                outcome["error"] = None
            except REFUSALS as exc:
                outcome["error"] = str(exc)

        thread = threading.Thread(target=mover, daemon=True)
        thread.start()
        time.sleep(0.8)
        platform.motors[1]._transport.set_offline(True)
        thread.join(timeout=30)
        platform.motors[1]._transport.set_offline(False)

        message = outcome.get("error") or ""
        halted = all(
            abs(m.get_position_mm() - 20.0) > 0.5
            for i, m in enumerate(platform.motors) if i != 1)
        return DrillResult(
            "one motor lost mid-move halts the other two",
            bool(message) and halted,
            "unplugged the East motor part-way through a 20 mm move",
            (message or "the move reported success") +
            ("" if halted else " -- but the other two carried on to the target"),
            "halt every axis and report, rather than racking the plate about "
            "its ball joints")
    finally:
        platform.disconnect()


DRILLS: List[Callable[[], DrillResult]] = [
    drill_move_releases_the_brakes,
    drill_move_refused_when_brakes_will_not_release,
    drill_release_refused_when_drives_are_passive,
    drill_the_focal_plane_falls_without_a_brake,
    drill_move_refused_without_drive_power,
    drill_focus_limit,
    drill_tilt_limit,
    drill_step_limit,
    drill_hard_stop_keeps_the_plate_flat,
    drill_stall_protection_stops_pushing,
    drill_a_lost_motor_halts_the_others,
]


def run_all(only: Optional[List[str]] = None,
            report: Optional[Callable[[DrillResult], None]] = None) -> DrillReport:
    """Run every drill, or the named ones, and collect the results."""
    result = DrillReport()
    for drill in DRILLS:
        if only and not any(word.lower() in drill.__name__.lower()
                            for word in only):
            continue
        try:
            outcome = drill()
        except Exception as exc:  # noqa: BLE001 -- a drill that crashes is a fail
            outcome = DrillResult(
                drill.__name__.replace("drill_", "").replace("_", " "),
                False, "ran the drill",
                f"the drill itself raised {type(exc).__name__}: {exc}")
        result.results.append(outcome)
        if report is not None:
            report(outcome)
    return result


__all__ = ["DrillResult", "DrillReport", "DRILLS", "run_all", "bench_config"]
