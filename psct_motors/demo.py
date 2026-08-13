"""
Single-motor exerciser: what one JVL motor can do, and what happens when it
goes wrong.

    python -m psct_motors.cli demo --motor A

Built for a bench setup with **one** motor, because that is what you have
before the other two arrive. It works in **counts, revolutions and degrees of
motor shaft**, never millimetres, so it needs no calibration, no gear ratio
and no knowledge of what the shaft is attached to. It never touches the
three-actuator kinematics.

Two halves
----------
**Capability drills** show what the motor and this software can do together:
connect, read, change mode, move, jog, run at different speeds, measure
repeatability, stop mid-move, control the brake.

**Fault drills** make things go wrong on purpose, so you can watch the error
handling work and satisfy yourself it does the right thing. Each one says what
it is about to break, what it expects the software to do, what actually
happened, and what to do if you meet it for real on the telescope.

Most faults are injected in software (see faults.py): they tamper with the
register traffic so the motor *appears* to misbehave. That proves the handling
is right, which is the part that has bugs in it. A few drills instead ask you
to break something physically -- unplug the Ethernet cable, open MacTalk --
because a genuine fault is worth seeing at least once. Those are clearly
marked and can be skipped individually.

Safety
------
- Nothing moves unless you pass `--allow-motion`, and you are asked to confirm
  once before the first motion drill.
- Every motion drill stays inside a band around wherever the shaft starts,
  `--range-revs` wide (2 revolutions by default). The band is computed once,
  at the start, and every target is checked against it.
- The motor is returned to its starting position and left passive at the end,
  including when a drill fails or you interrupt with Ctrl-C.
- Injection only ever tampers with values read back, and with whether a
  transaction succeeds. It never invents a write or moves anything.
"""

from __future__ import annotations

import time
import traceback
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from .config import ActuatorConfig, load_config
from .faults import Fault, FaultInjectingTransport, wrap_motor
from .jvl_motor import BrakeState, JVLMotor, MotorFault
from .registers import (
    REGISTERS, MotorMode, VERIFY, WordOrder, describe_errors, describe_mode,
)
from .transport import ModbusError

PASS = "PASS"
FAIL = "FAIL"
INFO = "INFO"
SKIP = "SKIP"


# --------------------------------------------------------------------------
# Results
# --------------------------------------------------------------------------

@dataclass
class DrillResult:
    verdict: str = INFO
    notes: List[str] = field(default_factory=list)
    #: Where to echo notes as they are added. Set by DemoContext.result().
    #:
    #: Findings stream out as they happen rather than being held until the
    #: drill ends. That matters for two reasons: a drill that prompts you
    #: part-way through (unplug the cable, open MacTalk) would otherwise print
    #: its prompts and its findings out of order, and a drill that takes half a
    #: minute would look like it had hung.
    echo: Optional[Callable[[str], None]] = None

    def note(self, text: str) -> None:
        self.notes.append(text)
        if self.echo is not None:
            self.echo(text)

    def ok(self, text: str) -> "DrillResult":
        self.verdict = PASS
        self.note(text)
        return self

    def bad(self, text: str) -> "DrillResult":
        self.verdict = FAIL
        self.note(text)
        return self

    def skipped(self, text: str) -> "DrillResult":
        self.verdict = SKIP
        self.note(text)
        return self


@dataclass
class Drill:
    name: str
    category: str
    summary: str
    #: What the software is supposed to do. Printed before the drill runs, so
    #: you know what you are looking for rather than judging after the fact.
    expectation: str
    run: Callable[["DemoContext"], DrillResult]
    needs_motion: bool = False
    #: Asks the operator to physically interfere with the hardware.
    needs_operator: bool = False
    #: What to do if this shows up for real, printed with the result.
    remediation: str = ""


# --------------------------------------------------------------------------
# Context
# --------------------------------------------------------------------------

class DemoContext:
    """Everything a drill needs: the motor, the safe band, and output."""

    def __init__(self, motor: JVLMotor, injector: FaultInjectingTransport,
                 out: Callable[[str], None], ask: Callable[[str], bool],
                 allow_motion: bool, range_revs: float):
        self.motor = motor
        self.injector = injector
        self.out = out
        self.ask = ask
        self.allow_motion = allow_motion
        self.range_revs = range_revs

        self.counts_per_rev = float(motor.cfg.counts_per_rev)
        self.home_counts: int = 0
        self.band_low: int = 0
        self.band_high: int = 0
        self._motion_confirmed = False

    def result(self) -> "DrillResult":
        """A result whose notes are printed as they are added."""
        return DrillResult(echo=lambda text: self.out(f"  {text}" if text else ""))

    # ---- units -----------------------------------------------------------

    def revs(self, counts: float) -> float:
        return counts / self.counts_per_rev

    def counts(self, revs: float) -> int:
        return int(round(revs * self.counts_per_rev))

    def describe_counts(self, counts: int) -> str:
        """Counts, with revolutions and shaft degrees alongside.

        No millimetres anywhere: on a bare shaft there is nothing to convert
        to, and inventing a number would be worse than omitting it.
        """
        revolutions = self.revs(counts)
        return (f"{counts} counts ({revolutions:+.4f} rev, "
                f"{revolutions * 360.0:+.2f} deg of shaft)")

    # ---- safe band -------------------------------------------------------

    def establish_band(self) -> None:
        """Fix the travel band around wherever the shaft is now."""
        self.home_counts = self.motor.get_position_counts()
        span = self.counts(self.range_revs)
        self.band_low = self.home_counts - span
        self.band_high = self.home_counts + span

    def check_band(self, target_counts: int) -> None:
        if not (self.band_low <= target_counts <= self.band_high):
            raise MotorFault(
                f"Demo refused a target of {target_counts} counts: outside the "
                f"bench band {self.band_low}..{self.band_high} counts "
                f"(+/- {self.range_revs} rev around where the shaft started)."
            )

    # ---- motion helpers --------------------------------------------------

    def confirm_motion(self) -> bool:
        """Ask once, before the first drill that turns the shaft."""
        if not self.allow_motion:
            return False
        if self._motion_confirmed:
            return True
        self.out("")
        self.out("  The next drills TURN THE MOTOR SHAFT.")
        self.out(f"  It started at {self.home_counts} counts and will stay between")
        self.out(f"  {self.band_low} and {self.band_high} counts "
                 f"(+/- {self.range_revs} rev, "
                 f"+/- {self.range_revs * 360.0:.0f} deg of shaft).")
        self.out("  Make sure nothing is attached that could be damaged by that.")
        self._motion_confirmed = self.ask("  Allow the shaft to turn?")
        return self._motion_confirmed

    def move_to(self, target_counts: int, wait: bool = True,
                timeout_s: float = 30.0) -> bool:
        """Move within the band, in counts."""
        self.check_band(target_counts)
        self.motor.clear_cancel()
        self.motor.ensure_position_mode()
        if self.motor.brake_is_software_controlled:
            status = self.motor.get_brake_status()
            if status.state is not BrakeState.RELEASED:
                self.motor.release_brake()
        self.motor.set_velocity(self.motor.cfg.velocity_raw)
        self.motor.command_position_counts(target_counts)
        if not wait:
            return True
        return self.motor.wait_for_in_position(timeout_s=timeout_s)

    def move_revs(self, revs: float, wait: bool = True) -> bool:
        return self.move_to(self.home_counts + self.counts(revs), wait=wait)

    def long_move_revs(self) -> float:
        """How far the 'catch it in flight' drills should travel.

        Three quarters of the band, so there is as much time as the bench
        allows to interrupt the move, without ever approaching the edge.
        """
        return min(1.5, self.range_revs * 0.75)

    def slow_velocity(self) -> int:
        """A deliberately low V_SOLL, for drills that interrupt a move."""
        return max(1, int(self.motor.cfg.velocity_raw // 10))

    def wait_until_progress(self, start_counts: int, target_counts: int,
                            fraction: float = 0.2,
                            timeout_s: float = 20.0) -> tuple:
        """Poll until the shaft has covered `fraction` of the way to target.

        Drills that interrupt a move cannot assume how fast the motor is: that
        depends on V_SOLL, the microstepping and whatever the shaft is attached
        to, none of which this software knows before it is calibrated. Waiting
        on observed progress instead of a fixed delay makes those drills work
        at any speed, and lets them say honestly when the move was simply too
        quick to catch.

        Returns (caught_in_flight, position).
        """
        span = target_counts - start_counts
        if span == 0:
            return False, start_counts
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            position = self.motor.get_position_counts()
            covered = (position - start_counts) / span
            if covered >= 1.0:
                return False, position          # finished before we caught it
            if covered >= fraction:
                return True, position
            time.sleep(0.05)
        return False, self.motor.get_position_counts()

    def return_home(self) -> None:
        try:
            self.motor.clear_cancel()
            self.motor.ensure_position_mode()
            self.move_to(self.home_counts)
        except (ModbusError, MotorFault) as exc:
            self.out(f"  Could not return to the starting position: {exc}")


# --------------------------------------------------------------------------
# Capability drills
# --------------------------------------------------------------------------

def drill_identity(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    motor = ctx.motor
    result.note(f"Connection: {motor.describe()}")
    result.note(f"Configured word order: {motor.word_order.value}")

    probe = motor.probe_word_order()
    for line in probe.evidence:
        result.note(f"  {line}")

    if probe.detected is None:
        # Inconclusive is not a failure. It means the probe found nothing to
        # go on -- which says nothing at all about whether the configured
        # order is right, so reporting it as a fault would be a false alarm.
        result.note("")
        result.note(f"Word order not determined: {probe.reason}")
        result.note("This is not evidence of a problem. Confirm it the direct "
                    "way instead: command a known move with the `small-move` "
                    "drill and check the shaft turns the expected quarter turn, "
                    "or compare a position read against MacTalk's display.")
        result.verdict = INFO
        return result

    if probe.detected is motor.word_order:
        result.note("")
        return result.ok(
            f"Word order confirmed as {probe.detected.value} from the register "
            "values themselves: every small configuration value has its high "
            "word where this setting says it should be."
        )
    return result.bad(
        f"Word order MISMATCH: the config says {motor.word_order.value}, but the "
        f"registers only make sense as {probe.detected.value}. Every position "
        "read from this motor is wrong until you fix it."
    )


def drill_registers(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    values = ctx.motor.read_diagnostics()
    unreadable = []
    for reg in REGISTERS:
        value = values.get(reg.name, "<missing>")
        marker = "  <-- VERIFY against MacTalk" if reg.confidence == VERIFY else ""
        result.note(f"{reg.number:>4}  {reg.name:<16} {value}{marker}")
        if str(value).startswith("<unreadable"):
            unreadable.append(reg.name)
    if unreadable:
        result.note("")
        result.note(
            f"Unreadable on this motor: {', '.join(unreadable)}. That is not "
            "necessarily wrong -- it may mean the register number is different "
            "on this firmware. Compare against MacTalk and correct "
            "psct_motors/registers.py."
        )
    result.note("")
    result.note(
        "Two of these are the only history the motor keeps: Follow Error Max "
        "(22) and Bus Voltage Min (98) are latched extremes that survive a "
        "cleared error. There is no event log in the drive -- Errors (35) and "
        "Warnings (36) are instantaneous only."
    )
    mismatch = ctx.motor.check_brake_configuration()
    if mismatch:
        result.note("")
        result.note(mismatch)
    result.verdict = INFO
    return result


def drill_position_stability(ctx: DemoContext) -> DrillResult:
    """Watch a stationary axis, to learn what 'not moving' looks like."""
    result = ctx.result()
    samples = []
    for _ in range(20):
        samples.append(ctx.motor.get_position_counts())
        time.sleep(0.1)
    spread = max(samples) - min(samples)
    result.note("20 samples over 2 s, stationary.")
    result.note(f"Position: {min(samples)} .. {max(samples)} counts")
    result.note(f"Spread: {spread} counts ({ctx.revs(spread) * 360.0:.4f} deg of shaft)")
    result.note("")
    result.note(
        "This is your noise floor. The in-position tolerance must be comfortably "
        "larger than this spread, or a settled axis will never be declared "
        "settled and every move will time out."
    )
    tolerance_counts = ctx.motor.cfg.in_position_tol_mm * ctx.motor.cfg.resolved_counts_per_mm
    result.note(f"Configured in-position tolerance: about {tolerance_counts:.0f} counts "
                f"({ctx.revs(tolerance_counts) * 360.0:.3f} deg of shaft).")
    if not ctx.motor.cfg.scale_is_measured:
        result.note(
            "Treat that count as indicative only: the tolerance is configured in "
            "millimetres and this actuator has not been calibrated, so the "
            "conversion uses an assumed drivetrain. It becomes meaningful after "
            f"`cli calibrate --motor {ctx.motor.name}` on the real mechanism."
        )
    if spread == 0:
        return result.ok("Rock steady: no jitter at all while stationary.")
    if spread < tolerance_counts / 2:
        return result.ok("Jitter is well inside the in-position tolerance.")
    return result.bad(
        f"Jitter ({spread} counts) is not comfortably inside the tolerance "
        f"({tolerance_counts:.0f} counts). Raise in_position_tol_mm, or moves "
        "will time out even when the axis has arrived."
    )


def drill_mode(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    motor = ctx.motor
    start = motor.get_mode()
    result.note(f"Mode at start: {describe_mode(start)}")
    try:
        motor.set_mode(MotorMode.POSITION)
        result.note(f"Set Position mode, read back: {describe_mode(motor.get_mode())}")
        motor.set_mode(MotorMode.PASSIVE)
        result.note(f"Set Passive mode, read back: {describe_mode(motor.get_mode())}")
    except MotorFault as exc:
        return result.bad(str(exc))
    finally:
        try:
            motor.write_register("MODE_REG", start)
        except (ModbusError, MotorFault):
            pass
    result.note(f"Restored the mode it started in ({describe_mode(start)}).")
    return result.ok("Mode changes take, and read back as written.")


def drill_small_move(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    if not ctx.confirm_motion():
        return result.skipped("Motion not permitted.")
    quarter = 0.25
    result.note(f"Moving +{quarter} rev, then back.")
    if not ctx.move_revs(quarter):
        return result.bad("The move did not report in-position before the timeout.")
    landed = ctx.motor.get_position_counts()
    result.note(f"Arrived at {ctx.describe_counts(landed - ctx.home_counts)} from start.")
    error = landed - (ctx.home_counts + ctx.counts(quarter))
    result.note(f"Position error at target: {error} counts.")
    if not ctx.move_revs(0.0):
        return result.bad("The return move did not complete.")
    back = ctx.motor.get_position_counts()
    result.note(f"Returned to within {back - ctx.home_counts} counts of the start.")
    return result.ok("Moved out and back, and reported arrival both times.")


def drill_repeatability(ctx: DemoContext) -> DrillResult:
    """Go away and come back several times; see where 'back' really is."""
    result = ctx.result()
    if not ctx.confirm_motion():
        return result.skipped("Motion not permitted.")
    cycles = 4
    landings = []
    for i in range(cycles):
        if not ctx.move_revs(0.5):
            return result.bad(f"Cycle {i + 1}: the outward move did not complete.")
        if not ctx.move_revs(0.0):
            return result.bad(f"Cycle {i + 1}: the return move did not complete.")
        landings.append(ctx.motor.get_position_counts() - ctx.home_counts)
        result.note(f"Cycle {i + 1}: returned to {landings[-1]:+d} counts from start.")
    spread = max(landings) - min(landings)
    result.note("")
    result.note(f"Return spread over {cycles} cycles: {spread} counts "
                f"({ctx.revs(spread) * 360.0:.4f} deg of shaft).")
    result.note(
        "On the telescope this spread is your positioning repeatability, and it "
        "sets the smallest tilt adjustment worth commanding."
    )
    return result.ok("Repeatability measured.")


def drill_velocity(ctx: DemoContext) -> DrillResult:
    """Same move at two speeds, timed: shows what V_SOLL actually buys."""
    result = ctx.result()
    if not ctx.confirm_motion():
        return result.skipped("Motion not permitted.")
    motor = ctx.motor
    original = motor.cfg.velocity_raw
    timings = []
    try:
        for speed in (original, max(1, original // 4)):
            motor.ensure_position_mode()
            ctx.move_revs(0.0)
            motor.set_velocity(speed)
            started = time.monotonic()
            motor.command_position_counts(ctx.home_counts + ctx.counts(0.5))
            done = motor.wait_for_in_position(timeout_s=60.0)
            elapsed = time.monotonic() - started
            timings.append((speed, elapsed, done))
            result.note(f"V_SOLL {speed:>6}: half a revolution in {elapsed:.2f} s"
                        + ("" if done else "  (TIMED OUT)"))
            ctx.move_revs(0.0)
    finally:
        motor.set_velocity(original)
    if len(timings) == 2 and timings[0][2] and timings[1][2]:
        fast, slow = timings[0][1], timings[1][1]
        if slow > fast:
            result.note(f"Quarter speed took {slow / fast:.1f}x as long, as expected.")
            return result.ok("V_SOLL controls speed as expected.")
        return result.bad(
            "The slower setting was not slower. V_SOLL may not be register 5 on "
            "this firmware, or the move is too short to measure -- check against "
            "MacTalk."
        )
    return result.bad("At least one timed move did not complete.")


def drill_stop(ctx: DemoContext) -> DrillResult:
    """Start a long move, stop it halfway, prove it holds."""
    result = ctx.result()
    if not ctx.confirm_motion():
        return result.skipped("Motion not permitted.")
    motor = ctx.motor
    original = motor.cfg.velocity_raw
    distance = ctx.long_move_revs()
    try:
        ctx.move_revs(0.0)
        motor.set_velocity(ctx.slow_velocity())
        start = motor.get_position_counts()
        target = ctx.home_counts + ctx.counts(distance)
        ctx.check_band(target)
        motor.command_position_counts(target)
        result.note(f"Commanded a {distance} rev move to {target} counts at "
                    f"V_SOLL {ctx.slow_velocity()}.")

        caught, moving_at = ctx.wait_until_progress(start, target, fraction=0.2)
        if not caught:
            return result.bad(
                f"The move reached {moving_at} counts before it could be "
                "interrupted -- it finished too quickly for this drill to test "
                "anything. Lower velocity_raw in the config, or raise "
                "--range-revs so there is further to travel."
            )
        result.note(f"Caught it in flight at {moving_at} counts, about "
                    f"{100.0 * (moving_at - start) / (target - start):.0f}% of the way.")

        motor.stop()
        time.sleep(0.5)
        stopped_at = motor.get_position_counts()
        settled_target = motor.get_target_counts()
        result.note(f"STOP issued. Position {stopped_at}, target now {settled_target}.")

        time.sleep(1.0)
        held_at = motor.get_position_counts()
        drift = abs(held_at - stopped_at)
        result.note(f"One second later the shaft is at {held_at} "
                    f"({drift} counts of drift).")
        result.note(f"Mode is still {describe_mode(motor.get_mode())} -- the drive "
                    "stayed enabled and is holding.")

        if abs(stopped_at - target) < ctx.counts(0.05):
            return result.bad(
                "The shaft ended up at the original target anyway, so STOP did "
                "not actually interrupt the move."
            )
        if motor.get_mode() != int(MotorMode.POSITION):
            return result.bad("STOP left the drive disabled; it should stay enabled.")
        if drift > ctx.counts(0.05):
            return result.bad(
                f"The shaft drifted {drift} counts after stopping. It should be "
                "actively held, not coasting."
            )
        return result.ok(
            "STOP halted the move partway, and the drive stayed enabled and "
            "holding rather than going passive."
        )
    finally:
        motor.set_velocity(original)
        motor.clear_cancel()


def drill_travel_limit(ctx: DemoContext) -> DrillResult:
    """Command something silly; check the software refuses and nothing moves."""
    result = ctx.result()
    motor = ctx.motor
    before = motor.get_position_counts()
    absurd = ctx.home_counts + ctx.counts(ctx.range_revs * 10)
    result.note(f"Asking the demo to move to {absurd} counts, far outside the "
                f"bench band {ctx.band_low}..{ctx.band_high}.")
    try:
        ctx.check_band(absurd)
    except MotorFault as exc:
        result.note(f"Refused: {exc}")
        after = motor.get_position_counts()
        if after != before:
            return result.bad(f"The shaft moved anyway: {before} -> {after}.")
        return result.ok("Refused before commanding anything, and nothing moved.")
    return result.bad("The out-of-range target was NOT refused.")


def drill_brake(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    motor = ctx.motor
    mode = motor.cfg.brake.mode
    result.note(f"Brake mode for this actuator: '{mode}'.")

    if mode == "none":
        result.note("No brake is configured, so there is nothing to exercise.")
        result.note("If this motor does have a brake, set brake.mode to 'auto' "
                    "or 'output' in the config and re-run.")
        return result.skipped("No brake configured.")

    if mode == "auto":
        motor.set_mode(MotorMode.PASSIVE)
        passive_state = motor.get_brake_status()
        motor.set_mode(MotorMode.POSITION)
        enabled_state = motor.get_brake_status()
        motor.set_mode(MotorMode.PASSIVE)
        result.note(f"Drive passive  -> brake reads {passive_state.state.value} "
                    f"({passive_state.detail})")
        result.note(f"Drive enabled  -> brake reads {enabled_state.state.value} "
                    f"({enabled_state.detail})")
        result.note("")
        result.note("Both of those are INFERRED from the mode, not measured. "
                    "Listen for the brake clicking as the mode changes -- if you "
                    "hear nothing, the brake is not wired to follow the drive and "
                    "brake.mode should not be 'auto'.")
        if passive_state.state is BrakeState.ENGAGED and \
                enabled_state.state is BrakeState.RELEASED:
            return result.ok("The inferred brake state tracks the drive mode.")
        return result.bad("The inferred brake state did not track the drive mode.")

    # mode == "output"
    motor.ensure_position_mode()
    motor.command_position_counts(motor.get_position_counts())
    for _ in range(2):
        motor.release_brake()
        released = motor.get_brake_status()
        result.note(f"Commanded release -> reads back {released.state.value}")
        time.sleep(0.5)
        motor.engage_brake()
        engaged = motor.get_brake_status()
        result.note(f"Commanded engage  -> reads back {engaged.state.value}")
        time.sleep(0.5)
    result.note("")
    result.note("Those states were READ BACK from the output register, not inferred.")

    motor.set_mode(MotorMode.PASSIVE)
    try:
        motor.release_brake()
        return result.bad(
            "The brake was released while the drive was passive. The interlock "
            "that prevents dropping a loaded axis is not working."
        )
    except MotorFault as exc:
        result.note(f"Interlock check: {exc}")
    return result.ok("Brake control works, reads back, and the passive-drive "
                     "interlock refuses to drop the load.")


# --------------------------------------------------------------------------
# Fault drills
# --------------------------------------------------------------------------

def drill_fault_bad_register(ctx: DemoContext) -> DrillResult:
    """A genuinely invalid request, no injection: what a Modbus error looks like."""
    result = ctx.result()
    bogus = 9999
    result.note(f"Reading JVL register {bogus}, which does not exist.")
    try:
        value = ctx.motor.read_register(bogus)
    except (ModbusError, MotorFault) as exc:
        result.note(f"Raised: {type(exc).__name__}")
        result.note(f"  {exc}")
        return result.ok("An invalid register read fails cleanly with a readable "
                         "message rather than returning a wrong number.")
    return result.bad(
        f"Register {bogus} returned {value} instead of failing. The motor is "
        "answering for a register that should not exist -- treat every register "
        "read from it with suspicion until you have checked against MacTalk."
    )


def drill_fault_comms_drop(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    with ctx.injector:
        ctx.injector.arm(Fault.COMMS_DROP)
        result.note("Injected: total communications loss.")

        status = ctx.motor.read_status()
        result.note(f"read_status() -> comms_error: {status.comms_error!r}")
        if not status.comms_error:
            return result.bad("A status read during a comms loss did not report "
                              "an error. A UI would show stale numbers as live.")
        result.note("Note that read_status() did NOT raise -- it reported the "
                    "failure in the returned object, so a polling loop survives.")

        try:
            ctx.motor.get_position_counts()
            return result.bad("A direct position read did not raise during a "
                              "comms loss.")
        except ModbusError as exc:
            result.note(f"A direct read raised ModbusError: {str(exc)[:90]}")

    result.note("")
    result.note("Injection cleared; checking the link recovers.")
    time.sleep(0.2)
    recovered = ctx.motor.read_status()
    if recovered.comms_error:
        return result.bad(f"Still failing after the injection was cleared: "
                          f"{recovered.comms_error}")
    result.note(f"Position reads again: {recovered.position_counts} counts.")
    return result.ok("Comms loss is detected, reported without crashing a poll "
                     "loop, and recovers by itself when the link returns.")


def drill_fault_error_bits(ctx: DemoContext) -> DrillResult:
    """The important one: a drive fault during a move."""
    result = ctx.result()
    if not ctx.confirm_motion():
        result.note("Motion not permitted, so this runs as a stationary check.")
        with ctx.injector:
            ctx.injector.arm(Fault.ERROR_BITS, error_bits_value=1 << 5)
            errors = ctx.motor.get_errors()
            result.note(f"ERR_BITS reads 0x{errors:08X} -> {describe_errors(errors)}")
        return result.ok("The error register is read and decoded. Re-run with "
                         "--allow-motion to see a fault interrupt a real move.")

    motor = ctx.motor
    original = motor.cfg.velocity_raw
    distance = ctx.long_move_revs()
    try:
        ctx.move_revs(0.0)
        motor.set_velocity(ctx.slow_velocity())
        start = motor.get_position_counts()
        target = ctx.home_counts + ctx.counts(distance)
        ctx.check_band(target)
        motor.command_position_counts(target)
        result.note(f"Started a slow {distance} rev move to {target} counts.")

        caught, moving_at = ctx.wait_until_progress(start, target, fraction=0.2)
        if not caught:
            return result.bad(
                "The move finished before a fault could be injected into it. "
                "Lower velocity_raw, or raise --range-revs."
            )
        result.note(f"Caught it in flight at {moving_at} counts.")

        with ctx.injector:
            ctx.injector.arm(Fault.ERROR_BITS, error_bits_value=1 << 1)
            result.note("Injected: ERR_BITS = 0x00000002 while the move is running.")
            try:
                motor.wait_for_in_position(timeout_s=10.0)
                return result.bad(
                    "The move completed without noticing the fault. A faulting "
                    "motor would have been driven to its target regardless."
                )
            except MotorFault as exc:
                result.note(f"Detected: {exc}")

        time.sleep(0.3)
        halted_at = motor.get_position_counts()
        target_now = motor.get_target_counts()
        result.note(f"After the fault, position {halted_at}, target {target_now}.")
        moved_on = abs(target_now - target) < ctx.counts(0.05)
        if moved_on:
            return result.bad(
                "The target is still the original one, so the motor is still "
                "driving towards it despite the reported fault."
            )
        result.note("The target was reset to the current position: the software "
                    "halted the axis rather than leaving it driving.")
        if abs(halted_at - target) < ctx.counts(0.1):
            result.note("(The move had nearly finished, so little travel was saved.)")
        result.note("")
        result.note(f"ERR_BITS now reads {describe_errors(motor.get_errors())} -- "
                    "the injected fault is gone with the injection.")
        return result.ok("A fault during a move is detected, the axis is halted "
                         "where it was, and the operator is told why.")
    finally:
        motor.set_velocity(original)
        motor.clear_cancel()


def drill_fault_mode_revert(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    with ctx.injector:
        ctx.injector.arm(Fault.MODE_REVERT)
        result.note("Injected: MODE_REG reads back Passive however it is written.")
        result.note("This is what a second client -- normally MacTalk still being "
                    "connected -- looks like from here.")
        try:
            ctx.motor.set_mode(MotorMode.POSITION, settle_s=0.1)
            return result.bad(
                "The mode change was accepted without checking it stuck. A move "
                "would then be commanded and quietly do nothing."
            )
        except MotorFault as exc:
            result.note(f"Refused: {exc}")
            if "MacTalk" not in str(exc):
                return result.bad("Detected, but the message does not point at "
                                  "the usual cause.")
    return result.ok("A mode that will not stick is caught immediately, and the "
                     "message names the likely cause.")


def drill_fault_stuck(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    if not ctx.confirm_motion():
        return result.skipped("Motion not permitted.")
    motor = ctx.motor
    try:
        ctx.move_revs(0.0)
        motor.ensure_position_mode()
        with ctx.injector:
            ctx.injector.arm(Fault.STUCK_POSITION)
            result.note("Injected: every position register frozen -- the axis appears not to move.")
            result.note("This is what a seized screw, an unreleased brake or a "
                        "dead encoder looks like.")
            target = ctx.home_counts + ctx.counts(0.25)
            ctx.check_band(target)
            motor.command_position_counts(target)
            started = time.monotonic()
            arrived = motor.wait_for_in_position(timeout_s=4.0)
            elapsed = time.monotonic() - started
            if arrived:
                return result.bad("The move reported success even though the "
                                  "position never changed.")
            result.note(f"Timed out after {elapsed:.1f} s without claiming arrival.")
        time.sleep(0.3)
        target_now = motor.get_target_counts()
        position_now = motor.get_position_counts()
        result.note(f"After the timeout, target {target_now}, position {position_now}.")
        if abs(target_now - target) < ctx.counts(0.05):
            return result.bad("The axis is still commanded to the unreachable "
                              "target rather than being halted.")
        result.note("The axis was halted rather than left straining towards a "
                    "target it was never reaching.")
        result.note("")
        result.note(
            "One subtlety worth knowing. The halt works by writing the position "
            "the motor reports as the new target. If the axis is genuinely "
            "stuck, that reported position is the true one and the axis stops "
            "where it is -- correct. If instead the ENCODER is dead while the "
            "shaft still turns, the reported position is stale, and the halt "
            "commands the shaft back to that stale value. That is why the two "
            "faults look identical here but are not: a frozen position with a "
            "warm motor and no ERR_BITS deserves a physical look before you "
            "command it again."
        )
        return result.ok("A stuck axis times out instead of hanging, and is "
                         "halted rather than left driving.")
    finally:
        motor.clear_cancel()
        ctx.return_home()


def drill_fault_word_order(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    # PROG_VERSION rather than the position: the shaft may be at zero,
    # and zero is the one value that survives a word swap unchanged, so it
    # would demonstrate nothing.
    good_version = ctx.motor.read_register("PROG_VERSION", signed=False)
    good_position = ctx.motor.get_position_counts()
    with ctx.injector:
        ctx.injector.arm(Fault.SWAPPED_WORDS)
        bad_version = ctx.motor.read_register("PROG_VERSION", signed=False)
        bad_position = ctx.motor.get_position_counts()
        result.note(f"PROG_VERSION  correct: {good_version}   swapped: {bad_version}")
        result.note(f"Position      correct: {good_position}   swapped: {bad_position}")
        result.note("")
        if good_position == 0:
            result.note("(The shaft is at zero, which is the one value a word "
                        "swap leaves unchanged -- hence checking PROG_VERSION, "
                        "which is never zero.)")
        result.note("The swapped position is what EVERY reading would look like "
                    "with the wrong word_order in the config. Note that it is "
                    "still a plausible-looking number: nothing about it "
                    "announces itself as wrong, which is why this is checked "
                    "automatically rather than left to be noticed.")
        try:
            detected = ctx.motor.detect_word_order()
            expected = (WordOrder.HIGH_LOW if ctx.motor.word_order is WordOrder.LOW_HIGH
                        else WordOrder.LOW_HIGH)
            result.note(f"detect_word_order() under the swap says: {detected.value}")
            if detected is expected:
                return result.ok(
                    "The word-order check spots the swap. Connecting with the "
                    "wrong setting is refused rather than silently misreporting "
                    "every position."
                )
            return result.bad("The word-order check did not spot the swap.")
        except MotorFault as exc:
            result.note(f"detect_word_order() refused to guess: {exc}")
            return result.ok("The swap is caught -- neither order gives a "
                             "plausible firmware version, so it refuses to guess.")


def drill_fault_flaky(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    rate = 0.15
    with ctx.injector:
        ctx.injector.arm(Fault.COMMS_FLAKY, failure_rate=rate)
        result.note(f"Injected: {rate:.0%} of individual transactions fail at random.")
        good = bad = 0
        for _ in range(20):
            status = ctx.motor.read_status()
            if status.comms_error:
                bad += 1
            else:
                good += 1
        result.note(f"20 status polls: {good} succeeded, {bad} reported an error.")
        if bad == 0:
            return result.bad("No poll reported an error, so the injection did "
                              "not take effect.")
        result.note("")
        result.note(
            f"Note how much worse than {rate:.0%} that is. One status poll makes "
            "about six separate register reads, and any one of them failing "
            "fails the whole poll. A link losing a small fraction of packets "
            "therefore loses a large fraction of readings -- which is why a "
            "flaky cable presents as 'nothing works' rather than 'things are "
            "occasionally slow'."
        )
        result.note("No poll raised, and none returned a stale reading as if it "
                    "were live.")
    recovered = ctx.motor.read_status()
    if recovered.comms_error:
        return result.bad("Still failing after the injection was cleared.")
    return result.ok("A marginal link degrades to intermittent errors rather "
                     "than crashing or reporting stale positions as live.")


# --------------------------------------------------------------------------
# Operator-participation drills
# --------------------------------------------------------------------------

def drill_real_unplug(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    ctx.out("")
    ctx.out("  This drill asks you to UNPLUG the motor's Ethernet cable.")
    ctx.out("  Nothing will be moving. You will be asked to plug it back in.")
    if not ctx.ask("  Run this drill?"):
        return result.skipped("Declined.")

    ctx.out("  Unplug the Ethernet cable from the motor now.")
    ctx.ask("  Press Enter (or y) once it is unplugged.")

    detected = False
    for attempt in range(15):
        status = ctx.motor.read_status()
        if status.comms_error:
            detected = True
            result.note(f"Detected after {attempt + 1} poll(s): {status.comms_error[:100]}")
            break
        time.sleep(1.0)
    if not detected:
        return result.bad(
            "15 polls over 15 s and none reported an error. Either the cable is "
            "still connected, or something is answering on that address that "
            "should not be."
        )

    ctx.out("  Now plug the cable back in.")
    ctx.ask("  Press Enter (or y) once it is reconnected.")

    for attempt in range(30):
        try:
            # reconnect(), not connect(): after a reset the client can hold a
            # socket it still believes is usable, so connect() succeeds and
            # every read then fails against the dead socket.
            ctx.motor.reconnect()
            position = ctx.motor.get_position_counts()
            result.note(f"Reconnected after {attempt + 1} attempt(s); "
                        f"position {position} counts.")
            return result.ok(
                "A real cable pull is detected, reported, and the connection "
                "comes back after a reconnect. Note that reconnecting is an "
                "explicit step -- the software does not silently reconnect and "
                "resume a move, which is deliberate."
            )
        except (ModbusError, MotorFault):
            time.sleep(1.0)
    return result.bad("Could not reconnect within 30 s of you reporting the "
                      "cable was back.")


def drill_real_mactalk(ctx: DemoContext) -> DrillResult:
    result = ctx.result()
    ctx.out("")
    ctx.out("  This drill asks you to connect MacTalk to this motor while this")
    ctx.out("  software is also connected, to see what a second client does.")
    if not ctx.ask("  Run this drill?"):
        return result.skipped("Declined.")

    ctx.out("  Open MacTalk and connect it to this motor now.")
    ctx.ask("  Press Enter (or y) once MacTalk is connected.")

    try:
        current = ctx.motor.get_mode()
        result.note(f"Mode reads {describe_mode(current)} with MacTalk attached.")
    except (ModbusError, MotorFault) as exc:
        result.note(f"Reads now fail with MacTalk attached: {exc}")
        result.note("Some configurations only allow one client at a time, which "
                    "is itself worth knowing.")
        ctx.out("  Disconnect MacTalk again.")
        ctx.ask("  Press Enter (or y) once MacTalk is disconnected.")
        return result.ok("With MacTalk attached this software cannot talk to the "
                         "motor at all. Close MacTalk before running this.")

    try:
        ctx.motor.set_mode(MotorMode.POSITION, settle_s=0.5)
        result.note("The mode change stuck even with MacTalk connected.")
        result.note("MacTalk is evidently not fighting for control in this state. "
                    "Try pressing something in MacTalk while this runs to see the "
                    "conflict.")
        verdict = result.ok("No conflict observed in this configuration.")
    except MotorFault as exc:
        result.note(f"Conflict detected: {exc}")
        verdict = result.ok("A second client fighting for control is detected and "
                            "named, rather than causing a move that silently does "
                            "nothing.")
    finally:
        try:
            ctx.motor.write_register("MODE_REG", int(MotorMode.PASSIVE))
        except (ModbusError, MotorFault):
            pass
    ctx.out("  Disconnect MacTalk again.")
    ctx.ask("  Press Enter (or y) once MacTalk is disconnected.")
    return verdict


# --------------------------------------------------------------------------
# The drill list
# --------------------------------------------------------------------------

def all_drills() -> List[Drill]:
    return [
        # ---- capability -------------------------------------------------
        Drill("identity", "capability",
              "Confirm the Modbus word order from the register values themselves.",
              "Small configuration values (currents, ramps, velocity limit) all "
              "put their zero high word on the side the configured order says, "
              "confirming it. An inconclusive result is informational, not a "
              "failure -- only a contradiction is a problem.",
              drill_identity,
              remediation="If the word order mismatches, fix 'word_order' for "
                          "this actuator in the config. Every position read is "
                          "wrong until you do."),
        Drill("registers", "capability",
              "Dump every register this software uses, with confidence markers.",
              "Confirmed registers return sensible values. VERIFY rows are the "
              "ones to compare against MacTalk.",
              drill_registers,
              remediation="Correct any wrong register number in "
                          "psct_motors/registers.py and promote the confidence "
                          "marker once you have checked it."),
        Drill("stability", "capability",
              "Watch a stationary axis for 2 s to find the position noise floor.",
              "The spread is well below the in-position tolerance.",
              drill_position_stability,
              remediation="If jitter approaches the tolerance, raise "
                          "in_position_tol_mm or every move will time out."),
        Drill("mode", "capability",
              "Change operating mode and confirm each change reads back.",
              "Position and Passive both take, and the original mode is restored.",
              drill_mode,
              remediation="A mode that will not stick almost always means "
                          "MacTalk is still connected."),
        Drill("small-move", "capability",
              "Move a quarter turn out and back.",
              "Both moves report arrival, and the shaft returns near the start.",
              drill_small_move, needs_motion=True,
              remediation="If the move never reports arrival, check the "
                          "in-position tolerance and that V_SOLL is non-zero."),
        Drill("repeatability", "capability",
              "Four out-and-back cycles; measure where 'back' really lands.",
              "The return spread is small and consistent.",
              drill_repeatability, needs_motion=True,
              remediation="A large spread limits the smallest tilt worth "
                          "commanding on the telescope."),
        Drill("velocity", "capability",
              "Time the same move at full and quarter speed.",
              "Quarter speed takes appreciably longer.",
              drill_velocity, needs_motion=True,
              remediation="If speed does not change, V_SOLL may not be register "
                          "5 on this firmware."),
        Drill("stop", "capability",
              "Start a long move, press STOP halfway, confirm it holds.",
              "The move halts partway, the target becomes the current position, "
              "and the drive stays enabled and holding.",
              drill_stop, needs_motion=True,
              remediation="If the drive ends up disabled, something is calling "
                          "passivate rather than stop."),
        Drill("limit", "capability",
              "Command a target far outside the safe band.",
              "It is refused before anything is written, and nothing moves.",
              drill_travel_limit,
              remediation="A refusal that still moves the shaft would mean the "
                          "check runs after the command, not before."),
        Drill("brake", "capability",
              "Exercise the brake and its interlock.",
              "The brake responds, its state is reported, and releasing it with "
              "the drive passive is refused.",
              drill_brake,
              remediation="If nothing clicks, brake.mode does not match how the "
                          "brake is actually wired."),

        # ---- injected faults --------------------------------------------
        Drill("fault-bad-register", "fault",
              "Read a register that does not exist.",
              "A clean, readable error rather than a wrong number.",
              drill_fault_bad_register,
              remediation="A motor that answers for a nonexistent register "
                          "cannot be trusted on register numbers at all."),
        Drill("fault-comms", "fault",
              "Simulate the cable being pulled.",
              "Status polls report the error without raising; direct reads "
              "raise; everything recovers when the link returns.",
              drill_fault_comms_drop,
              remediation="On the telescope: check the cable, the switch, and "
                          "that the motor still has power."),
        Drill("fault-errbits", "fault",
              "Make the drive report a fault in the middle of a move.",
              "The fault is detected, the axis is halted where it was, and the "
              "reason is reported.",
              drill_fault_error_bits, needs_motion=True,
              remediation="For a real fault: read ERR_BITS, clear it with "
                          "`cli clear-errors`, and if it will not clear it is "
                          "latched and needs MacTalk or a power cycle."),
        Drill("fault-mode", "fault",
              "Simulate another client overriding the mode.",
              "The mode change is refused with a message naming MacTalk.",
              drill_fault_mode_revert,
              remediation="Disconnect MacTalk from this motor and retry."),
        Drill("fault-stuck", "fault",
              "Simulate an axis that does not move when commanded.",
              "The move times out instead of hanging, and the axis is halted.",
              drill_fault_stuck, needs_motion=True,
              remediation="For real: check for an unreleased brake, a seized "
                          "screw, or a dead encoder before commanding it again."),
        Drill("fault-word-order", "fault",
              "Swap the register words, as a wrong word_order would.",
              "The word-order check spots it rather than reporting plausible "
              "but wrong positions.",
              drill_fault_word_order,
              remediation="This is why connecting verifies the word order "
                          "instead of trusting the config."),
        Drill("fault-flaky", "fault",
              "Simulate a marginal link losing a fraction of transactions.",
              "Polls degrade to intermittent errors, never stale readings "
              "presented as live.",
              drill_fault_flaky,
              remediation="Intermittent errors in the field mean cabling or "
                          "switch trouble, not motor trouble."),

        # ---- real faults, operator required -----------------------------
        Drill("real-unplug", "real-fault",
              "Actually unplug the Ethernet cable.",
              "The loss is detected within a few polls and the link recovers "
              "after reconnecting.",
              drill_real_unplug, needs_operator=True,
              remediation="Reconnection is deliberately an explicit step: the "
                          "software will not silently resume an interrupted move."),
        Drill("real-mactalk", "real-fault",
              "Actually connect MacTalk alongside this software.",
              "Either the conflict is detected and named, or you learn that "
              "only one client can connect at a time.",
              drill_real_mactalk, needs_operator=True,
              remediation="Close MacTalk before running this software against a "
                          "motor."),
    ]


# --------------------------------------------------------------------------
# Runner
# --------------------------------------------------------------------------

class DemoRunner:
    def __init__(self, motor: JVLMotor, out: Callable[[str], None],
                 ask: Callable[[str], bool], allow_motion: bool = False,
                 range_revs: float = 2.0):
        self.motor = motor
        self.out = out
        self.injector = wrap_motor(motor)
        self.ctx = DemoContext(motor, self.injector, out, ask,
                               allow_motion, range_revs)
        self.results: List[tuple] = []

    def run(self, drills: List[Drill]) -> int:
        self.ctx.establish_band()

        self._banner(drills)

        for index, drill in enumerate(drills, start=1):
            self._header(index, len(drills), drill)
            if drill.needs_motion and not self.ctx.allow_motion:
                result = self.ctx.result().skipped(
                    "Needs motion. Re-run with --allow-motion to include it."
                )
            else:
                result = self._run_one(drill)
            self._report(drill, result)
            self.results.append((drill, result))

        self._cleanup()
        return self._summary()

    def _run_one(self, drill: Drill) -> DrillResult:
        try:
            return drill.run(self.ctx)
        except KeyboardInterrupt:
            raise
        except (ModbusError, MotorFault) as exc:
            return self.ctx.result().bad(f"{type(exc).__name__}: {exc}")
        except Exception as exc:  # noqa: BLE001 - a bad drill must not end the run
            result = self.ctx.result().bad(f"Unexpected {type(exc).__name__}: {exc}")
            result.note(traceback.format_exc().strip().splitlines()[-1])
            return result
        finally:
            # A drill that fails partway must not leave a fault armed for the
            # next one, or every later result is nonsense.
            self.injector.clear()

    # ---- output ----------------------------------------------------------

    def _banner(self, drills: List[Drill]) -> None:
        ctx = self.ctx
        self.out("=" * 74)
        self.out("  Single-motor demo and fault drills")
        self.out("=" * 74)
        self.out(f"  Motor            : {self.motor.name} @ {self.injector.inner.describe()}")
        self.out(f"  Counts per rev   : {ctx.counts_per_rev:.0f}")
        self.out(f"  Starting position: {ctx.home_counts} counts")
        self.out(f"  Bench band       : {ctx.band_low} .. {ctx.band_high} counts "
                 f"(+/- {ctx.range_revs} rev)")
        self.out(f"  Motion           : {'ALLOWED' if ctx.allow_motion else 'NOT allowed (--allow-motion to enable)'}")
        self.out(f"  Drills           : {len(drills)}")
        self.out("")
        self.out("  Everything is reported in counts, revolutions and degrees of")
        self.out("  motor shaft. No millimetres: on a bare shaft there is nothing")
        self.out("  to convert to, so no calibration is needed to run this.")

    def _header(self, index: int, total: int, drill: Drill) -> None:
        tags = []
        if drill.needs_motion:
            tags.append("MOVES THE SHAFT")
        if drill.needs_operator:
            tags.append("NEEDS YOU")
        tag = ("   [" + ", ".join(tags) + "]") if tags else ""
        self.out("")
        self.out("=" * 74)
        self.out(f"[{index}/{total}] {drill.name}   ({drill.category}){tag}")
        self.out("-" * 74)
        self.out(f"  What: {drill.summary}")
        self.out(f"  Expect: {drill.expectation}")
        self.out("")

    def _report(self, drill: Drill, result: DrillResult) -> None:
        # Notes were echoed as the drill produced them, so only the verdict is
        # left. A result built without an echo still gets its notes shown here.
        if result.echo is None:
            for note in result.notes:
                self.out(f"  {note}" if note else "")
        self.out("")
        self.out(f"  ---> {result.verdict}")
        if result.verdict == FAIL and drill.remediation:
            self.out(f"  What to do: {drill.remediation}")
        elif result.verdict == PASS and drill.remediation:
            self.out(f"  If it fails on the telescope: {drill.remediation}")

    def _cleanup(self) -> None:
        self.out("")
        self.out("=" * 74)
        self.out("  Cleaning up")
        self.out("-" * 74)
        self.injector.clear()
        try:
            if not self.motor.connected or not self._link_alive():
                self.out("  Link is down; trying to reconnect so the motor can "
                         "be left in a known state.")
                try:
                    self.motor.reconnect()
                    self.out("  Reconnected.")
                except (ModbusError, MotorFault) as exc:
                    self.out(f"  Could not reconnect: {exc}")
                    self.out("  The motor has NOT been returned or passivated. "
                             "Check it before leaving it.")
                    return
            if self.ctx.allow_motion:
                self.ctx.return_home()
                self.out(f"  Returned to {self.motor.get_position_counts()} counts "
                         f"(started at {self.ctx.home_counts}).")
            self.motor.write_register("MODE_REG", int(MotorMode.PASSIVE))
            self.out("  Motor left in Passive mode.")
        except (ModbusError, MotorFault) as exc:
            self.out(f"  Cleanup could not complete: {exc}")

    def _link_alive(self) -> bool:
        try:
            self.motor.get_mode()
            return True
        except (ModbusError, MotorFault):
            return False

    def _summary(self) -> int:
        counts = {PASS: 0, FAIL: 0, INFO: 0, SKIP: 0}
        for _, result in self.results:
            counts[result.verdict] += 1
        self.out("")
        self.out("=" * 74)
        self.out("  Summary")
        self.out("-" * 74)
        for drill, result in self.results:
            self.out(f"  {result.verdict:<5} {drill.name}")
        self.out("")
        self.out(f"  {counts[PASS]} passed, {counts[FAIL]} failed, "
                 f"{counts[INFO]} informational, {counts[SKIP]} skipped.")
        if counts[FAIL]:
            self.out("")
            self.out("  Failures are listed above with what to do about each.")
            self.out("  A failure here is a real finding: either the software is")
            self.out("  wrong, or the config does not match this motor.")
        return 1 if counts[FAIL] else 0


# --------------------------------------------------------------------------
# Entry point used by the CLI
# --------------------------------------------------------------------------

def build_motor(config_path: Optional[str], motor_name: str,
                simulate: bool) -> JVLMotor:
    """One motor from the config, with no platform and no kinematics."""
    cfg = load_config(config_path)
    actuator: ActuatorConfig = cfg.actuator(motor_name)
    if simulate:
        from .simulator import simulated_motor
        return simulated_motor(actuator, start_mm=None)
    return JVLMotor(actuator, timeout_s=cfg.modbus_timeout_s)


def select_drills(only: Optional[List[str]] = None,
                  categories: Optional[List[str]] = None,
                  include_operator: bool = True) -> List[Drill]:
    drills = all_drills()
    if only:
        wanted = {n.lower() for n in only}
        known = {d.name for d in drills}
        unknown = wanted - known
        if unknown:
            raise ValueError(
                f"Unknown drill(s): {sorted(unknown)}. Known: {sorted(known)}"
            )
        drills = [d for d in drills if d.name.lower() in wanted]
    if categories:
        wanted_cats = {c.lower() for c in categories}
        drills = [d for d in drills if d.category in wanted_cats]
    if not include_operator:
        drills = [d for d in drills if not d.needs_operator]
    return drills


__all__ = [
    "Drill", "DrillResult", "DemoContext", "DemoRunner",
    "all_drills", "select_drills", "build_motor",
    "PASS", "FAIL", "INFO", "SKIP",
]
