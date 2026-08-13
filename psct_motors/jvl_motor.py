"""
One JVL MIS23x motor, spoken to in its own vocabulary.

This layer turns Modbus words into JVL registers, and JVL registers into the
things an operator cares about: where the actuator is in millimetres, whether
it has finished moving, whether the brake is holding, and what is wrong.

Nothing above this file should need to know about word order, register
doubling, or MODE_REG values.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Dict, List, Optional, Union

from .config import ActuatorConfig
from .registers import (
    MotorMode, WordOrder, describe_errors, describe_mode, describe_status,
    int32_to_words, modbus_address, register, words_to_int32,
)
from .transport import ModbusError, PymodbusTransport, Transport


class MotorFault(RuntimeError):
    """The motor is reachable but refused to do what was asked."""


#: Registers used to determine the word order. Each holds a small,
#: non-negative configuration value on a healthy motor, so its high word is
#: zero -- which is what makes the two Modbus words distinguishable. Values
#: observed on the pSCT motor: V_SOLL 10000, A_SOLL 100, RUN_CURRENT 511,
#: STANDBY_TIME 500, STANDBY_CURRENT 128.
#:
#: Deliberately excludes position registers (which are legitimately large or
#: negative) and register 4, which reads 0x06080000 on the real motor and
#: would vote the wrong way.
WORD_ORDER_PROBE_REGISTERS = (
    "V_SOLL", "A_SOLL", "RUN_CURRENT", "STANDBY_TIME", "STANDBY_CURRENT",
)


@dataclass(frozen=True)
class WordOrderProbe:
    """What the word-order probe concluded, and what it saw.

    `detected` is None when the probe could not reach a verdict, which is a
    different thing from finding a mismatch and must not be treated as one.
    """

    detected: Optional[WordOrder]
    evidence: List[str]
    reason: str = ""

    @property
    def conclusive(self) -> bool:
        return self.detected is not None


# --------------------------------------------------------------------------
# Brake
# --------------------------------------------------------------------------

class BrakeState(str, Enum):
    ENGAGED = "engaged"       # brake is holding the axis
    RELEASED = "released"     # brake is off, the axis is free to move
    UNKNOWN = "unknown"       # not under software control, or not readable

    @property
    def label(self) -> str:
        return {
            BrakeState.ENGAGED: "ENGAGED (holding)",
            BrakeState.RELEASED: "RELEASED (free)",
            BrakeState.UNKNOWN: "UNKNOWN",
        }[self]


@dataclass(frozen=True)
class BrakeStatus:
    state: BrakeState
    #: True when the state was deduced from the motor mode rather than read
    #: back from an output. Inferred states are shown differently in the GUI
    #: so nobody mistakes a deduction for a measurement.
    inferred: bool
    detail: str = ""


# --------------------------------------------------------------------------
# Status snapshot
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class MotorStatus:
    """One consistent read of everything the UI shows for a motor."""

    name: str
    connected: bool
    position_counts: int = 0
    position_mm: float = 0.0
    target_counts: int = 0
    target_mm: float = 0.0
    velocity_raw: int = 0
    #: Where the profile generator got to (register 10). Reaches the target by
    #: construction, so it is shown beside the encoder position, never instead.
    projected_counts: int = 0
    #: Projected minus encoder (register 20). A small standing value is normal.
    follow_error: int = 0
    mode: int = 0
    mode_text: str = ""
    error_bits: int = 0
    error_text: str = ""
    brake: BrakeStatus = BrakeStatus(BrakeState.UNKNOWN, True)
    in_position: bool = False
    #: Populated when the read itself failed, in which case the numeric fields
    #: are stale/zero and must not be displayed as live values.
    comms_error: str = ""

    @property
    def healthy(self) -> bool:
        return self.connected and not self.comms_error and self.error_bits == 0

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "connected": self.connected,
            "position_counts": self.position_counts,
            "position_mm": self.position_mm,
            "target_counts": self.target_counts,
            "target_mm": self.target_mm,
            "velocity_raw": self.velocity_raw,
            "projected_counts": self.projected_counts,
            "follow_error": self.follow_error,
            "mode": self.mode,
            "mode_text": self.mode_text,
            "error_bits": self.error_bits,
            "error_text": self.error_text,
            "brake": self.brake.state.value,
            "brake_inferred": self.brake.inferred,
            "in_position": self.in_position,
            "comms_error": self.comms_error,
            "healthy": self.healthy,
        }


# --------------------------------------------------------------------------
# The motor
# --------------------------------------------------------------------------

class JVLMotor:
    """A single JVL MIS23x actuator on the focal plane.

    All register access is serialised by an internal lock, so the GUI's
    background poller and an operator's button press cannot interleave two
    halves of a 32-bit transaction.
    """

    def __init__(self, cfg: ActuatorConfig, transport: Optional[Transport] = None,
                 timeout_s: float = 2.0, retries: int = 1,
                 logger: Optional[Callable[[str], None]] = None):
        self.cfg = cfg
        self.word_order = WordOrder.parse(cfg.word_order)
        self._transport = transport or PymodbusTransport(
            cfg.ip, cfg.port, cfg.unit_id, timeout_s=timeout_s, retries=retries
        )
        self._lock = threading.RLock()
        self._log = logger or (lambda msg: None)
        self._connected = False
        self._warned_no_encoder = False
        #: Set by stop()/abort so a move loop waiting for in-position gives up
        #: instead of waiting out its full timeout on a motor that was halted.
        self._cancel = threading.Event()

    # ------------------------------------------------------------ properties

    @property
    def name(self) -> str:
        return self.cfg.name

    @property
    def connected(self) -> bool:
        return self._connected and self._transport.is_open()

    def describe(self) -> str:
        return f"{self.name} @ {self._transport.describe()}"

    # ------------------------------------------------------------ lifecycle

    def connect(self, verify_word_order: bool = True) -> None:
        with self._lock:
            if not self._transport.connect():
                raise ModbusError(
                    f"Could not open a Modbus TCP connection to {self.describe()}. "
                    "Check the IP address, that the motor is powered, and that "
                    "nothing else (MacTalk) holds the connection."
                )
            self._connected = True
            self._cancel.clear()
        if verify_word_order:
            self._check_word_order()

    def disconnect(self) -> None:
        with self._lock:
            self._connected = False
            self._transport.close()

    def reconnect(self, verify_word_order: bool = False) -> None:
        """Rebuild the connection after the link has failed.

        Not the same as calling connect() again. When a TCP connection is
        reset -- a pulled cable, a power-cycled switch, the motor rebooting --
        the client is left holding a socket it still believes is usable.
        connect() on it then succeeds while every transaction that follows
        fails, so a retry loop spins until it gives up on a link that has
        actually come back. This tears the connection down first.
        """
        with self._lock:
            self._connected = False
            self._transport.reconnect()
            self._connected = True
            self._cancel.clear()
        if verify_word_order:
            self._check_word_order()

    def __enter__(self) -> "JVLMotor":
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        self.disconnect()

    # -------------------------------------------------------- register access

    def _resolve(self, reg: Union[int, str]) -> int:
        if isinstance(reg, str):
            return register(reg).number
        return int(reg)

    def read_register(self, reg: Union[int, str], signed: bool = True) -> int:
        """Read one JVL register (always 2 Modbus words wide)."""
        number = self._resolve(reg)
        with self._lock:
            words = self._transport.read_holding(modbus_address(number), 2)
        return words_to_int32(words, self.word_order, signed=signed)

    def write_register(self, reg: Union[int, str], value: int) -> None:
        """Write one JVL register.

        Always two words: a JVL register is a 32-bit slot even when the value
        it holds is 16-bit, and a single-word write to MODE_REG is rejected by
        the motor.
        """
        number = self._resolve(reg)
        words = int32_to_words(value, self.word_order)
        with self._lock:
            self._transport.write_holding(modbus_address(number), words)

    def read_registers(self, regs: List[Union[int, str]]) -> Dict[str, int]:
        """Read several registers, keyed by the name or number given."""
        out: Dict[str, int] = {}
        for reg in regs:
            out[str(reg)] = self.read_register(reg)
        return out

    # ------------------------------------------------------------ word order

    def probe_word_order(self) -> WordOrderProbe:
        """Work out the word order empirically, without moving the motor.

        How it works
        ------------
        Several JVL registers hold small, non-negative configuration values --
        currents, ramp times, velocity limits. Any value below 65536 has a
        high word of zero, so of the two 16-bit Modbus words that come back,
        the one that is consistently zero *is* the high word. That fixes the
        order, and it does so from the structure of the data rather than from
        a guess about what any particular value should be.

        Several registers are used and their votes compared, because any one
        of them might legitimately be zero (an idle MODE_REG) or unexpectedly
        large, and either would make a single-register test useless.

        Why not just read the firmware version
        --------------------------------------
        An earlier version of this compared a firmware-version read against a
        "looks like a version number" range. On the real pSCT motor register 1
        reads 540777, which needs 20 bits and so cannot be the 16-bit version
        field it was assumed to be -- probably that register is not
        PROG_VERSION on this firmware at all. The check then reported it could
        not determine the word order on a motor whose word order was provably
        correct. Structure beats plausibility.

        Returns a probe result rather than raising, because "I could not tell"
        and "the config is wrong" call for very different responses and the
        caller has to be able to tell them apart.
        """
        evidence: List[str] = []
        votes = {WordOrder.LOW_HIGH: 0, WordOrder.HIGH_LOW: 0}

        for name in WORD_ORDER_PROBE_REGISTERS:
            try:
                number = register(name).number
                with self._lock:
                    words = self._transport.read_holding(modbus_address(number), 2)
            except (ModbusError, MotorFault, KeyError):
                continue                       # absent on this firmware; skip
            first, second = words[0], words[1]
            if second == 0 and first != 0:
                votes[WordOrder.LOW_HIGH] += 1
                evidence.append(f"{name}: [0x{first:04X}, 0x{second:04X}] "
                                "-- second word zero, so it is the high word")
            elif first == 0 and second != 0:
                votes[WordOrder.HIGH_LOW] += 1
                evidence.append(f"{name}: [0x{first:04X}, 0x{second:04X}] "
                                "-- first word zero, so it is the high word")
            # Both zero, or both non-zero: carries no information either way.

        low, high = votes[WordOrder.LOW_HIGH], votes[WordOrder.HIGH_LOW]
        if low and high:
            return WordOrderProbe(
                None, evidence,
                f"Contradictory evidence ({low} for Low-High, {high} for "
                "High-Low). That usually means one of the probe registers is "
                "not what this software thinks it is on this firmware. Compare "
                "the register numbers against MacTalk.",
            )
        if not low and not high:
            return WordOrderProbe(
                None, evidence,
                "No register carried usable evidence -- every probe register "
                "read as zero, or none had a zero word. Nothing is necessarily "
                "wrong; there was just nothing to go on.",
            )
        detected = WordOrder.LOW_HIGH if low else WordOrder.HIGH_LOW
        return WordOrderProbe(detected, evidence, "")

    def detect_word_order(self) -> WordOrder:
        """The word order, raising if it cannot be determined."""
        probe = self.probe_word_order()
        if probe.detected is None:
            raise MotorFault(f"{self.name}: could not detect word order. {probe.reason}")
        return probe.detected

    def _check_word_order(self) -> None:
        """Refuse to run with a demonstrably wrong word order.

        Only a positive contradiction is fatal. An inconclusive probe is
        logged and allowed through: refusing to connect because a check could
        not reach a verdict would make a diagnostic tool the reason you cannot
        diagnose anything.
        """
        try:
            probe = self.probe_word_order()
        except (ModbusError, MotorFault) as exc:
            self._log(f"{self.name}: word-order check skipped: {exc}")
            return
        if probe.detected is None:
            self._log(f"{self.name}: word order not confirmed -- {probe.reason}")
            return
        if probe.detected is not self.word_order:
            raise MotorFault(
                f"{self.name}: configured word order is {self.word_order.value}, but "
                f"the motor's registers only make sense as {probe.detected.value}. "
                "Every position you read would be wrong. Fix 'word_order' for this "
                "actuator in the config.\n  Evidence:\n    "
                + "\n    ".join(probe.evidence)
            )

    # ----------------------------------------------------------------- mode

    def get_mode(self) -> int:
        return self.read_register("MODE_REG")

    def set_mode(self, mode: MotorMode, verify: bool = True,
                 settle_s: float = 0.2) -> None:
        """Set MODE_REG and, by default, read it back.

        The read-back matters: if MacTalk is still connected it will fight for
        control and quietly put the mode back, and the symptom is a move
        command that is accepted and then does nothing at all.
        """
        self.write_register("MODE_REG", int(mode))
        if not verify:
            return
        time.sleep(settle_s)
        actual = self.get_mode()
        if actual != int(mode):
            raise MotorFault(
                f"{self.name}: asked for {describe_mode(int(mode))} but MODE_REG "
                f"reads back {describe_mode(actual)}. Something is overriding the "
                "mode -- the usual cause is MacTalk still being connected to this "
                "motor. Disconnect it there and try again."
            )

    def ensure_position_mode(self) -> None:
        """Make sure the motor is in Position mode, setting it only if needed."""
        if self.get_mode() == int(MotorMode.POSITION):
            return
        self.set_mode(MotorMode.POSITION)
        self._log(f"{self.name}: switched to Position mode.")

    # ------------------------------------------------------------- position

    def get_position_counts(self) -> int:
        """Where the shaft actually is, from the encoder.

        Register 16 ('Actual Encoder Position'), not register 10 ('Projected
        Position'). Register 10 is the profile generator's output: it arrives
        at the requested position by construction, whether or not the shaft
        followed, so checking arrival against it can never fail. On the real
        pSCT motor register 10 read 204800 -- exactly the requested position --
        while the encoder read 204569, a standing following error of 231 counts
        that register 10 gave no hint of.

        Falls back to the projected position if the encoder register cannot be
        read, so an open-loop or differently-configured motor still works, but
        says so once.
        """
        try:
            return self.read_register("P_ENCODER")
        except ModbusError:
            if not self._warned_no_encoder:
                self._warned_no_encoder = True
                self._log(
                    f"{self.name}: Actual Encoder Position (register 16) could "
                    "not be read; falling back to Projected Position (register "
                    "10). Positions will show where the profile generator got "
                    "to, not where the shaft is."
                )
            return self.read_register("P_PROJECTED")

    def get_projected_position_counts(self) -> int:
        """The profile generator's output -- what to freeze on a stop."""
        return self.read_register("P_PROJECTED")

    def get_follow_error(self) -> int:
        """Projected minus actual, straight from the motor (register 20)."""
        return self.read_register("FLWERR")

    def get_position_mm(self) -> float:
        return self.cfg.counts_to_mm(self.get_position_counts())

    def get_target_counts(self) -> int:
        return self.read_register("P_SOLL")

    def get_target_mm(self) -> float:
        return self.cfg.counts_to_mm(self.get_target_counts())

    def set_velocity(self, velocity_raw: int) -> None:
        v = max(1, min(32767, int(round(velocity_raw))))
        self.write_register("V_SOLL", v)

    def set_acceleration(self, acceleration_raw: int) -> None:
        self.write_register("A_SOLL", max(1, min(32767, int(round(acceleration_raw)))))

    def command_position_counts(self, counts: int) -> None:
        """Write P_SOLL. Assumes mode and brake have already been dealt with."""
        self.write_register("P_SOLL", int(counts))

    def command_position_mm(self, mm: float) -> None:
        self.command_position_counts(self.cfg.mm_to_counts(mm))

    def check_travel_limit(self, mm: float) -> None:
        """Raise if `mm` is outside this actuator's configured soft limits."""
        lo, hi = self.cfg.min_travel_mm, self.cfg.max_travel_mm
        if not (lo <= mm <= hi):
            raise MotorFault(
                f"{self.name}: target {mm:.4f} mm is outside the soft travel "
                f"limits {lo:.3f}..{hi:.3f} mm. Nothing was commanded."
            )

    # ------------------------------------------------------------ motion end

    def is_in_position(self, tol_mm: Optional[float] = None) -> bool:
        """True when the move has finished AND the shaft is really there.

        Three conditions, because each catches a different lie:

        1. The profile generator has reached the target. Register 10 is what
           says so, and on its own it says nothing about the shaft -- it
           arrives by construction.
        2. The following error is inside its window. This is the condition
           register 10 cannot express: profile finished, shaft 231 counts
           short. Without it, a stalled axis reports a completed move.
        3. Velocity has settled, so a sample taken while coasting through the
           target does not count as arrival.
        """
        tol_counts = self._tolerance_counts(tol_mm)
        try:
            projected = self.get_projected_position_counts()
            target = self.get_target_counts()
        except ModbusError:
            return False
        if abs(projected - target) > tol_counts:
            return False

        try:
            follow_error = self.get_follow_error()
            if abs(follow_error) > self.cfg.follow_error_window_counts:
                return False
        except ModbusError:
            pass          # not fatal; the other two conditions still apply

        try:
            return abs(self.read_register("V_IST")) <= 1
        except ModbusError:
            self._log(
                f"{self.name}: Actual Velocity unreadable, using position-only "
                "settle test."
            )
            return True

    def _tolerance_counts(self, tol_mm: Optional[float] = None) -> float:
        tol = self.cfg.in_position_tol_mm if tol_mm is None else tol_mm
        try:
            return abs(tol * self.cfg.resolved_counts_per_mm)
        except ValueError:
            # No usable millimetre scale (a bare motor on a bench). Fall back
            # to the following-error window, which is already in counts.
            return float(self.cfg.follow_error_window_counts)

    def wait_for_in_position(self, timeout_s: Optional[float] = None,
                             poll_s: float = 0.1,
                             stable_polls: int = 3,
                             halt_on_failure: bool = True) -> bool:
        """Block until the move finishes. Returns False on timeout or cancel.

        `stable_polls` consecutive in-position reads are required, so a single
        sample taken as the axis coasts through its target does not count as
        arrival.

        With `halt_on_failure` (the default), a fault or a timeout halts this
        axis before reporting. Noticing that something has gone wrong and then
        leaving the drive commanded to a target it is not reaching is the worst
        of both worlds: the operator has been told the move failed while the
        motor is still trying to complete it.
        """
        timeout = self.cfg.move_timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout
        stable = 0
        while time.monotonic() < deadline:
            if self._cancel.is_set():
                return False
            errors = self.get_errors()
            if errors:
                if halt_on_failure:
                    self.stop_quietly("an error was reported during the move")
                raise MotorFault(
                    f"{self.name}: motor reported an error during the move -- "
                    f"{describe_errors(errors)}"
                )
            if self.is_in_position():
                stable += 1
                if stable >= stable_polls:
                    return True
            else:
                stable = 0
            time.sleep(poll_s)
        if halt_on_failure:
            self.stop_quietly(f"the move did not complete within {timeout:.0f} s")
        return False

    # ------------------------------------------------------------- stopping

    def stop(self) -> None:
        """Controlled stop: decelerate and hold where you are.

        This is deliberately *not* MODE_REG=0. Going passive cuts the drive,
        which on a loaded actuator means the axis is held by nothing but the
        screw's friction and whatever the brake happens to be doing. Writing
        the current position as the new target makes the motor ramp down on
        its own deceleration profile and then actively hold. Use `passivate()`
        when you genuinely want the drive off.
        """
        self._cancel.set()
        # The PROJECTED position, deliberately. Writing the encoder reading
        # instead would command a small step equal to the standing following
        # error -- a stop that produces motion. The profile output is where the
        # generator currently is, so freezing it is what actually stops.
        frozen = self.get_projected_position_counts()
        self.command_position_counts(frozen)
        self._log(f"{self.name}: STOP -- holding at {frozen} counts.")

    def stop_quietly(self, reason: str) -> bool:
        """Stop, swallowing any failure. For use on an error path.

        The situation this exists for is a fault that is itself a comms
        failure: the stop will not get through either, and letting that second
        failure replace the first would hide the thing that actually went
        wrong. Returns whether the stop was delivered.
        """
        try:
            self.stop()
            self._log(f"{self.name}: halted because {reason}.")
            return True
        except (ModbusError, MotorFault) as exc:
            self._log(
                f"{self.name}: tried to halt because {reason}, but the stop "
                f"could not be delivered either ({exc}). If the motor is still "
                "powered and moving, use MacTalk or remove drive power."
            )
            return False

    def passivate(self, engage_brake_first: bool = True) -> None:
        """Drive off. Engages the brake first when the brake is controllable."""
        self._cancel.set()
        if engage_brake_first and self.cfg.brake.mode == "output":
            try:
                self.engage_brake()
            except (ModbusError, MotorFault) as exc:
                self._log(f"{self.name}: could not engage brake before passivating: {exc}")
        self.write_register("MODE_REG", int(MotorMode.PASSIVE))
        self._log(f"{self.name}: PASSIVE -- drive output off.")

    def clear_cancel(self) -> None:
        self._cancel.clear()

    @property
    def cancelled(self) -> bool:
        return self._cancel.is_set()

    # ---------------------------------------------------------------- errors

    def get_errors(self) -> int:
        return self.read_register("ERR_BITS", signed=False)

    def error_text(self) -> str:
        return describe_errors(self.get_errors())

    def clear_errors(self) -> int:
        """Best-effort error clear, returning ERR_BITS afterwards.

        Writing 0 to ERR_BITS clears the non-latching bits on JVL drives. A
        latched or hardware fault survives this and needs MacTalk's own clear
        or a power cycle; the returned value tells you which happened.
        """
        try:
            self.write_register("ERR_BITS", 0)
        except ModbusError as exc:
            self._log(f"{self.name}: direct ERR_BITS clear rejected ({exc}).")
        time.sleep(0.2)
        return self.get_errors()

    # ----------------------------------------------------------------- brake

    def read_brake_output_assignment(self) -> Optional[int]:
        """Which digital output the drive drives the brake from (register 179).

        0 means none is assigned, so nothing software does to the outputs will
        move a brake. Returns None if the register cannot be read.
        """
        try:
            return self.read_register("BRAKE_OUTPUT")
        except (ModbusError, MotorFault):
            return None

    def check_brake_configuration(self) -> str:
        """Compare the configured brake mode against what the drive is set up
        to do. Returns "" when they agree, otherwise an explanation."""
        assignment = self.read_brake_output_assignment()
        if assignment is None:
            return ""
        mode = self.cfg.brake.mode
        if assignment == 0 and mode == "output":
            return (
                f"{self.name}: brake.mode is 'output', but the drive's Brake "
                "Output (register 179) is 0, meaning no digital output is "
                "assigned to the brake. Toggling an output will not move a "
                "brake. Assign one in MacTalk, or set brake.mode to 'none'."
            )
        if assignment == 0 and mode == "auto":
            return (
                f"{self.name}: brake.mode is 'auto', but the drive's Brake "
                "Output (register 179) is 0, so the drive is not driving a "
                "brake from any output. The inferred brake state would be a "
                "guess with nothing behind it -- set brake.mode to 'none' "
                "unless the brake is wired some other way."
            )
        if assignment != 0 and mode == "none":
            return (
                f"{self.name}: the drive has Brake Output (register 179) set to "
                f"{assignment}, so it IS driving a brake, but brake.mode is "
                "'none' so this software will not show or control it."
            )
        return ""

    def get_brake_status(self) -> BrakeStatus:
        cfg = self.cfg.brake
        if cfg.mode == "none":
            return BrakeStatus(
                BrakeState.UNKNOWN, inferred=True,
                detail="No brake configured for this actuator.",
            )
        if cfg.mode == "auto":
            # The drive releases the brake whenever it is enabled, so the mode
            # register tells us the brake state -- by deduction, not by
            # measurement, which is why `inferred` is True.
            mode = self.get_mode()
            engaged = mode == int(MotorMode.PASSIVE)
            return BrakeStatus(
                BrakeState.ENGAGED if engaged else BrakeState.RELEASED,
                inferred=True,
                detail=f"Inferred from {describe_mode(mode)}.",
            )
        # mode == "output": read the output back.
        outputs = self.read_register(cfg.output_register, signed=False)
        energized = bool(outputs & (1 << cfg.output_bit))
        released = energized if cfg.energized_releases else not energized
        return BrakeStatus(
            BrakeState.RELEASED if released else BrakeState.ENGAGED,
            inferred=False,
            detail=(
                f"Output register {cfg.output_register} bit {cfg.output_bit} = "
                f"{int(energized)}."
            ),
        )

    def _set_brake_output(self, release: bool) -> None:
        cfg = self.cfg.brake
        outputs = self.read_register(cfg.output_register, signed=False)
        energize = release if cfg.energized_releases else not release
        mask = 1 << cfg.output_bit
        new = (outputs | mask) if energize else (outputs & ~mask)
        if new != outputs:
            self.write_register(cfg.output_register, new & 0xFFFFFFFF)
        time.sleep(cfg.settle_s)

    def release_brake(self, force: bool = False) -> None:
        """Release the holding brake.

        Refuses while the drive is passive unless forced. On a loaded actuator,
        releasing the brake with no holding torque behind it lets the load
        drive the screw -- exactly the situation the brake exists to prevent.
        """
        cfg = self.cfg.brake
        if cfg.mode == "none":
            raise MotorFault(
                f"{self.name}: no brake is configured, so it cannot be released. "
                "Set brake.mode in the config if this actuator has one."
            )
        if cfg.mode == "auto":
            raise MotorFault(
                f"{self.name}: the brake is in 'auto' mode, meaning the drive "
                "controls it. Enable the drive (Position mode) to release it."
            )
        if not force and self.get_mode() == int(MotorMode.PASSIVE):
            raise MotorFault(
                f"{self.name}: refusing to release the brake while the drive is "
                "passive -- nothing would be holding the actuator. Enable "
                "Position mode first, or pass force=True if you really mean it."
            )
        self._set_brake_output(release=True)
        self._log(f"{self.name}: brake RELEASED.")

    def engage_brake(self) -> None:
        cfg = self.cfg.brake
        if cfg.mode == "none":
            raise MotorFault(f"{self.name}: no brake is configured.")
        if cfg.mode == "auto":
            raise MotorFault(
                f"{self.name}: the brake is in 'auto' mode; it engages when the "
                "drive goes passive. Use Passivate/Emergency stop."
            )
        self._set_brake_output(release=False)
        self._log(f"{self.name}: brake ENGAGED.")

    @property
    def brake_is_software_controlled(self) -> bool:
        return self.cfg.brake.mode == "output"

    # ---------------------------------------------------------------- zeroing

    def set_zero_here(self) -> int:
        """Define the current position as 0 mm of travel.

        Nothing is written to the motor: the offset lives in this software's
        config as `zero_counts`. That is deliberate. The alternative, writing
        the motor's own position registers, changes state inside the drive
        that MacTalk and any other client would then disagree with. MacTalk's
        register list has no documented 'set position' register on this
        firmware anyway -- register 4, the obvious candidate, is unnamed even
        by JVL.
        """
        counts = self.get_position_counts()
        self.cfg.zero_counts = counts
        self._log(f"{self.name}: zero set at {counts} counts.")
        return counts

    # ---------------------------------------------------------------- status

    def read_status(self) -> MotorStatus:
        """One snapshot for the UI. Never raises; comms failures are reported
        in the returned object so a poll loop cannot die on a dropped packet."""
        if not self.connected:
            return MotorStatus(name=self.name, connected=False,
                               comms_error="not connected")
        try:
            counts = self.get_position_counts()
            target = self.get_target_counts()
            mode = self.get_mode()
            errors = self.get_errors()
            try:
                velocity = self.read_register("V_IST")
            except ModbusError:
                velocity = 0
            try:
                projected = self.get_projected_position_counts()
            except ModbusError:
                projected = counts
            try:
                follow_error = self.get_follow_error()
            except ModbusError:
                follow_error = 0
            brake = self.get_brake_status()
            position_mm = self.cfg.counts_to_mm(counts)
            target_mm = self.cfg.counts_to_mm(target)
            return MotorStatus(
                name=self.name,
                connected=True,
                position_counts=counts,
                position_mm=position_mm,
                target_counts=target,
                target_mm=target_mm,
                velocity_raw=velocity,
                projected_counts=projected,
                follow_error=follow_error,
                mode=mode,
                mode_text=describe_mode(mode),
                error_bits=errors,
                error_text=describe_errors(errors),
                brake=brake,
                in_position=(
                    abs(projected - target) <= self._tolerance_counts()
                    and abs(follow_error) <= self.cfg.follow_error_window_counts
                    and abs(velocity) <= 1
                ),
            )
        except (ModbusError, MotorFault) as exc:
            return MotorStatus(name=self.name, connected=True, comms_error=str(exc))

    def read_diagnostics(self) -> Dict[str, str]:
        """Everything readable, for the CLI's verify-registers command."""
        from .registers import REGISTERS
        out: Dict[str, str] = {}
        for reg in REGISTERS:
            try:
                value = self.read_register(reg.number, signed=reg.signed)
            except (ModbusError, MotorFault) as exc:
                out[reg.name] = f"<unreadable: {exc}>"
                continue
            text = str(value)
            if reg.name == "ERR_BITS":
                text = describe_errors(value)
            elif reg.name == "STATUSBITS":
                text = describe_status(value)
            elif reg.name == "MODE_REG":
                text = f"{value} ({describe_mode(value)})"
            out[reg.name] = text
        return out


__all__ = ["JVLMotor", "MotorStatus", "BrakeState", "BrakeStatus", "MotorFault"]
