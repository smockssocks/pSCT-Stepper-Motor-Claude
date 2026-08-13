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
                 timeout_s: float = 2.0,
                 logger: Optional[Callable[[str], None]] = None):
        self.cfg = cfg
        self.word_order = WordOrder.parse(cfg.word_order)
        self._transport = transport or PymodbusTransport(
            cfg.ip, cfg.port, cfg.unit_id, timeout_s=timeout_s
        )
        self._lock = threading.RLock()
        self._log = logger or (lambda msg: None)
        self._connected = False
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

    def detect_word_order(self) -> WordOrder:
        """Work out the word order empirically, without moving the motor.

        PROG_VERSION is a small positive firmware version number. Read with
        the wrong word order it comes back as a huge value, because what
        should be the zero high word lands in the low half. Whichever order
        yields a plausible version number is the right one.
        """
        number = register("PROG_VERSION").number
        with self._lock:
            words = self._transport.read_holding(modbus_address(number), 2)
        candidates = {}
        for order in (WordOrder.LOW_HIGH, WordOrder.HIGH_LOW):
            candidates[order] = words_to_int32(words, order, signed=False)
        plausible = [o for o, v in candidates.items() if 0 < v < 100000]
        if len(plausible) == 1:
            return plausible[0]
        if not plausible:
            raise MotorFault(
                f"{self.name}: could not detect word order. PROG_VERSION read as "
                f"{candidates} in both orders, neither of which looks like a "
                "firmware version. Check that the register numbers and the unit "
                "id are right for this motor."
            )
        # Both plausible happens only when the raw words are symmetric, e.g.
        # both zero. Nothing to distinguish them, so keep what is configured.
        return self.word_order

    def _check_word_order(self) -> None:
        try:
            detected = self.detect_word_order()
        except (ModbusError, MotorFault) as exc:
            self._log(f"{self.name}: word-order check skipped: {exc}")
            return
        if detected is not self.word_order:
            raise MotorFault(
                f"{self.name}: configured word order is {self.word_order.value} but "
                f"the motor's firmware version register only makes sense as "
                f"{detected.value}. Every position you read would be wrong. Fix "
                f"'word_order' in the config for this actuator."
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
        return self.read_register("P_IST")

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
        """True when the actuator has reached its target and stopped.

        Both halves matter. Position alone can pass momentarily as the axis
        overshoots through the target; velocity alone can pass before the move
        has been picked up at all. Requiring both avoids each failure.
        """
        tol = self.cfg.in_position_tol_mm if tol_mm is None else tol_mm
        try:
            error_mm = abs(self.get_position_mm() - self.get_target_mm())
        except ModbusError:
            return False
        if error_mm > tol:
            return False
        try:
            return abs(self.read_register("V_IST")) <= 1
        except ModbusError:
            # V_IST is DOCUMENTED rather than CONFIRMED on this hardware. If
            # it cannot be read, fall back to the position test alone rather
            # than blocking the move -- but say so, once.
            self._log(
                f"{self.name}: V_IST unreadable, using position-only settle test."
            )
            return True

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
        actual = self.get_position_counts()
        self.command_position_counts(actual)
        self._log(f"{self.name}: STOP -- holding at {actual} counts.")

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
        P_NEW to renumber the motor's own position, changes state inside the
        drive that MacTalk and any other client would then disagree with, and
        P_NEW is only VERIFY-level confidence here anyway.
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
                mode=mode,
                mode_text=describe_mode(mode),
                error_bits=errors,
                error_text=describe_errors(errors),
                brake=brake,
                in_position=(abs(position_mm - target_mm) <= self.cfg.in_position_tol_mm
                             and abs(velocity) <= 1),
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
