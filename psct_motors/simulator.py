"""
A fake JVL motor that behaves like the real one over the same interface.

This exists so the whole stack -- kinematics, coordinated moves, limit checks,
the GUI, the LabVIEW bridge -- can be exercised with no hardware attached, and
so you can rehearse a procedure at your desk before running it on the
telescope.

It substitutes at the transport boundary, so the code under test is the real
driver: the same register doubling, the same word-order handling, the same
32-bit packing. Only the wire is fake.

What it models
--------------
* The JVL register file, addressed the way the real motor addresses it.
* Position mode: P_IST ramps towards P_SOLL at a rate set by V_SOLL, and
  V_IST reports non-zero while it moves.
* Passive mode: the axis does not move, and if `gravity_counts_per_s` is set
  it drifts, which is how you check that your brake interlocks actually work.
* A digital output register, so brake mode "output" can be tested.
* Optional faults: `inject_error()` sets ERR_BITS, `set_offline()` makes every
  transaction fail the way a pulled cable does.

What it does not model
----------------------
Acceleration ramps, following error, current limits, or thermal behaviour.
It is a control-flow test double, not a mechanical simulation.
"""

from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional

from .config import ActuatorConfig
from .jvl_motor import JVLMotor
from .registers import MotorMode, WordOrder, int32_to_words, words_to_int32
from .transport import ModbusError


class SimulatedJVLTransport:
    """Transport-compatible stand-in for one JVL motor."""

    #: Counts per second per unit of V_SOLL. Chosen so the default V_SOLL of
    #: 1000 gives a visible-but-not-instant move on a 409600 count/rev motor.
    COUNTS_PER_SECOND_PER_VSOLL = 100.0

    def __init__(self, name: str = "SIM", word_order: WordOrder = WordOrder.LOW_HIGH,
                 start_counts: int = 0, firmware_version: int = 1030,
                 gravity_counts_per_s: float = 0.0):
        self.name = name
        self.word_order = word_order
        self.gravity_counts_per_s = gravity_counts_per_s
        self._lock = threading.RLock()
        self._open = False
        self._offline = False
        self._last_update = time.monotonic()
        self._position = float(start_counts)

        # Values chosen to match what the real pSCT motor returns, so the
        # simulator is a fair rehearsal rather than an idealised one. In
        # particular registers 4 and 25 hold the large, un-position-like and
        # un-status-like values actually observed on the hardware, so anything
        # that tries to interpret them meets the same difficulty here.
        self.registers: Dict[int, int] = {
            1: firmware_version,      # register 1 -- identity unconfirmed
            2: int(MotorMode.PASSIVE),
            3: int(start_counts),     # P_SOLL
            4: 0x06080000,            # as observed; not position-shaped
            5: 1000,                  # V_SOLL
            6: 100,                   # A_SOLL
            7: 511,                   # RUN_CURRENT
            8: 500,                   # STANDBY_TIME
            9: 128,                   # STANDBY_CURRENT
            10: int(start_counts),    # P_IST
            12: 0,                    # V_IST
            19: 0,                    # outputs (brake lives here in output mode)
            20: 0,                    # FLWERR
            25: 0x8A476C14,           # as observed on a passive, idle motor
            35: 0,                    # ERR_BITS
            36: 0,                    # WARN_BITS
            38: -100000,              # P_HOME
        }

    # ------------------------------------------------------------- test hooks

    def set_offline(self, offline: bool = True) -> None:
        """Simulate a pulled cable: every transaction fails."""
        self._offline = offline

    def inject_error(self, error_bits: int) -> None:
        with self._lock:
            self.registers[35] = int(error_bits)

    @property
    def position_counts(self) -> int:
        with self._lock:
            self._advance()
            return int(round(self._position))

    # -------------------------------------------------------------- physics

    def _advance(self) -> None:
        """Move the simulated axis forward to now."""
        now = time.monotonic()
        dt = now - self._last_update
        self._last_update = now
        if dt <= 0:
            return

        mode = self.registers.get(2, 0)
        if mode == int(MotorMode.POSITION) and self.registers.get(35, 0) == 0:
            target = float(self.registers.get(3, 0))
            speed = max(1.0, abs(self.registers.get(5, 1000))) * self.COUNTS_PER_SECOND_PER_VSOLL
            delta = target - self._position
            step = speed * dt
            if abs(delta) <= step:
                self._position = target
                self.registers[12] = 0
            else:
                direction = 1.0 if delta > 0 else -1.0
                self._position += direction * step
                self.registers[12] = int(direction * speed / self.COUNTS_PER_SECOND_PER_VSOLL)
        else:
            # Drive off. If a load is configured and the brake output is not
            # holding, the axis creeps -- the failure mode the interlocks exist
            # to prevent.
            self.registers[12] = 0
            if self.gravity_counts_per_s and not self._brake_output_engaged():
                self._position -= self.gravity_counts_per_s * dt

        self.registers[10] = int(round(self._position))

    def _brake_output_engaged(self) -> bool:
        """True when output bit 0 is low, i.e. a fail-safe brake is holding."""
        return not bool(self.registers.get(19, 0) & 1)

    # ------------------------------------------------------------- transport

    def connect(self) -> bool:
        if self._offline:
            return False
        self._open = True
        self._last_update = time.monotonic()
        return True

    def close(self) -> None:
        self._open = False

    def reconnect(self) -> bool:
        self.close()
        return self.connect()

    def is_open(self) -> bool:
        return self._open and not self._offline

    def describe(self) -> str:
        return f"simulated://{self.name}"

    def _check(self) -> None:
        if self._offline:
            raise ModbusError(f"{self.name}: simulated motor is offline")
        if not self._open:
            raise ModbusError(f"{self.name}: not connected")

    def read_holding(self, address: int, count: int) -> List[int]:
        with self._lock:
            self._check()
            if count != 2:
                raise ModbusError(
                    f"{self.name}: JVL registers are 32-bit; expected a 2-word "
                    f"read, got {count}"
                )
            if address % 2:
                raise ModbusError(
                    f"{self.name}: address {address} is not on a JVL register "
                    "boundary (JVL register numbers map to even Modbus addresses)"
                )
            self._advance()
            number = address // 2
            if number not in self.registers:
                raise ModbusError(
                    f"{self.name}: JVL register {number} does not exist on this motor"
                )
            return int32_to_words(self.registers[number], self.word_order)

    def write_holding(self, address: int, values: List[int]) -> None:
        with self._lock:
            self._check()
            if len(values) != 2:
                raise ModbusError(
                    f"{self.name}: JVL registers are 32-bit; expected a 2-word "
                    f"write, got {len(values)}"
                )
            if address % 2:
                raise ModbusError(
                    f"{self.name}: address {address} is not on a JVL register boundary"
                )
            self._advance()
            number = address // 2
            if number not in self.registers:
                raise ModbusError(
                    f"{self.name}: JVL register {number} does not exist on this motor"
                )
            if number in (1, 10, 12, 20):  # read-only on the real hardware
                raise ModbusError(
                    f"{self.name}: JVL register {number} is read-only"
                )
            self.registers[number] = words_to_int32(values, self.word_order, signed=True)


def simulated_motor(cfg: ActuatorConfig, start_mm: Optional[float] = None,
                    **kwargs) -> JVLMotor:
    """A JVLMotor backed by a simulated transport, positioned at `start_mm`."""
    start_counts = cfg.mm_to_counts(start_mm) if start_mm is not None else cfg.zero_counts
    transport = SimulatedJVLTransport(
        name=cfg.name,
        word_order=WordOrder.parse(cfg.word_order),
        start_counts=start_counts,
        **kwargs,
    )
    motor = JVLMotor(cfg, transport=transport)
    return motor


__all__ = ["SimulatedJVLTransport", "simulated_motor"]
