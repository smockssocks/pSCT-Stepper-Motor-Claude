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
* Position mode: the projected position ramps towards the requested
  position at a rate set by Max Velocity, the encoder follows it with a
  realistic standing lag, and Actual Velocity reports non-zero while it
  moves.
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
                 start_counts: int = 0, firmware_version: int = 540777,
                 gravity_counts_per_s: float = 0.0,
                 follow_error_counts: int = 0,
                 hard_stop_low: Optional[int] = None,
                 hard_stop_high: Optional[int] = None):
        self.name = name
        self.word_order = word_order
        self.gravity_counts_per_s = gravity_counts_per_s
        #: Standing lag of the encoder behind the profile output, in counts.
        #:
        #: Zero by default, so the simulator is an ideal motor and a test that
        #: asserts an exact position is testing the thing it means to. Set it
        #: to model a real one -- the pSCT motor sits at 231 counts when
        #: settled -- which is how the in-position logic's following-error
        #: condition gets exercised.
        self.follow_error_counts = follow_error_counts

        #: Mechanical end stops, in counts. The shaft cannot pass them, and
        #: torque climbs while the drive pushes against one -- which is what
        #: the pSCT calibration procedure ("run it out until it stops") relies
        #: on, and what `seek_hard_stop` has to detect.
        self.hard_stop_low = hard_stop_low
        self.hard_stop_high = hard_stop_high
        #: Torque as a fraction of CL: Current Max while unobstructed. 337/2048
        #: is about 16%, which is what the real motor reads.
        self.idle_torque = 337
        self.stalled_torque = 1600
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
            1: firmware_version,      # Program Version
            2: int(MotorMode.PASSIVE),
            3: int(start_counts),     # Requested Position
            5: 1000,                  # Max Velocity
            6: 100,                   # Acceleration
            7: 511,                   # Running Current
            8: 500,                   # Standby Time
            9: 128,                   # Standby Current
            10: int(start_counts),    # Projected Position
            12: 0,                    # Actual Velocity
            13: 1000,                 # Start Velocity
            14: 409600,               # Gear Output
            15: 2048,                 # Gear Input
            16: int(start_counts),    # Actual Encoder Position
            18: 0,                    # Digital Inputs
            19: 0,                    # Digital Outputs
            20: 0,                    # Follow Error
            22: 0,                    # Follow Error Max
            25: 0x8A472C14,           # Status Bits, as observed
            26: 18,                   # Temperature, low res
            28: 0,                    # Position Limit Min
            30: 0,                    # Position Limit Max
            32: 10000,                # Error Deceleration
            33: 20000,                # 'In Position' Window
            34: 2,                    # 'In Position' Retries
            35: 0,                    # Errors
            36: 0,                    # Warnings
            37: 2,                    # Startup Operating Mode
            38: -100000,              # Homing Position Offset
            40: -5000,                # Homing Velocity
            42: 0,                    # Homing Mode
            46: int(start_counts),    # Abs Encoder Position
            # Above the acceptance threshold, i.e. the 60 V supply is on.
            # The dumped motor read 1794 -- below acceptance -- because that
            # dump was taken with the 60 V off, which is a fault state, not
            # the state a test double should start in.
            97: 4485,                 # Bus voltage (P+)
            98: 565,                  # Bus Voltage Min
            99: 4,                    # Encoder Type
            110: 100,                 # Position Settling Time
            121: 6168,                # Modbus Setup
            125: 255,                 # Digital I/O Setup
            129: 0,                   # Negative Limit Input
            130: 0,                   # Positive Limit Input
            132: 8,                   # Homing Sensor Input
            137: 0,                   # 'In Position' Output
            138: 0,                   # 'Error' Output
            139: 2054,                # Acceptance Voltage
            151: 153,                 # Motor Type
            152: 314852,              # Motor Serial Number
            156: 21,                  # Hardware Revision
            173: 100000,              # Threshold Stall Detection
            174: 0,                   # Deceleration
            177: 10,                  # 'In Target Position' Time
            179: 0,                   # Brake Output -- unassigned, as observed
            199: 0,                   # ModBus Slave Timeout
            200: 0,                   # ModBus Slave Action
            202: 6674785,             # Ticks
            212: 2048,                # CL: Current Max
            217: 337,                 # Actual Torque
            238: 21,                  # Motor Rotations
            246: 41687,               # Temperature
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
            self._apply_hard_stops()
        else:
            # Drive off. If a load is configured and the brake output is not
            # holding, the axis creeps -- the failure mode the interlocks exist
            # to prevent.
            self.registers[12] = 0
            if self.gravity_counts_per_s and not self._brake_output_engaged():
                self._position -= self.gravity_counts_per_s * dt

        projected = int(round(self._position))
        self.registers[10] = projected
        # The encoder lags the profile by a small standing amount, as the real
        # motor does (231 counts when settled), so anything that reads the
        # encoder or the following error meets realistic numbers.
        encoder = projected - self.follow_error_counts
        self.registers[16] = encoder
        self.registers[46] = encoder
        self.registers[20] = projected - encoder
        self.registers[22] = max(self.registers.get(22, 0), abs(projected - encoder))

    def _apply_hard_stops(self) -> None:
        """Clamp the shaft at an end stop, and raise torque while it is held.

        Torque is what a real drive does when it is commanded past an
        obstruction: it pushes harder. Modelling that is the only way the
        stall detection can be exercised without a real end stop.
        """
        pressing = False
        if self.hard_stop_high is not None and self._position > self.hard_stop_high:
            self._position = float(self.hard_stop_high)
            pressing = True
        if self.hard_stop_low is not None and self._position < self.hard_stop_low:
            self._position = float(self.hard_stop_low)
            pressing = True

        if pressing:
            target = float(self.registers.get(3, 0))
            # Only under load while the drive is still commanded past the stop.
            beyond = (
                (self.hard_stop_high is not None and target > self.hard_stop_high)
                or (self.hard_stop_low is not None and target < self.hard_stop_low)
            )
            self.registers[217] = self.stalled_torque if beyond else self.idle_torque
            if beyond:
                self.registers[12] = 0
        else:
            self.registers[217] = self.idle_torque

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
            if number in (1, 10, 12, 16, 20, 46, 202, 217):  # read-only
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
