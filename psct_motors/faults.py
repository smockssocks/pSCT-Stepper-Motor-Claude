"""
Fault injection, for proving the error handling works.

The problem this solves
-----------------------
Most of the failures this software is written to survive cannot be produced on
demand on a healthy motor. You cannot politely ask a drive to report a
following error, and you certainly should not stall a screw to provoke one.
But the code that *responds* to those failures is ordinary software, and it can
be exercised the moment something makes the motor appear to fail.

`FaultInjectingTransport` wraps a real (or simulated) transport and tampers
with the register traffic on the way past. Everything above it -- the driver,
the platform, the GUI, the bridge -- sees exactly what it would see if the
motor really had that fault, and reacts exactly as it would in the field.

What this does and does not prove
---------------------------------
It proves your *handling* is right: that a fault is noticed, that the move is
abandoned, that motion is halted, that the operator is told something useful,
and that recovery works. That is the part with bugs in it.

It does not prove the motor sets the bit you think it sets. Only the hardware
can tell you that, which is why `ERROR_BITS` in registers.py is marked VERIFY
and why the demo prints raw hex next to every decoded name.

Safety
------
Injection only ever tampers with what is *read back*, plus the ability to make
transactions fail. It never invents a write, never changes a target position,
and never enables a drive. The worst it can do is make the software believe
something is wrong and take the cautious path -- which is the point.

One consequence worth understanding: injecting a fault mid-move makes the
software stop the motor, because that is what it does when a motor faults for
real. The motion ends early by design.
"""

from __future__ import annotations

import random
import threading
import time
from enum import Enum
from typing import List, Optional

from .registers import WordOrder, int32_to_words, modbus_address, register
from .transport import ModbusError, Transport


class Fault(str, Enum):
    """The kinds of failure that can be injected."""

    NONE = "none"
    #: Every transaction fails immediately, as if the cable were pulled.
    COMMS_DROP = "comms-drop"
    #: Every transaction hangs for `delay_s`, then fails: a dead-but-not-absent
    #: link, which is the more annoying real-world case because it is slow.
    COMMS_TIMEOUT = "comms-timeout"
    #: A fraction of transactions fail at random: a marginal cable or a
    #: congested switch.
    COMMS_FLAKY = "comms-flaky"
    #: ERR_BITS reads back a non-zero value, as if the drive had faulted.
    ERROR_BITS = "error-bits"
    #: MODE_REG reads back Passive however it is written, which is what a
    #: second client (MacTalk) holding the motor looks like.
    MODE_REVERT = "mode-revert"
    #: P_IST stops changing, as if the shaft were held or the encoder dead.
    STUCK_POSITION = "stuck-position"
    #: The two 16-bit words come back swapped, which is what a wrong word-order
    #: setting looks like.
    SWAPPED_WORDS = "swapped-words"

    @property
    def label(self) -> str:
        return {
            Fault.NONE: "no fault",
            Fault.COMMS_DROP: "communications lost (cable pulled)",
            Fault.COMMS_TIMEOUT: "communications hanging then failing",
            Fault.COMMS_FLAKY: "intermittent communications",
            Fault.ERROR_BITS: "drive reporting an error",
            Fault.MODE_REVERT: "another client overriding the mode",
            Fault.STUCK_POSITION: "axis not moving / encoder frozen",
            Fault.SWAPPED_WORDS: "wrong word order",
        }[self]


class FaultInjectingTransport:
    """Wraps a transport and makes the motor appear to misbehave.

    Pass-through when no fault is armed, so it is safe to leave permanently in
    the stack. The demo does exactly that.
    """

    def __init__(self, inner: Transport, word_order: WordOrder = WordOrder.LOW_HIGH,
                 seed: Optional[int] = 1234):
        self.inner = inner
        #: Injected values have to be encoded the way the motor encodes them,
        #: or the software decodes the injection into a different number than
        #: was injected. `wrap_motor` takes this from the motor it wraps.
        self.word_order = WordOrder.parse(word_order)
        self._lock = threading.RLock()
        self._fault = Fault.NONE
        self._random = random.Random(seed)

        # Fault parameters.
        self.error_bits_value: int = 1 << 1     # "follow error" by default
        self.delay_s: float = 2.0
        self.failure_rate: float = 0.5
        self._frozen_position: Optional[int] = None

        #: Counts of what the fault actually did, so a drill can assert that
        #: injection was really exercised rather than silently doing nothing.
        self.injected_reads = 0
        self.injected_failures = 0

        # Addresses we may need to intercept, resolved once.
        self._addr_err_bits = modbus_address(register("ERR_BITS").number)
        self._addr_mode = modbus_address(register("MODE_REG").number)
        self._addr_p_ist = modbus_address(register("P_IST").number)

    # ------------------------------------------------------------- arming

    @property
    def fault(self) -> Fault:
        return self._fault

    def arm(self, fault: Fault, **params) -> None:
        """Arm a fault. Keyword arguments override the fault's parameters."""
        with self._lock:
            self._fault = Fault(fault)
            for key, value in params.items():
                if not hasattr(self, key):
                    raise ValueError(f"Unknown fault parameter {key!r}")
                setattr(self, key, value)
            self._frozen_position = None
            self.injected_reads = 0
            self.injected_failures = 0

    def clear(self) -> None:
        """Disarm. The next transaction behaves normally again."""
        with self._lock:
            self._fault = Fault.NONE
            self._frozen_position = None

    def __enter__(self) -> "FaultInjectingTransport":
        return self

    def __exit__(self, *exc) -> None:
        self.clear()

    # ---------------------------------------------------------- pass-through

    def connect(self) -> bool:
        if self._fault in (Fault.COMMS_DROP, Fault.COMMS_TIMEOUT):
            self.injected_failures += 1
            return False
        return self.inner.connect()

    def close(self) -> None:
        self.inner.close()

    def is_open(self) -> bool:
        if self._fault is Fault.COMMS_DROP:
            return False
        return self.inner.is_open()

    def describe(self) -> str:
        base = self.inner.describe()
        return base if self._fault is Fault.NONE else f"{base} [FAULT: {self._fault.value}]"

    # -------------------------------------------------------------- traffic

    def _maybe_fail(self, what: str) -> None:
        """Raise if the armed fault should make this transaction fail."""
        fault = self._fault
        if fault is Fault.COMMS_DROP:
            self.injected_failures += 1
            raise ModbusError(
                f"[injected] {what} failed: no response from the motor. "
                "This is what a pulled cable, a powered-down motor or a wrong "
                "IP address looks like."
            )
        if fault is Fault.COMMS_TIMEOUT:
            time.sleep(self.delay_s)
            self.injected_failures += 1
            raise ModbusError(
                f"[injected] {what} timed out after {self.delay_s:.1f} s. "
                "This is what a link that is up but not answering looks like."
            )
        if fault is Fault.COMMS_FLAKY and self._random.random() < self.failure_rate:
            self.injected_failures += 1
            raise ModbusError(
                f"[injected] {what} failed intermittently. This is what a "
                "marginal cable or a congested switch looks like."
            )

    def read_holding(self, address: int, count: int) -> List[int]:
        with self._lock:
            self._maybe_fail(f"read at {address}")
            words = self.inner.read_holding(address, count)
            return self._tamper(address, words)

    def write_holding(self, address: int, values: List[int]) -> None:
        with self._lock:
            self._maybe_fail(f"write at {address}")
            # MODE_REVERT lets the write reach the motor and then lies about
            # the read-back, which is exactly how a second client fighting for
            # control presents: the write succeeds, the mode does not stick.
            self.inner.write_holding(address, values)

    def _tamper(self, address: int, words: List[int]) -> List[int]:
        """Alter what a read returns, according to the armed fault."""
        fault = self._fault
        if fault is Fault.NONE:
            return words

        if fault is Fault.SWAPPED_WORDS:
            self.injected_reads += 1
            return list(reversed(words))

        if fault is Fault.ERROR_BITS and address == self._addr_err_bits:
            self.injected_reads += 1
            return self._as_words(self.error_bits_value)

        if fault is Fault.MODE_REVERT and address == self._addr_mode:
            self.injected_reads += 1
            return self._as_words(0)            # Passive, whatever was written

        if fault is Fault.STUCK_POSITION and address == self._addr_p_ist:
            self.injected_reads += 1
            if self._frozen_position is None:
                self._frozen_position = list(words)
            return list(self._frozen_position)

        return words

    def _as_words(self, value: int) -> List[int]:
        """Encode `value` the way this motor encodes it.

        The injector has to use the motor's own word order. Encoding a fault
        value low-word-first into a high-low motor would have the software
        decode 0x0042 as 0x00420000 -- an injected fault that does not say what
        it meant to say, and a fault drill that tests the wrong thing.
        """
        return int32_to_words(int(value) & 0xFFFFFFFF, self.word_order)


def wrap_motor(motor, seed: Optional[int] = 1234) -> FaultInjectingTransport:
    """Insert a fault injector in front of an existing motor's transport.

    Returns the injector so the caller can arm and clear faults. The motor
    keeps working exactly as before until something is armed. The motor's word
    order is picked up here so injected values decode to what was injected.
    """
    injector = FaultInjectingTransport(motor._transport,
                                       word_order=motor.word_order, seed=seed)
    motor._transport = injector
    return injector


__all__ = ["Fault", "FaultInjectingTransport", "wrap_motor"]
