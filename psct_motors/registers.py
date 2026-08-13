"""
JVL register definitions for the MIS23x / SMC75 family.

Scope
-----
This module contains *only* facts about the motor's register interface: which
JVL register number holds what, how JVL register numbers map onto Modbus
addresses, and how to decode the bit-field registers into human-readable text.
No I/O happens here, so this file is safe to import and unit-test anywhere.

How JVL register numbers map to Modbus addresses
------------------------------------------------
JVL documents its registers by a *register number* (MODE_REG is "register 2",
P_SOLL is "register 3", and so on). The register file is 32-bit wide: every
JVL register occupies one 32-bit slot, whether or not the value inside it
needs all 32 bits.

Modbus, however, addresses 16-bit words. So one JVL register = two consecutive
Modbus holding registers, and:

    modbus_address = jvl_register_number * 2

This is why a 1-word write to MODE_REG gets rejected by the motor even though
MODE_REG only holds a small number: you are writing half of a 32-bit slot.
Every access in this package is therefore 2 Modbus words wide. That behaviour
was already observed on the real hardware before this code was written, and it
is consistent with JVL's documented addressing.

Word order
----------
Within the 32-bit slot, the two 16-bit Modbus words can be transmitted low
word first or high word first. On the pSCT motors this was confirmed as
LOW_HIGH (low word first) by comparing a live read of P_IST against MacTalk's
own position display. `WordOrder` keeps both options because a firmware or
module change could flip it, and `jvl_motor.detect_word_order()` can determine
it empirically at runtime.

Confidence markers
------------------
Every register below carries a `confidence` field, because this code was
written without access to a copy of the JVL manual:

    CONFIRMED : verified against the real pSCT motors and/or MacTalk.
    DOCUMENTED: taken from JVL's published MIS23x/SMC75 register overview.
    VERIFY    : plausible from the JVL register conventions but NOT yet
                checked on hardware. Anything marked VERIFY must be confirmed
                against MacTalk before it is trusted in an unattended system.

`python -m psct_motors.cli verify-registers` prints this table next to live
values read from a motor, which is the intended way to promote a VERIFY entry
to CONFIRMED. Please update the markers here as you confirm them.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Iterable, List, Tuple


# --------------------------------------------------------------------------
# Word order
# --------------------------------------------------------------------------

class WordOrder(str, Enum):
    """Order of the two 16-bit Modbus words inside one 32-bit JVL register."""

    LOW_HIGH = "Low-High"   # low word at the lower Modbus address (pSCT motors)
    HIGH_LOW = "High-Low"

    @classmethod
    def parse(cls, value) -> "WordOrder":
        if isinstance(value, cls):
            return value
        text = str(value).strip().lower().replace("_", "-")
        if text in ("low-high", "lowhigh", "little", "lsw-first"):
            return cls.LOW_HIGH
        if text in ("high-low", "highlow", "big", "msw-first"):
            return cls.HIGH_LOW
        raise ValueError(f"Unrecognised word order {value!r}; use 'Low-High' or 'High-Low'")


def modbus_address(jvl_register: int) -> int:
    """Modbus holding-register address for a JVL register number."""
    if jvl_register < 0:
        raise ValueError(f"JVL register number must be >= 0, got {jvl_register}")
    return int(jvl_register) * 2


def words_to_int32(words: Iterable[int], word_order: WordOrder, signed: bool = True) -> int:
    """Combine two 16-bit Modbus words into one 32-bit JVL register value."""
    w = list(words)
    if len(w) != 2:
        raise ValueError(f"Expected exactly 2 Modbus words, got {len(w)}")
    if word_order is WordOrder.HIGH_LOW:
        high, low = w[0], w[1]
    else:
        high, low = w[1], w[0]
    combined = ((high & 0xFFFF) << 16) | (low & 0xFFFF)
    if signed and combined >= 2 ** 31:
        combined -= 2 ** 32
    return combined


def int32_to_words(value: int, word_order: WordOrder) -> List[int]:
    """Split a 32-bit JVL register value into two 16-bit Modbus words."""
    unsigned = int(value) & 0xFFFFFFFF
    high = (unsigned >> 16) & 0xFFFF
    low = unsigned & 0xFFFF
    return [high, low] if word_order is WordOrder.HIGH_LOW else [low, high]


# --------------------------------------------------------------------------
# Register table
# --------------------------------------------------------------------------

CONFIRMED = "CONFIRMED"
DOCUMENTED = "DOCUMENTED"
VERIFY = "VERIFY"


@dataclass(frozen=True)
class RegisterDef:
    """One entry in the JVL register file."""

    number: int
    name: str
    width: int              # native width in bits (8/16/32) -- the Modbus slot is always 32
    signed: bool
    writable: bool
    confidence: str
    description: str
    unit: str = ""

    @property
    def address(self) -> int:
        return modbus_address(self.number)


#: The registers this package actually uses, plus a few that are useful when
#: diagnosing a motor. This is deliberately a small, curated subset of JVL's
#: ~150-register file rather than a full transcription, so that everything
#: here can carry an honest confidence marker.
REGISTERS: Tuple[RegisterDef, ...] = (
    RegisterDef(1, "PROG_VERSION", 32, False, False, VERIFY,
                "Believed to be the firmware version, but on the pSCT motor this "
                "reads 540777 (0x00084069), which needs 20 bits and so cannot be "
                "the 16-bit version field JVL's overview describes. Register 1 may "
                "well be something else on this firmware. Not used for anything; "
                "kept because it is worth comparing against MacTalk's version "
                "display to settle what it is."),
    RegisterDef(2, "MODE_REG", 16, True, True, CONFIRMED,
                "Operating mode. See MotorMode."),
    RegisterDef(3, "P_SOLL", 32, True, True, CONFIRMED,
                "Target position. The motor drives towards this in Position mode.",
                unit="counts"),
    RegisterDef(4, "P_NEW", 32, True, True, VERIFY,
                "Believed to redefine the current position without moving. On the "
                "pSCT motor it reads 101187584 (0x06080000) while P_IST is 600, "
                "which is not position-shaped, so register 4 is probably not P_NEW "
                "here. Nothing in this package writes it -- set-zero keeps its "
                "offset in the config instead.",
                unit="counts"),
    RegisterDef(5, "V_SOLL", 16, True, True, CONFIRMED,
                "Maximum velocity for position moves.", unit="raw"),
    RegisterDef(6, "A_SOLL", 16, True, True, DOCUMENTED,
                "Acceleration/deceleration ramp.", unit="raw"),
    RegisterDef(7, "RUN_CURRENT", 16, True, True, DOCUMENTED,
                "Motor current while moving.", unit="raw"),
    RegisterDef(8, "STANDBY_TIME", 16, True, True, DOCUMENTED,
                "Delay before dropping to standby current.", unit="ms"),
    RegisterDef(9, "STANDBY_CURRENT", 16, True, True, DOCUMENTED,
                "Holding current when stationary. On a vertical/loaded axis this "
                "is what actually holds position when the brake is released.",
                unit="raw"),
    RegisterDef(10, "P_IST", 32, True, False, CONFIRMED,
                "Actual position. This is the number the GUI displays and the "
                "kinematics are driven from.", unit="counts"),
    RegisterDef(12, "V_IST", 16, True, False, DOCUMENTED,
                "Actual velocity. Reads ~0 when the axis has settled, which is "
                "half of the motion-complete test.", unit="raw"),
    RegisterDef(19, "OUTPUTS", 32, False, True, VERIFY,
                "Digital output states, one bit per output. This is where a "
                "brake wired to an output lives (see BrakeConfig). The register "
                "number, the bit and the polarity are all installation details "
                "-- confirm them with `cli probe-brake` before relying on the "
                "brake indicator."),
    RegisterDef(20, "FLWERR", 32, True, False, VERIFY,
                "Following error (commanded minus encoder position).", unit="counts"),
    RegisterDef(25, "STATUSBITS", 32, False, False, VERIFY,
                "Believed to be a status bit field, but on the pSCT motor it reads "
                "0x8A476C14 while the drive is passive and the shaft stationary. "
                "That is not what an idle status word looks like, so either the bit "
                "layout is nothing like the obvious one or register 25 is not "
                "STATUSBITS on this firmware. No bit names are claimed for it -- see "
                "STATUS_BITS. Read but never acted upon."),
    RegisterDef(35, "ERR_BITS", 32, False, True, CONFIRMED,
                "Error bit field. 0 means no error. Individual bit meanings in "
                "ERROR_BITS are VERIFY-level."),
    RegisterDef(36, "WARN_BITS", 32, False, False, VERIFY,
                "Warning bit field."),
    RegisterDef(38, "P_HOME", 32, True, True, VERIFY,
                "Position value loaded after a homing/zero-search run.",
                unit="counts"),
)

REGISTERS_BY_NAME: Dict[str, RegisterDef] = {r.name: r for r in REGISTERS}
REGISTERS_BY_NUMBER: Dict[int, RegisterDef] = {r.number: r for r in REGISTERS}


def register(name: str) -> RegisterDef:
    try:
        return REGISTERS_BY_NAME[name]
    except KeyError:
        raise KeyError(
            f"Unknown JVL register {name!r}. Known: {sorted(REGISTERS_BY_NAME)}"
        ) from None


def registers_needing_verification() -> List[RegisterDef]:
    return [r for r in REGISTERS if r.confidence == VERIFY]


# --------------------------------------------------------------------------
# MODE_REG values
# --------------------------------------------------------------------------

class MotorMode(int, Enum):
    """Values for MODE_REG.

    PASSIVE / VELOCITY / POSITION / GEAR are confirmed for the MIS23x family.
    Modes 11 and 13-15 are internal zero-search modes; this package never
    writes them, because starting a zero-search on a telescope actuator with
    no physical home switch would drive the axis into its end stop.
    """

    PASSIVE = 0
    VELOCITY = 1
    POSITION = 2
    GEAR = 3

    @property
    def label(self) -> str:
        return {
            MotorMode.PASSIVE: "Passive (drive off)",
            MotorMode.VELOCITY: "Velocity",
            MotorMode.POSITION: "Position",
            MotorMode.GEAR: "Gear",
        }[self]


def describe_mode(value: int) -> str:
    try:
        return MotorMode(value).label
    except ValueError:
        if value in (11, 13, 14, 15):
            return f"Zero-search / internal mode {value}"
        return f"Unknown mode {value}"


# --------------------------------------------------------------------------
# Bit-field decoding
# --------------------------------------------------------------------------

#: Bit meanings for ERR_BITS. VERIFY-level: the register itself is confirmed
#: (0 = healthy), but this bit-to-text mapping has not been checked against a
#: real fault on the pSCT motors. Decoded text is always shown next to the raw
#: hex value in the UI so an unmapped or mis-mapped bit is still visible.
ERROR_BITS: Dict[int, str] = {
    0: "General error",
    1: "Follow error / position error too large",
    2: "Position limit exceeded",
    3: "Low bus voltage",
    4: "Over voltage",
    5: "Temperature too high",
    6: "Internal/self-test error",
    7: "Encoder error",
    8: "Driver over-current",
    9: "Driver disabled",
    10: "Communication error",
}

#: Bit meanings for STATUSBITS -- deliberately empty.
#:
#: This used to hold a guessed layout (bit 0 "In position", bit 2
#: "Decelerating", and so on). On the real pSCT motor register 25 reads
#: 0x8A476C14 while the drive is passive and the shaft is stationary, and that
#: guess decoded it as "Decelerating, Motion running" -- a confident, wrong
#: statement about a motor that was doing nothing at all.
#:
#: A plausible-looking wrong answer is worse than no answer, so the names are
#: gone and `describe_status` now reports the raw value only. Fill this in from
#: the JVL manual or by watching the bits change in MacTalk, and nothing else
#: needs to change.
STATUS_BITS: Dict[int, str] = {}


def decode_bits(value: int, meanings: Dict[int, str]) -> List[str]:
    """List the set bits of `value`, naming the ones we have text for."""
    if value == 0:
        return []
    out: List[str] = []
    unsigned = int(value) & 0xFFFFFFFF
    for bit in range(32):
        if unsigned & (1 << bit):
            out.append(meanings.get(bit, f"bit {bit} (unmapped)"))
    return out


def describe_errors(value: int) -> str:
    """Decode ERR_BITS.

    The raw hex always leads, and the decoded names are explicitly marked as
    unverified. That the register means "error" is confirmed -- 0 is healthy on
    the real motor -- but which bit means what has not been checked against a
    real fault, and an operator reading a fault message deserves to know which
    half of it is solid.
    """
    if value == 0:
        return "No errors"
    names = decode_bits(value, ERROR_BITS)
    return (f"0x{int(value) & 0xFFFFFFFF:08X}: " + ", ".join(names)
            + " [bit names UNVERIFIED -- check against MacTalk]")


def describe_status(value: int) -> str:
    raw = f"0x{int(value) & 0xFFFFFFFF:08X}"
    if not STATUS_BITS:
        return f"{raw} (no verified bit meanings for this register)"
    names = decode_bits(value, STATUS_BITS)
    return raw + (": " + ", ".join(names) if names else "")


__all__ = [
    "WordOrder", "modbus_address", "words_to_int32", "int32_to_words",
    "RegisterDef", "REGISTERS", "REGISTERS_BY_NAME", "REGISTERS_BY_NUMBER",
    "register", "registers_needing_verification",
    "MotorMode", "describe_mode",
    "ERROR_BITS", "STATUS_BITS", "decode_bits", "describe_errors", "describe_status",
    "CONFIRMED", "DOCUMENTED", "VERIFY",
]
