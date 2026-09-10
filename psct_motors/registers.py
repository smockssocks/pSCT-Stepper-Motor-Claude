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
LOW_HIGH (low word first): every small configuration register puts its zero
high word in the second Modbus word. `WordOrder` keeps both options because a firmware or
module change could flip it, and `jvl_motor.detect_word_order()` can determine
it empirically at runtime.

Where these names come from
---------------------------
Every name and description below was taken from a MacTalk register dump of the
pSCT motor itself -- serial 314852, firmware 6.09.00, an Ethernet module
running Modbus TCP -- so the names are JVL's own rather than inferences.
Observed values are quoted in the descriptions where they are surprising or
where they contradict what the name suggests.

Two of them are worth reading before anything else:

* Register 10 is 'Projected Position', not the actual position. It is the
  profile generator's output and reaches the requested position by
  construction. Register 16, 'Actual Encoder Position', is where the shaft is.
  On the dumped motor these read 204800 and 204569 -- a settled following
  error of 231 counts that register 10 gave no hint of.
* Register 179 is 'Brake Output', which selects WHICH digital output drives
  the brake, not the brake's state. It reads 0, so no output does.

What the motor does NOT have
----------------------------
There is no error history, event log or fault buffer anywhere in the 254
registers. 35 ('Errors') and 36 ('Warnings') are instantaneous bit fields: a
fault that has cleared leaves no trace in the drive. Two registers are latched
extremes and are the closest thing to a black box:

    22  'Follow Error Max'  -- the worst lag ever seen
    98  'Bus Voltage Min'   -- the lowest supply ever seen

Both survive a cleared error and a completed move, and both can be reset by
writing 0, which turns a value with no timestamp into one with a known
starting point. Everything finer-grained has to be recorded externally, which
is what `eventlog.py` is for.

Confidence markers
------------------
    CONFIRMED : name taken from MacTalk's register list for this motor, and/or
                the value verified on the hardware.
    DOCUMENTED: from JVL's published overview but not seen on this motor.
    VERIFY    : inferred, not checked. Nothing carries this marker any more --
                the MacTalk dump settled every register listed here.

`python -m psct_motors.cli verify-registers` prints this table next to live
values, and `motor-report` gives the same information organised by what you
would be looking for.
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
    # ---- identity ------------------------------------------------------
    RegisterDef(1, "PROG_VERSION", 32, False, False, CONFIRMED,
                "Program Version. Reads 540777 on the pSCT motor -- a packed "
                "value, not a plain version number, so it is displayed but not "
                "interpreted."),
    RegisterDef(151, "MOTOR_TYPE", 32, False, False, CONFIRMED, "Motor Type."),
    RegisterDef(152, "MOTOR_SERIAL", 32, False, False, CONFIRMED,
                "Motor Serial Number."),
    RegisterDef(156, "HARDWARE_REV", 32, False, False, CONFIRMED,
                "Hardware Revision."),
    RegisterDef(99, "ENCODER_TYPE", 32, False, False, CONFIRMED,
                "Encoder Type. 4 = closed-loop absolute multiturn on this motor."),

    # ---- command and mode ----------------------------------------------
    RegisterDef(2, "MODE_REG", 16, True, True, CONFIRMED,
                "Operating Mode. See MotorMode."),
    RegisterDef(37, "STARTUP_MODE", 16, True, True, CONFIRMED,
                "Startup Operating Mode -- the mode the motor enters at power "
                "on. 2 on the pSCT motor, so it comes up ready to position."),
    RegisterDef(3, "P_SOLL", 32, True, True, CONFIRMED,
                "Requested Position -- the target the motor drives towards.",
                unit="counts"),
    RegisterDef(5, "V_SOLL", 16, True, True, CONFIRMED,
                "Max Velocity for position moves.", unit="raw"),
    RegisterDef(6, "A_SOLL", 16, True, True, CONFIRMED,
                "Acceleration.", unit="raw"),
    RegisterDef(174, "DECELERATION", 32, True, True, CONFIRMED,
                "Deceleration. 0 means the acceleration value is used for both.",
                unit="raw"),
    RegisterDef(13, "V_START", 32, True, True, CONFIRMED,
                "Start Velocity.", unit="raw"),
    RegisterDef(32, "ERROR_DECELERATION", 32, True, True, CONFIRMED,
                "Error Deceleration -- the ramp used when a fault stops a move.",
                unit="raw"),

    # ---- position feedback ---------------------------------------------
    RegisterDef(10, "P_PROJECTED", 32, True, False, CONFIRMED,
                "Projected Position: where the profile generator has got to. "
                "This reaches the requested position by construction, whether or "
                "not the shaft followed, so it is NOT a measurement of where the "
                "motor is. Used as the stop-here value, never as the position.",
                unit="counts"),
    RegisterDef(16, "P_ENCODER", 32, True, False, CONFIRMED,
                "Actual Encoder Position -- where the shaft actually is. This is "
                "the position this software reports and checks arrival against.",
                unit="counts"),
    RegisterDef(46, "P_ENCODER_ABS", 32, True, False, CONFIRMED,
                "Abs Encoder Position, from the absolute multiturn encoder.",
                unit="counts"),
    RegisterDef(12, "V_IST", 16, True, False, CONFIRMED,
                "Actual Velocity. ~0 when the axis has settled.", unit="raw"),
    RegisterDef(20, "FLWERR", 32, True, False, CONFIRMED,
                "Follow Error = Projected Position - Actual Encoder Position. "
                "Reads 231 on a settled pSCT motor, so a small standing value is "
                "normal and only a large or growing one is a problem.",
                unit="counts"),
    RegisterDef(22, "FLWERR_MAX", 32, True, True, CONFIRMED,
                "Follow Error Max -- the largest following error seen since it "
                "was last cleared. A latched high-water mark, and one of only two "
                "pieces of history this motor keeps. Write 0 to reset it.",
                unit="counts"),
    RegisterDef(238, "MOTOR_ROTATIONS", 32, True, False, CONFIRMED,
                "Motor Rotations."),

    # ---- errors and status ---------------------------------------------
    RegisterDef(35, "ERR_BITS", 32, False, True, CONFIRMED,
                "Errors. 0 means healthy. Instantaneous only -- the motor keeps "
                "no error history, which is why this software records its own."),
    RegisterDef(36, "WARN_BITS", 32, False, False, CONFIRMED,
                "Warnings. Instantaneous, like Errors."),
    RegisterDef(25, "STATUSBITS", 32, False, False, CONFIRMED,
                "Status Bits. Confirmed as the status word, but the bit layout is "
                "not published in MacTalk's register list, so no bit names are "
                "claimed. Reads 0x8A47xxxx on the pSCT motor -- including while the "
                "drive is passive and the shaft stationary -- and does change "
                "between samples. Displayed raw."),

    # ---- arrival criteria ------------------------------------------------
    RegisterDef(33, "IN_POSITION_WINDOW", 32, True, True, CONFIRMED,
                "'In Position' Window, in counts -- the motor's own idea of "
                "arrival. 20000 on the pSCT motor, which is far wider than the "
                "tolerance this software applies.", unit="counts"),
    RegisterDef(34, "IN_POSITION_RETRIES", 32, True, True, CONFIRMED,
                "'In Position' Retries."),
    RegisterDef(110, "SETTLING_TIME", 32, True, True, CONFIRMED,
                "Position Settling Time.", unit="ms"),
    RegisterDef(177, "IN_TARGET_TIME", 32, True, True, CONFIRMED,
                "'In Target Position' Time.", unit="ms"),

    # ---- travel limits ---------------------------------------------------
    RegisterDef(28, "POS_LIMIT_MIN", 32, True, True, CONFIRMED,
                "Position Limit Min. Both limits 0 on the pSCT motor, which "
                "means no software travel limit is active in the drive itself.",
                unit="counts"),
    RegisterDef(30, "POS_LIMIT_MAX", 32, True, True, CONFIRMED,
                "Position Limit Max.", unit="counts"),
    RegisterDef(129, "NEG_LIMIT_INPUT", 32, True, True, CONFIRMED,
                "Negative Limit Input -- which digital input is the negative "
                "limit switch. 0 = none assigned."),
    RegisterDef(130, "POS_LIMIT_INPUT", 32, True, True, CONFIRMED,
                "Positive Limit Input. 0 = none assigned."),

    # ---- current and load ------------------------------------------------
    RegisterDef(7, "RUN_CURRENT", 16, True, True, CONFIRMED,
                "Running Current.", unit="raw"),
    RegisterDef(8, "STANDBY_TIME", 16, True, True, CONFIRMED,
                "Standby Time -- delay before dropping to standby current.",
                unit="ms"),
    RegisterDef(9, "STANDBY_CURRENT", 16, True, True, CONFIRMED,
                "Standby Current -- what holds position when stationary.",
                unit="raw"),
    RegisterDef(217, "ACTUAL_TORQUE", 32, True, False, CONFIRMED,
                "Actual Torque. Rising torque at a standstill means the axis is "
                "fighting something."),
    RegisterDef(212, "CURRENT_MAX", 32, True, True, CONFIRMED,
                "CL: Current Max. Reads 2048 on the pSCT motor, and Actual "
                "Torque is expressed against it: 337/2048 is about 16%, which "
                "matches the 15% the pSCT motion-control procedure reports as "
                "typical. So torque percent = ACTUAL_TORQUE / CURRENT_MAX.",
                unit="raw"),
    RegisterDef(173, "STALL_THRESHOLD", 32, True, True, CONFIRMED,
                "Threshold Stall Detection.", unit="counts"),

    # ---- supply and temperature -----------------------------------------
    RegisterDef(97, "BUS_VOLTAGE", 32, True, False, CONFIRMED,
                "Bus voltage (P+), in raw units.", unit="raw"),
    RegisterDef(98, "BUS_VOLTAGE_MIN", 32, True, True, CONFIRMED,
                "Bus Voltage Min -- the lowest supply voltage seen since it was "
                "last cleared. A latched low-water mark, and the other piece of "
                "history this motor keeps. A brown-out that caused a fault hours "
                "ago is still visible here. Write to reset.", unit="raw"),
    RegisterDef(139, "ACCEPTANCE_VOLTAGE", 32, True, True, CONFIRMED,
                "Acceptance Voltage -- the supply threshold the drive requires.",
                unit="raw"),
    RegisterDef(246, "TEMPERATURE", 32, True, False, CONFIRMED,
                "Temperature, raw. Register 26 gives a low-resolution degrees "
                "value (18 C on the pSCT motor)."),
    RegisterDef(26, "TEMPERATURE_LOW_RES", 32, True, False, CONFIRMED,
                "Temperature, low resolution.", unit="C"),

    # ---- I/O and brake ---------------------------------------------------
    RegisterDef(18, "INPUTS", 32, False, False, CONFIRMED, "Digital Inputs."),
    RegisterDef(19, "OUTPUTS", 32, False, True, CONFIRMED, "Digital Outputs."),
    RegisterDef(125, "IO_SETUP", 32, False, True, CONFIRMED, "Digital I/O Setup."),
    RegisterDef(179, "BRAKE_OUTPUT", 32, True, True, CONFIRMED,
                "Brake Output -- WHICH digital output drives the brake, not the "
                "brake state. 0 means no output is assigned, so on the pSCT motor "
                "as configured today the brake is not under output control. Same "
                "pattern as 'In Position' Output (137) and 'Error' Output (138)."),
    RegisterDef(137, "IN_POSITION_OUTPUT", 32, True, True, CONFIRMED,
                "'In Position' Output -- which output signals arrival. 0 = none."),
    RegisterDef(138, "ERROR_OUTPUT", 32, True, True, CONFIRMED,
                "'Error' Output -- which output signals a fault. 0 = none."),

    # ---- comms watchdog --------------------------------------------------
    RegisterDef(199, "MODBUS_TIMEOUT_MS", 32, True, True, CONFIRMED,
                "ModBus Slave Timeout, ms. If non-zero, the drive takes the "
                "action in register 200 when it stops being polled -- a way for a "
                "motor to change mode by itself and appear to stop accepting "
                "commands. 0 on the pSCT motor, so the watchdog is off.",
                unit="ms"),
    RegisterDef(200, "MODBUS_ACTION", 32, True, True, CONFIRMED,
                "ModBus Slave Action -- what the drive does when the Modbus "
                "timeout expires."),
    RegisterDef(121, "MODBUS_SETUP", 32, False, True, CONFIRMED, "Modbus Setup."),

    # ---- homing ----------------------------------------------------------
    RegisterDef(38, "HOME_OFFSET", 32, True, True, CONFIRMED,
                "Homing Position Offset.", unit="counts"),
    RegisterDef(40, "HOME_VELOCITY", 32, True, True, CONFIRMED,
                "Homing Velocity.", unit="raw"),
    RegisterDef(42, "HOME_MODE", 32, True, True, CONFIRMED, "Homing Mode."),
    RegisterDef(132, "HOME_SENSOR_INPUT", 32, True, True, CONFIRMED,
                "Homing Sensor Input."),

    # ---- gearing ---------------------------------------------------------
    RegisterDef(14, "GEAR_OUTPUT", 32, True, True, CONFIRMED,
                "Gear Output. 409600 on the pSCT motor, the same number as the "
                "counts per revolution."),
    RegisterDef(15, "GEAR_INPUT", 32, True, True, CONFIRMED,
                "Gear Input. 2048 on the pSCT motor."),

    # ---- uptime ----------------------------------------------------------
    RegisterDef(202, "TICKS", 32, True, False, CONFIRMED,
                "Ticks -- a free-running counter. Useful only to tell whether the "
                "motor has restarted between two readings."),
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
