"""
"It has stopped taking position commands." Why?

This module answers that question against a live motor, in the order the
causes actually occur, and says what to do about each.

The failure mode it is built for
--------------------------------
A JVL drive almost never refuses a position command. It accepts the write,
returns success, and then does nothing -- because the drive is passive, or an
error is latched, or the velocity limit is zero, or the run current is zero.
From the application's side all four look identical: the write succeeded and
the shaft did not move. Nothing in the Modbus response distinguishes them.

So rather than guess, read the handful of registers that can each independently
prevent motion, and report which one is doing it.

Checks are ordered by what would stop motion *first*, and each one is
classified:

    BLOCKING  this alone stops the motor moving. Fix it.
    SUSPECT   not fatal by itself, but a plausible contributor.
    OK        checked, and fine.
    UNKNOWN   could not be checked, and why.

Everything here is read-only except one optional probe, `target_readback`,
which writes P_SOLL with the position the motor is *already at*. That commands
no motion by construction, and it is the only way to find out whether writes
are reaching the register at all.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List

from .jvl_motor import BrakeState, JVLMotor, MotorFault
from .registers import MotorMode, describe_errors, describe_mode
from .transport import ModbusError

BLOCKING = "BLOCKING"
SUSPECT = "SUSPECT"
OK = "OK"
UNKNOWN = "UNKNOWN"


@dataclass
class Finding:
    verdict: str
    title: str
    detail: str
    remedy: str = ""
    data: dict = field(default_factory=dict)

    @property
    def blocking(self) -> bool:
        return self.verdict == BLOCKING

    def as_dict(self) -> dict:
        return {"verdict": self.verdict, "title": self.title,
                "detail": self.detail, "remedy": self.remedy, "data": self.data}


@dataclass
class Diagnosis:
    findings: List[Finding] = field(default_factory=list)

    @property
    def blockers(self) -> List[Finding]:
        return [f for f in self.findings if f.verdict == BLOCKING]

    @property
    def suspects(self) -> List[Finding]:
        return [f for f in self.findings if f.verdict == SUSPECT]

    @property
    def healthy(self) -> bool:
        return not self.blockers

    def summary(self) -> str:
        if self.blockers:
            first = self.blockers[0]
            more = (f" (and {len(self.blockers) - 1} more)"
                    if len(self.blockers) > 1 else "")
            return f"Motion is blocked: {first.title}{more}"
        if self.suspects:
            return (f"Nothing is blocking motion, but {len(self.suspects)} thing(s) "
                    "look worth checking.")
        return "Nothing found that would stop this motor moving."

    def as_dict(self) -> dict:
        return {
            "summary": self.summary(),
            "healthy": self.healthy,
            "findings": [f.as_dict() for f in self.findings],
        }

    def as_text(self) -> str:
        lines = [self.summary(), ""]
        for finding in self.findings:
            lines.append(f"[{finding.verdict:<8}] {finding.title}")
            lines.append(f"           {finding.detail}")
            if finding.remedy and finding.verdict in (BLOCKING, SUSPECT):
                lines.append(f"           -> {finding.remedy}")
        return "\n".join(lines)


def diagnose(motor: JVLMotor, probe_writes: bool = True) -> Diagnosis:
    """Work out what, if anything, is stopping this motor from moving.

    `probe_writes` allows the one write this performs: P_SOLL set to the
    motor's current position, which cannot cause motion but does reveal
    whether writes are landing. Turn it off if you would rather nothing at all
    be written.
    """
    findings: List[Finding] = []

    # ---- 1. Is it answering at all? ---------------------------------------
    try:
        position = motor.get_position_counts()
        findings.append(Finding(
            OK, "Communications",
            f"The motor is answering. P_IST = {position} counts.",
            data={"position": position},
        ))
    except (ModbusError, MotorFault) as exc:
        findings.append(Finding(
            BLOCKING, "Communications",
            f"The motor is not answering: {exc}",
            "Check the cable, that the motor is powered, and the IP address. "
            "If the link was lost and has come back, use Reconnect -- after a "
            "reset the client can hold a dead socket that never recovers on "
            "its own.",
        ))
        return Diagnosis(findings)      # nothing further can be read

    # ---- 2. Latched errors ------------------------------------------------
    try:
        errors = motor.get_errors()
        if errors:
            findings.append(Finding(
                BLOCKING, "Error bits set",
                f"ERR_BITS = {describe_errors(errors)}. A JVL drive with an "
                "active error will accept position commands and not act on them.",
                "Clear the errors. If they come straight back, the cause is "
                "still present -- read the bits and check the mechanism, the "
                "supply voltage and the temperature before retrying.",
                data={"err_bits": errors},
            ))
        else:
            findings.append(Finding(OK, "Error bits", "ERR_BITS = 0, no errors.",
                                    data={"err_bits": 0}))
    except (ModbusError, MotorFault) as exc:
        findings.append(Finding(UNKNOWN, "Error bits",
                                f"Could not read ERR_BITS: {exc}"))

    # ---- 3. Operating mode ------------------------------------------------
    try:
        mode = motor.get_mode()
        if mode == int(MotorMode.POSITION):
            findings.append(Finding(OK, "Operating mode",
                                    "MODE_REG = 2 (Position). Position commands "
                                    "will be acted on.", data={"mode": mode}))
        elif mode == int(MotorMode.PASSIVE):
            findings.append(Finding(
                BLOCKING, "Drive is passive",
                "MODE_REG = 0 (Passive). The drive output is off. Writes to "
                "P_SOLL succeed and are ignored, which is the single most "
                "common reason a motor 'stops taking position commands'.",
                "Enable Position mode. If it will not stick, something else is "
                "writing the register -- MacTalk being still connected is the "
                "usual cause. It is also worth checking whether the drive went "
                "passive by itself, which the event log will show.",
                data={"mode": mode},
            ))
        else:
            findings.append(Finding(
                BLOCKING, "Wrong operating mode",
                f"MODE_REG = {mode} ({describe_mode(mode)}), not Position mode. "
                "Position commands are not acted on in this mode.",
                "Set Position mode.",
                data={"mode": mode},
            ))
    except (ModbusError, MotorFault) as exc:
        findings.append(Finding(UNKNOWN, "Operating mode",
                                f"Could not read MODE_REG: {exc}"))

    # ---- 4. Settings that silently prevent motion -------------------------
    findings.extend(_check_zero_setting(
        motor, "V_SOLL", "Velocity limit",
        "The motor will accept a target and approach it at zero speed -- that "
        "is, never move. Nothing reports an error.",
        "Set a non-zero velocity. If it became zero on its own, something "
        "wrote it; the event log records V_SOLL changes for exactly this reason.",
    ))
    findings.extend(_check_zero_setting(
        motor, "A_SOLL", "Acceleration",
        "With zero acceleration the motor cannot ramp up to its velocity, so "
        "it never starts.",
        "Set a non-zero acceleration.",
    ))
    findings.extend(_check_zero_setting(
        motor, "RUN_CURRENT", "Run current",
        "With no run current the motor has no torque and will not turn, even "
        "though everything else looks correct.",
        "Set a run current appropriate for this motor.",
    ))

    # ---- 5. Is it simply already there? -----------------------------------
    try:
        target = motor.get_target_counts()
        delta = position - target
        if abs(delta) <= 1:
            findings.append(Finding(
                OK, "Already at target",
                f"P_SOLL = {target} and P_IST = {position}. The motor is not "
                "moving because it has nothing to do.",
                data={"target": target, "position": position},
            ))
        else:
            findings.append(Finding(
                OK, "Target differs from position",
                f"P_SOLL = {target}, P_IST = {position} ({delta:+d} counts away). "
                "There is a move outstanding.",
                data={"target": target, "position": position, "delta": delta},
            ))
    except (ModbusError, MotorFault) as exc:
        findings.append(Finding(UNKNOWN, "Target", f"Could not read P_SOLL: {exc}"))

    # ---- 6. Do writes actually land? --------------------------------------
    if probe_writes:
        findings.append(_probe_target_write(motor, position))

    # ---- 7. Brake ---------------------------------------------------------
    findings.append(_check_brake(motor))

    # ---- 8. Following error ----------------------------------------------
    try:
        follow_error = motor.read_register("FLWERR")
        if abs(follow_error) > 0:
            findings.append(Finding(
                SUSPECT, "Following error",
                f"FLWERR = {follow_error}. The commanded and actual positions "
                "disagree, which means the motor is being asked for motion it "
                "is not achieving.",
                "Check for a mechanical obstruction, an unreleased brake, or a "
                "run current too low for the load. Note that FLWERR is a "
                "VERIFY-level register here -- confirm the number means what "
                "it appears to before acting on it.",
                data={"flwerr": follow_error},
            ))
        else:
            findings.append(Finding(OK, "Following error", "FLWERR = 0.",
                                    data={"flwerr": 0}))
    except (ModbusError, MotorFault):
        findings.append(Finding(UNKNOWN, "Following error",
                                "FLWERR could not be read on this motor."))

    return Diagnosis(findings)


def _check_zero_setting(motor: JVLMotor, register_name: str, title: str,
                        consequence: str, remedy: str) -> List[Finding]:
    try:
        value = motor.read_register(register_name)
    except (ModbusError, MotorFault) as exc:
        return [Finding(UNKNOWN, title, f"Could not read {register_name}: {exc}")]
    if value == 0:
        return [Finding(BLOCKING, f"{title} is zero",
                        f"{register_name} = 0. {consequence}", remedy,
                        data={register_name: 0})]
    return [Finding(OK, title, f"{register_name} = {value}.",
                    data={register_name: value})]


def _probe_target_write(motor: JVLMotor, position: int) -> Finding:
    """Write P_SOLL = current position and check it sticks.

    Commanding the position the motor is already at cannot produce motion, so
    this is safe to run at any time, and it is the only way to distinguish
    "the write is being rejected or overwritten" from "the write landed and
    something else is stopping the motor".
    """
    try:
        original = motor.get_target_counts()
    except (ModbusError, MotorFault) as exc:
        return Finding(UNKNOWN, "Write probe", f"Could not read P_SOLL: {exc}")
    try:
        motor.command_position_counts(position)
        time.sleep(0.15)
        readback = motor.get_target_counts()
    except (ModbusError, MotorFault) as exc:
        return Finding(BLOCKING, "Writes are failing",
                       f"Writing P_SOLL raised: {exc}",
                       "The link is up for reads but not writes. Check whether "
                       "another client holds the motor.")
    if readback == position:
        return Finding(
            OK, "Writes land",
            f"P_SOLL was set to the current position ({position}) and read back "
            "unchanged, so position commands are reaching the register.",
            data={"target": readback},
        )
    return Finding(
        BLOCKING, "Writes are not sticking",
        f"P_SOLL was set to {position} but reads back {readback}. Something is "
        "overwriting the target, so every position command is being undone.",
        "The usual cause is a second client -- MacTalk, another copy of this "
        "software, or a PLC -- writing the same register. Disconnect the "
        "others and retry.",
        data={"wrote": position, "read_back": readback, "was": original},
    )


def _check_brake(motor: JVLMotor) -> Finding:
    try:
        status = motor.get_brake_status()
    except (ModbusError, MotorFault) as exc:
        return Finding(UNKNOWN, "Brake", f"Could not read the brake state: {exc}")

    if status.state is BrakeState.ENGAGED and not status.inferred:
        return Finding(
            BLOCKING, "Brake is engaged",
            f"The brake output reads engaged. {status.detail}",
            "Release the brake before commanding motion. The drive must be "
            "enabled and holding first.",
            data={"brake": status.state.value},
        )
    if status.state is BrakeState.ENGAGED:
        return Finding(
            SUSPECT, "Brake may be engaged",
            f"The brake is inferred to be engaged. {status.detail} This is a "
            "deduction from the drive mode, not a measurement.",
            "If the drive is passive the brake is holding, which is expected. "
            "Enable Position mode and check again.",
            data={"brake": status.state.value},
        )
    return Finding(OK, "Brake",
                   f"{status.state.label}. {status.detail}",
                   data={"brake": status.state.value})


__all__ = ["diagnose", "Diagnosis", "Finding",
           "BLOCKING", "SUSPECT", "OK", "UNKNOWN"]
