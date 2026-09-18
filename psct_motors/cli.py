"""
Command line tool for the pSCT focal-plane actuators.

This is the commissioning interface. Everything you need to do *before* the
GUI is useful -- confirming the register map, proving the word order, working
out how many counts make a millimetre, finding which way each actuator pushes,
checking the brakes actually click -- lives here.

    python -m psct_motors.cli --help

Every command takes `--simulate` to run against the built-in fake motors, so
you can rehearse a procedure at your desk. Nothing in this file writes to a
motor without either an explicit flag or a typed confirmation.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from typing import Dict, List, Optional

from .config import (
    default_config, default_config_path, load_config, save_config,
)
from .jvl_motor import MotorFault
from .kinematics import Orientation
from .platform import FocalPlanePlatform, PlatformError
from .registers import REGISTERS, VERIFY
from .transport import ModbusError


# --------------------------------------------------------------------------
# Output helpers
# --------------------------------------------------------------------------

def out(msg: str = "") -> None:
    print(msg, flush=True)


def rule(title: str = "") -> None:
    out("-" * 72 if not title else f"--- {title} " + "-" * max(0, 67 - len(title)))


def confirm(prompt: str, assume_yes: bool = False) -> bool:
    if assume_yes:
        out(f"{prompt} [auto-yes]")
        return True
    try:
        return input(f"{prompt} [y/N] ").strip().lower() in ("y", "yes")
    except EOFError:
        return False


def ask_float(prompt: str) -> Optional[float]:
    try:
        raw = input(f"{prompt}: ").strip()
    except EOFError:
        return None
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        out(f"'{raw}' is not a number.")
        return None


def make_platform(args) -> FocalPlanePlatform:
    cfg = load_config(args.config)
    apply_bench(cfg, getattr(args, "bench", None))
    if getattr(args, "sim_speed", None):
        cfg.simulated_speed_mm_per_s = args.sim_speed
    if getattr(args, "poll", None):
        cfg.poll_interval_s = args.poll
        cfg.idle_poll_interval_s = max(args.poll, cfg.idle_poll_interval_s)
    from .history import default_history_path
    # The same position record the GUI keeps, so a move made from the command
    # line shows up in the window's log and vice versa.
    platform = FocalPlanePlatform(cfg=cfg, simulate=args.simulate, logger=out,
                                  config_path=args.config,
                                  history_path=default_history_path(args.config))
    if platform.is_mixed:
        out("BENCH MODE: " + ", ".join(platform.simulated_names)
            + " are simulated; only "
            + ", ".join(m.name for m in platform.motors
                        if m.name not in platform.simulated_names)
            + " is a real motor.")
        out("Anything the simulated axes report is made up. Use this to")
        out("exercise the application, not to believe its numbers.")
    return platform


def apply_bench(cfg, bench: Optional[str]):
    """Mark every actuator except `bench` as simulated.

    One motor on a bench is what the site actually has, and without this the
    entire three-axis half of the application -- kinematics, coordinated moves,
    the hard-stop search, the emergency interlocks -- cannot be exercised
    against real hardware at all until all three are wired.
    """
    if not bench:
        return cfg
    names = [a.name for a in cfg.actuators]
    matches = [a for a in cfg.actuators if a.name.lower() == bench.lower()]
    if not matches:
        raise ValueError(f"No actuator named {bench!r}. Known: {names}")
    for actuator in cfg.actuators:
        actuator.simulated = actuator is not matches[0]
    return cfg


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------

def cmd_init_config(args) -> int:
    path = args.config or default_config_path()
    import os
    if os.path.exists(path) and not args.force:
        out(f"{path} already exists. Use --force to overwrite it.")
        return 1
    cfg = default_config()
    cfg.validate()
    written = save_config(cfg, path)
    out(f"Wrote a starting configuration to {written}")
    out("")
    out("Before you connect to hardware, edit that file and set:")
    out("  * ip           -- the address of each of the three motors")
    out("  * azimuth_deg  -- where each actuator sits around the optical axis")
    out("  * radius_mm    -- its distance from the axis, from the camera drawing")
    out("  * brake.mode   -- 'auto' if the drive releases the brake, 'output' if")
    out("                    it is wired to a digital output you control")
    out("")
    out("Then run, in order:")
    out("  python -m psct_motors.cli verify-registers")
    out("  python -m psct_motors.cli check-direction --motor Top")
    out("  python -m psct_motors.cli calibrate --motor Top")
    return 0


def cmd_show_config(args) -> int:
    cfg = load_config(args.config)
    out(json.dumps({"path": args.config or default_config_path()}, indent=2))
    from .config import config_to_dict
    out(json.dumps(config_to_dict(cfg), indent=2))
    return 0


def cmd_status(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        state = platform.read_state()
        if args.json:
            out(json.dumps(state.as_dict(), indent=2))
            return 0

        rule("Actuators")
        header = (f"{'name':<6}{'mm':>12}{'counts':>14}{'target mm':>12}  "
                  f"{'mode':<20}{'brake':<12}{'load':>6}{'supply':>12}")
        out(header)
        for m in state.motors:
            if m.comms_error:
                out(f"{m.name:<6}  ** {m.comms_error}")
                continue
            brake = m.brake.state.value + ("?" if m.brake.inferred else "")
            load = "--" if m.torque_percent is None else f"{m.torque_percent:.0f}%"
            if m.bus_voltage is None:
                supply = "--"
            elif m.supply_volts is not None:
                supply = f"{m.supply_volts:.1f} V"
            else:
                supply = f"{m.bus_voltage} raw"
            out(f"{m.name:<6}{m.position_mm:>12.4f}{m.position_counts:>14}"
                f"{m.target_mm:>12.4f}  {m.mode_text:<20}{brake:<12}"
                f"{load:>6}{supply:>12}")
            if m.error_bits:
                out(f"        errors: {m.error_text}")

        rule("Focal plane")
        if state.orientation_valid and state.orientation:
            out("  " + state.orientation.describe())
        else:
            out("  " + (state.message or "unavailable"))
        rule()
        out(f"moving: {state.moving}   errors: {state.any_error}")
        return 0
    finally:
        platform.disconnect()


def cmd_verify_registers(args) -> int:
    """Print the register table beside live values, so VERIFY entries can be
    checked against MacTalk and promoted."""
    rule("Register map")
    out(f"{'reg':>4} {'modbus':>7}  {'name':<16}{'bits':>5} {'confidence':<11} description")
    for r in REGISTERS:
        out(f"{r.number:>4} {r.address:>7}  {r.name:<16}{r.width:>5} {r.confidence:<11} "
            f"{r.description.splitlines()[0][:60]}")
    out("")
    out("CONFIRMED = checked on these motors.  DOCUMENTED = from JVL's published")
    out("register overview.  VERIFY = plausible but unchecked -- compare the live")
    out("value below against the same field in MacTalk before trusting it.")

    if args.offline:
        return 0

    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out("")
        out(f"Could not connect: {exc}")
        out("Re-run with --offline to see the table alone.")
        return 1
    try:
        for motor in platform.motors:
            rule(f"Live values from {motor.name}")
            for name, value in motor.read_diagnostics().items():
                marker = ""
                reg = next((r for r in REGISTERS if r.name == name), None)
                if reg and reg.confidence == VERIFY:
                    marker = "   <-- VERIFY against MacTalk"
                out(f"  {name:<16} {value}{marker}")
    finally:
        platform.disconnect()
    return 0


def cmd_detect(args) -> int:
    """Read PROG_VERSION both ways and report which word order is right."""
    platform = make_platform(args)
    failures = 0
    for motor in platform.motors:
        try:
            # Connect without the word-order check, which is what we are testing.
            motor.connect(verify_word_order=False)
        except (ModbusError, MotorFault) as exc:
            out(f"{motor.name}: could not connect -- {exc}")
            failures += 1
            continue
        try:
            detected = motor.detect_word_order()
            configured = motor.word_order
            verdict = "matches config" if detected is configured else (
                f"** MISMATCH: config says {configured.value} **")
            out(f"{motor.name}: detected {detected.value}  ({verdict})")
            if detected is not configured:
                failures += 1
        except (ModbusError, MotorFault) as exc:
            out(f"{motor.name}: detection failed -- {exc}")
            failures += 1
        finally:
            motor.disconnect()
    if failures:
        out("")
        out("Fix 'word_order' in the config for any mismatched actuator; until you "
            "do, every position read from it is wrong.")
    return 1 if failures else 0


def cmd_check_direction(args) -> int:
    """Find out which way an actuator pushes the focal plane.

    Direction signs are the classic silent failure: get one wrong and a pure
    focus command becomes a tilt, with everything still reporting success.
    """
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        motor = platform.motor(args.motor)
        start = motor.get_position_mm()
        out(f"{motor.name} is at {start:.4f} mm ({motor.get_position_counts()} counts).")
        out(f"About to move it {args.mm:+.3f} mm as this software currently "
            "understands the direction.")
        if not confirm("Watch the actuator. Proceed?", args.yes):
            return 1

        platform.move_actuator_mm(motor.name, args.mm, relative=True)
        end = motor.get_position_mm()
        out(f"{motor.name} is now at {end:.4f} mm.")
        out("")
        out("Did the focal plane move in the +z direction (the direction you want")
        out("a POSITIVE focus number to mean -- normally away from the secondary)?")
        if confirm("Moved the way the software says?", args.yes):
            out(f"Good. direction stays {motor.cfg.direction:+d} for {motor.name}.")
            return 0
        motor.cfg.direction = -motor.cfg.direction
        save_config(platform.cfg, args.config)
        out(f"Flipped direction for {motor.name} to {motor.cfg.direction:+d} and saved.")
        out("Re-run this command to confirm the new sign is right.")
        return 0
    finally:
        platform.disconnect()


def cmd_calibrate(args) -> int:
    """Measure counts-per-millimetre directly, without knowing the gear ratio.

    Move a known number of counts, measure the travel with a dial indicator,
    divide. This absorbs the gearbox, the screw lead and any fixed scaling
    error in one number, which is why it beats deriving the value from a
    drivetrain you would otherwise have to chase down.
    """
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        motor = platform.motor(args.motor)
        cfg = motor.cfg
        counts = int(args.counts)

        out(f"Calibrating {motor.name}.")
        out(f"  current scale : {cfg.resolved_counts_per_mm:.4f} counts/mm "
            f"({'measured' if cfg.scale_is_measured else 'derived from the drivetrain'})")
        out(f"  test move     : {counts} counts "
            f"(about {counts / cfg.resolved_counts_per_mm:.4f} mm at the current scale)")
        out("")
        out("Set up a dial indicator against the moving flange and zero it now.")
        if not confirm("Ready to move?", args.yes):
            return 1

        motor.ensure_position_mode()
        if motor.brake_is_software_controlled:
            motor.release_brake()
        start = motor.get_position_counts()
        target_counts = start + counts * cfg.direction

        target_mm_check = cfg.counts_to_mm(target_counts)
        try:
            motor.check_travel_limit(target_mm_check)
        except MotorFault as exc:
            out(str(exc))
            out("Use a smaller --counts, or move the actuator away from its limit first.")
            return 1

        motor.set_velocity(cfg.velocity_raw)
        motor.command_position_counts(target_counts)
        if not motor.wait_for_in_position():
            out("The move did not complete. Calibration abandoned.")
            return 1
        moved_counts = motor.get_position_counts() - start
        out(f"Moved {moved_counts} counts.")

        measured = args.measured_mm
        if measured is None:
            measured = ask_float("Displacement you measured, in mm (unsigned)")
        if measured is None or measured == 0:
            out("No measurement given; nothing was changed.")
            return 1

        new_scale = abs(moved_counts) / abs(measured)
        old_scale = cfg.resolved_counts_per_mm
        out("")
        out(f"  measured      : {abs(measured):.4f} mm for {abs(moved_counts)} counts")
        out(f"  new scale     : {new_scale:.4f} counts/mm")
        out(f"  previous      : {old_scale:.4f} counts/mm "
            f"({100.0 * (new_scale - old_scale) / old_scale:+.2f}% change)")
        # A wildly different answer usually means the indicator moved, the
        # wrong actuator was watched, or the units were mixed up -- worth
        # saying out loud rather than silently saving.
        if not 0.2 < new_scale / old_scale < 5.0:
            out("")
            out("  ** That is a long way from the previous value. Check that you")
            out("     measured the right actuator, in millimetres, and that the")
            out("     indicator did not slip.")
        if not confirm("Save this scale?", args.yes):
            out("Not saved.")
            return 1
        cfg.counts_per_mm = new_scale
        save_config(platform.cfg, args.config)
        out(f"Saved counts_per_mm = {new_scale:.4f} for {motor.name}.")
        out("Repeat for the other two actuators; they are not necessarily identical.")
        return 0
    finally:
        platform.disconnect()


def cmd_probe_brake(args) -> int:
    """Toggle the configured brake output and let the operator confirm it."""
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        motor = platform.motor(args.motor)
        cfg = motor.cfg.brake
        out(f"{motor.name}: brake mode is '{cfg.mode}'.")
        if cfg.mode == "none":
            out("No brake is configured for this actuator, so there is nothing to probe.")
            out("If it does have a brake, set brake.mode to 'auto' or 'output'.")
            return 1
        if cfg.mode == "auto":
            out("In 'auto' mode the drive controls the brake and software cannot")
            out("toggle it. Watch the brake while the mode changes instead:")
            out("  it should engage when the drive goes passive and release when")
            out("  Position mode is enabled.")
            status = motor.get_brake_status()
            out(f"Currently inferred as {status.state.label} -- {status.detail}")
            return 0

        out(f"Brake is on output register {cfg.output_register}, bit {cfg.output_bit}, "
            f"{'energised = released' if cfg.energized_releases else 'energised = engaged'}.")
        out("The drive will be enabled first, so the motor holds the load while the")
        out("brake is off. Listen for the brake clicking.")
        if not confirm("Proceed?", args.yes):
            return 1

        motor.ensure_position_mode()
        # stop() freezes the PROFILE output. Commanding the encoder reading
        # instead would step the axis by the standing following error -- a
        # "hold still" that moves, which is the last thing wanted while the
        # brake is being taken off and put back on.
        motor.stop()
        for _ in range(args.cycles):
            motor.release_brake()
            out(f"  released -- reads back {motor.get_brake_status().state.value}")
            time.sleep(1.0)
            motor.engage_brake()
            out(f"  engaged  -- reads back {motor.get_brake_status().state.value}")
            time.sleep(1.0)
        out("")
        if confirm("Did the brake click in time with those messages?", args.yes):
            out("Brake control confirmed for this actuator.")
            return 0
        out("Then the wiring does not match the config. Check output_register,")
        out("output_bit and energized_releases, and re-run.")
        return 1
    finally:
        platform.disconnect()


def cmd_preview(args) -> int:
    """Show what a move would do, touching no hardware."""
    cfg = load_config(args.config)
    platform = FocalPlanePlatform(cfg=cfg, simulate=True, config_path=args.config)
    target = _orientation_from_args(args)
    out(f"Target: {target.describe()}")
    rule("Actuator targets")
    for name, mm in platform.preview(target).items():
        actuator = cfg.actuator(name)
        inside = actuator.min_travel_mm <= mm <= actuator.max_travel_mm
        flag = "" if inside else "   ** OUTSIDE TRAVEL LIMITS **"
        out(f"  {name:<4} {mm:>10.4f} mm{flag}")
    rule("Travel budget")
    span = platform.geometry.actuator_span_for_tilt(target.total_tilt_deg)
    out(f"  a {target.total_tilt_deg:.4f} deg tilt costs {span:.3f} mm of "
        "peak-to-peak actuator travel")
    return 0


def _orientation_from_args(args) -> Orientation:
    if getattr(args, "tilt_magnitude", None) is not None:
        return Orientation.from_polar_tilt(
            args.focus, args.tilt_magnitude, args.azimuth or 0.0
        )
    return Orientation(args.focus, args.tip or 0.0, args.tilt or 0.0)


def cmd_move(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        current = platform.read_orientation()
        target = _orientation_from_args(args)
        out(f"Now:    {current.describe()}")
        out(f"Target: {target.describe()}")
        rule("Actuator targets")
        for name, mm in platform.preview(target).items():
            out(f"  {name:<4} {mm:>10.4f} mm")
        try:
            platform.check_orientation(target, current=current)
        except PlatformError as exc:
            out(str(exc))
            return 1
        if not confirm("Command this move?", args.yes):
            return 1
        state = platform.move_to_orientation(target, wait=not args.no_wait)
        if state.orientation:
            out(f"Done:   {state.orientation.describe()}")
        return 0
    except PlatformError as exc:
        out(f"Move failed: {exc}")
        return 1
    finally:
        platform.disconnect()


def cmd_move_relative(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        current = platform.read_orientation()
        target = current.offset_by(args.dfocus, args.dtip, args.dtilt)
        out(f"Now:    {current.describe()}")
        out(f"Target: {target.describe()}")
        try:
            platform.check_orientation(target, current=current)
        except PlatformError as exc:
            out(str(exc))
            return 1
        if not confirm("Command this move?", args.yes):
            return 1
        state = platform.move_to_orientation(target, wait=not args.no_wait)
        if state.orientation:
            out(f"Done:   {state.orientation.describe()}")
        return 0
    except PlatformError as exc:
        out(f"Move failed: {exc}")
        return 1
    finally:
        platform.disconnect()


def cmd_history(args) -> int:
    """Where the focal plane has been: the position record, newest first."""
    from .history import PositionHistory, default_history_path
    path = default_history_path(args.config)
    history = PositionHistory(path=path, logger=out)
    records = history.records(newest_first=True)
    if args.json:
        out(json.dumps([r.as_dict() for r in records[:args.last]], indent=2))
        return 0
    rule("Position log")
    out(f"  {path}")
    if not records:
        out("  No moves on record yet. Every move made from the GUI or the "
            "command line is written here as it happens.")
        return 0
    out("")

    def cell(o):
        if o is None:
            return "--"
        if abs(o.tip_deg) < 5e-6 and abs(o.tilt_deg) < 5e-6:
            return f"{o.focus_mm:+.4f}"
        return f"{o.focus_mm:+.4f} ({o.tip_deg:+.4f}/{o.tilt_deg:+.4f} deg)"

    out(f"  {'when':<19} {'what':<10}{'before':>12}{'sent to':>12}{'after':>12}"
        f"  result")
    for record in records[:args.last]:
        result = "done" if record.completed else record.outcome
        out(f"  {record.when:<19} {record.kind:<10}{cell(record.before):>12}"
            f"{cell(record.commanded):>12}{cell(record.after):>12}  {result}")
        if record.note:
            out(f"  {'':<19} {'':<10}  {record.note}")
    out("")
    out(f"  {len(records)} move(s) on record, showing the latest "
        f"{min(args.last, len(records))}. Focus in mm from zero.")
    return 0


def cmd_go_back(args) -> int:
    """Return to where the focal plane was before the most recent move."""
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        record = platform.history.last_with_before()
        if record is None:
            out("There is no previous position on record to go back to.")
            return 1
        current = platform.read_orientation()
        out(f"Now:    {current.describe()}")
        out(f"Back to where it was before the {record.kind} at {record.when}:")
        out(f"Target: {record.before.describe()}")
        try:
            platform.check_orientation(record.before, current=current)
        except PlatformError as exc:
            out(str(exc))
            return 1
        if not confirm("Command this move?", args.yes):
            return 1
        state = platform.go_back(wait=not args.no_wait)
        if state.orientation:
            out(f"Done:   {state.orientation.describe()}")
        return 0
    except PlatformError as exc:
        out(f"Move failed: {exc}")
        return 1
    finally:
        platform.disconnect()


def cmd_jog(args) -> int:
    """Move one actuator, for commissioning only."""
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        motor = platform.motor(args.motor)
        out(f"{motor.name}: {motor.get_position_mm():.4f} mm -> "
            f"{motor.get_position_mm() + args.mm:.4f} mm")
        if not confirm("Jog this single actuator?", args.yes):
            return 1
        status = platform.move_actuator_mm(motor.name, args.mm, relative=True)
        out(f"{motor.name} now at {status.position_mm:.4f} mm.")
        return 0
    except (PlatformError, MotorFault) as exc:
        out(f"Jog failed: {exc}")
        return 1
    finally:
        platform.disconnect()


def cmd_set_zero(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        out("This defines the CURRENT mechanical position as focus 0, tip 0, tilt 0.")
        out("Do it only with the focal plane at a position you have independently")
        out("established -- every later command is measured from here.")
        for motor in platform.motors:
            out(f"  {motor.name}: {motor.get_position_counts()} counts")
        if not confirm("Set this as the zero reference?", args.yes):
            return 1
        result = platform.set_zero_here(persist=True)
        out(f"Zero set: {result}")
        return 0
    finally:
        platform.disconnect()


def cmd_stop(args) -> int:
    """Controlled stop: decelerate and hold, drives stay enabled."""
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        platform.stop()
        out("All three actuators commanded to stop and hold.")
        return 0
    finally:
        platform.disconnect()


def cmd_passivate(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        if args.force:
            out("--force: the drives will be turned OFF whatever the brakes say.")
            out("With the drives off and the brakes not holding, the focal plane")
            out("rests on screw friction alone and can sink.")
            if not confirm("Turn the drives off anyway?", args.yes):
                return 1
            problems = platform.passivate_all(force=True)
            if problems:
                out("Trouble on: " + "; ".join(problems))
                return 1
            out("Drives off.")
            return 0

        out("Halts all three, engages the brakes, and turns the drives off only")
        out("if the brakes are confirmed holding. Use --force to turn them off")
        out("regardless.")
        if not confirm("Emergency stop all three motors?", args.yes):
            return 1
        result = platform.emergency_stop()
        out("")
        for line in result.summary().splitlines():
            out(line)
        return 0 if result.stopped and result.holding else 1
    finally:
        platform.disconnect()


def cmd_brake(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        if args.action == "status":
            for motor in platform.motors:
                status = motor.get_brake_status()
                tag = " (inferred)" if status.inferred else ""
                out(f"  {motor.name:<4} {status.state.label}{tag}   {status.detail}")
            return 0
        engage = args.action == "engage"
        if not engage:
            out("Releasing a brake removes the mechanical hold on the actuator.")
            out("The drives must be enabled and holding first.")
            if not confirm("Release the brakes?", args.yes):
                return 1
        results = platform.set_all_brakes(engaged=engage)
        failed = False
        for name, result in results.items():
            out(f"  {name:<4} {result}")
            failed = failed or result != "ok"
        return 1 if failed else 0
    finally:
        platform.disconnect()


def cmd_clear_errors(args) -> int:
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        for name, value in platform.clear_all_errors().items():
            if value == 0:
                out(f"  {name:<4} clear")
            elif value < 0:
                out(f"  {name:<4} could not be read")
            else:
                out(f"  {name:<4} still 0x{value:08X} -- latched fault; use MacTalk's "
                    "clear or power-cycle the motor")
        return 0
    finally:
        platform.disconnect()


def cmd_demo(args) -> int:
    """Exercise ONE motor: capabilities, then deliberate faults.

    Deliberately does not use the platform or the kinematics -- it works on a
    single motor in counts and revolutions, so it is useful on a bench with one
    motor and no calibration done.
    """
    from .demo import DemoRunner, all_drills, build_motor, select_drills

    if args.list:
        rule("Available drills")
        out(f"{'name':<22}{'category':<12}{'flags':<22}summary")
        for drill in all_drills():
            flags = []
            if drill.needs_motion:
                flags.append("moves")
            if drill.needs_operator:
                flags.append("needs you")
            out(f"{drill.name:<22}{drill.category:<12}{','.join(flags):<22}"
                f"{drill.summary}")
        out("")
        out("Run a subset with:  --only identity,fault-comms")
        out("Or a whole category with:  --category fault")
        return 0

    try:
        drills = select_drills(
            only=[n.strip() for n in args.only.split(",")] if args.only else None,
            categories=[c.strip() for c in args.category.split(",")] if args.category else None,
            include_operator=not args.no_operator,
        )
    except ValueError as exc:
        out(str(exc))
        return 2
    if not drills:
        out("No drills matched that selection. Use --list to see what exists.")
        return 2

    motor = build_motor(args.config, args.motor, args.simulate)
    try:
        motor.connect(verify_word_order=False)
    except (ModbusError, MotorFault) as exc:
        out(f"Could not connect to motor {args.motor}: {exc}")
        out("")
        out("Check the IP address for this actuator in the config, that the motor")
        out("is powered, and that MacTalk is not holding the connection.")
        out("To try the demo with no hardware at all, add --simulate.")
        return 1

    def ask(prompt: str) -> bool:
        return confirm(prompt, args.yes)

    runner = DemoRunner(motor, out=out, ask=ask,
                        allow_motion=args.allow_motion,
                        range_revs=args.range_revs,
                        passivate_at_end=args.passivate_at_end)
    try:
        return runner.run(drills)
    except KeyboardInterrupt:
        out("")
        out("Interrupted. Stopping the motor and leaving it holding.")
        try:
            runner.injector.clear()
            motor.stop()
            # Deliberately not passivate(): on a loaded axis the drive is the
            # only thing holding it. Ctrl-C should not drop the camera.
            if args.passivate_at_end:
                motor.passivate()
                out("Drive off, as --passivate-at-end asked.")
            else:
                out("The motor is stopped and still holding position.")
        except (ModbusError, MotorFault) as exc:
            out(f"Could not stop cleanly: {exc}")
            out("If the shaft is still turning, remove drive power.")
        return 130
    finally:
        motor.disconnect()


def cmd_safety_check(args) -> int:
    """Provoke each dangerous situation and check the software refuses it."""
    from .safety import run_all

    rule("Safety drills")
    out("Each drill sets up one dangerous situation in simulation and checks")
    out("the software refuses it, and says something useful when it does.")
    out("No hardware is touched: every drill builds its own simulated platform.")
    out("")

    def report(result):
        out(f"[{result.verdict}] {result.name}")
        out(f"        did:      {result.what_was_done}")
        out(f"        result:   {result.what_happened}")
        if result.expected and not result.passed:
            out(f"        expected: {result.expected}")
        out("")

    outcome = run_all(only=args.only.split(",") if args.only else None,
                      report=None if args.json else report)
    if args.json:
        out(json.dumps(outcome.as_dict(), indent=2))
        return 0 if outcome.passed else 1

    rule("Summary")
    passed = len(outcome.results) - len(outcome.failures)
    out(f"  {passed} of {len(outcome.results)} drills passed.")
    if outcome.failures:
        out("")
        out("  A failing drill means a guard is missing or has stopped working:")
        for failure in outcome.failures:
            out(f"    * {failure.name}")
        return 1
    out("")
    out("  Every guard fired. Note what this does NOT prove: the drills use")
    out("  simulated brakes, because the real brake device's protocol is not")
    out("  known yet. They show the interlock logic is right, not that the")
    out("  wiring is.")
    return 0


def cmd_torque_profile(args) -> int:
    """Measure what each motor's torque actually does, and set the threshold.

    The stall limit ships at 45% because that is a reasonable guess from one
    motor's idle reading. A guess is not good enough for the thing that stops
    the actuators driving into their end stops, so this measures the real
    numbers on this machine: what torque reads at rest, what it reads while
    moving freely, and -- if asked -- what it reads pressed against the end.
    A threshold is then recommended from the gap between them.
    """
    import statistics

    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1

    samples: Dict[str, Dict[str, List[float]]] = {
        m.name: {"at rest": [], "moving out": [], "moving back": [],
                 "at the stop": []}
        for m in platform.motors
    }
    # Sampling runs in its own thread throughout, so the interesting moment --
    # the instant an axis meets the stop and torque climbs -- is caught even
    # though it happens in the middle of a blocking call. Sampling only
    # between steps misses it entirely and makes the stop look like a normal
    # move.
    phase = ["at rest"]
    sampling = threading.Event()
    sampling.set()

    def sampler() -> None:
        while sampling.is_set():
            current = phase[0]
            for motor in platform.motors:
                try:
                    percent = motor.get_torque_percent()
                except (ModbusError, MotorFault):
                    percent = None
                if percent is not None:
                    samples[motor.name][current].append(percent)
            time.sleep(0.05)

    sampler_thread = threading.Thread(target=sampler, name="torque-sampler",
                                      daemon=True)

    try:
        rule("Torque profile")
        out("Measures what torque actually reads on these motors, so the stall")
        out("threshold is set from data rather than from a default.")
        out("")
        out(f"  move size     {args.mm} mm, all three together")
        out(f"  current limit "
            f"{platform.motors[0].get_current_limit()} (CL: Current Max, reg 212)")
        if args.to_stop:
            out(f"  and then      approach the end stop in the "
                f"{args.direction} direction")
        out("")
        if not confirm("Run the torque profile?", args.yes):
            return 1

        platform._prepare_for_motion()
        platform.stop()
        sampler_thread.start()

        out("  holding still...")
        time.sleep(2.0)

        out(f"  moving {args.mm:+.3f} mm...")
        phase[0] = "moving out"
        platform.move_relative(d_focus_mm=args.mm)

        out(f"  moving {-args.mm:+.3f} mm back...")
        phase[0] = "moving back"
        platform.move_relative(d_focus_mm=-args.mm)

        found_a_stop = False
        if args.to_stop:
            out("  approaching the end stop...")
            phase[0] = "at the stop"
            direction = 1 if args.direction == "+" else -1
            try:
                result = platform.seek_hard_stop_together(
                    direction=direction, step_mm=args.step_mm,
                    budget_mm=args.budget_mm)
                found_a_stop = bool(result.stopped_by)
                out(f"  stopped by {', '.join(result.stopped_by)}.")
            except (PlatformError, MotorFault) as exc:
                out(f"  the search ended without finding a stop: {exc}")

        sampling.clear()
        sampler_thread.join(timeout=2.0)

        # ---------------------------------------------------------- report
        out("")
        rule("What torque read")
        out(f"{'motor':<7}{'phase':<14}{'n':>5}{'min':>8}{'median':>8}{'max':>8}")
        peaks: Dict[str, float] = {}
        stop_peaks: Dict[str, float] = {}
        for name, phases in samples.items():
            for phase, values in phases.items():
                if not values:
                    continue
                out(f"{name:<7}{phase:<14}{len(values):>5}"
                    f"{min(values):>7.1f}%{statistics.median(values):>7.1f}%"
                    f"{max(values):>7.1f}%")
                if phase.startswith("moving"):
                    peaks[name] = max(peaks.get(name, 0.0), max(values))
                elif phase == "at the stop":
                    stop_peaks[name] = max(stop_peaks.get(name, 0.0), max(values))
            out("")

        rule("What to set")
        if not peaks:
            out("  No torque readings at all. These motors are not reporting")
            out("  Actual Torque (217) or CL: Current Max (212), so stall")
            out("  protection cannot work and `stall_protection` should be set")
            out("  false rather than left on and trusted.")
            return 1

        worst_moving = max(peaks.values())
        # Never below what a healthy move already draws, whatever the stop
        # data says -- a threshold under that aborts ordinary moves.
        floor = max(worst_moving * 1.3, worst_moving + 5.0)
        recommended = min(95.0, max(floor, worst_moving * 1.5))
        out(f"  Hardest a free move worked:  {worst_moving:.1f}%"
            f"  (worst of {', '.join(f'{n} {v:.1f}%' for n, v in peaks.items())})")

        if not args.to_stop:
            out("  No end-stop reading taken; re-run with --to-stop to get one.")
            out("  Without it this recommendation has margin above normal moves")
            out("  but is not known to be below what an obstruction produces.")
        elif not found_a_stop:
            out("  The search did not reach a stop, so nothing was measured")
            out("  pressed against the end. Re-run with a larger --budget-mm.")
        elif not stop_peaks:
            out("  A stop was found but no torque was sampled there.")
        else:
            worst_stop = min(stop_peaks.values())
            out(f"  Lowest reading at the stop:  {worst_stop:.1f}%")
            if worst_stop <= worst_moving * 1.2:
                out("")
                out("  ** These overlap. Torque against the end stop is not")
                out("  ** clearly higher than during a normal move, so no")
                out("  ** threshold separates them: set it low and ordinary")
                out("  ** moves abort, set it high and the stop is never")
                out("  ** noticed. On this machine the 'commanded a step and")
                out("  ** barely moved' check is what will find the stop.")
                out("  ** Keep stall_protection on as a backstop; do not rely")
                out("  ** on it alone, and do not lower the threshold to try")
                out("  ** to make it fire.")
            else:
                midpoint = (worst_moving + worst_stop) / 2
                recommended = min(max(floor, midpoint), 95.0)
                out("  Clear separation: a threshold between them will work.")

        out("")
        out(f"  Recommended stall_torque_percent: {recommended:.0f}")
        out(f"  Currently configured:             "
            f"{platform.motors[0].cfg.stall_torque_percent:.0f}")
        out("")
        out("  Put it in the config under each actuator:")
        out(f'      "stall_torque_percent": {recommended:.0f},')
        out(f'      "torque_warn_percent": {max(worst_moving * 1.2, 5.0):.0f},')
        out("")
        out("  The second one only colours the GUI's load bars; it stops")
        out("  nothing. It is worth setting so that 'normal' on the bars means")
        out("  normal for this machine.")
        return 0
    except (PlatformError, MotorFault) as exc:
        out("")
        out(f"Stopped: {exc}")
        return 1
    finally:
        sampling.clear()
        try:
            platform.stop()
        except Exception:  # noqa: BLE001
            pass
        platform.disconnect()


def cmd_supply(args) -> int:
    """Record what the supply reads when it is healthy, in raw units and volts.

    Register 97 is in the drive's own units and this software does not know the
    scale. Two registers in raw units cannot be compared unless they are known
    to share one, which is why the old check -- register 97 against register
    139, 'Acceptance Voltage' -- was wrong: on the bench motor they read 1794
    and 2054, so a motor running happily at 48 V looked like it was below
    threshold and every move would have been refused.

    So the scale is measured instead: read the register with the supply known
    good, write down the voltage beside it, and after that the software can
    both report volts and tell a real supply failure from a healthy reading.
    """
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        rule("Supply voltage")
        out("Register 97 ('Bus voltage') is in the drive's own raw units, and")
        out("nothing in the register map says what they are worth in volts.")
        out("This records the pair once, so the software can report volts and")
        out("recognise a supply that has actually failed.")
        out("")
        out("Do this with the supply ON and healthy.")
        out("")

        wanted = getattr(args, "motor", None)
        if wanted:
            chosen = [m for m in platform.motors if m.name == wanted]
            if not chosen:
                out(f"No actuator called {wanted!r}. "
                    f"Configured: {', '.join(platform.names)}.")
                return 1
            out(f"Recording for {wanted} only.")
            out("")
        else:
            chosen = list(platform.motors)

        readings = {}
        for motor in chosen:
            try:
                raw = motor.read_register("BUS_VOLTAGE")
                acceptance = motor.read_register("ACCEPTANCE_VOLTAGE")
                lowest = motor.read_register("BUS_VOLTAGE_MIN")
            except (ModbusError, MotorFault) as exc:
                out(f"  {motor.name:<6} could not be read: {exc}")
                continue
            readings[motor.name] = raw
            existing = motor.cfg.supply_raw_at_nominal
            out(f"  {motor.name:<6} reads {raw:>8}   "
                f"(acceptance register {acceptance}, lowest ever {lowest})")
            if existing:
                volts = motor.get_supply_volts(raw)
                out(f"         previously recorded {existing} = "
                    f"{motor.cfg.supply_nominal_v:g} V"
                    + (f", so this reading is {volts:.1f} V" if volts else ""))

        if not readings:
            out("")
            out("Nothing could be read. Check the connection first.")
            return 1

        out("")
        if args.volts is not None:
            volts = args.volts
            out(f"Using --volts {volts:g}.")
        else:
            volts = ask_float(
                "What is the supply actually at, in volts (a meter, or the "
                "supply's own display)?")
        if not volts or volts <= 0:
            out("No voltage given, so nothing was recorded.")
            return 1

        for motor in chosen:
            raw = readings.get(motor.name)
            if raw is None:
                continue
            motor.cfg.supply_nominal_v = float(volts)
            motor.cfg.supply_raw_at_nominal = int(raw)

        out("")
        rule("Recorded")
        for motor in chosen:
            if motor.name not in readings:
                continue
            floor = readings[motor.name] * motor.cfg.supply_low_fraction
            out(f"  {motor.name:<6} {readings[motor.name]} raw = {volts:g} V")
            out(f"         a move will be refused below {floor:.0f} raw "
                f"({volts * motor.cfg.supply_low_fraction:.1f} V, "
                f"{motor.cfg.supply_low_fraction:.0%})")
        out("")
        if confirm("Save to the configuration file?", args.yes):
            out(f"  Saved to {platform.save()}")
        return 0
    finally:
        platform.disconnect()


def cmd_find_stop(args) -> int:
    """Run all three actuators out together until the travel ends.

    Always all three. Driving one actuator into its end stop on its own tilts
    the focal plane about the other two ball joints, and the site's experience
    is that this can break something, so there is no option to do it.
    """
    platform = make_platform(args)
    try:
        platform.connect()
    except PlatformError as exc:
        out(str(exc))
        return 1
    try:
        direction = 1 if args.direction == "+" else -1
        towards = "M1 (primary)" if direction > 0 else "M2 (secondary)"
        limits = platform.cfg.limits

        rule("Find hard stop -- all three actuators together")
        out(f"  direction     {args.direction}  towards {towards}")
        out(f"  speed         {args.speed:.0%} of the configured velocity, "
            f"matched in mm/s across all three")
        out(f"  give up after {args.budget_mm} mm")
        out(f"  torque limit  "
            f"{platform.cfg.actuators[0].stall_torque_percent:.0f}% of the "
            f"drive's current limit, over "
            f"{platform.cfg.actuators[0].stall_persist_samples} consecutive readings")
        out(f"  tilt guard    abandon if the three drift more than "
            f"{limits.max_hard_stop_spread_mm:.3f} mm apart")
        for name, mm in zip(platform.names, platform.read_actuator_positions_mm()):
            out(f"  starting at   {name:<5} {mm:+9.4f} mm")
        out("")
        out("All three run out together, continuously, at the same speed. The")
        out("first one to stop halts the other two in the same instant, and they")
        out("are then backed off to match it so the plate ends flat.")
        if not confirm("Run them into the stop?", args.yes):
            return 1

        last = [0.0]

        def progress(step):
            # Continuous motion produces a reading every 50 ms; printing all of
            # them would bury the numbers that matter.
            now = time.monotonic()
            if now - last[0] < 0.5:
                return
            last[0] = now
            out("    " + "  ".join(f"{n} {mm:+9.4f}"
                                   for n, mm in step.positions_mm.items())
                + f"   apart by {step.spread_mm:.4f} mm"
                + f"   load {max(step.torque_percent.values()):.0f}%")

        result = platform.seek_hard_stop_together(
            direction=direction, budget_mm=args.budget_mm,
            speed_fraction=args.speed, progress=progress,
        )
        out("")
        rule("Result")
        for line in result.summary().splitlines():
            out("  " + line if not line.startswith(" ") else line)
        # The end of travel is a measurement; the soft limit was a guess. So
        # the limit follows the stop rather than the other way round.
        focus_mm = sum(result.stop_mm.values()) / len(result.stop_mm)
        out("")
        rule("Limits")
        for note in platform.adopt_hard_stop(direction, focus_mm):
            out(f"  {note}")
        limits = platform.cfg.limits
        out("")
        out(f"  Focus limits are now {limits.min_focus_mm:+.4f} to "
            f"{limits.max_focus_mm:+.4f} mm.")
        if confirm("Save the new limits to the configuration file?", args.yes):
            out(f"  Saved to {platform.save()}")
            out("  The ends of travel are drawn on the GUI's gauge from now on.")

        out("")
        out("Next: `set-zero` wherever you want the reference to be. If the far")
        out("end was derived from the published travel rather than measured,")
        out("run this again in the other direction to confirm it.")
        return 0
    except (PlatformError, MotorFault) as exc:
        out("")
        out(f"Search stopped: {exc}")
        return 1
    finally:
        platform.disconnect()


def cmd_motor_report(args) -> int:
    """Everything the motor will tell you about itself, in one page.

    Includes the only two pieces of history the drive keeps: Follow Error Max
    and Bus Voltage Min. Both are latched extremes with no timestamp, but they
    survive an error being cleared and a move completing, so after an
    intermittent fault they are often the only evidence left.
    """
    from .demo import build_motor
    from .registers import (REGISTERS, describe_errors, describe_mode,
                            describe_status, describe_warnings)

    motor = build_motor(args.config, args.motor, args.simulate)
    try:
        motor.connect(verify_word_order=False)
    except (ModbusError, MotorFault) as exc:
        out(f"Could not connect to motor {args.motor}: {exc}")
        return 1
    try:
        def value(name, default=None):
            try:
                return motor.read_register(name)
            except (ModbusError, MotorFault):
                return default

        rule(f"Motor {args.motor}")
        out(f"  connection        {motor.describe()}")
        out(f"  serial number     {value('MOTOR_SERIAL', '?')}")
        out(f"  motor type        {value('MOTOR_TYPE', '?')}")
        out(f"  hardware revision {value('HARDWARE_REV', '?')}")
        out(f"  program version   {value('PROG_VERSION', '?')}")
        out(f"  encoder type      {value('ENCODER_TYPE', '?')}")

        rule("Where it is")
        encoder = value("P_ENCODER")
        projected = value("P_PROJECTED")
        requested = value("P_SOLL")
        follow = value("FLWERR")
        out(f"  requested position (reg 3)    {requested}")
        out(f"  projected position (reg 10)   {projected}   <- profile output, "
            "reaches the target by construction")
        out(f"  encoder position   (reg 16)   {encoder}   <- where the shaft is")
        out(f"  follow error       (reg 20)   {follow}")
        out(f"  actual velocity    (reg 12)   {value('V_IST')}")
        out(f"  actual torque      (reg 217)  {value('ACTUAL_TORQUE')}")

        rule("What it is doing")
        mode = value("MODE_REG")
        out(f"  operating mode     {describe_mode(mode) if mode is not None else '?'}")
        out(f"  startup mode       {describe_mode(value('STARTUP_MODE', -1))}")
        out(f"  max velocity       {value('V_SOLL')}")
        out(f"  acceleration       {value('A_SOLL')}")
        out(f"  running current    {value('RUN_CURRENT')}")
        out(f"  standby current    {value('STANDBY_CURRENT')}")

        rule("Health now")
        errors = value("ERR_BITS", 0)
        warnings = value("WARN_BITS", 0)
        out(f"  errors   (reg 35)  {describe_errors(errors)}")
        out(f"  warnings (reg 36)  {describe_warnings(warnings)}")
        out(f"  status   (reg 25)  {describe_status(value('STATUSBITS', 0))}")
        out(f"  temperature        {value('TEMPERATURE_LOW_RES')} C "
            f"(raw {value('TEMPERATURE')})")

        rule("History the motor keeps")
        out("  These two registers are latched extremes. They have no timestamp,")
        out("  but they survive a cleared error and a completed move, so after an")
        out("  intermittent fault they are often the only evidence left.")
        out("")
        out(f"  follow error max (reg 22)  {value('FLWERR_MAX')}"
            "   <- worst lag ever seen")
        out(f"  bus voltage min  (reg 98)  {value('BUS_VOLTAGE_MIN')}"
            "   <- lowest supply ever seen")
        out(f"  bus voltage now  (reg 97)  {value('BUS_VOLTAGE')}")
        out(f"  ticks            (reg 202) {value('TICKS')}"
            "   <- resets when the motor restarts")
        out("")
        out("  The motor keeps NO error history. Registers 35 and 36 are")
        out("  instantaneous, so a fault that has cleared leaves no trace in the")
        out("  drive at all. Use `cli watch` to record one yourself.")

        rule("Things that silently stop motion")
        out(f"  position limit min/max (28/30)  {value('POS_LIMIT_MIN')} / "
            f"{value('POS_LIMIT_MAX')}   (0/0 = no drive limit)")
        out(f"  modbus slave timeout   (199)    {value('MODBUS_TIMEOUT_MS')} ms"
            "   (0 = watchdog off)")
        out(f"  modbus slave action    (200)    {value('MODBUS_ACTION')}")
        out(f"  brake output           (179)    {value('BRAKE_OUTPUT')}"
            "   (0 = no output drives a brake)")
        out(f"  in-position window     (33)     {value('IN_POSITION_WINDOW')} counts")
        out(f"  negative/positive limit inputs  {value('NEG_LIMIT_INPUT')} / "
            f"{value('POS_LIMIT_INPUT')}   (0 = none assigned)")

        mismatch = motor.check_brake_configuration()
        if mismatch:
            out("")
            out(f"  NOTE: {mismatch}")

        if args.all_registers:
            rule("Every register this software knows")
            for reg in REGISTERS:
                out(f"  {reg.number:>4}  {reg.name:<22} {value(reg.name, '<unreadable>')}")
        return 0
    finally:
        motor.disconnect()


def cmd_motor_gui(args) -> int:
    """Bench GUI for a single motor."""
    from .single_gui import main as single_main
    return single_main(motor_name=args.motor, config_path=args.config,
                       simulate=args.simulate, log_path=args.log,
                       poll_interval_s=args.poll)


def cmd_diagnose(args) -> int:
    """Answer 'why is this motor not taking position commands?'"""
    from .demo import build_motor
    from .diagnostics import diagnose

    motor = build_motor(args.config, args.motor, args.simulate)
    try:
        motor.connect(verify_word_order=False)
    except (ModbusError, MotorFault) as exc:
        out(f"Could not connect to motor {args.motor}: {exc}")
        return 1
    try:
        result = diagnose(motor, probe_writes=not args.no_write_probe)
        rule(f"Diagnosis for motor {args.motor}")
        out(result.as_text())
        out("")
        if result.blockers:
            out(f"{len(result.blockers)} thing(s) would stop this motor moving.")
            return 1
        out("Nothing found that would stop this motor moving.")
        return 0
    finally:
        motor.disconnect()


def cmd_watch(args) -> int:
    """Record what the motor does, so a later hang can be explained."""
    from .demo import build_motor
    from .eventlog import EventLog, MotorWatcher, default_log_path

    path = args.log or default_log_path(args.motor)
    motor = build_motor(args.config, args.motor, args.simulate)
    try:
        motor.connect(verify_word_order=False)
    except (ModbusError, MotorFault) as exc:
        out(f"Could not connect to motor {args.motor}: {exc}")
        return 1

    log = EventLog(path=path, on_event=lambda e: out(e.as_line()))
    watcher = MotorWatcher(motor, log, interval_s=args.interval,
                           slow_transaction_s=args.slow_threshold)
    out(f"Recording motor {args.motor} to {path}")
    out("Only changes are logged, not every poll. Press Ctrl-C to stop.")
    out("")
    watcher.start()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        out("")
    finally:
        watcher.stop()
        log.close()
        motor.disconnect()
        out(f"Recording saved to {path}")
    return 0


def cmd_show_log(args) -> int:
    """Replay a recorded log, newest last."""
    from .eventlog import read_log
    events = read_log(args.file)
    if not events:
        out(f"No events in {args.file}")
        return 1
    from .eventlog import _SEVERITY_ORDER
    floor = _SEVERITY_ORDER.get(args.severity, 0)
    shown = [e for e in events if _SEVERITY_ORDER.get(e.severity, 0) >= floor]
    rule(f"{args.file}  ({len(shown)} of {len(events)} events)")
    for event in shown:
        out(event.as_line())
        for key, value in event.data.items():
            out(f"{'':>12}{key} = {value}")
    return 0


def cmd_gui(args) -> int:
    from .gui import main as gui_main
    return gui_main(config_path=args.config, simulate=args.simulate,
                    bench=args.bench, sim_speed=args.sim_speed,
                    poll_interval_s=args.poll)


# --------------------------------------------------------------------------
# Argument parsing
# --------------------------------------------------------------------------

def _add_global_args(p: argparse.ArgumentParser,
                     suppress_defaults: bool = False) -> None:
    """Declare the options that every command accepts.

    `suppress_defaults` is for the copy attached to each subcommand: with
    SUPPRESS, argparse only sets the attribute when the flag is actually
    given, so the copy cannot overwrite a value the top-level parser already
    took from the same flag written before the subcommand.
    """
    extra = {"default": argparse.SUPPRESS} if suppress_defaults else {}
    p.add_argument("--config", help="path to the configuration JSON", **extra)
    p.add_argument("--simulate", action="store_true", **extra,
                   help="run against built-in fake motors, no hardware needed")
    p.add_argument("-y", "--yes", action="store_true", **extra,
                   help="answer yes to confirmations (for scripts)")
    p.add_argument("--bench", metavar="MOTOR", **extra,
                   help="bench mode: MOTOR is real, the other two are "
                        "simulated. Lets the whole three-axis application be "
                        "exercised against the one motor you have.")
    p.add_argument("--sim-speed", type=float, metavar="MM_PER_S", **extra,
                   help="how fast a simulated actuator runs at full velocity, "
                        "in mm/s (default 2). Only affects simulated axes.")
    p.add_argument("--poll", type=float, metavar="SECONDS", **extra,
                   help="seconds between status polls while moving (default "
                        "0.15). Lower is more responsive and more Modbus "
                        "traffic.")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m psct_motors.cli",
        description="Control and commissioning for the pSCT focal-plane actuators.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
commissioning order
-------------------
  init-config          write a starting config, then edit it
  detect               confirm the Modbus word order on each motor
  verify-registers     check the register map against MacTalk
  check-direction      confirm which way each actuator pushes  (per motor)
  calibrate            measure counts per millimetre           (per motor)
  probe-brake          confirm brake control and polarity      (per motor)
  supply               record what the bus-voltage register reads with the
                       supply healthy, so volts can be shown and a failed
                       supply can be told from a normal reading
  set-zero             define the reference orientation
  status / move        normal operation
  history / go-back    where the focal plane has been, and back to the
                       position before the last move

one motor on a bench
--------------------
  motor-gui            live GUI: state, errors, fault injection, event log
  motor-report         one page of everything the motor reports
  find-stop            drive an actuator into its end stop, watching torque
  demo                 exercise a single motor and deliberately provoke
                       faults, to see the error handling work
  diagnose             explain why a motor is not taking position commands
  watch                record changes to a log file, to explain a later hang
  show-log             replay a recording
""",
    )
    _add_global_args(parser)

    # The same options are accepted on either side of the subcommand, so that
    # both `cli --simulate gui` and `cli gui --simulate` work. Argparse will
    # not do that on its own: an option declared only on the top-level parser
    # is rejected once the subcommand has been seen. So they are declared a
    # second time on every subcommand, via this parent -- with SUPPRESS
    # defaults, so that the copy leaves the value alone when the flag was
    # given before the subcommand instead of overwriting it with its default.
    common = argparse.ArgumentParser(add_help=False)
    _add_global_args(common, suppress_defaults=True)

    sub = parser.add_subparsers(dest="command", required=True)

    def command(name: str, **kwargs) -> argparse.ArgumentParser:
        return sub.add_parser(name, parents=[common], **kwargs)

    p = command("init-config", help="write a starting configuration file")
    p.add_argument("--force", action="store_true", help="overwrite an existing file")
    p.set_defaults(func=cmd_init_config)

    p = command("show-config", help="print the active configuration")
    p.set_defaults(func=cmd_show_config)

    p = command("status", help="read positions, orientation, brakes and errors")
    p.add_argument("--json", action="store_true", help="machine-readable output")
    p.set_defaults(func=cmd_status)

    p = command("verify-registers",
                       help="print the register map beside live values")
    p.add_argument("--offline", action="store_true",
                   help="print the table only, without connecting")
    p.set_defaults(func=cmd_verify_registers)

    p = command("detect", help="detect the Modbus word order on each motor")
    p.set_defaults(func=cmd_detect)

    p = command("check-direction",
                       help="confirm which way an actuator moves the focal plane")
    p.add_argument("--motor", required=True, help="actuator name, e.g. Top")
    p.add_argument("--mm", type=float, default=0.5,
                   help="test move size in mm (default 0.5)")
    p.set_defaults(func=cmd_check_direction)

    p = command("calibrate", help="measure counts per millimetre")
    p.add_argument("--motor", required=True, help="actuator name, e.g. Top")
    p.add_argument("--counts", type=int, default=409600,
                   help="counts to move for the test (default 409600, one motor rev)")
    p.add_argument("--measured-mm", type=float,
                   help="measured displacement, if you already have it")
    p.set_defaults(func=cmd_calibrate)

    p = command("probe-brake", help="toggle a brake and confirm it responds")
    p.add_argument("--motor", required=True)
    p.add_argument("--cycles", type=int, default=2)
    p.set_defaults(func=cmd_probe_brake)

    p = command("preview", help="show a move's actuator targets without moving")
    _add_orientation_args(p)
    p.set_defaults(func=cmd_preview)

    p = command("move", help="move to an absolute orientation")
    _add_orientation_args(p)
    p.add_argument("--no-wait", action="store_true",
                   help="return as soon as the move is commanded")
    p.set_defaults(func=cmd_move)

    p = command("move-rel", help="move relative to the current orientation")
    p.add_argument("--dfocus", type=float, default=0.0, help="mm")
    p.add_argument("--dtip", type=float, default=0.0, help="degrees about +x")
    p.add_argument("--dtilt", type=float, default=0.0, help="degrees about +y")
    p.add_argument("--no-wait", action="store_true")
    p.set_defaults(func=cmd_move_relative)

    p = command("go-back", help="return to where the focal plane was before "
                                "the most recent move")
    p.add_argument("--no-wait", action="store_true")
    p.set_defaults(func=cmd_go_back)

    p = command("history", help="list where the focal plane has been, "
                                "newest first")
    p.add_argument("--last", type=int, default=30,
                   help="how many moves to show (default 30)")
    p.add_argument("--json", action="store_true")
    p.set_defaults(func=cmd_history)

    p = command("jog", help="move a single actuator (commissioning)")
    p.add_argument("--motor", required=True)
    p.add_argument("--mm", type=float, required=True, help="relative move in mm")
    p.set_defaults(func=cmd_jog)

    p = command("set-zero", help="define the current position as the reference")
    p.set_defaults(func=cmd_set_zero)

    p = command("stop", help="controlled stop: decelerate and hold")
    p.set_defaults(func=cmd_stop)

    p = command(
        "passivate",
        help="emergency stop: halt, brakes on, drives off only if it is safe",
        description=(
            "Halts all three actuators and keeps them holding, engages the "
            "brakes, and removes drive power only when the brakes are "
            "confirmed engaged. On this telescope the drives are usually the "
            "only thing holding the focal plane, so turning them off with the "
            "brakes released lets it sink. --force overrides that check."
        ),
    )
    p.add_argument("--force", action="store_true",
                   help="turn the drives off even if the brakes cannot be "
                        "confirmed (for maintenance, after checking by hand)")
    p.set_defaults(func=cmd_passivate)

    p = command("brake", help="engage, release or report the brakes")
    p.add_argument("action", choices=["status", "engage", "release"])
    p.set_defaults(func=cmd_brake)

    p = command("clear-errors", help="best-effort error clear on all motors")
    p.set_defaults(func=cmd_clear_errors)

    p = command(
        "demo",
        help="exercise ONE motor: capabilities and deliberate faults",
        description=(
            "Single-motor demo and fault drills. Works on a bench with one "
            "motor, in counts and revolutions, with no calibration needed. "
            "Nothing turns the shaft unless you pass --allow-motion."
        ),
    )
    p.add_argument("--motor", default="Top",
                   help="actuator name from the config (default Top)")
    p.add_argument("--allow-motion", action="store_true",
                   help="permit drills that turn the shaft")
    p.add_argument("--range-revs", type=float, default=2.0,
                   help="how far the shaft may turn either way, in revolutions "
                        "(default 2)")
    p.add_argument("--only", help="comma-separated drill names to run")
    p.add_argument("--category", help="comma-separated categories: capability, "
                                      "fault, real-fault")
    p.add_argument("--no-operator", action="store_true",
                   help="skip drills that ask you to unplug things")
    p.add_argument("--list", action="store_true", help="list the drills and exit")
    p.add_argument("--passivate-at-end", action="store_true",
                   help="turn the drive off when the run finishes. Off by "
                        "default: on a loaded axis, passivating removes the "
                        "only thing holding it")
    p.set_defaults(func=cmd_demo)

    p = command(
        "motor-gui",
        help="bench GUI for ONE motor: live state, errors, fault injection, log",
        description=(
            "Single-motor bench GUI. Works in revolutions, needs no "
            "calibration, records an event log to disk as it runs."
        ),
    )
    p.add_argument("--motor", default="Top", help="actuator name (default Top)")
    p.add_argument("--log", help="path for the JSONL event log")
    p.set_defaults(func=cmd_motor_gui)

    p = command(
        "safety-check",
        help="provoke each dangerous situation and check it is refused",
        description=(
            "Runs the safety drills: brakes on, brake supply off, no drive "
            "power, a brake released with nothing holding the load, limits, "
            "stalls, and a motor lost mid-move. Everything runs against "
            "simulated motors, so it is safe to run at any time and proves "
            "the guards still fire."
        ),
    )
    p.add_argument("--only", help="comma-separated words to match drill names")
    p.add_argument("--json", action="store_true", help="machine-readable output")
    p.set_defaults(func=cmd_safety_check)

    p = command(
        "torque-profile",
        help="measure what torque really reads, and set the stall threshold",
        description=(
            "The stall limit ships at 45%, which is a guess from one motor's "
            "idle reading. This measures the real numbers on your machine -- "
            "at rest, moving freely, and optionally pressed against the end "
            "stop -- and recommends a threshold from the gap between them. "
            "Run it before trusting the over-torque protection, and again "
            "after anything mechanical changes."
        ),
    )
    p.add_argument("--mm", type=float, default=0.5,
                   help="how far to move while sampling (default 0.5)")
    p.add_argument("--to-stop", action="store_true",
                   help="also approach the end stop, to see what it reads "
                        "there. Moves all three together.")
    p.add_argument("--direction", choices=["+", "-"], default="+",
                   help="which end to approach with --to-stop")
    p.add_argument("--step-mm", type=float, default=0.2)
    p.add_argument("--budget-mm", type=float, default=30.0)
    p.set_defaults(func=cmd_torque_profile)

    p = command(
        "supply",
        help="record what the supply reads when healthy, so volts can be shown",
        description=(
            "Register 97 is in the drive's own raw units and the register map "
            "does not say what they are worth. Read it with the supply known "
            "good, say what the voltage actually is, and the software can then "
            "report volts and tell a real supply failure from a normal "
            "reading. Until this is done it cannot do either, and says so "
            "rather than guessing."
        ),
    )
    p.add_argument("--volts", type=float,
                   help="the supply voltage, if you would rather not be asked")
    p.add_argument("--motor",
                   help="record for this actuator only (default: all of them). "
                        "Use this on the bench, where only one motor is real: "
                        "a reading taken from a simulated stand-in is not a "
                        "measurement of anything")
    p.set_defaults(func=cmd_supply)

    p = command(
        "find-stop",
        help="run the actuators out into their end stops, watching torque",
        description=(
            "The site's calibration procedure -- run the actuators out until "
            "they stop -- with torque watched so they stop when something "
            "resists rather than continuing to push. All three move together, "
            "continuously and at a matched speed; there is no way to do it "
            "with one, because taking a single actuator to its stop tilts the "
            "focal plane about the other two ball joints."
        ),
    )
    p.add_argument("--direction", choices=["+", "-"], default="+",
                   help="+ towards M1 (primary), - towards M2 (secondary)")
    p.add_argument("--speed", type=float, default=0.25,
                   help="fraction of each actuator's configured velocity to "
                        "use, matched in mm/s across all three (default 0.25)")
    p.add_argument("--budget-mm", type=float, default=30.0,
                   help="give up after this much travel (default 30)")
    p.set_defaults(func=cmd_find_stop)

    p = command(
        "motor-report",
        help="one page of everything the motor reports, including its latched history",
    )
    p.add_argument("--motor", default="Top")
    p.add_argument("--all-registers", action="store_true",
                   help="also dump every register this software knows about")
    p.set_defaults(func=cmd_motor_report)

    p = command(
        "diagnose",
        help="explain why a motor is not taking position commands",
    )
    p.add_argument("--motor", default="Top")
    p.add_argument("--no-write-probe", action="store_true",
                   help="skip the P_SOLL readback probe (which commands the "
                        "position the motor is already at, so cannot move it)")
    p.set_defaults(func=cmd_diagnose)

    p = command(
        "watch",
        help="record mode, error, target and timing changes to a log file",
    )
    p.add_argument("--motor", default="Top")
    p.add_argument("--log", help="path for the JSONL event log")
    p.add_argument("--interval", type=float, default=0.5,
                   help="seconds between polls (default 0.5)")
    p.add_argument("--slow-threshold", type=float, default=1.0,
                   help="log any poll slower than this many seconds (default 1)")
    p.set_defaults(func=cmd_watch)

    p = command("show-log", help="replay a recorded event log")
    p.add_argument("file")
    p.add_argument("--severity", default="DEBUG",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    p.set_defaults(func=cmd_show_log)

    p = command("gui", help="launch the three-motor focal-plane application")
    p.set_defaults(func=cmd_gui)


    return parser


def _add_orientation_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--focus", type=float, required=True, help="mm along the optical axis")
    p.add_argument("--tip", type=float, help="degrees about +x")
    p.add_argument("--tilt", type=float, help="degrees about +y")
    p.add_argument("--tilt-magnitude", type=float,
                   help="total tilt in degrees (polar form; use with --azimuth)")
    p.add_argument("--azimuth", type=float,
                   help="azimuth of the tilt in degrees CCW from +x")


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except KeyboardInterrupt:
        out("")
        out("Interrupted. Note that this did NOT stop a move already running on the")
        out("motors -- run `stop` if anything is still in motion.")
        return 130
    except (PlatformError, MotorFault, ModbusError) as exc:
        out(f"Error: {exc}")
        return 1
    except (ValueError, KeyError) as exc:
        out(f"Configuration error: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
